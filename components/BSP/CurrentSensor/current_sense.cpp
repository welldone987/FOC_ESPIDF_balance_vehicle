#include "current_sense.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "board_pins.hpp"
#include "current_sensor_config.hpp"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace vehicle {
namespace current_sensor {
namespace {
/*
 * 静态资源保存ADC1的oneshot校准单元、连续DMA句柄、线性校准句柄和四路零偏。
 * Initialize()用oneshot完成零偏校准后切换到adc_continuous(DMA)，运行期由硬件写池。
 * 回调保存整帧和交付时间；Read()排空池后原子读取最新快照，不做逐通道等待。
 */
// current_pins按M0 A/B、M1 A/B顺序排列四路相电流采样引脚。
constexpr std::array<gpio_num_t, 4> current_pins{
    pins::current_sensor_out_a_M0, pins::current_sensor_out_b_M0,
    pins::current_sensor_out_a_M1, pins::current_sensor_out_b_M1};
// BytesPerConversion和ScanBytes按ESP32的结果格式推导：一次转换结果占2字节。
constexpr unsigned BytesPerConversion = SOC_ADC_DIGI_RESULT_BYTES;
constexpr unsigned ScanBytes = BytesPerConversion * current_pins.size();
static_assert(BytesPerConversion == 2U);
static_assert(DmaFrameBytes % BytesPerConversion == 0U);
static_assert(DmaFrameBytes == ScanBytes * DmaScansPerFrame);
static_assert(ScanBytes < DmaReadBufferBytes);
// oneshot只服务上电零偏校准。
adc_oneshot_unit_handle_t oneshot = nullptr;
// dma保存ADC1连续采样句柄；运行期只由ControlTask访问。
adc_continuous_handle_t dma = nullptr;
// calibration保存line-fitting线性校准句柄，DMA原始值同样使用。
adc_cali_handle_t calibration = nullptr;
// ready为true时Read()才允许访问DMA；started标记DMA已启动。
bool ready = false;
bool started = false;
// channels保存四路GPIO解析出的ADC1通道号。
std::array<adc_channel_t, 4> channels{};
// offsets_mv保存零偏校准得到的每路静态电压，单位mV。
std::array<float, 4> offsets_mv{};

// 回调复制整帧与交付时间，任务原子取走同一份现场；不借用驱动缓冲指针。
struct DmaFrame {
    std::array<std::uint8_t, DmaFrameBytes> bytes{};
    std::int64_t delivered_us{};
    std::uint32_t sequence{};
    bool valid{};
};
portMUX_TYPE frame_mux = portMUX_INITIALIZER_UNLOCKED;
DmaFrame delivered_frame{};
DmaFrame selected_frame{}; // 仅ControlTask访问。
std::uint32_t consumed_sequence = 0;

bool IRAM_ATTR OnDmaFrame(adc_continuous_handle_t, const adc_continuous_evt_data_t *event, void *)
{
    const auto delivered_us = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&frame_mux);
    delivered_frame.delivered_us = delivered_us;
    ++delivered_frame.sequence;
    delivered_frame.valid = event && event->conv_frame_buffer && event->size == DmaFrameBytes;
    if (delivered_frame.valid) {
        std::memcpy(delivered_frame.bytes.data(), event->conv_frame_buffer, DmaFrameBytes);
    }
    portEXIT_CRITICAL_ISR(&frame_mux);
    return false; // GPTimer独占控制通知；回调不打印、不计算电流、不唤醒任务。
}
// staging用于清理驱动池；latest_scan来自selected_frame最后一次完整扫描。
// 两个缓冲显式4字节对齐；结果按2字节步长解码。
alignas(4) std::array<std::uint8_t, DmaReadBufferBytes> staging{};
alignas(4) std::array<std::uint8_t, ScanBytes> latest_scan{};
// sample_started_us记录交付时刻减标称扫描跨度的采样时间估计，单位us。
std::int64_t sample_started_us = 0;

// ReadOneshotMillivolts()按current_pins顺序读取四路电压，仅用于上电零偏校准。
esp_err_t ReadOneshotMillivolts(std::array<int, 4> &values_mv, ErrorInfo *error)
{
    const auto started_us = esp_timer_get_time();
    for (unsigned i = 0; i < values_mv.size(); ++i) {
        int raw_counts = 0;
        esp_err_t result = adc_oneshot_read(oneshot, channels[i], &raw_counts);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_raw,result, raw_counts, 0, i, ErrorValue | ErrorChannel); }
        result = adc_cali_raw_to_voltage(calibration, raw_counts, &values_mv[i]);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_mv,result, raw_counts, 0, i, ErrorValue | ErrorChannel); }
        if (values_mv[i] < AdcMin_mV || values_mv[i] > AdcMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_range,0, values_mv[i], values_mv[i] < AdcMin_mV ? AdcMin_mV : AdcMax_mV, i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
    }
    return esp_timer_get_time() - started_us <= ReadMaxDuration_us
        ? ESP_OK : VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_timeout,0, esp_timer_get_time()-started_us, ReadMaxDuration_us, -1, ErrorValue | ErrorThreshold);
}

// 参照官方例程排空驱动池；实际控制样本取自带交付时间的回调快照。
esp_err_t DrainDmaPool(ErrorInfo *error)
{
    unsigned drained_bytes = 0;
    for (unsigned attempt = 0; attempt < DmaDrainReadLimit; ++attempt) {
        std::uint32_t got = 0;
        const esp_err_t rc = adc_continuous_read(dma, staging.data(), staging.size(), &got, 0);
        if (rc == ESP_ERR_TIMEOUT) { return ESP_OK; }
        if (rc != ESP_OK) {
            return VEHICLE_ERROR(error, rc, current_dma_read, rc, 0, 0, -1, ErrorValue);
        }
        drained_bytes += got;
        // 短读也可能只到达环形缓冲的尾部，继续读取直到非阻塞超时确认池空。
    }
    return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_dma_read, 0,
        drained_bytes, DmaStoreBytes, -1, ErrorValue | ErrorThreshold);
}

} // namespace

void Release()
{
    ready=false;
    if (dma) {
        if (started) { adc_continuous_stop(dma); }
        adc_continuous_deinit(dma);
        dma = nullptr;
    }
    started=false;
    if (calibration) { adc_cali_delete_scheme_line_fitting(calibration); calibration = nullptr; }
    if (oneshot) { adc_oneshot_del_unit(oneshot); oneshot = nullptr; }
}

esp_err_t Initialize(ErrorInfo *error)
{
    ready=false; started=false;
    adc_oneshot_unit_init_cfg_t unit{};
    // 四路采样引脚全部属于ADC1。
    unit.unit_id = ADC_UNIT_1;
    esp_err_t result = adc_oneshot_new_unit(&unit, &oneshot);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_unit,result); }
    adc_oneshot_chan_cfg_t channel{};
    // 四路通道统一使用12dB衰减和12位分辨率。
    channel.atten = ADC_ATTEN_DB_12;
    channel.bitwidth = ADC_BITWIDTH_12;
    for (unsigned i = 0; i < channels.size(); ++i) {
        adc_unit_t actual_unit{};
        result = adc_oneshot_io_to_channel(current_pins[i], &actual_unit, &channels[i]);
        if (result != ESP_OK || actual_unit != ADC_UNIT_1) { return VEHICLE_ERROR(error, result != ESP_OK ? result : ESP_ERR_INVALID_ARG, current_map,result, 0, 0, i, ErrorChannel); }
        result = adc_oneshot_config_channel(oneshot, channels[i], &channel);
        if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_channel,result, 0, 0, i, ErrorChannel); }
    }
    adc_cali_line_fitting_config_t cal{};
    cal.unit_id = ADC_UNIT_1;
    cal.atten = ADC_ATTEN_DB_12;
    cal.bitwidth = ADC_BITWIDTH_12;
    cal.default_vref = AdcDefaultVref_mV;
    result = adc_cali_create_scheme_line_fitting(&cal, &calibration);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_calibration,result); }
    // min_mv和max_mv记录校准期间逐路电压极值，用于检查峰峰噪声。
    std::array<int, 4> min_mv{};
    std::array<int, 4> max_mv{};
    min_mv.fill(AdcMax_mV);
    offsets_mv.fill(0.0f);
    for (unsigned sample_index = 0; sample_index < OffsetSamples; ++sample_index) {
        std::array<int, 4> batch_mv{};
        result = ReadOneshotMillivolts(batch_mv, error);
        if (result != ESP_OK) { return result; }
        for (unsigned i = 0; i < batch_mv.size(); ++i) {
            offsets_mv[i] += batch_mv[i];
            min_mv[i] = std::min(min_mv[i], batch_mv[i]);
            max_mv[i] = std::max(max_mv[i], batch_mv[i]);
        }
        // 仅启动零偏校准使用延时，此时驱动器保持关闭。
        vTaskDelay(1);
    }
    for (unsigned i = 0; i < offsets_mv.size(); ++i) {
        // offsets_mv保存OffsetSamples批次的平均电压，单位mV。
        offsets_mv[i] /= OffsetSamples;
        if (offsets_mv[i] < OffsetMin_mV ||
            offsets_mv[i] > OffsetMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_mean,0, offsets_mv[i], offsets_mv[i] < OffsetMin_mV ? OffsetMin_mV : OffsetMax_mV, i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
    }
    for (unsigned i=0; i<offsets_mv.size(); ++i) {
        if (max_mv[i]-min_mv[i] > OffsetNoise_mV) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, offset_noise,0, max_mv[i]-min_mv[i], OffsetNoise_mV, i, ErrorValue | ErrorThreshold | ErrorChannel); }
    }
    // 校准结束后释放oneshot：连续模式与oneshot不能同时占用ADC1。
    adc_oneshot_del_unit(oneshot); oneshot = nullptr;
    adc_continuous_handle_cfg_t dma_config{};
    dma_config.max_store_buf_size = DmaStoreBytes;
    dma_config.conv_frame_size = DmaFrameBytes;
    // 池满时丢弃最旧数据，保证排空后拿到的是最新扫描。
    dma_config.flags.flush_pool = true;
    result = adc_continuous_new_handle(&dma_config, &dma);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_init,result); }
    // pattern顺序与current_pins一致；DMA存储顺序可能不同，运行期按通道标签归位。
    std::array<adc_digi_pattern_config_t, 4> pattern{};
    for (unsigned i = 0; i < channels.size(); ++i) {
        pattern[i].atten = ADC_ATTEN_DB_12;
        pattern[i].channel = static_cast<std::uint8_t>(channels[i]);
        pattern[i].unit = ADC_UNIT_1;
        pattern[i].bit_width = ADC_BITWIDTH_12;
    }
    adc_continuous_config_t continuous{};
    continuous.pattern_num = pattern.size();
    continuous.adc_pattern = pattern.data();
    continuous.sample_freq_hz = DmaSampleFreq_Hz;
    continuous.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    result = adc_continuous_config(dma, &continuous);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_init,result); }
    adc_continuous_evt_cbs_t callbacks{};
    callbacks.on_conv_done = OnDmaFrame;
    result = adc_continuous_register_event_callbacks(dma, &callbacks, nullptr);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_init,result); }
    ready=true;
    return ESP_OK;
}

esp_err_t Start(ErrorInfo *error)
{
    if (!ready || !dma) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, current_state,0); }
    if (started) { return ESP_OK; }
    esp_err_t result = adc_continuous_flush_pool(dma);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_start,result); }
    // 此时DMA尚未启动，不存在并发回调。
    delivered_frame = {};
    selected_frame = {};
    consumed_sequence = 0;
    result = adc_continuous_start(dma);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_start,result); }
    started = true;
    return ESP_OK;
}

esp_err_t Suspend(ErrorInfo *error)
{
    if (!dma || !started) { return ESP_OK; }
    const esp_err_t result = adc_continuous_stop(dma);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_start,result); }
    started = false;
    return ESP_OK;
}

esp_err_t Read(Sample *out, ErrorInfo *error)
{
    if (!out) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_ARG, current_raw,0); }
    *out = {};
    if (!ready) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_STATE,current_state,0); }
    if (!started) { return VEHICLE_ERROR(error,ESP_ERR_INVALID_STATE,current_state,0); }
    const std::int64_t read_started_us = esp_timer_get_time();
    const esp_err_t drain_result = DrainDmaPool(error);
    if (drain_result != ESP_OK) { return drain_result; }
    portENTER_CRITICAL(&frame_mux);
    selected_frame = delivered_frame;
    portEXIT_CRITICAL(&frame_mux);
    if (!selected_frame.valid || selected_frame.sequence == consumed_sequence) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_dma_read, 0,
            0, DmaFrameBytes, -1, ErrorValue | ErrorThreshold);
    }
    consumed_sequence = selected_frame.sequence;
    std::memcpy(latest_scan.data(), selected_frame.bytes.data() + DmaFrameBytes - ScanBytes, ScanBytes);
    std::array<adc_continuous_data_t, 4> parsed{};
    std::uint32_t parsed_count = 0;
    const esp_err_t parse_result = adc_continuous_parse_data(dma, latest_scan.data(), ScanBytes,
        parsed.data(), &parsed_count);
    if (parse_result != ESP_OK || parsed_count != parsed.size()) {
        return VEHICLE_ERROR(error, parse_result != ESP_OK ? parse_result : ESP_ERR_INVALID_SIZE,
            current_dma_read, parse_result);
    }
    // 四个结果按通道标签归位，拒绝未知/重复通道，保证每路恰好出现一次。
    std::array<int, 4> counts{};
    std::array<bool, 4> seen{};
    for (unsigned i = 0; i < counts.size(); ++i) {
        const auto &entry = parsed[i];
        unsigned slot = 0;
        while (slot < channels.size() && entry.channel != channels[slot]) { ++slot; }
        if (!entry.valid || entry.unit != ADC_UNIT_1 || slot == channels.size() || seen[slot]) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_dma_read,0,
                entry.channel, channels[i], i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
        seen[slot] = true;
        counts[slot] = static_cast<int>(entry.raw_data);
    }
    // phase_mv把原始计数换算为校准后的电压，单位mV。
    std::array<int, 4> phase_mv{};
    for (unsigned i = 0; i < phase_mv.size(); ++i) {
        const esp_err_t rc = adc_cali_raw_to_voltage(calibration, counts[i], &phase_mv[i]);
        if (rc != ESP_OK) { return VEHICLE_ERROR(error, rc, current_mv,rc, counts[i], 0, i, ErrorValue | ErrorChannel); }
        if (phase_mv[i] < AdcMin_mV || phase_mv[i] > AdcMax_mV) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_range,0, phase_mv[i], phase_mv[i] < AdcMin_mV ? AdcMin_mV : AdcMax_mV, i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
    }
    if (esp_timer_get_time() - read_started_us > ReadMaxDuration_us) {
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_timeout,0, esp_timer_get_time()-read_started_us, ReadMaxDuration_us, -1, ErrorValue | ErrorThreshold);
    }
    // currents_A把四路mV减去零偏后换算为相电流，单位A。
    std::array<float, 4> currents_A{};
    std::array<PhaseCurrents,2> phase_currents{};
    for (unsigned i = 0; i < currents_A.size(); ++i) {
        currents_A[i] = (phase_mv[i] - offsets_mv[i]) * 0.001f * Polarity /
            (Shunt_Ohm * AmplifierGain);
        if (!std::isfinite(currents_A[i]) || std::abs(currents_A[i]) > PhaseTrip_A) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, phase_limit,0, currents_A[i], PhaseTrip_A, i, ErrorValue | ErrorThreshold | ErrorChannel); }
    }
    for (unsigned i = 0; i < phase_currents.size(); ++i) {
        const float measured_a_A = currents_A[2 * i];
        const float measured_b_A = currents_A[2 * i + 1];
        if (std::abs(measured_a_A + measured_b_A) > PhaseTrip_A) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, reconstructed_limit,0, -measured_a_A-measured_b_A, PhaseTrip_A, i, ErrorValue | ErrorThreshold | ErrorChannel); }
        // 两相测量重构第三相，仅用于保护，不建立Id环。
        phase_currents[i] = {measured_a_A, measured_b_A, -measured_a_A - measured_b_A};
    }
    // 与原始帧绑定的交付时间减最后扫描跨度；不再从读取时间固定回推整帧周期。
    // ISR延迟仍未计入，该时间不是ADC硬件时间戳，实际采样年龄仍待实测。
    sample_started_us = selected_frame.delivered_us - DmaScanPeriod_us;
    *out = {phase_currents, sample_started_us, true};
    return ESP_OK;
}

} // namespace current_sensor
} // namespace vehicle
