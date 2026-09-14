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
 * Read()只做非阻塞排空，取最新一次四通道扫描，不做逐通道等待。
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

// carry保存不足一次扫描的尾巴，保证扫描边界永不失步。
std::array<std::uint8_t, ScanBytes - 1U> carry{};
unsigned carry_length = 0;
// staging是每次排空读取的目标缓冲；latest_scan保存最新一次完整四通道扫描。
// 两个缓冲显式4字节对齐；结果按2字节步长解码。
alignas(4) std::array<std::uint8_t, DmaReadBufferBytes> staging{};
alignas(4) std::array<std::uint8_t, ScanBytes> latest_scan{};
// sample_started_us记录本批采样的保守时刻，单位us。
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

// EntryAt()按结果大小复制一次转换，避免把DMA传输字宽当成结果步长。
adc_digi_output_data_t EntryAt(const std::uint8_t *scan, unsigned index)
{
    adc_digi_output_data_t entry{};
    static_assert(sizeof(entry) >= BytesPerConversion);
    std::memcpy(&entry, scan + index * BytesPerConversion, BytesPerConversion);
    return entry;
}

// DrainDmaScan()非阻塞排空DMA池并保留最新一次完整扫描；无完整扫描时返回false。
bool DrainDmaScan(ErrorInfo *error)
{
    bool have_scan = false;
    for (unsigned attempt = 0; attempt < DmaDrainReadLimit; ++attempt) {
        unsigned prefix = 0;
        if (carry_length != 0) {
            std::memcpy(staging.data(), carry.data(), carry_length);
            prefix = carry_length;
        }
        // room保持4字节对齐，满足驱动返回长度的对齐要求。
        const std::uint32_t room = DmaReadBufferBytes - prefix;
        std::uint32_t got = 0;
        const esp_err_t rc = adc_continuous_read(dma, staging.data() + prefix, room, &got, 0);
        if (rc == ESP_ERR_TIMEOUT) { break; }
        if (rc != ESP_OK) {
            VEHICLE_ERROR(error, rc, current_dma_read, rc, 0, 0, -1, ErrorValue);
            return false;
        }
        const unsigned total = prefix + got;
        const unsigned scans = total / ScanBytes;
        if (scans != 0) {
            std::memcpy(latest_scan.data(), staging.data() + (scans - 1U) * ScanBytes, ScanBytes);
            have_scan = true;
        }
        carry_length = total - scans * ScanBytes;
        if (carry_length != 0) {
            std::memmove(carry.data(), staging.data() + scans * ScanBytes, carry_length);
        }
        // 本次未读满说明池已排空；数据不足一次扫描时保留carry结束。
        if (got < room || scans == 0) { break; }
    }
    return have_scan;
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
    // pattern顺序与current_pins一致，运行期按同一顺序解析。
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
    carry_length = 0;
    ready=true;
    return ESP_OK;
}

esp_err_t Start(ErrorInfo *error)
{
    if (!ready || !dma) { return VEHICLE_ERROR(error, ESP_ERR_INVALID_STATE, current_state,0); }
    if (started) { return ESP_OK; }
    esp_err_t result = adc_continuous_flush_pool(dma);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_start,result); }
    result = adc_continuous_start(dma);
    if (result != ESP_OK) { return VEHICLE_ERROR(error, result, current_dma_start,result); }
    carry_length = 0;
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
    if (!DrainDmaScan(error)) {
        // 池里没有一次完整扫描：DMA停止或读取被长时间抢占，按传感器故障上报。
        return VEHICLE_ERROR(error, ESP_ERR_TIMEOUT, current_dma_read,0, 0, static_cast<float>(ScanBytes), -1, ErrorValue | ErrorThreshold);
    }
    // counts保存本次扫描的四路原始计数，通道顺序必须与pattern一致。
    std::array<int, 4> counts{};
    for (unsigned i = 0; i < counts.size(); ++i) {
        const auto entry = EntryAt(latest_scan.data(), i);
        if (entry.type1.channel != channels[i]) {
            return VEHICLE_ERROR(error, ESP_ERR_INVALID_RESPONSE, current_dma_read,0,
                entry.type1.channel, channels[i], i, ErrorValue | ErrorThreshold | ErrorChannel);
        }
        counts[i] = static_cast<int>(entry.type1.data);
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
    // DMA没有逐样本时间戳：最新一帧的转换时刻落在[read-DmaScanPeriod_us, read]内，
    // 取read-DmaScanPeriod_us作为保守上界；该常量在相邻采样做差时抵消，不影响dt。
    sample_started_us = read_started_us - DmaScanPeriod_us;
    *out = {phase_currents, sample_started_us, true};
    return ESP_OK;
}

} // namespace current_sensor
} // namespace vehicle
