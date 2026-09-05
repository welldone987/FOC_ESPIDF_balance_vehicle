#include "ble_idf_service.h"

#include "command_input.h"
#include "vehicle_config.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <atomic>
#include <stddef.h>

namespace vehicle {
namespace {

// kLogTag标识同步初始化失败日志，蓝牙回调保持无日志输出。
constexpr char kLogTag[]{"ble_idf_service"};

// kGattApplicationId标识本服务向GATTS注册的应用编号。
constexpr uint16_t kGattApplicationId{0U};
// kServiceInstanceId标识静态属性表使用的服务实例编号。
constexpr uint8_t kServiceInstanceId{0U};
// kAdvertisingDataPending标记广播数据配置回调尚未完成。
constexpr uint8_t kAdvertisingDataPending{1U << 0U};
// kScanResponseDataPending标记扫描响应配置回调尚未完成。
constexpr uint8_t kScanResponseDataPending{1U << 1U};

// AttributeIndex固定静态属性表与运行时句柄数组的对应顺序。
enum AttributeIndex : size_t {
  kServiceAttribute,
  kCommandDeclarationAttribute,
  kCommandValueAttribute,
  kAttributeCount,
};

// service_uuid按config::ble_service_uuid的128-bit UUID低字节在前保存。
uint8_t service_uuid[ESP_UUID_LEN_128]{
    0x9eU, 0xcaU, 0xdcU, 0x24U, 0x0eU, 0xe5U, 0xa9U, 0xe0U,
    0x93U, 0xf3U, 0xa3U, 0xb5U, 0x01U, 0x00U, 0x40U, 0x6eU};
// command_uuid按config::ble_characteristic_uuid的128-bit UUID低字节在前保存。
uint8_t command_uuid[ESP_UUID_LEN_128]{
    0x9eU, 0xcaU, 0xdcU, 0x24U, 0x0eU, 0xe5U, 0xa9U, 0xe0U,
    0x93U, 0xf3U, 0xa3U, 0xb5U, 0x02U, 0x00U, 0x40U, 0x6eU};

// primary_service_uuid标识GATT主服务属性的16-bit UUID。
uint16_t primary_service_uuid{ESP_GATT_UUID_PRI_SERVICE};
// characteristic_declaration_uuid标识GATT特征声明属性的16-bit UUID。
uint16_t characteristic_declaration_uuid{ESP_GATT_UUID_CHAR_DECLARE};
// command_properties为命令特征启用READ和WRITE属性。
uint8_t command_properties{
    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE};
// welcome_value保存命令特征的初始可读值。
char welcome_value[]{"欢迎来到平衡车"};

// gatt_database按服务、命令特征声明和命令特征值顺序定义静态属性表。
esp_gatts_attr_db_t gatt_database[kAttributeCount]{
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16,
      reinterpret_cast<uint8_t *>(&primary_service_uuid),
      ESP_GATT_PERM_READ,
      sizeof(service_uuid),
      sizeof(service_uuid),
      service_uuid}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16,
      reinterpret_cast<uint8_t *>(&characteristic_declaration_uuid),
      ESP_GATT_PERM_READ,
      sizeof(command_properties),
      sizeof(command_properties),
      &command_properties}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_128,
      command_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      static_cast<uint16_t>(config::maximum_ble_command_length),
      static_cast<uint16_t>(sizeof(welcome_value) - 1U),
      reinterpret_cast<uint8_t *>(welcome_value)}}};

// advertising_data保存设备名和发射功率所在的广播数据配置。
esp_ble_adv_data_t advertising_data{};
// scan_response_data保存扫描响应中携带的服务UUID配置。
esp_ble_adv_data_t scan_response_data{};
// advertising_parameters保存GAP启动广播时使用的参数。
esp_ble_adv_params_t advertising_parameters{};

// service_state原子保存BLE服务生命周期状态，供其他执行上下文读取。
std::atomic<BleServiceState> service_state{BleServiceState::kStopped};
// output_queue接收BLE回调发布的最新RemoteCommand快照。
QueueHandle_t output_queue{nullptr};
// producer_command保存BLE回调待发布的最新命令状态。
RemoteCommand producer_command{};
// attribute_handles保存静态属性表创建后返回的运行时句柄。
uint16_t attribute_handles[kAttributeCount]{};
// pending_advertising_configuration记录两项异步广播配置的待完成状态。
uint8_t pending_advertising_configuration{0U};
// service_started标记GATT主服务是否已启动。
bool service_started{false};
// advertising_start_pending标记广播启动请求是否等待完成回调。
bool advertising_start_pending{false};

// setServiceState原子更新供其他执行上下文读取的BLE生命周期状态。
void setServiceState(BleServiceState state) {
  service_state.store(state, std::memory_order_release);
}

// setFault清除广播启动待处理标记并把服务置为故障状态。
void setFault() {
  advertising_start_pending = false;
  setServiceState(BleServiceState::kFault);
}

// publishProducerCommand用producer_command覆盖Queue中的最近命令快照。
void publishProducerCommand() {
  if (output_queue != nullptr) {
    (void)xQueueOverwrite(output_queue, &producer_command);
  }
}

// resetRuntimeState清空句柄、命令和状态，并重建广播参数。
void resetRuntimeState() {
  output_queue = nullptr;
  producer_command = RemoteCommand{};
  for (size_t index{0U}; index < kAttributeCount; ++index) {
    attribute_handles[index] = 0U;
  }

  pending_advertising_configuration = 0U;
  service_started = false;
  advertising_start_pending = false;

  advertising_data = esp_ble_adv_data_t{};
  advertising_data.set_scan_rsp = false;
  advertising_data.include_name = true;
  advertising_data.include_txpower = true;
  advertising_data.min_interval = 0x20;
  advertising_data.max_interval = 0x40;
  advertising_data.flag =
      ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;

  scan_response_data = esp_ble_adv_data_t{};
  scan_response_data.set_scan_rsp = true;
  scan_response_data.service_uuid_len = sizeof(service_uuid);
  scan_response_data.p_service_uuid = service_uuid;

  advertising_parameters = esp_ble_adv_params_t{};
  advertising_parameters.adv_int_min = 0x20;
  advertising_parameters.adv_int_max = 0x40;
  advertising_parameters.adv_type = ADV_TYPE_IND;
  advertising_parameters.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
  advertising_parameters.channel_map = ADV_CHNL_ALL;
  advertising_parameters.adv_filter_policy =
      ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
}

// rollbackBluetoothStack按Bluedroid到Controller的逆序释放已初始化的协议栈。
void rollbackBluetoothStack() {
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
    (void)esp_bluedroid_disable();
  }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
    (void)esp_bluedroid_deinit();
  }

  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    (void)esp_bt_controller_disable();
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    (void)esp_bt_controller_deinit();
  }
}

// startAdvertisingWhenReady在GATT服务和两项广播配置就绪后发起广播。
void startAdvertisingWhenReady() {
  const BleServiceState current_state{
      service_state.load(std::memory_order_acquire)};
  // 广播请求仅在服务启动、两项配置完成、未连接且无重复请求时发起。
  if (current_state == BleServiceState::kFault ||
      current_state == BleServiceState::kConnected ||
      !service_started || pending_advertising_configuration != 0U ||
      advertising_start_pending) {
    return;
  }

  // 广播启动请求返回成功后等待完成事件，再进入可广播状态。
  const esp_err_t result{
      esp_ble_gap_start_advertising(&advertising_parameters)};
  if (result != ESP_OK) {
    setFault();
    return;
  }
  advertising_start_pending = true;
}

// handleCommandWrite把命令特征的普通写入解析为最新RemoteCommand。
void handleCommandWrite(
    const esp_ble_gatts_cb_param_t::gatts_write_evt_param &write) {
  // 普通命令写入需匹配命令特征、非prepared、零偏移并携带载荷。
  if (write.handle != attribute_handles[kCommandValueAttribute] ||
      write.is_prep || write.offset != 0U || write.len == 0U ||
      write.value == nullptr) {
    return;
  }

  const CommandParseResult result{parseRemoteCommand(write.value, write.len)};
  // 保留旧接口语义：非空写入总是覆盖控制值，只有含逗号时更新时间戳和序号。
  producer_command.steering_voltage_v = result.command.steering_voltage_v;
  producer_command.throttle_velocity_rad_s =
      result.command.throttle_velocity_rad_s;
  if (result.has_separator) {
    // esp_timer_get_time()的微秒值换算为RemoteCommand接收时刻，单位ms。
    producer_command.received_ms =
        static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
    producer_command.sequence += 1U;
  }
  publishProducerCommand();
}

// gapCallback处理广告数据配置和广播启动的异步回调。
void gapCallback(esp_gap_ble_cb_event_t event,
                 esp_ble_gap_cb_param_t *parameters) {
  if (parameters == nullptr) {
    // parameters缺失时把服务状态更新为故障状态。
    setFault();
    return;
  }

  switch (event) {
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    if (parameters->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      setFault();
      return;
    }
    // 广播数据配置完成后清除对应待处理标记。
    pending_advertising_configuration &=
        static_cast<uint8_t>(~kAdvertisingDataPending);
    startAdvertisingWhenReady();
    break;

  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    if (parameters->scan_rsp_data_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      setFault();
      return;
    }
    // 扫描响应配置完成后清除对应待处理标记。
    pending_advertising_configuration &=
        static_cast<uint8_t>(~kScanResponseDataPending);
    startAdvertisingWhenReady();
    break;

  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    // 广播启动完成事件清除待处理标记。
    advertising_start_pending = false;
    if (parameters->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      setFault();
      return;
    }
    // 广播启动成功后发布可广播状态。
    setServiceState(BleServiceState::kAdvertising);
    break;

  default:
    break;
  }
}

// gattsCallback处理GATT注册、属性表创建、连接和特征写入事件。
void gattsCallback(esp_gatts_cb_event_t event,
                   esp_gatt_if_t gatts_interface,
                   esp_ble_gatts_cb_param_t *parameters) {
  if (parameters == nullptr) {
    // parameters缺失时把服务状态更新为故障状态。
    setFault();
    return;
  }

  switch (event) {
  case ESP_GATTS_REG_EVT:
    if (parameters->reg.status != ESP_GATT_OK ||
        parameters->reg.app_id != kGattApplicationId) {
      setFault();
      return;
    }

    // 注册成功后配置设备名、广告数据和静态GATT属性表。
    if (esp_ble_gap_set_device_name(config::ble_device_name) != ESP_OK) {
      setFault();
      return;
    }

    // 两项广告配置均需收到完成事件后才能启动广播。
    pending_advertising_configuration =
        kAdvertisingDataPending | kScanResponseDataPending;
    if (esp_ble_gap_config_adv_data(&advertising_data) != ESP_OK ||
        esp_ble_gap_config_adv_data(&scan_response_data) != ESP_OK ||
        esp_ble_gatts_create_attr_tab(gatt_database,
                                      gatts_interface,
                                      kAttributeCount,
                                      kServiceInstanceId) != ESP_OK) {
      setFault();
    }
    break;

  case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    if (parameters->add_attr_tab.status != ESP_GATT_OK ||
        parameters->add_attr_tab.num_handle != kAttributeCount ||
        parameters->add_attr_tab.handles == nullptr) {
      setFault();
      return;
    }
    // ESP-IDF返回的句柄顺序与AttributeIndex保持一致。
    for (size_t index{0U}; index < kAttributeCount; ++index) {
      attribute_handles[index] = parameters->add_attr_tab.handles[index];
    }
    if (esp_ble_gatts_start_service(
            attribute_handles[kServiceAttribute]) != ESP_OK) {
      setFault();
    }
    break;

  case ESP_GATTS_START_EVT:
    if (parameters->start.status != ESP_GATT_OK) {
      setFault();
      return;
    }
    // 主服务启动完成后等待广告配置回调再开始广播。
    service_started = true;
    startAdvertisingWhenReady();
    break;

  case ESP_GATTS_CONNECT_EVT:
    // 新连接发布connected状态并停止重复广播。
    producer_command.connected = true;
    publishProducerCommand();
    advertising_start_pending = false;
    setServiceState(BleServiceState::kConnected);
    break;

  case ESP_GATTS_DISCONNECT_EVT:
    // 断连保留最近控制值，更新connected状态并恢复广播。
    producer_command.connected = false;
    publishProducerCommand();
    if (service_state.load(std::memory_order_acquire) ==
        BleServiceState::kFault) {
      return;
    }
    setServiceState(BleServiceState::kStarting);
    startAdvertisingWhenReady();
    break;

  case ESP_GATTS_WRITE_EVT:
    handleCommandWrite(parameters->write);
    break;

  default:
    break;
  }
}

} // namespace

// beginBleIdfCommandService按Controller、Bluedroid、GAP和GATTS顺序启动服务。
esp_err_t beginBleIdfCommandService(QueueHandle_t command_queue) {
  // 调用方必须提供用于发布RemoteCommand的Queue。
  if (command_queue == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }
  // 仅在协议栈和本地服务均处于未启动状态时接受初始化。
  if (service_state.load(std::memory_order_acquire) !=
          BleServiceState::kStopped ||
      esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE ||
      esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    return ESP_ERR_INVALID_STATE;
  }

  resetRuntimeState();
  // output_queue复用调用方Queue，初始空命令先交付给控制侧。
  output_queue = command_queue;
  publishProducerCommand();
  setServiceState(BleServiceState::kStarting);

  esp_bt_controller_config_t controller_config =
      BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  // 依次初始化Controller、Bluedroid、GAP回调、GATTS回调和应用注册。
  const char *initialization_stage{"esp_bt_controller_init"};
  esp_err_t result{esp_bt_controller_init(&controller_config)};
  if (result == ESP_OK) {
    initialization_stage = "esp_bt_controller_enable";
    result = esp_bt_controller_enable(
        static_cast<esp_bt_mode_t>(controller_config.mode));
  }
  if (result == ESP_OK) {
    initialization_stage = "esp_bluedroid_init";
    result = esp_bluedroid_init();
  }
  if (result == ESP_OK) {
    initialization_stage = "esp_bluedroid_enable";
    result = esp_bluedroid_enable();
  }
  if (result == ESP_OK) {
    initialization_stage = "esp_ble_gap_register_callback";
    result = esp_ble_gap_register_callback(gapCallback);
  }
  if (result == ESP_OK) {
    initialization_stage = "esp_ble_gatts_register_callback";
    result = esp_ble_gatts_register_callback(gattsCallback);
  }
  if (result == ESP_OK) {
    initialization_stage = "esp_ble_gatts_app_register";
    result = esp_ble_gatts_app_register(kGattApplicationId);
  }

  if (result != ESP_OK) {
    // 在启动调用上下文中报告原始失败阶段和错误码，再执行资源清理。
    ESP_LOGE(kLogTag, "BLE init failed at %s: %s (0x%x)",
             initialization_stage, esp_err_to_name(result),
             static_cast<unsigned int>(result));
    // 任一步骤失败都释放已经初始化的协议栈并发布故障状态。
    rollbackBluetoothStack();
    setFault();
  }
  return result;
}

BleServiceState bleIdfCommandServiceState() {
  return service_state.load(std::memory_order_acquire);
}

} // namespace vehicle
