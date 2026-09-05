#include "wifi_telemetry.h"
#include "vehicle_config.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <stdint.h>
#include <stddef.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <stdio.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <esp_wifi_default.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <lwip/ip4_addr.h>
#include <algorithm>
#include <cstring>
#include <lwip/inet.h>
#include <lwip/sockets.h>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>


namespace vehicle {

// 网络只接收值快照；时间为ESP-IDF启动后的单调微秒，线上转换为秒。
struct TelemetryFrame {
  int64_t device_time_us;
  float pitch_deg;
  float left_velocity_rad_s;
  float right_velocity_rad_s;
  float velocity_difference_rad_s;
  float left_target_v;
  float right_target_v;
};

} // namespace vehicle


namespace vehicle {

// WifiStationConfig保存Station连接凭据与重连退避；共存省电策略由模块固定。
struct WifiStationConfig {
  // 构造函数保存Station凭据与重连退避。
  WifiStationConfig(const char *ssidValue,
                    const char *passwordValue,
                    uint32_t initialReconnectDelayMillisecondsValue,
                    uint32_t maximumReconnectDelayMillisecondsValue)
      : ssid{ssidValue},
        password{passwordValue},
        initialReconnectDelayMilliseconds{
            initialReconnectDelayMillisecondsValue},
        maximumReconnectDelayMilliseconds{
            maximumReconnectDelayMillisecondsValue} {}

  // ssid指向待连接AP名称的以null结尾字符串。
  const char *ssid;
  // password指向待连接AP密码的以null结尾字符串。
  const char *password;
  // initialReconnectDelayMilliseconds保存首次重连等待时长，单位ms。
  uint32_t initialReconnectDelayMilliseconds;
  // maximumReconnectDelayMilliseconds限制指数退避的最大等待时长，单位ms。
  uint32_t maximumReconnectDelayMilliseconds;
};

// ESP32只有一个Wi-Fi射频；本模块把Station生命周期设计为单实例。
// beginWifiStation()只应由一个任务调用一次。
// beginWifiStation()初始化Station并启动首次连接流程；配置或初始化失败返回false。
bool beginWifiStation(const WifiStationConfig &config);
// pollWifiStation()处理事件位、重置退避状态并按时发起连接尝试。
void pollWifiStation();
// wifiStationHasIpAddress()报告Station是否已经获得IPv4地址。
bool wifiStationHasIpAddress();
// getWifiStationIpAddress()把当前IPv4地址写入目标缓冲区并返回写入是否成功。
bool getWifiStationIpAddress(char *destination, size_t capacity);

} // namespace vehicle




namespace vehicle {

// TcpTelemetryServerConfig保存TCP监听、慢客户端和连接保活参数。
struct TcpTelemetryServerConfig {
  // 构造函数保存TCP监听、慢客户端和保活参数。
  TcpTelemetryServerConfig(uint16_t portValue,
                           uint32_t stalledClientTimeoutMillisecondsValue,
                           int keepAliveIdleSecondsValue,
                           int keepAliveIntervalSecondsValue,
                           int keepAliveProbeCountValue)
      : port{portValue},
        stalledClientTimeoutMilliseconds{
            stalledClientTimeoutMillisecondsValue},
        keepAliveIdleSeconds{keepAliveIdleSecondsValue},
        keepAliveIntervalSeconds{keepAliveIntervalSecondsValue},
        keepAliveProbeCount{keepAliveProbeCountValue} {}

  // port保存TCP遥测服务监听端口。
  uint16_t port;
  // stalledClientTimeoutMilliseconds保存发送持续阻塞后断开客户端的时限，单位ms。
  uint32_t stalledClientTimeoutMilliseconds;
  // keepAliveIdleSeconds保存TCP空闲多久后发送保活探测，单位s。
  int keepAliveIdleSeconds;
  // keepAliveIntervalSeconds保存TCP保活探测之间的间隔，单位s。
  int keepAliveIntervalSeconds;
  // keepAliveProbeCount保存无响应时允许的保活探测次数。
  int keepAliveProbeCount;
};

// TcpTelemetryServer把TelemetryFrame格式化为CSV并通过单客户端非阻塞TCP连接发送。
class TcpTelemetryServer {
public:
  // 构造函数复制TCP服务配置，socket在update()首次需要时创建。
  explicit TcpTelemetryServer(const TcpTelemetryServerConfig &config);
  // 析构函数关闭客户端、监听器和待发送缓存。
  ~TcpTelemetryServer();

  TcpTelemetryServer(const TcpTelemetryServer &) = delete;
  TcpTelemetryServer &operator=(const TcpTelemetryServer &) = delete;
  TcpTelemetryServer(TcpTelemetryServer &&) = delete;
  TcpTelemetryServer &operator=(TcpTelemetryServer &&) = delete;

  // update()根据网络就绪状态维护监听器、客户端和待发送数据。
  void update(bool networkReady, uint32_t nowMilliseconds);
  // publish()把一帧最新遥测装入固定缓存并尝试非阻塞发送。
  void publish(const TelemetryFrame &frame, uint32_t nowMilliseconds);
  // hasClient()报告当前是否持有已接受的客户端socket。
  bool hasClient() const;

private:
  // pendingBufferCapacity限制协议头或单帧CSV的待发送缓存大小，单位B。
  static constexpr size_t pendingBufferCapacity{256U};

  bool createListener();
  void acceptClient();
  bool setNonBlocking(int socketDescriptor);
  void queueProtocolHeader();
  void flushPending(uint32_t nowMilliseconds);
  void closeClient();
  void closeAllSockets();

  // config_保存本服务实例使用的网络参数。
  TcpTelemetryServerConfig config_;
  // listenerSocket_保存监听连接请求的非阻塞socket，负值表示未创建。
  int listenerSocket_{-1};
  // clientSocket_保存唯一客户端socket，负值表示当前没有客户端。
  int clientSocket_{-1};
  // pendingBuffer_保存尚未完整发送的协议头或CSV字节。
  char pendingBuffer_[pendingBufferCapacity]{};
  // pendingLength_保存待发送数据总长度，单位B。
  size_t pendingLength_{0U};
  // pendingOffset_保存下次send()的起始偏移，单位B。
  size_t pendingOffset_{0U};
  // clientIsBlocked_表示最近一次send()因发送缓冲区不可写而暂缓。
  bool clientIsBlocked_{false};
  // 同一客户端不重复排入同一快照。
  int64_t lastQueuedTimeUs_{-1};
  // clientBlockedSinceMilliseconds_保存本轮发送阻塞的起始时刻，单位ms。
  uint32_t clientBlockedSinceMilliseconds_{0U};
};

} // namespace vehicle

// 仅本翻译单元启用连接日志，不修改预编译SDK的CONFIG。


namespace vehicle {

/*
 * Wi-Fi Station模块把ESP-IDF事件回调转换为事件位，再由网络任务轮询连接状态。
 * 回调只更新EventGroup；重连调用、指数退避和IPv4地址读取均归属于调用pollWifiStation()的任务。
 * 模块保存单一Station生命周期，避免多个调用者同时管理默认网络接口。
 */
namespace {
// stationStartedBit表示ESP-IDF已经发出Station启动事件。
constexpr EventBits_t stationStartedBit{BIT0};
// stationDisconnectedBit表示最近收到Station断开事件。
constexpr EventBits_t stationDisconnectedBit{BIT1};
// stationIpReadyBit表示Station已经获得IPv4地址。
constexpr EventBits_t stationIpReadyBit{BIT2};

// stationEventGroupStorage为事件位提供静态控制块存储。
StaticEventGroup_t stationEventGroupStorage{};
// stationEventGroup保存Wi-Fi回调与网络任务之间的事件状态。
EventGroupHandle_t stationEventGroup{nullptr};
// stationNetworkInterface保存默认Station网络接口。
esp_netif_t *stationNetworkInterface{nullptr};
// 两个handler实例用于保留ESP-IDF事件注册返回的生命周期句柄。
esp_event_handler_instance_t wifiEventHandlerInstance{nullptr};
esp_event_handler_instance_t ipEventHandlerInstance{nullptr};

// activeConfig保存当前Station使用的连接和重连配置副本。
WifiStationConfig activeConfig{nullptr, nullptr, 1000U, 10000U};
// stationInitialized表示ESP-IDF Station初始化流程是否完成。
bool stationInitialized{false};
// connectionAttemptInProgress表示最近一次esp_wifi_connect()仍等待事件结果。
bool connectionAttemptInProgress{false};
// ipAddressReported抑制同一次联网期间重复打印IPv4地址。
bool ipAddressReported{false};
// reconnectDelayMilliseconds保存下一次断线后的退避等待时长，单位ms。
uint32_t reconnectDelayMilliseconds{1000U};
// reconnectWaitMilliseconds保存当前连接尝试前需要等待的时长，单位ms。
uint32_t reconnectWaitMilliseconds{0U};
// lastConnectionAttemptMilliseconds保存最近一次连接尝试的设备时间，单位ms。
uint32_t lastConnectionAttemptMilliseconds{0U};

// monotonicMilliseconds()提供重连策略使用的设备毫秒计数。
uint32_t monotonicMilliseconds() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// logEspError()把ESP-IDF错误码转换为可读名称并输出操作上下文。
void logEspError(const char *operation, esp_err_t error) {
  ESP_LOGE("vehicle_wifi", "%s failed: %s (%d)",
                operation,
                esp_err_to_name(error),
                static_cast<int>(error));
}

// checkEspResult()在ESP_OK时返回成功，否则记录错误并返回false。
bool checkEspResult(const char *operation, esp_err_t error) {
  if (error == ESP_OK) {
    return true;
  }
  logEspError(operation, error);
  return false;
}

// credentialsAreValid()检查凭据长度以及重连退避参数是否满足驱动输入边界。
bool credentialsAreValid(const WifiStationConfig &config) {
  if (config.ssid == nullptr || config.password == nullptr) {
    return false;
  }

  // 上限多检查一个字节，用于识别超过ESP32字段容量的字符串。
  const size_t ssidLength{strnlen(config.ssid, 33U)};
  const size_t passwordLength{strnlen(config.password, 65U)};
  return ssidLength > 0U && ssidLength <= 32U &&
         passwordLength <= 64U &&
         config.initialReconnectDelayMilliseconds > 0U &&
         config.maximumReconnectDelayMilliseconds >=
             config.initialReconnectDelayMilliseconds;
}

// wifiEventHandler()只把Station启动和断开事件映射为EventGroup状态位。
void wifiEventHandler(void *,
                      esp_event_base_t eventBase,
                      int32_t eventId,
                      void *) {
  if (stationEventGroup == nullptr) {
    return;
  }

  if (eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START) {
    // 网络任务消费stationStartedBit后才开始首次连接尝试。
    xEventGroupSetBits(stationEventGroup, stationStartedBit);
  } else if (eventBase == WIFI_EVENT &&
             eventId == WIFI_EVENT_STA_DISCONNECTED) {
    // 断线立即清除IP就绪状态，并通知网络任务安排重连。
    xEventGroupClearBits(stationEventGroup, stationIpReadyBit);
    xEventGroupSetBits(stationEventGroup, stationDisconnectedBit);
  }
}

// ipEventHandler()把获得IPv4地址事件映射为网络可用状态。
void ipEventHandler(void *,
                    esp_event_base_t eventBase,
                    int32_t eventId,
                    void *) {
  if (stationEventGroup == nullptr) {
    return;
  }

  if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
    // 获得IP后清除断线标记并置位网络就绪标记。
    xEventGroupClearBits(stationEventGroup, stationDisconnectedBit);
    xEventGroupSetBits(stationEventGroup, stationIpReadyBit);
  }
}

// scheduleReconnect()记录当前退避等待，并把下一次等待时间按上限指数增加。
void scheduleReconnect(uint32_t nowMilliseconds) {
  connectionAttemptInProgress = false;
  reconnectWaitMilliseconds = reconnectDelayMilliseconds;
  lastConnectionAttemptMilliseconds = nowMilliseconds;
  // remainingDelay限制本次翻倍不超过配置的最大退避时长，单位ms。
  const uint32_t remainingDelay{
      activeConfig.maximumReconnectDelayMilliseconds -
      reconnectDelayMilliseconds};
  reconnectDelayMilliseconds +=
      std::min(reconnectDelayMilliseconds, remainingDelay);
}

// reportIpAddressOnce()在每次获得IP后只读取和打印一次IPv4地址。
void reportIpAddressOnce() {
  if (ipAddressReported) {
    return;
  }

  char address[16]{};
  if (getWifiStationIpAddress(address, sizeof(address))) {
    ESP_LOGI("vehicle_wifi", "station ready, IPv4=%s", address);
  }
  ipAddressReported = true;
}
} // namespace

bool beginWifiStation(const WifiStationConfig &config) {
  esp_log_level_set("vehicle_wifi", ESP_LOG_INFO);
  // 已初始化时复用单一Station生命周期，不重复创建默认接口或注册回调。
  if (stationInitialized) {
    return true;
  }
  if (!credentialsAreValid(config)) {
    ESP_LOGI("vehicle_wifi", "invalid station configuration");
    return false;
  }

  // activeConfig保存退避策略所需的配置，并初始化首次连接计时基准。
  activeConfig = config;
  reconnectDelayMilliseconds = config.initialReconnectDelayMilliseconds;
  reconnectWaitMilliseconds = 0U;
  lastConnectionAttemptMilliseconds = monotonicMilliseconds();

  // 事件组使用静态存储，供ESP-IDF回调发布状态位。
  stationEventGroup =
      xEventGroupCreateStatic(&stationEventGroupStorage);
  if (stationEventGroup == nullptr) {
    ESP_LOGI("vehicle_wifi", "event group creation failed");
    return false;
  }

  // 先初始化网络接口和默认事件循环，再创建Station接口。
  if (!checkEspResult("esp_netif_init", esp_netif_init())) {
    return false;
  }

  const esp_err_t eventLoopResult{esp_event_loop_create_default()};
  if (eventLoopResult != ESP_OK &&
      eventLoopResult != ESP_ERR_INVALID_STATE) {
    logEspError("esp_event_loop_create_default", eventLoopResult);
    return false;
  }

  // 默认Station接口负责承接DHCP获得的IPv4信息。
  stationNetworkInterface = esp_netif_create_default_wifi_sta();
  if (stationNetworkInterface == nullptr) {
    ESP_LOGI("vehicle_wifi", "default station interface creation failed");
    return false;
  }

  // Wi-Fi驱动任务固定在Core 0，与网络应用任务使用相同核心。
  wifi_init_config_t initializationConfig = WIFI_INIT_CONFIG_DEFAULT();
  initializationConfig.wifi_task_core_id = 0;
  if (!checkEspResult("esp_wifi_init",
                      esp_wifi_init(&initializationConfig))) {
    return false;
  }
  if (!checkEspResult("esp_wifi_set_storage",
                      esp_wifi_set_storage(WIFI_STORAGE_RAM))) {
    return false;
  }

  // 回调只发布事件位，不在ESP-IDF事件上下文中执行连接或socket操作。
  if (!checkEspResult(
          "register WIFI_EVENT handler",
          esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              wifiEventHandler,
                                              nullptr,
                                              &wifiEventHandlerInstance))) {
    return false;
  }
  if (!checkEspResult(
          "register IP_EVENT handler",
          esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              ipEventHandler,
                                              nullptr,
                                              &ipEventHandlerInstance))) {
    return false;
  }

  // stationConfig把本地凭据复制到ESP-IDF要求的固定字段中。
  wifi_config_t stationConfig{};
  const size_t ssidLength{std::strlen(config.ssid)};
  const size_t passwordLength{std::strlen(config.password)};
  std::memcpy(stationConfig.sta.ssid, config.ssid, ssidLength);
  std::memcpy(stationConfig.sta.password,
              config.password,
              passwordLength);
  // 快速扫描按信号强度选择满足认证门限的AP。
  stationConfig.sta.scan_method = WIFI_FAST_SCAN;
  stationConfig.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  stationConfig.sta.threshold.rssi = -127;
  stationConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  stationConfig.sta.pmf_cfg.capable = true;
  stationConfig.sta.pmf_cfg.required = false;

  if (!checkEspResult("esp_wifi_set_mode",
                      esp_wifi_set_mode(WIFI_MODE_STA)) ||
      !checkEspResult("esp_wifi_set_config",
                      esp_wifi_set_config(WIFI_IF_STA, &stationConfig)) ||
      !checkEspResult("esp_wifi_start", esp_wifi_start())) {
    return false;
  }

  // 当前ESP-IDF 4.4.7的Wi-Fi/BLE共存要求开启modem sleep。
  // 固定使用MIN_MODEM，避免调用方关闭省电后在Wi-Fi任务内触发abort。
  if (!checkEspResult("esp_wifi_set_ps(WIFI_PS_MIN_MODEM)",
                      esp_wifi_set_ps(WIFI_PS_MIN_MODEM))) {
    return false;
  }

  // 驱动启动成功后，后续连接状态由pollWifiStation()推进。
  stationInitialized = true;
  ESP_LOGI("vehicle_wifi", "ESP-IDF station initialized");
  return true;
}

void pollWifiStation() {
  if (!stationInitialized || stationEventGroup == nullptr) {
    return;
  }

  const uint32_t nowMilliseconds{monotonicMilliseconds()};
  EventBits_t eventBits{xEventGroupGetBits(stationEventGroup)};

  if ((eventBits & stationIpReadyBit) != 0U) {
    // IP有效时清零连接尝试和退避状态，保持网络任务快速返回。
    connectionAttemptInProgress = false;
    reconnectDelayMilliseconds =
        activeConfig.initialReconnectDelayMilliseconds;
    reconnectWaitMilliseconds = 0U;
    reportIpAddressOnce();
    return;
  }

  // 离开IP就绪状态后允许下一次联网重新打印地址。
  ipAddressReported = false;

  if ((eventBits & stationStartedBit) != 0U) {
    // 首次启动事件把连接尝试和等待时间复位到立即可尝试状态。
    xEventGroupClearBits(stationEventGroup, stationStartedBit);
    connectionAttemptInProgress = false;
    reconnectWaitMilliseconds = 0U;
    lastConnectionAttemptMilliseconds = nowMilliseconds;
  }

  eventBits = xEventGroupGetBits(stationEventGroup);
  if ((eventBits & stationDisconnectedBit) != 0U) {
    // 断开事件消费一次后进入当前退避等待。
    xEventGroupClearBits(stationEventGroup, stationDisconnectedBit);
    scheduleReconnect(nowMilliseconds);
  }

  // 正在等待事件或尚未到达退避截止时间时不重复调用连接API。
  if (connectionAttemptInProgress ||
      (nowMilliseconds - lastConnectionAttemptMilliseconds) <
          reconnectWaitMilliseconds) {
    return;
  }

  // 连接API只发起异步尝试，最终结果由Wi-Fi/IP事件回调发布。
  const esp_err_t connectResult{esp_wifi_connect()};
  lastConnectionAttemptMilliseconds = nowMilliseconds;
  if (connectResult == ESP_OK) {
    connectionAttemptInProgress = true;
    ESP_LOGI("vehicle_wifi", "connecting to configured access point");
  } else {
    // 同步调用失败也走退避路径，避免网络任务连续快速重试。
    logEspError("esp_wifi_connect", connectResult);
    scheduleReconnect(nowMilliseconds);
  }
}

bool wifiStationHasIpAddress() {
  return stationInitialized && stationEventGroup != nullptr &&
         (xEventGroupGetBits(stationEventGroup) & stationIpReadyBit) != 0U;
}

bool getWifiStationIpAddress(char *destination, size_t capacity) {
  if (destination == nullptr || capacity < 16U ||
      stationNetworkInterface == nullptr ||
      !wifiStationHasIpAddress()) {
    return false;
  }

  // ipInformation保存默认Station接口当前的IPv4配置。
  esp_netif_ip_info_t ipInformation{};
  if (esp_netif_get_ip_info(stationNetworkInterface,
                            &ipInformation) != ESP_OK) {
    return false;
  }

  // IPSTR把IPv4地址写入调用方提供的至少16字节缓冲区。
  const int written{snprintf(destination,
                             capacity,
                             IPSTR,
                             IP2STR(&ipInformation.ip))};
  return written > 0 && static_cast<size_t>(written) < capacity;
}

} // namespace vehicle

// 仅本翻译单元启用连接日志，不修改预编译SDK的CONFIG。


namespace vehicle {

/*
 * TCP遥测模块把TelemetryFrame格式化为七字段ASCII CSV，并通过单客户端socket发送。
 * listener、client和pendingBuffer_均由网络任务维护，发送使用非阻塞socket避免网络反压传播到采样任务。
 * 新客户端先接收协议头，待发送内容在固定缓存中按偏移续传。
 */
namespace {
// protocolHeader声明协议版本和七个CSV字段的顺序与单位。
constexpr char protocolHeader[]{
    "#balancing_vehicle_tcp,v1\n"
    "#time_s,pitch_deg,left_velocity_rad_s,right_velocity_rad_s,"
    "velocity_difference_rad_s,left_target_v,right_target_v\n"};

// isWouldBlockError()识别非阻塞socket暂时没有可写空间的错误。
bool isWouldBlockError(int error) {
  return error == EAGAIN || error == EWOULDBLOCK;
}

// formatTelemetryCsv()把TelemetryFrame转换为固定格式的单行CSV。
int formatTelemetryCsv(const TelemetryFrame &frame,
                       char *destination,
                       size_t capacity) {
  if (destination == nullptr || capacity == 0U) {
    return -1;
  }

  // 整数拆分64位单调微秒，避免float时间精度损失与32位时钟回绕。
  const int written{snprintf(
      destination, capacity,
      "%lld.%06lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      static_cast<long long>(frame.device_time_us / 1000000),
      static_cast<long long>(frame.device_time_us % 1000000),
      static_cast<double>(frame.pitch_deg),
      static_cast<double>(frame.left_velocity_rad_s),
      static_cast<double>(frame.right_velocity_rad_s),
      static_cast<double>(frame.velocity_difference_rad_s),
      static_cast<double>(frame.left_target_v),
      static_cast<double>(frame.right_target_v))};
  // 格式化失败或超出缓存时清空目标，调用方不会发送不完整帧。
  if (written < 0 || static_cast<size_t>(written) >= capacity) {
    destination[0] = '\0';
    return -1;
  }
  return written;
}
} // namespace

TcpTelemetryServer::TcpTelemetryServer(
    const TcpTelemetryServerConfig &config)
    : config_{config} {
  esp_log_level_set("vehicle_tcp", ESP_LOG_INFO);
}

TcpTelemetryServer::~TcpTelemetryServer() {
  closeAllSockets();
}

void TcpTelemetryServer::update(bool networkReady,
                                uint32_t nowMilliseconds) {
  if (!networkReady) {
    closeAllSockets();
    return;
  }

  if (listenerSocket_ < 0 && !createListener()) {
    return;
  }
  if (clientSocket_ < 0) {
    acceptClient();
  }
  if (clientSocket_ >= 0 && pendingOffset_ < pendingLength_) {
    flushPending(nowMilliseconds);
  }
}

void TcpTelemetryServer::publish(const TelemetryFrame &frame,
                                 uint32_t nowMilliseconds) {
  if (clientSocket_ < 0) {
    return;
  }

  if (pendingOffset_ < pendingLength_) {
    flushPending(nowMilliseconds);
    if (clientSocket_ < 0 || pendingOffset_ < pendingLength_) {
      return;
    }
  }

  if (frame.device_time_us == lastQueuedTimeUs_) {
    return;
  }

  const int formattedLength{
      formatTelemetryCsv(frame, pendingBuffer_, sizeof(pendingBuffer_))};
  if (formattedLength <= 0) {
    ESP_LOGI("vehicle_tcp", "telemetry formatting failed");
    return;
  }

  lastQueuedTimeUs_ = frame.device_time_us;
  pendingLength_ = static_cast<size_t>(formattedLength);
  pendingOffset_ = 0U;
  flushPending(nowMilliseconds);
}

bool TcpTelemetryServer::hasClient() const {
  return clientSocket_ >= 0;
}

bool TcpTelemetryServer::createListener() {
  // listener创建IPv4 TCP监听socket，后续所有操作都在非阻塞模式下完成。
  const int listener{socket(AF_INET, SOCK_STREAM, IPPROTO_IP)};
  if (listener < 0) {
    ESP_LOGI("vehicle_tcp", "socket failed: errno=%d", errno);
    return false;
  }

  // 允许网络恢复后快速重新绑定相同端口。
  int reuseAddress{1};
  if (setsockopt(listener,
                 SOL_SOCKET,
                 SO_REUSEADDR,
                 &reuseAddress,
                 sizeof(reuseAddress)) != 0) {
    ESP_LOGI("vehicle_tcp", "SO_REUSEADDR failed: errno=%d", errno);
    ::close(listener);
    return false;
  }
  // 监听socket不可阻塞，update()可以在没有连接请求时立即返回。
  if (!setNonBlocking(listener)) {
    ::close(listener);
    return false;
  }

  // serverAddress监听所有本地IPv4接口和配置端口。
  sockaddr_in serverAddress{};
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddress.sin_port = htons(config_.port);

  if (bind(listener,
           reinterpret_cast<sockaddr *>(&serverAddress),
           sizeof(serverAddress)) != 0) {
    ESP_LOGI("vehicle_tcp", "bind failed: errno=%d", errno);
    ::close(listener);
    return false;
  }
  // backlog为1与当前单客户端模型保持一致。
  if (listen(listener, 1) != 0) {
    ESP_LOGI("vehicle_tcp", "listen failed: errno=%d", errno);
    ::close(listener);
    return false;
  }

  listenerSocket_ = listener;
  ESP_LOGI("vehicle_tcp", "telemetry server listening on port %u",
                static_cast<unsigned>(config_.port));
  return true;
}

void TcpTelemetryServer::acceptClient() {
  // acceptClient()接收一个客户端，并把它配置为单客户端非阻塞连接。
  sockaddr_in clientAddress{};
  socklen_t clientAddressLength{sizeof(clientAddress)};
  // acceptedSocket保存本轮接受的客户端socket描述符。
  const int acceptedSocket{
      accept(listenerSocket_,
             reinterpret_cast<sockaddr *>(&clientAddress),
             &clientAddressLength)};
  if (acceptedSocket < 0) {
    if (!isWouldBlockError(errno)) {
      ESP_LOGI("vehicle_tcp", "accept failed: errno=%d", errno);
      closeAllSockets();
    }
    return;
  }

  if (!setNonBlocking(acceptedSocket)) {
    ::close(acceptedSocket);
    return;
  }

  // acceptedSocket关闭Nagle并启用TCP保活，以减少小帧等待并发现失联客户端。
  int enabled{1};
  if (setsockopt(acceptedSocket,
                 IPPROTO_TCP,
                 TCP_NODELAY,
                 &enabled,
                 sizeof(enabled)) != 0 ||
      setsockopt(acceptedSocket,
                 SOL_SOCKET,
                 SO_KEEPALIVE,
                 &enabled,
                 sizeof(enabled)) != 0 ||
      setsockopt(acceptedSocket,
                 IPPROTO_TCP,
                 TCP_KEEPIDLE,
                 &config_.keepAliveIdleSeconds,
                 sizeof(config_.keepAliveIdleSeconds)) != 0 ||
      setsockopt(acceptedSocket,
                 IPPROTO_TCP,
                 TCP_KEEPINTVL,
                 &config_.keepAliveIntervalSeconds,
                 sizeof(config_.keepAliveIntervalSeconds)) != 0 ||
      setsockopt(acceptedSocket,
                 IPPROTO_TCP,
                 TCP_KEEPCNT,
                 &config_.keepAliveProbeCount,
                 sizeof(config_.keepAliveProbeCount)) != 0) {
    ESP_LOGI("vehicle_tcp", "client socket configuration failed: errno=%d",
                  errno);
    ::close(acceptedSocket);
    return;
  }

  // 新客户端先排入协议头，数据发送由后续flushPending()逐步完成。
  clientSocket_ = acceptedSocket;
  lastQueuedTimeUs_ = -1;
  queueProtocolHeader();

  char clientIpAddress[16]{};
  inet_ntoa_r(clientAddress.sin_addr,
              clientIpAddress,
              sizeof(clientIpAddress));
  ESP_LOGI("vehicle_tcp", "telemetry client connected from %s",
                clientIpAddress);
}

bool TcpTelemetryServer::setNonBlocking(int socketDescriptor) {
  // setNonBlocking()为监听器或客户端保留原标志并追加O_NONBLOCK。
  const int currentFlags{fcntl(socketDescriptor, F_GETFL, 0)};
  if (currentFlags < 0 ||
      fcntl(socketDescriptor,
            F_SETFL,
            currentFlags | O_NONBLOCK) != 0) {
    ESP_LOGI("vehicle_tcp", "fcntl(O_NONBLOCK) failed: errno=%d", errno);
    return false;
  }
  return true;
}

void TcpTelemetryServer::queueProtocolHeader() {
  // queueProtocolHeader()把协议头复制到固定缓存，等待非阻塞发送。
  const size_t headerLength{sizeof(protocolHeader) - 1U};
  static_assert(headerLength <= pendingBufferCapacity,
                "TCP protocol header does not fit pending buffer");
  std::memcpy(pendingBuffer_, protocolHeader, headerLength);
  pendingLength_ = headerLength;
  pendingOffset_ = 0U;
  clientIsBlocked_ = false;
}

void TcpTelemetryServer::flushPending(uint32_t nowMilliseconds) {
  // flushPending()从pendingOffset_开始发送缓存，直到发送完、暂时阻塞或连接关闭。
  while (clientSocket_ >= 0 && pendingOffset_ < pendingLength_) {
    // remaining保存本轮尚未发送的字节数，单位B。
    const size_t remaining{pendingLength_ - pendingOffset_};
    // send()只尝试当前非阻塞socket，不等待客户端接收窗口。
    const int sent{send(clientSocket_,
                        pendingBuffer_ + pendingOffset_,
                        remaining,
                        MSG_DONTWAIT)};
    if (sent > 0) {
      // sent表示本轮实际发送的字节数，单位B。
      pendingOffset_ += static_cast<size_t>(sent);
      clientIsBlocked_ = false;
      continue;
    }

    if (sent < 0 && isWouldBlockError(errno)) {
      // 暂时不可写时记录阻塞起点，后续网络轮询继续尝试同一缓存。
      if (!clientIsBlocked_) {
        clientIsBlocked_ = true;
        clientBlockedSinceMilliseconds_ = nowMilliseconds;
      } else if ((nowMilliseconds - clientBlockedSinceMilliseconds_) >=
                 config_.stalledClientTimeoutMilliseconds) {
        ESP_LOGI("vehicle_tcp", "client stalled; disconnecting");
        closeClient();
      }
      return;
    }

    // 其他send()错误表示当前客户端不能继续承载待发送数据。
    ESP_LOGI("vehicle_tcp", "send failed or peer closed: errno=%d", errno);
    closeClient();
    return;
  }

  if (pendingOffset_ >= pendingLength_) {
    // 缓存发送完成后清零长度、偏移和阻塞状态，准备接收下一帧。
    pendingLength_ = 0U;
    pendingOffset_ = 0U;
    clientIsBlocked_ = false;
  }
}

void TcpTelemetryServer::closeClient() {
  // closeClient()释放客户端socket并丢弃尚未发送的旧数据。
  if (clientSocket_ >= 0) {
    shutdown(clientSocket_, SHUT_RDWR);
    ::close(clientSocket_);
    clientSocket_ = -1;
    ESP_LOGI("vehicle_tcp", "client disconnected");
  }
  pendingLength_ = 0U;
  pendingOffset_ = 0U;
  clientIsBlocked_ = false;
}

void TcpTelemetryServer::closeAllSockets() {
  // closeAllSockets()先关闭客户端，再关闭监听器，恢复到未联网状态。
  closeClient();
  if (listenerSocket_ >= 0) {
    ::close(listenerSocket_);
    listenerSocket_ = -1;
    ESP_LOGI("vehicle_tcp", "listener closed");
  }
}

} // namespace vehicle
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
namespace WifiCredentials { constexpr char ssid[]{""}; constexpr char password[]{""}; }
#endif
namespace vehicle {
namespace { bool telemetry_started{false}; }
bool beginWifiTelemetry() {
  const WifiStationConfig config{WifiCredentials::ssid, WifiCredentials::password, 1000U, 10000U};
  telemetry_started = beginWifiStation(config);
  return telemetry_started;
}
void serviceWifiTelemetry(const TelemetrySnapshot &snapshot) {
  if (!telemetry_started) return;
  static TcpTelemetryServer server{{config::telemetry_tcp_port, 1000U, 5, 2, 3}};
  pollWifiStation();
  const bool ready{wifiStationHasIpAddress()};
  const uint32_t now_ms{static_cast<uint32_t>(esp_timer_get_time() / 1000)};
  server.update(ready, now_ms);
  if (!ready || snapshot.sequence == 0U) return;
  const float left{config::motor0_direction * snapshot.wheels.left_velocity_rad_s};
  const float right{config::motor1_direction * snapshot.wheels.right_velocity_rad_s};
  const TelemetryFrame frame{snapshot.device_time_us, snapshot.attitude.pitch_deg,
      left, right, left - right, snapshot.control.motor_voltage.left_target_v,
      snapshot.control.motor_voltage.right_target_v};
  server.publish(frame, now_ms);
}
} // namespace vehicle
