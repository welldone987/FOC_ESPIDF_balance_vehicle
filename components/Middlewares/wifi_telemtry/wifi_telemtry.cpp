#include "wifi_telemtry.hpp"

#include "sdkconfig.h"
#include "vehicle_config.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include <fcntl.h>
#include <unistd.h>

namespace vehicle {
namespace wifi_telemtry {
namespace {

/*
 * Wi-Fi事件回调只更新联网状态位，socket建立和数据发送由service()顺序执行。
 * pending保存部分发送帧，sequence保证同一快照不会重复编码到发送队列。
 */

// kWifiReady表示STA已经通过IP_EVENT_STA_GOT_IP获得可用地址。
constexpr EventBits_t kWifiReady = BIT0;
constexpr EventBits_t kConnectRequested = BIT1;
constexpr char kTag[] = "wifi_telemetry";
// 重连、发送阻塞和文本缓冲区参数的单位分别为ms、ms和字节。
constexpr std::uint32_t kReconnectPeriodMs = 1000U;
constexpr std::uint32_t kClientTimeoutMs = 1000U;
constexpr std::size_t kBufferSize = 256U;
constexpr char kHeader[] =
    "#balancing_vehicle_tcp,v1\n"
    "#time_s,pitch_deg,left_velocity_rad_s,right_velocity_rad_s,"
    "velocity_difference_rad_s,left_target_v,right_target_v\n";

StaticEventGroup_t wifi_events_storage{};
// wifi_events由事件回调设置，由服务任务读取。
EventGroupHandle_t wifi_events = nullptr;
esp_event_handler_instance_t wifi_handler = nullptr;
esp_event_handler_instance_t ip_handler = nullptr;
int listener_socket = -1;
int client_socket = -1;
// pending保存TCP尚未发送完的协议头或遥测行。
std::array<char, kBufferSize> pending{};
std::size_t pending_length = 0U;
std::size_t pending_offset = 0U;
std::uint32_t blocked_since_ms = 0U;
std::uint32_t last_connect_ms = 0U;
// 仅由service访问；连接请求成功后等待事件，不重复打断关联或DHCP。
bool reconnect_pending = false;
std::uint32_t last_sequence = 0U;

// nowMilliseconds()把ESP高精度计时器转换为回绕可接受的毫秒时基。
std::uint32_t nowMilliseconds()
{
    return static_cast<std::uint32_t>(esp_timer_get_time() / 1000LL);
}

// wifiEvent()把获得IP和断开事件映射为kWifiReady位的置位或清零。
void wifiEvent(void *, esp_event_base_t base, std::int32_t id, void *event_data)
{
    if (wifi_events == nullptr) {
        return;
    }
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_events, kWifiReady);
        const auto *event = static_cast<const ip_event_got_ip_t *>(event_data);
        ESP_LOGI(kTag, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(wifi_events, kConnectRequested);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_events, kWifiReady);
        const auto *event = static_cast<const wifi_event_sta_disconnected_t *>(event_data);
        ESP_LOGW(kTag, "Disconnected: reason=%u", static_cast<unsigned>(event->reason));
        xEventGroupSetBits(wifi_events, kConnectRequested);
    }
}

// closeClient()关闭当前客户端并清空部分发送状态。
void closeClient()
{
    if (client_socket >= 0) {
        shutdown(client_socket, SHUT_RDWR);
        ::close(client_socket);
        client_socket = -1;
    }
    pending_length = 0U;
    pending_offset = 0U;
    blocked_since_ms = 0U;
}

// closeSockets()同时释放客户端和监听socket。
void closeSockets()
{
    closeClient();
    if (listener_socket >= 0) {
        ::close(listener_socket);
        listener_socket = -1;
    }
}

// setNonBlocking()把socket切换为非阻塞模式，避免服务任务等待网络发送。
bool setNonBlocking(int socket_descriptor)
{
    const int flags = fcntl(socket_descriptor, F_GETFL, 0);
    return flags >= 0 &&
           fcntl(socket_descriptor, F_SETFL, flags | O_NONBLOCK) == 0;
}

} // namespace

esp_err_t initialize()
{
    // SSID和密码长度先按Wi-Fi字段容量校验，再复制到ESP-IDF配置结构。
    const std::size_t ssid_length = strnlen(CONFIG_VEHICLE_WIFI_SSID, 33U);
    const std::size_t password_length =
        strnlen(CONFIG_VEHICLE_WIFI_PASSWORD, 65U);
    if (ssid_length == 0U || ssid_length > 32U || password_length > 64U) {
        return ESP_ERR_INVALID_ARG;
    }

    // 事件组使用静态存储，联网状态由Wi-Fi/IP事件回调发布。
    wifi_events = xEventGroupCreateStatic(&wifi_events_storage);
    if (wifi_events == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // 网络栈和默认STA接口只在本模块初始化阶段建立一次。
    esp_err_t result = esp_netif_init();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }
    result = esp_event_loop_create_default();
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        return result;
    }
    if (esp_netif_create_default_wifi_sta() == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    result = esp_wifi_init(&wifi_init);
    if (result != ESP_OK) {
        return result;
    }
    result = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEvent, nullptr, &wifi_handler);
    if (result == ESP_OK) {
        result = esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, wifiEvent, nullptr, &ip_handler);
    }
    if (result != ESP_OK) {
        return result;
    }

    // wifi_config保存Kconfig提供的station凭据和认证模式。
    wifi_config_t wifi_config{};
    std::memcpy(wifi_config.sta.ssid, CONFIG_VEHICLE_WIFI_SSID, ssid_length);
    std::memcpy(
        wifi_config.sta.password,
        CONFIG_VEHICLE_WIFI_PASSWORD,
        password_length);
    wifi_config.sta.threshold.authmode =
        password_length == 0U ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    // 先启动STA，再由service()按事件状态发起连接和TCP监听。
    result = esp_wifi_set_mode(WIFI_MODE_STA);
    if (result == ESP_OK) {
        result = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    if (result == ESP_OK) {
        result = esp_wifi_start();
    }
    if (result == ESP_OK) {
        result = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    }
    return result;
}

void service(const TelemetrySnapshot *snapshot)
{
    // service()在非阻塞模式下推进完整的Wi-Fi和TCP状态机。
    const std::uint32_t now_ms = nowMilliseconds();
    const bool wifi_ready =
        (xEventGroupGetBits(wifi_events) & kWifiReady) != 0U;
    if (!wifi_ready) {
        // 启动或断线事件触发一次延时重试；连接中和等待DHCP期间不重复请求。
        closeSockets();
        const EventBits_t events = xEventGroupClearBits(wifi_events, kConnectRequested);
        if ((events & kConnectRequested) != 0U) {
            reconnect_pending = true;
            last_connect_ms = now_ms;
        }
        if (reconnect_pending && (now_ms - last_connect_ms) >= kReconnectPeriodMs) {
            reconnect_pending = false;
            const esp_err_t result = esp_wifi_connect();
            if (result != ESP_OK) {
                ESP_LOGW(kTag, "Connect failed: %s", esp_err_to_name(result));
                reconnect_pending = true;
                last_connect_ms = now_ms;
            }
        }
        return;
    }

    reconnect_pending = false;
    if (!config::kWifiTcpDebugEnabled) {
        closeSockets();
        return;
    }

    if (listener_socket < 0) {
        // 监听socket只接受一个客户端，所有socket操作均保持非阻塞。
        listener_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        int reuse = 1;
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(CONFIG_VEHICLE_WIFI_TELEMETRY_PORT);
        if (listener_socket < 0 || !setNonBlocking(listener_socket) ||
            setsockopt(
                listener_socket,
                SOL_SOCKET,
                SO_REUSEADDR,
                &reuse,
                sizeof(reuse)) != 0 ||
            bind(
                listener_socket,
                reinterpret_cast<sockaddr *>(&address),
                sizeof(address)) != 0 ||
            listen(listener_socket, 1) != 0) {
            closeSockets();
            return;
        }
    }

    if (client_socket < 0) {
        // 新客户端先接收协议头，随后再发送带序号的新遥测快照。
        client_socket = accept(listener_socket, nullptr, nullptr);
        if (client_socket < 0) {
            return;
        }
        int enabled = 1;
        if (!setNonBlocking(client_socket) ||
            setsockopt(
                client_socket,
                IPPROTO_TCP,
                TCP_NODELAY,
                &enabled,
                sizeof(enabled)) != 0) {
            closeClient();
            return;
        }
        static_assert(sizeof(kHeader) - 1U <= kBufferSize);
        std::memcpy(pending.data(), kHeader, sizeof(kHeader) - 1U);
        pending_length = sizeof(kHeader) - 1U;
        pending_offset = 0U;
        last_sequence = 0U;
    }

    // 先排空上一轮待发送数据，避免覆盖仍未完成的TCP帧。
    while (pending_offset < pending_length) {
        const int sent = send(
            client_socket,
            pending.data() + pending_offset,
            pending_length - pending_offset,
            MSG_DONTWAIT);
        if (sent > 0) {
            pending_offset += static_cast<std::size_t>(sent);
            blocked_since_ms = 0U;
            continue;
        }
        if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // 客户端持续阻塞超过超时阈值时断开，释放服务任务的发送状态。
            if (blocked_since_ms == 0U) {
                blocked_since_ms = now_ms;
            } else if ((now_ms - blocked_since_ms) >= kClientTimeoutMs) {
                closeClient();
            }
            return;
        }
        closeClient();
        return;
    }
    pending_length = 0U;
    pending_offset = 0U;

    // 空快照或已发送版本不产生重复遥测行。
    if (snapshot == nullptr || snapshot->sequence == last_sequence) {
        return;
    }

    // 遥测速度沿用控制器的左右方向约定，velocity_difference为左减右。
    const float left = config::kMotor0Direction * snapshot->left_velocity_rad_s;
    const float right = config::kMotor1Direction * snapshot->right_velocity_rad_s;
    // device_time_us拆成秒和微秒字段，保持LF分隔的文本协议格式。
    const int length = std::snprintf(
        pending.data(),
        pending.size(),
        "%lld.%06lld,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        static_cast<long long>(snapshot->device_time_us / 1000000LL),
        static_cast<long long>(snapshot->device_time_us % 1000000LL),
        static_cast<double>(snapshot->pitch_deg),
        static_cast<double>(left),
        static_cast<double>(right),
        static_cast<double>(left - right),
        static_cast<double>(snapshot->left_target_v),
        static_cast<double>(snapshot->right_target_v));
    // 只有整行适合缓冲区时才提交为下一次非阻塞发送帧。
    if (length > 0 && static_cast<std::size_t>(length) < pending.size()) {
        pending_length = static_cast<std::size_t>(length);
        last_sequence = snapshot->sequence;
    }
}

} // namespace wifi_telemtry
} // namespace vehicle
