#include "diagnostics.hpp"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
COREDUMP_DRAM_ATTR vehicle::diagnostics::CrashState g_diag_crash{};
namespace vehicle::diagnostics {
namespace { portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED; }
void initialize() { portENTER_CRITICAL(&lock); g_diag_crash = {}; portEXIT_CRITICAL(&lock); }
void boot(BootStep step, const char *state, esp_err_t rc)
{
    portENTER_CRITICAL(&lock); g_diag_crash.boot_step=step; portEXIT_CRITICAL(&lock);
    ESP_LOGI("boot", "boot_step=%u %s rc=%s (0x%x)", static_cast<unsigned>(step), state, esp_err_to_name(rc), rc);
}
void record(const ErrorInfo &error, bool fatal)
{ portENTER_CRITICAL(&lock); commit(g_diag_crash,error,fatal); portEXIT_CRITICAL(&lock); }
void bleError(const ErrorInfo &error)
{ portENTER_CRITICAL(&lock); g_diag_crash.last_ble_error=error; portEXIT_CRITICAL(&lock); }
void bleState(bool connected, bool status, bool diag, std::uint16_t mtu)
{
    portENTER_CRITICAL(&lock);
    g_diag_crash.connected=connected; g_diag_crash.status_subscribed=status;
    g_diag_crash.diag_subscribed=diag; g_diag_crash.mtu=mtu;
    portEXIT_CRITICAL(&lock);
}
void controlSnapshot(const ControlSnapshot &s)
{ portENTER_CRITICAL(&lock); g_diag_crash.last_control=s; portEXIT_CRITICAL(&lock); }
void completeBoot()
{ portENTER_CRITICAL(&lock); g_diag_crash.boot_complete=true; portEXIT_CRITICAL(&lock); }
bool replay(const ReplayCursor &cursor, Event &event, bool &first)
{ portENTER_CRITICAL(&lock); bool found=prepareReplay(g_diag_crash,cursor,event,first); portEXIT_CRITICAL(&lock); return found; }
void metadata(std::uint8_t (&packet)[20])
{ portENTER_CRITICAL(&lock); encodeMetadata(g_diag_crash,packet); portEXIT_CRITICAL(&lock); }
}
