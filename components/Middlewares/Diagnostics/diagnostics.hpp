#pragma once
#include "diagnostic_store.hpp"
extern vehicle::diagnostics::CrashState g_diag_crash;
namespace vehicle::diagnostics {
void initialize();
void boot(BootStep step, const char *state, esp_err_t rc=ESP_OK);
void record(const ErrorInfo &error, bool fatal=false);
void bleError(const ErrorInfo &error);
void bleState(bool connected, bool status, bool diag, std::uint16_t mtu);
void controlSnapshot(const ControlSnapshot &snapshot);
void completeBoot();
bool replay(const ReplayCursor &cursor, Event &event, bool &first);
void metadata(std::uint8_t (&packet)[20]);
}
