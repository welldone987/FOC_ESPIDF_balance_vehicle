#include "esp_log.h"

namespace {
constexpr char kTag[] = "app";
constexpr int kCppBuildProbe = 42;
} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "ESP-IDF C++ build probe passed: %d", kCppBuildProbe);
}
