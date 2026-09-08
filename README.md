[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC Balance Vehicle

Native ESP-IDF firmware for a two-wheel self-balancing vehicle based on the **DengFOC V4** board (ESP32-WROOM-32). Pure C++ application built with **ESP-IDF v6.0.2**, migrated from the original Arduino/PlatformIO reference project.

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet) ![FOC](https://img.shields.io/badge/FOC-motor--control-orange) [![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## Features

- **1 kHz cascade control** — wheel-speed outer loop outputs the target pitch angle, attitude inner loop outputs the motor voltage; steering is applied as a differential voltage
- **FOC motor drive** — dual 7-pole-pair BLDC with I2C encoders, driven through `espressif/esp_simplefoc`
- **IMU** — BMI160 attitude estimation over I2C (accelerometer + gyroscope)
- **BLE remote control** — Nordic-UART-style service with sequence numbers, 300 ms command timeout, arm/disarm states and latching emergency stop; compatible with the vendor's web control page
- **Wi-Fi TCP telemetry** — single-client TCP server (default port 3333) streaming LF-separated text frames: pitch, wheel velocities, motor target voltages
- **Dual-core FreeRTOS** — 1 kHz control task pinned to core 1; BLE, Wi-Fi telemetry and diagnostics tasks on core 0; fully static allocation

## Hardware

| Item | Detail |
|------|--------|
| MCU | ESP32-WROOM-32 (classic ESP32) |
| Board | DengFOC V4 — dual 3-phase PWM, phase current sensing, battery voltage ADC |
| IMU | BMI160 @ 0x69 on I2C0 (400 kHz) |
| Motors | 2 × BLDC (7 pole pairs) with I2C encoders |
| Supply | 12 V nominal |

All GPIO assignments are centralized in `components/BSP/Board/board_pins.hpp`.

## Project Layout

```
components/
├── BSP/          # Board pin map, BMI160 IMU, SimpleFOC motor service, power monitor
└── Middlewares/  # BLE protocol, balance controller, Wi-Fi telemetry, FreeRTOS tasks, diagnostics
main/             # app_main: initialization checks and task creation
```

## Build and Flash

Requires [ESP-IDF v6.0.2](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/index.html), target fixed to `esp32`.

```bash
idf.py build
idf.py -p COMx flash monitor   # replace COMx with your serial port
```

Wi-Fi credentials are set via menuconfig: `CONFIG_VEHICLE_WIFI_SSID`, `CONFIG_VEHICLE_WIFI_PASSWORD` (telemetry port defaults to 3333). All controller gains, limits and filters are defined in `components/BSP/Common/vehicle_config.hpp`.

CI builds the firmware with ESP-IDF v6.0.2 on every push/PR to `main`.
