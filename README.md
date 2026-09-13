[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC Balance Vehicle

Native C++ firmware for a two-wheel self-balancing vehicle based on **DengFOC V4** and ESP32-WROOM-32. The repository builds with **ESP-IDF v6.0.2**; local design notes and vendor material under `docs/` are intentionally excluded from Git, except for the browser controller.

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet) ![FOC](https://img.shields.io/badge/FOC-motor--control-orange) [![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## Features

- **Three-loop current cascade** — 100 Hz wheel-speed PI produces a pitch reference, the 200 Hz attitude PD produces the balance-current request, and the 500 Hz BSP current loop performs Iq PI and six-sector SVPWM
- **FOC motor drive** — two 7-pole-pair BLDC motors indexed by board channels M0/M1, each with an AS5600 encoder; `espressif/esp_simplefoc` is used for startup alignment while BSP owns the runtime current loop
- **Current and safety checks** — four INA240A2 ADC1 channels, per-channel startup offset calibration, reconstructed third-phase current, per-wheel ±1 A request limiting and a 1.3 A phase-current trip
- **IMU** — BMI160 at 800 Hz ODR with 200 Hz software reads and a complementary pitch estimator using the measured sample interval
- **BLE control and diagnostics** — Nordic-UART-style `.002` X,Y commands with a 300 ms freshness timeout, `.007` 10 Hz binary telemetry, and `.008` acknowledged `ErrorInfo` diagnostics; there is no wireless arm or emergency-stop command
- **Wi-Fi TCP telemetry** — optional single-client TCP v2 server (default port 3333) publishing 21 LF-separated fields for attitude, wheel speed, current, voltage and sample status
- **Dual-core FreeRTOS** — the 500 Hz control task is pinned to core 1; BLE and optional Wi-Fi service tasks run on core 0; application tasks and queues use static allocation, with diagnostics recorded in fixed RAM storage rather than a separate task

## Hardware

| Item | Detail |
|------|--------|
| MCU | ESP32-WROOM-32 (classic ESP32) |
| Board | DengFOC V4 — dual 3-phase PWM, phase current sensing, battery voltage ADC |
| IMU | BMI160 @ `0x69` on I2C0 (400 kHz), shared with the M0 encoder |
| Motors | 2 × BLDC (7 pole pairs) with AS5600 encoders @ `0x36` |
| Supply | 12 V nominal |

PWM is M0 `32/33/25` and M1 `26/27/14`; both drivers share active-high enable GPIO12. The AS5600 buses use SDA/SCL `19/18` and `23/5`; phase-current ADC inputs are M0 `39/36` and M1 `35/34`. All GPIO assignments are centralized in `components/BSP/Board/board_pins.hpp`.

The source currently sets `CurrentHardwareVerified=true`, which permits GPIO12 enable and M1-then-M0 motor alignment during startup. This is a software gate, not hardware evidence. Alignment can move the wheels and runs before periodic current protection is active.

## Project Layout

```
components/
├── BSP/          # Board, Encoder, CurrentSensor, IMU, Motor, Power
└── Middlewares/  # BLE, Control, Diagnostics, FreeRTOS tasks, Wi-Fi telemetry
main/             # app_main: initialization checks and task creation
```

## Build

Requires [ESP-IDF v6.0.2](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/index.html), target fixed to `esp32`.

```powershell
esp-idf-602
idf.py build
```

The current source was built successfully with ESP-IDF v6.0.2 on 2026-09-13 using `idf.py --no-ccache build`. The application image is `0x13d570` bytes, leaving 59% of the smallest app partition free. This verifies compilation and image capacity only; it does not verify timing, wiring, motor safety, or balance stability.

Flashing and motor operation are not part of the normal build workflow. Use `idf.py -p COMx flash monitor` only for an explicitly authorized hardware session.

Wi-Fi credentials are set via menuconfig: `CONFIG_VEHICLE_WIFI_SSID`, `CONFIG_VEHICLE_WIFI_PASSWORD` (telemetry port defaults to 3333). Configuration constants live with their owning modules, such as `components/Middlewares/Control/control_config.hpp` and `components/BSP/Motor/motor_config.hpp`.

CI builds the firmware with ESP-IDF v6.0.2 on every push/PR to `main`. The tracked [browser controller](docs/平衡车控制界面.html) provides the BLE user interface.
