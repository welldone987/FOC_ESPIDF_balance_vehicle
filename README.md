[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC Balance Vehicle

A two-wheel self-balancing vehicle on **ESP32 + ESP-IDF**: cascaded control loops, FOC motor drive, dual-core FreeRTOS scheduling, BLE control and real-time telemetry. It runs on the **DengFOC V4** board with two BLDC motors, AS5600 encoders, a BMI160 IMU and INA240 phase-current sensing.

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet)
![FOC](https://img.shields.io/badge/FOC-motor--control-orange)
[![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## Control Architecture

```text
BLE command → Wheel speed PI (100 Hz) → Attitude PD (200 Hz) → Iq current PI (500 Hz) → SVPWM → Dual BLDC
```

Feedback: AS5600 wheel speed · BMI160 pitch · INA240 phase current · ADC bus voltage

The outer speed loop turns the commanded velocity into a pitch reference, the attitude loop converts balance error into a current demand, and the inner current loop drives the inverter with six-sector SVPWM. Once initialization completes the vehicle balances at zero speed by itself; BLE only supplies motion targets.

## Features

- **Cascaded control** — 100 Hz speed PI, 200 Hz attitude PD, 500 Hz Iq current PI
- **FOC drive** — two 7-pole-pair BLDC motors, startup electrical alignment through `esp_simplefoc`, six-sector SVPWM in the BSP
- **Current sensing** — four INA240A2 channels, ADC1 continuous DMA at 20 kHz (5 kHz per channel), ±1 A Iq, 4.8 V Uq and 1.3 A phase-current trip
- **Attitude estimation** — BMI160 at 800 Hz ODR, 200 Hz software sampling, complementary filter using measured sample intervals
- **Protection** — latched shutdown, 70° fall-angle stop, encoder and current-sample age gates, 16-event RAM diagnostics
- **BLE** — Nordic-UART-style service, X/Y commands, 20-byte telemetry at 10 Hz, diagnostic characteristic
- **Wi-Fi telemetry** — always-on single-client TCP CSV stream on port `3333`
- **Dual-core FreeRTOS** — control task on Core 1 released by a 2000 us GPTimer, BLE and Wi-Fi on Core 0, static tasks and queues

## Hardware

| Item | Detail |
|------|--------|
| MCU | ESP32-WROOM-32 |
| Control board | DengFOC V4 |
| Motors | 2 × 7-pole-pair BLDC |
| Encoders | 2 × AS5600 |
| IMU | BMI160 |
| Current sensing | INA240A2 |
| Supply | 12 V nominal |

| Function | M0 | M1 |
|----------|----|----|
| PWM | `32 / 33 / 25` | `26 / 27 / 14` |
| Encoder SDA/SCL | `19 / 18` | `23 / 5` |
| Phase-current ADC | `39 / 36` | `35 / 34` |

`GPIO13` senses the bus voltage (ADC2) and `GPIO12` is the shared active-high motor-driver enable. All pin definitions live in `components/BSP/Board/board_pins.hpp`.

## Software Layout

```text
components/BSP           Board, Encoder, CurrentSensor, IMU, Motor, Power
components/Middlewares   Control, BLE, Diagnostics, FreeRTOS, wifi_telemetry
main                     app_main
docs                     平衡车控制界面.html
```

The BSP owns hardware-facing drivers; Middlewares holds the control loops, communication and diagnostics, keeping the dependency direction `main → Middlewares → BSP`.

## Runtime

```text
Core 0   BLE service · Wi-Fi / TCP telemetry
Core 1   Control task 500 Hz: IMU 200 Hz · encoders 500 Hz · speed and yaw 100 Hz · attitude 200 Hz · current loop 500 Hz · SVPWM
```

## Communication

BLE (browser controller at `docs/平衡车控制界面.html`):

```text
Command characteristic      WRITE       X,Y percent input
Telemetry characteristic    NOTIFY      20-byte little-endian at 10 Hz
Diagnostic characteristic   READ/WRITE  ErrorInfo text with 4-byte sequence ack
```

UUIDs use the Nordic-UART-style base `6e4000xx-b5a3-f393-e0a9-e50e24dcca9e` (`01` service, `02` / `07` / `08` characteristics); the byte sequences are the protocol contract.

Wi-Fi streams the latest control snapshot as a 21-column CSV, with channel-prefixed `m0_` / `m1_` columns and a two-line header. Credentials come from menuconfig:

```text
CONFIG_VEHICLE_WIFI_SSID
CONFIG_VEHICLE_WIFI_PASSWORD
```

## Build

ESP-IDF v6.0.2, target fixed to `esp32` (do not run `set-target`):

```powershell
idf.py build
idf.py -p COMx flash monitor
```

Run `idf.py reconfigure` after adding or removing source files. Tuning lives with its module in `components/Middlewares/Control/control_config.hpp` and `components/BSP/Motor/motor_config.hpp`.
