[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC Balance Vehicle

A two-wheel self-balancing vehicle built on **ESP32 + ESP-IDF**, featuring cascaded control loops, FOC motor control, dual-core FreeRTOS scheduling, BLE control, and real-time telemetry.

The project runs on the **DengFOC V4** control board with two BLDC motors, AS5600 magnetic encoders, a BMI160 IMU, and phase-current sensing. The firmware is written in C++ on **ESP-IDF v6.0.2**, with the runtime motor-control stack implemented around a BSP/Middleware architecture.

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet)
![FOC](https://img.shields.io/badge/FOC-motor--control-orange)
[![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## Overview

The vehicle uses a cascaded control architecture:

```text
BLE Command
    │
    ▼
Wheel Speed PI          100 Hz
    │
    │ Pitch Reference
    ▼
Attitude PD             200 Hz
    │
    │ Balance Current Reference
    ▼
Iq Current PI           500 Hz
    │
    ▼
SVPWM
    │
    ▼
Dual BLDC Motors
```

Feedback is provided by:

```text
AS5600 ─────► Wheel Position / Speed
BMI160 ─────► Pitch / Angular Velocity
INA240 ─────► Phase Current
ADC ────────► Battery Voltage
```

The outer speed loop converts the commanded vehicle velocity into a pitch reference. The attitude controller then generates the motor torque/current demand required to maintain balance, while the inner current loop regulates motor current and drives the inverter using six-sector SVPWM.

## Features

- **Cascaded balance control**
  - 100 Hz wheel-speed PI loop
  - 200 Hz attitude PD loop
  - 500 Hz Iq current PI loop
- **Dual BLDC FOC drive**
  - Two 7-pole-pair BLDC motors
  - AS5600 magnetic encoders
  - Startup electrical alignment through `esp_simplefoc`
  - Runtime current regulation and SVPWM implemented in the BSP
- **Phase-current sensing**
  - Four INA240A2 current-sense channels
  - ADC startup offset calibration
  - Third-phase current reconstruction
  - Per-wheel current limiting and phase-current protection
- **Attitude estimation**
  - BMI160 IMU
  - 800 Hz sensor ODR
  - 200 Hz software sampling
  - Complementary-filter pitch estimation using measured sample intervals
- **BLE control**
  - Nordic-UART-style communication
  - X/Y velocity and steering commands
  - Command freshness timeout
  - Binary real-time telemetry
  - Structured diagnostic messages
- **Wi-Fi telemetry**
  - Always-on TCP telemetry server (single client)
  - Real-time attitude, M0/M1 wheel speed, current targets and measurements, phase currents and sampling validity
  - Default TCP port: `3333`
- **Dual-core FreeRTOS**
  - Control task pinned to Core 1
  - BLE and Wi-Fi services on Core 0
  - Static task and queue allocation
  - Fixed-memory diagnostic storage
- **Continuous Integration**
  - GitHub Actions build on every push and pull request to `main`
  - ESP-IDF v6.0.2 build environment

## Hardware

| Item | Detail |
|------|--------|
| MCU | ESP32-WROOM-32 |
| Control Board | DengFOC V4 |
| Motors | 2 × 7-pole-pair BLDC |
| Encoders | 2 × AS5600 |
| IMU | BMI160 |
| Current Sensing | INA240A2 |
| Motor Drive | Dual 3-phase inverter |
| Supply | 12 V nominal |

### GPIO Assignment

| Function | M0 | M1 |
|----------|----|----|
| PWM | `32 / 33 / 25` | `26 / 27 / 14` |
| Encoder SDA/SCL | `19 / 18` | `23 / 5` |
| Phase Current ADC | `39 / 36` | `35 / 34` |

Both motor drivers share the active-high enable pin:

```text
GPIO12
```

All board-level pin definitions are centralized in:

```text
components/BSP/Board/board_pins.hpp
```

## Software Architecture

The firmware separates hardware-facing drivers from higher-level vehicle logic.

```text
FOC_ESPIDF_balance_vehicle/
│
├── components/
│   │
│   ├── BSP/
│   │   ├── Board/
│   │   ├── Encoder/
│   │   ├── CurrentSensor/
│   │   ├── IMU/
│   │   ├── Motor/
│   │   └── Power/
│   │
│   └── Middlewares/
│       ├── BLE/
│       ├── Control/
│       ├── Diagnostics/
│       ├── FreeRTOS/
│       └── wifi_telemetry/
│
├── main/
│   └── app_main
│
└── docs/
```

### BSP

The Board Support Package owns hardware-specific functionality:

- GPIO and board configuration
- Motor PWM and SVPWM
- Encoder acquisition
- Phase-current sampling
- IMU communication
- Battery-voltage measurement

### Middlewares

The middleware layer contains hardware-independent vehicle functionality:

- Cascaded control loops
- BLE command handling
- Wi-Fi telemetry
- FreeRTOS task organization
- Runtime diagnostics

This keeps control logic separated from board-level implementation and makes individual modules easier to test and replace.

## Runtime Architecture

The ESP32 dual-core architecture is used to separate control from communication workloads.

```text
ESP32
│
├── Core 0
│   ├── BLE Service
│   └── Wi-Fi / TCP Telemetry
│
└── Core 1
    └── Control Task
        ├── Encoder Update
        ├── IMU Update
        ├── Speed Loop
        ├── Attitude Loop
        ├── Current Loop
        └── SVPWM Update
```

The main control task targets **500 Hz**, while slower control loops are scheduled at their corresponding rates inside the control system.

## Communication

### BLE

BLE is used for vehicle commands, telemetry and diagnostics.

The browser-based controller is available at:

```text
docs/平衡车控制界面.html
```

Current protocol characteristics:

```text
Command characteristic      WRITE       X,Y text input
Telemetry characteristic    NOTIFY      20-byte little-endian at 10 Hz
Diagnostic characteristic   READ/WRITE  ErrorInfo text with 4-byte sequence ack
```

Characteristic UUIDs use the Nordic-UART-style base `6e4000xx-b5a3-f393-e0a9-e50e24dcca9e` with `01` for the service and `02` / `07` / `08` for the three characteristics; the UUID byte sequences are the protocol contract.

### Wi-Fi TCP

Wi-Fi telemetry provides a higher-bandwidth debugging interface for control-system development and data analysis.

The TCP server publishes data including:

- pitch angle
- M0/M1 wheel speed and velocity difference
- current targets and measured currents
- applied voltage and phase currents
- sampling interval, sample age and validity

Default port:

```text
3333
```

Wi-Fi credentials are configured through ESP-IDF menuconfig:

```text
CONFIG_VEHICLE_WIFI_SSID
CONFIG_VEHICLE_WIFI_PASSWORD
```

## Build

The project targets the classic ESP32 and uses **ESP-IDF v6.0.2**. The target is already fixed to `esp32`; do not run `set-target`, as it clears the build state and regenerates configuration.

```powershell
idf.py build
```

Flash and monitor:

```powershell
idf.py -p COMx flash monitor
```

Project configuration:

```powershell
idf.py menuconfig
```

Control and motor parameters are kept with their corresponding modules. Important configuration files include:

```text
components/Middlewares/Control/control_config.hpp
components/BSP/Motor/motor_config.hpp
```

## Development Focus

This project is being developed as a complete embedded control platform rather than a single balance-controller demo. Its main areas of work include:

- cascaded control-loop design and tuning
- FOC and current-loop implementation
- attitude estimation
- deterministic FreeRTOS scheduling
- real-time telemetry and diagnostics
- ESP32 dual-core task partitioning
- modular BSP / middleware architecture
- hardware-software integration

The goal is to keep the control path observable and modular, making the platform suitable for continued experimentation with motor control, state estimation and real-time embedded systems.
