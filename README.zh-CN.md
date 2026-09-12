[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC 平衡车

基于 **DengFOC V4** 板（ESP32-WROOM-32）的两轮自平衡车原生 ESP-IDF 固件。纯 C++ 应用，使用 **ESP-IDF v6.0.2** 构建，由原 Arduino/PlatformIO 参考工程迁移而来。

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet) ![FOC](https://img.shields.io/badge/FOC-motor--control-orange) [![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## 功能特性

- **三环电流级联** — 100 Hz 轮速 PI 输出目标俯仰角，200 Hz 姿态 PD 输出平衡电流请求，500 Hz BSP 电流环执行 Iq PI 与六扇区 SVPWM
- **FOC 电机驱动** — 双 7 极对无刷电机与两路独立 AS5600 I2C 编码器；`espressif/esp_simplefoc` 仅保留用于启动对齐，运行电流环由 BSP 实现
- **电流采样与保护** — 四路 INA240A2 ADC1 采样、逐通道启动零偏校准、第三相电流重构、每轮 ±1 A 请求限幅及 1.3 A 相电流停机门
- **IMU** — BMI160 配置为 800 Hz ODR，软件以 200 Hz 读取，并按实测采样间隔进行互补滤波
- **BLE 控制与诊断** — Nordic-UART 风格 `.002` X,Y 指令采用 300 ms 时效；`.007` 以 10 Hz 回传二进制遥测，`.008` 读取并确认统一 `ErrorInfo`；当前没有无线 ARM 或急停指令
- **Wi-Fi TCP 遥测** — 可选单客户端 TCP v2 服务器（默认端口 3333），以 21 列 LF 文本帧输出姿态、轮速、电流、电压和采样状态
- **双核 FreeRTOS** — 500 Hz 控制任务固定在核 1；BLE 与可选 Wi-Fi 服务任务运行在核 0；应用任务和队列静态分配，诊断写入固定 RAM 区而不创建独立任务

## 硬件

| 项目 | 说明 |
|------|------|
| MCU | ESP32-WROOM-32（经典 ESP32） |
| 板卡 | DengFOC V4 — 双三相 PWM、相电流采样、母线电压 ADC |
| IMU | BMI160 @ `0x69`，I2C0（400 kHz），与 M0 编码器共用总线 |
| 电机 | 2 × 无刷电机（7 极对），AS5600 编码器 @ `0x36` |
| 供电 | 标称 12 V |

三相 PWM 为 M0 `32/33/25`、M1 `26/27/14`，两个驱动共用高有效 GPIO12；AS5600 的 SDA/SCL 分别为 `19/18`、`23/5`；相电流 ADC 为 M0 `39/36`、M1 `35/34`。所有 GPIO 分配集中在 `components/BSP/Board/board_pins.hpp`。

当前源码设置 `kCurrentHardwareVerified=true`，因此启动时允许拉高 GPIO12，并依次执行右轮、左轮对齐。它只是软件放行门，不代表相序、电流极性、时序或整车稳定性已经过实板验证；对齐可能转动车轮，且此阶段尚未进入周期电流保护。

## 项目结构

```
components/
├── BSP/          # Board、Encoder、CurrentSensor、IMU、Motor、Power
└── Middlewares/  # BLE、Control、Diagnostics、FreeRTOS 任务、Wi-Fi 遥测
main/             # app_main：初始化检查与任务创建
```

## 构建与烧录

需要 [ESP-IDF v6.0.2](https://docs.espressif.com/projects/esp-idf/zh_CN/v6.0.2/esp32/index.html)，目标芯片固定为 `esp32`。

```powershell
esp-idf-602
idf.py build
```

正常构建流程不包含烧录或电机运行。只有在选定实际串口并明确决定开展硬件操作后，才执行 `idf.py -p COMx flash monitor`。

Wi-Fi 凭据通过 menuconfig 配置：`CONFIG_VEHICLE_WIFI_SSID`、`CONFIG_VEHICLE_WIFI_PASSWORD`（遥测端口默认 3333）。配置常量随所属模块存放，例如 `components/Middlewares/Control/control_config.hpp` 与 `components/BSP/Motor/motor_config.hpp`。

每次推送到 `main` 或提交 PR 时，CI 会使用 ESP-IDF v6.0.2 完成整包构建。

源码对应的架构、板级约束和调试边界见[项目架构](docs/ADR-docs/architecture.md)、[当前架构决策](docs/ADR-docs/architecture-decisions.md)、[DengFOC V4 板级映射](docs/codex/hardware/board_dengfoc_v4.md)及[三环电流调试](docs/codex/control/current_cascade_tuning.md)。
