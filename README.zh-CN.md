[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC 平衡车

基于 **DengFOC V4** 板（ESP32-WROOM-32）的两轮自平衡车原生 ESP-IDF 固件。纯 C++ 应用，使用 **ESP-IDF v6.0.2** 构建，由原 Arduino/PlatformIO 参考工程迁移而来。

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C) ![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet) ![FOC](https://img.shields.io/badge/FOC-motor--control-orange) [![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## 功能特性

- **1 kHz 级联控制** — 速度外环输出目标俯仰角，姿态内环输出电机电压；转向以左右差分电压叠加
- **FOC 电机驱动** — 双 7 极对无刷电机 + I2C 编码器，通过 `espressif/esp_simplefoc` 驱动
- **IMU** — BMI160 姿态解算（I2C，加速度计 + 陀螺仪）
- **BLE 遥控** — Nordic-UART 风格服务，带序号校验、300 ms 指令超时、使能/失能状态与急停锁存；兼容厂商网页控制端协议
- **Wi-Fi TCP 遥测** — 单客户端 TCP 服务器（默认端口 3333），以 LF 分隔文本帧输出俯仰角、轮速和电机目标电压
- **双核 FreeRTOS** — 1 kHz 控制任务固定在核 1；BLE、Wi-Fi 遥测与诊断任务运行在核 0；全部静态分配

## 硬件

| 项目 | 说明 |
|------|------|
| MCU | ESP32-WROOM-32（经典 ESP32） |
| 板卡 | DengFOC V4 — 双三相 PWM、相电流采样、母线电压 ADC |
| IMU | BMI160 @ 0x69，I2C0（400 kHz） |
| 电机 | 2 × 无刷电机（7 极对），I2C 编码器 |
| 供电 | 标称 12 V |

所有 GPIO 分配集中在 `components/BSP/Board/board_pins.hpp`。

## 项目结构

```
components/
├── BSP/          # 板级引脚映射、BMI160 IMU、SimpleFOC 电机服务、电源监测
└── Middlewares/  # BLE 协议、平衡控制器、Wi-Fi 遥测、FreeRTOS 任务、诊断
main/             # app_main：初始化检查与任务创建
```

## 构建与烧录

需要 [ESP-IDF v6.0.2](https://docs.espressif.com/projects/esp-idf/zh_CN/v6.0.2/esp32/index.html)，目标芯片固定为 `esp32`。

```bash
idf.py build
idf.py -p COMx flash monitor   # 将 COMx 替换为实际串口号
```

Wi-Fi 凭据通过 menuconfig 配置：`CONFIG_VEHICLE_WIFI_SSID`、`CONFIG_VEHICLE_WIFI_PASSWORD`（遥测端口默认 3333）。控制器增益、限幅和滤波参数集中在 `components/BSP/Common/vehicle_config.hpp`。

每次推送到 `main` 或提交 PR 时，CI 会使用 ESP-IDF v6.0.2 完成整包构建。
