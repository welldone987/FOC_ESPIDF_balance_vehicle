[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC 平衡车

基于 **ESP32 + ESP-IDF** 的两轮自平衡车：级联控制环、FOC 电机驱动、双核 FreeRTOS 调度、BLE 控制与实时遥测。运行于 **DengFOC V4** 控制板，包含两台 BLDC 电机、AS5600 编码器、BMI160 IMU 与 INA240 相电流采样。

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet)
![FOC](https://img.shields.io/badge/FOC-motor--control-orange)
[![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## 控制架构

```text
BLE 指令 → 轮速 PI（100 Hz）→ 姿态 PD（200 Hz）→ Iq 电流 PI（500 Hz）→ SVPWM → 双 BLDC
```

反馈链路：AS5600 轮速 · BMI160 俯仰角 · INA240 相电流 · ADC 母线电压

外层速度环把目标车速转换为目标俯仰角，姿态环把平衡误差转换为电流请求，最内层电流环通过六扇区 SVPWM 驱动逆变桥。初始化完成后车辆自行保持零速平衡，BLE 只提供运动目标。

## 功能特性

- **级联控制** — 100 Hz 轮速 PI、200 Hz 姿态 PD、500 Hz Iq 电流 PI
- **FOC 驱动** — 两台 7 极对 BLDC，使用 `esp_simplefoc` 完成启动对齐，BSP 内实现六扇区 SVPWM
- **电流采样** — 四路 INA240A2，ADC1 连续 DMA 总转换 20 kHz（每通道 5 kHz），每轮 ±1 A Iq、4.8 V Uq、1.3 A 相电流停机
- **姿态估计** — BMI160 ODR 800 Hz、软件采样 200 Hz，按实测采样间隔进行互补滤波
- **保护** — 锁存停机、70° 倾倒门、编码器与电流采样年龄门、16 条 RAM 诊断事件
- **BLE** — Nordic-UART 风格服务，X/Y 指令，20 字节遥测（10 Hz），诊断特征
- **Wi-Fi 遥测** — 常开单客户端 TCP CSV 数据流，端口 `3333`
- **双核 FreeRTOS** — 控制任务固定在 Core 1，由 2000 us GPTimer 释放；BLE 与 Wi-Fi 在 Core 0；任务与队列静态分配

## 硬件

| 项目 | 说明 |
|------|------|
| MCU | ESP32-WROOM-32 |
| 控制板 | DengFOC V4 |
| 电机 | 2 × 7 极对 BLDC |
| 编码器 | 2 × AS5600 |
| IMU | BMI160 |
| 电流采样 | INA240A2 |
| 供电 | 标称 12 V |

| 功能 | M0 | M1 |
|------|----|----|
| PWM | `32 / 33 / 25` | `26 / 27 / 14` |
| 编码器 SDA/SCL | `19 / 18` | `23 / 5` |
| 相电流 ADC | `39 / 36` | `35 / 34` |

`GPIO13` 采集母线电压（ADC2），`GPIO12` 为两轮共享的高电平有效驱动使能。全部引脚定义位于 `components/BSP/Board/board_pins.hpp`。

## 软件结构

```text
components/BSP           Board、Encoder、CurrentSensor、IMU、Motor、Power
components/Middlewares   Control、BLE、Diagnostics、FreeRTOS、wifi_telemetry
main                     app_main
docs                     平衡车控制界面.html
```

BSP 负责硬件相关驱动，Middlewares 承载控制环、通信与诊断，依赖方向保持 `main → Middlewares → BSP`。

## 运行时

```text
Core 0   BLE 服务 · Wi-Fi / TCP 遥测
Core 1   控制任务 500 Hz：IMU 200 Hz · 编码器 500 Hz · 速度与偏航 100 Hz · 姿态 200 Hz · 电流环 500 Hz · SVPWM
```

## 通信

BLE（浏览器控制界面：`docs/平衡车控制界面.html`）：

```text
命令特征      WRITE       X,Y 百分比输入
遥测特征      NOTIFY      20 字节小端，10 Hz
诊断特征      READ/WRITE  ErrorInfo 文本，4 字节序号确认
```

UUID 使用 Nordic-UART 风格基址 `6e4000xx-b5a3-f393-e0a9-e50e24dcca9e`（`01` 为主服务，`02` / `07` / `08` 为三个特征），字节序列是协议契约。

Wi-Fi 以 21 列 CSV 输出最新控制快照，列名带 `m0_` / `m1_` 通道前缀并含两行表头。凭据通过 menuconfig 配置：

```text
CONFIG_VEHICLE_WIFI_SSID
CONFIG_VEHICLE_WIFI_PASSWORD
```

## 构建

ESP-IDF v6.0.2，目标已固定为 `esp32`（不要执行 `set-target`）：

```powershell
idf.py build
idf.py -p COMx flash monitor
```

新增或删除源文件后先执行 `idf.py reconfigure`。调参位于各模块内部：`components/Middlewares/Control/control_config.hpp` 与 `components/BSP/Motor/motor_config.hpp`。
