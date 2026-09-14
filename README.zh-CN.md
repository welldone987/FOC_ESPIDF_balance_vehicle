[English](README.md) | [简体中文](README.zh-CN.md)

# ESP-IDF FOC 平衡车

一个基于 **ESP32 + ESP-IDF** 的两轮自平衡车项目，包含级联控制、FOC 电机控制、双核 FreeRTOS 调度、BLE 控制与实时遥测。

项目运行于 **DengFOC V4** 控制板，使用两台 BLDC 电机、AS5600 磁编码器、BMI160 IMU 与相电流采样。固件采用 C++ 编写，基于 **ESP-IDF v6.0.2**，整体按照 BSP / Middleware 分层组织运行时电机控制与整车逻辑。

![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-E7352C)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.2-blue)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-dual--core-blueviolet)
![FOC](https://img.shields.io/badge/FOC-motor--control-orange)
[![Build](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml/badge.svg)](https://github.com/welldone987/FOC_ESPIDF_balance_vehicle/actions/workflows/esp-idf-build.yml)

## 项目概览

整车采用级联控制架构：

```text
BLE 指令
    │
    ▼
轮速 PI                 100 Hz
    │
    │ 目标俯仰角
    ▼
姿态 PD                 200 Hz
    │
    │ 平衡电流目标
    ▼
Iq 电流 PI              500 Hz
    │
    ▼
SVPWM
    │
    ▼
双 BLDC 电机
```

反馈链路包括：

```text
AS5600 ─────► 车轮位置 / 速度
BMI160 ─────► 俯仰角 / 角速度
INA240 ─────► 相电流
ADC ────────► 电池电压
```

外层速度环将目标车速转换为目标俯仰角；姿态控制器根据平衡需求生成电机转矩 / 电流请求；最内层电流环负责调节电机电流，并通过六扇区 SVPWM 驱动逆变桥。

## 功能特性

- **级联平衡控制**
  - 100 Hz 轮速 PI 环
  - 200 Hz 姿态 PD 环
  - 500 Hz Iq 电流 PI 环
- **双 BLDC FOC 驱动**
  - 两台 7 极对 BLDC 电机
  - AS5600 磁编码器
  - 使用 `esp_simplefoc` 完成启动电气对齐
  - BSP 负责运行时电流调节与 SVPWM
- **相电流采样**
  - 四路 INA240A2 电流采样
  - ADC 启动零偏校准
  - 第三相电流重构
  - 单轮电流限幅与相电流保护
- **姿态估计**
  - BMI160 IMU
  - 800 Hz 传感器 ODR
  - 200 Hz 软件采样
  - 使用实测采样间隔进行互补滤波俯仰角估计
- **BLE 控制**
  - Nordic-UART 风格通信
  - X/Y 速度与转向指令
  - 指令时效检测
  - 二进制实时遥测
  - 结构化诊断信息
- **Wi-Fi 遥测**
  - 常开 TCP 遥测服务器（单客户端）
  - 实时输出姿态、M0/M1 轮速、电流目标与实测、相电流与采样有效性
  - 默认 TCP 端口：`3333`
- **双核 FreeRTOS**
  - 控制任务固定在 Core 1
  - BLE 与 Wi-Fi 服务运行在 Core 0
  - 任务与队列采用静态分配
  - 诊断信息存储于固定内存区域
- **持续集成**
  - 每次 push 与针对 `main` 的 Pull Request 自动执行 GitHub Actions 构建
  - 使用 ESP-IDF v6.0.2 构建环境

## 硬件

| 项目 | 说明 |
|------|------|
| MCU | ESP32-WROOM-32 |
| 控制板 | DengFOC V4 |
| 电机 | 2 × 7 极对 BLDC |
| 编码器 | 2 × AS5600 |
| IMU | BMI160 |
| 电流采样 | INA240A2 |
| 电机驱动 | 双三相逆变桥 |
| 供电 | 标称 12 V |

### GPIO 分配

| 功能 | M0 | M1 |
|------|----|----|
| PWM | `32 / 33 / 25` | `26 / 27 / 14` |
| 编码器 SDA/SCL | `19 / 18` | `23 / 5` |
| 相电流 ADC | `39 / 36` | `35 / 34` |

两路电机驱动共用高电平有效使能引脚：

```text
GPIO12
```

所有板级 GPIO 定义集中在：

```text
components/BSP/Board/board_pins.hpp
```

## 软件架构

固件将硬件相关驱动与上层整车逻辑分离。

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

Board Support Package 负责硬件相关功能：

- GPIO 与板级配置
- 电机 PWM 与 SVPWM
- 编码器采集
- 相电流采样
- IMU 通信
- 电池电压测量

### Middlewares

Middleware 层负责与具体硬件解耦的整车功能：

- 级联控制环
- BLE 指令处理
- Wi-Fi 遥测
- FreeRTOS 任务组织
- 运行时诊断

这种分层使控制逻辑与板级实现保持分离，也便于独立测试和替换各个模块。

## 运行时架构

项目利用 ESP32 双核架构，将实时控制与通信负载分离。

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

主控制任务目标为 **500 Hz**，其内部按各自频率调度速度环、姿态环等较低频控制逻辑。

## 通信

### BLE

BLE 用于整车控制指令、遥测与诊断。

浏览器控制界面位于：

```text
docs/平衡车控制界面.html
```

当前协议特征包括：

```text
命令特征      WRITE       X,Y 文本输入
遥测特征      NOTIFY      20 字节小端，10 Hz
诊断特征      READ/WRITE  ErrorInfo 文本，4 字节序号确认
```

特征 UUID 使用 Nordic-UART 风格基址 `6e4000xx-b5a3-f393-e0a9-e50e24dcca9e`，`01` 为主服务，`02` / `07` / `08` 分别对应三个特征；UUID 字节序列是协议契约。

### Wi-Fi TCP

Wi-Fi 遥测为控制系统开发和数据分析提供更高带宽的调试链路。

TCP 服务输出的数据包括：

- 俯仰角
- M0/M1 轮速与轮速差
- 电流目标与实测电流
- 施加电压与相电流
- 采样间隔、样本年龄与有效性

默认端口：

```text
3333
```

Wi-Fi 凭据通过 ESP-IDF menuconfig 配置：

```text
CONFIG_VEHICLE_WIFI_SSID
CONFIG_VEHICLE_WIFI_PASSWORD
```

## 构建

项目目标平台为经典 ESP32，使用 **ESP-IDF v6.0.2**。目标已固定为 `esp32`，不要执行 `set-target`，该命令会清除构建状态并重新生成配置。

```powershell
idf.py build
```

烧录并打开串口监视器：

```powershell
idf.py -p COMx flash monitor
```

项目配置：

```powershell
idf.py menuconfig
```

控制器和电机相关参数随对应模块组织，主要配置文件包括：

```text
components/Middlewares/Control/control_config.hpp
components/BSP/Motor/motor_config.hpp
```

## 开发重点

这个项目的目标并不只是实现一个简单的平衡车 Demo，而是构建一个完整的嵌入式控制平台。当前主要开发内容包括：

- 级联控制器设计与调参
- FOC 与电流环实现
- 姿态估计
- 确定性的 FreeRTOS 调度
- 实时遥测与诊断
- ESP32 双核任务划分
- 模块化 BSP / Middleware 架构
- 软硬件联调

项目整体强调控制链路的可观测性与模块化，为后续继续实验电机控制、状态估计与实时嵌入式系统提供基础。
