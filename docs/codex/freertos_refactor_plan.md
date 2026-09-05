# balancing_vehicle_work FreeRTOS 重构规划

## 1. 目标与边界

本次重构先解决执行时序、硬件所有权、跨任务通信和安全状态管理，再进行控制器升级。首版保留现有的 SimpleFOC 电压模式、直立 PID 和速度 PID，避免同时改变软件架构与控制规律，导致实机问题无法归因。

约束如下：

- 目标硬件为双核 ESP32（Wemos LOLIN32 Lite，240 MHz）。
- PlatformIO 平台为 `espressif32@6.10.0`，Arduino-ESP32 为 `2.0.17`，SimpleFOC 为 `2.3.3`。
- FreeRTOS tick 为 1000 Hz，抢占和同优先级时间片已启用，优先级范围为 0～24，tickless idle 未启用。
- Arduino `loopTask` 已由框架固定在 Core 1、优先级 1、栈 8192 字节。
- 只使用框架现有 API，不新增或修改 `FreeRTOSConfig.h`、`sdkconfig`、`sdkconfig.h`，也不通过 `build_flags` 重定义 `CONFIG_FREERTOS_*`。
- `balancing_vehicle/` 是只读行为锚点，所有实现只进入 `balancing_vehicle_work/`。

## 2. 当前实现的主要问题

模块重构后，`loop()` 只调用 `VehicleControlRuntime::runOnce()`，但运行时仍将以下工作串在一个不受控的高速循环中：

1. 两台电机的 `loopFOC()` 和 `move()`；
2. BMI160 加速度计、陀螺仪读取与互补滤波；
3. 速度 PID、直立 PID、油门与转向滤波；
4. 左右电机目标电压混合。

这带来以下风险：

- FOC、姿态、速度环的实际频率都由 I²C、BLE 中断和其他框架负载共同决定，无法稳定复现。
- BMI160 当前 ODR 为 100 Hz，但姿态滤波在每次 `loop()` 中重复运行；固定 `0.98/0.02` 权重的等效截止频率会随循环频率变化。
- 姿态估计器的输入时间戳仍为毫秒级，不能准确描述亚毫秒抖动。
- BLE 回调和控制循环改为共享一个最新 `RemoteCommand` 快照，但尚未建立Queue或其他同步协议；跨核一致性仍未验证。
- BLE 断开后没有立即清零油门和转向，也没有命令超时保护。
- 电池电压只在启动时检查，运行中没有欠压监测。
- 直立输出与转向输出直接相加，理论最大值可超过母线或电机电压限制；独立钳位会破坏左右轮共同的平衡力矩。
- `motor.PID_velocity` 在 `MotionControlType::torque` 下不参与当前控制，容易误以为存在 SimpleFOC 内部速度闭环。
- `checkVinVolt()` 在 `Serial.begin()` 之前输出，启动等待信息不可见。

另一个决定任务边界的硬约束是：电机 0 的 AS5600 与 BMI160 共用 `Wire`，电机 1 的 AS5600 使用 `Wire1`。SimpleFOC 的 `loopFOC()` 会在内部调用编码器 `update()`。若将 FOC 与 IMU 读取直接放到不同任务或不同核，就必须让高频控制路径等待 I²C 互斥锁，并存在 I²C 驱动重入和优先级反转风险。

因此首版采用“模块细分、实时执行单一所有者”的方案：控制模块可以拆开，但两路 I²C、SimpleFOC 对象、状态估计器和控制器只由一个 Core 1 实时任务调用。

## 3. 当前模块结构与后续调度壳

```text
balancing_vehicle_work/
├── include/
│   ├── app_types.h            # Command、Sample、Estimate、Output、Telemetry 值类型
│   ├── vehicle_config.h       # 引脚、方向、供电、控制和协议配置
│   ├── command_input.h        # BLE 适配、解析器和最新命令聚合域
│   ├── attitude_system.h      # ImuDriver 与 AttitudeEstimator 聚合域
│   ├── motor_foc_service.h    # SimpleFOC/AS5600 唯一代码所有者
│   ├── vehicle_safety.h       # PowerMonitor；后续扩展 SafetyManager
│   ├── vehicle_control.h      # 两级控制、混合器与运行时组合根
│   └── app_tasks.h            # 后续阶段才新增：静态Task与通信对象
├── src/
│   ├── main.cpp               # 当前只调用 VehicleControlRuntime
│   ├── command_input.cpp
│   ├── attitude_system.cpp
│   ├── motor_foc_service.cpp
│   ├── vehicle_safety.cpp
│   ├── vehicle_control.cpp
│   └── app_tasks.cpp          # 后续阶段才新增
├── test/
│   ├── test_command_parser/
│   ├── test_control_mixer/
│   ├── test_safety_manager/
│   └── test_estimator_math/
└── docs/
    └── freertos_refactor_plan.md
```

当前聚合结构刻意避免“一个类一个文件”。模块拆分不等于每个模块都创建任务；任务只用于隔离具有不同实时性、核亲和性或阻塞行为的执行域。

## 4. 双核任务分配

| 执行域 | 核 | 优先级 | 周期/触发 | 初始栈预算 | 职责 |
|---|---:|---:|---:|---:|---|
| `control_fast` | 1 | 6 | 1 ms，1000 Hz | 6144 B | 独占 I²C、编码器、BMI160、SimpleFOC、估计器和控制器；内部多速率调度 |
| `supervisor` | 0 | 4 | 10 ms，100 Hz | 3072 B | 电池采样、慢速健康检查、产生粘滞故障位，不直接操作电机 |
| `telemetry` | 0 | 2 | 50 ms，20 Hz | 4096 B | 读取最新快照，执行 Serial/BLE 遥测和低优先级诊断 |
| Arduino `loopTask` | 1 | 1 | 启动后永久阻塞 | 框架 8192 B | 运行 `setup()`；任务启动成功后调用 `vTaskDelay(portMAX_DELAY)` |
| ESP-IDF Bluetooth Controller/Host | 0（当前预编译 SDK 固定） | 框架管理；Controller 约 23 | 事件驱动 | 框架管理 | BLE 改用 ESP-IDF 原生 Bluedroid GATTS API；回调只校验定长命令、写入长度 1 的最新值队列，不打印、不控制电机 |

优先级 6 足以让控制任务压过本项目的普通任务，同时避免无理由占用 20 以上的系统级优先级。Core 1 不再安排与 `control_fast` 同优先级的任务，从而避免时间片带来的控制抖动。

初始栈是保守预算，不是最终结论。应使用本 ESP-IDF FreeRTOS 端口的“栈深度按字节”接口约定创建静态任务，并在 BLE 压力测试和最长日志格式化路径下记录各任务 high-water mark；稳定运行后仍应保留至少 25% 余量。

## 5. `control_fast` 内部多速率方案

`control_fast` 使用 `vTaskDelayUntil()`，因为当前 1 ms tick 可以直接产生 1000 Hz 基准周期。FreeRTOS 软件定时器任务只有优先级 1、队列长度 10，不用于闭环控制。

| 子功能 | 频率 | 调度方式 | 控制/处理方案 |
|---|---:|---|---|
| 两电机 `loopFOC()` | 1000 Hz 起步 | 每个基准周期 | 读取两个 AS5600 并进行电角度换相；实机测得有余量后再评估 2000 Hz |
| 轮速更新与 `move()` | 1000 Hz | 每个基准周期 | 保持 SimpleFOC `torque + voltage` 模式；`move(targetVoltage)` 只负责更新 q 轴电压命令 |
| BMI160 六轴连续读取 | 500 Hz | 每 2 个基准周期 | 使用一次 `readMotionSensor()` 连续读取；BMI160 ODR 建议设为 800 Hz，保证每次控制读取都有新样本 |
| 姿态估计 | 500 Hz | 紧随 IMU 新样本 | 首版使用按实际 `dt` 计算权重的互补滤波；等效时间常数从原 100 Hz、0.98 权重换算为约 0.49 s |
| 直立内环 | 500 Hz | 紧随姿态估计 | `pitch error -> q-axis voltage` 的 PID/PD；输出先限幅，再进入混合器 |
| 速度外环 | 100 Hz | 每 10 个基准周期 | 平均有符号轮速与油门目标之差进入 PI/PID，输出受限目标俯仰角 |
| 油门/转向整形 | 100 Hz | 与速度外环同周期 | 一阶低通、斜率限制、死区；BLE 原始值不能直接成为电机电压 |
| 快速安全判定 | 1000/500 Hz | 每周期/每个姿态样本 | 命令时效、有限数检查、控制任务超期、倾角越界；故障后由控制任务置零并关闭电机 |
| 遥测快照发布 | 100 Hz | 每 10 个基准周期 | `xQueueOverwrite()` 发布最新值，绝不在实时任务中格式化或打印 |

1000 Hz FOC 是第一阶段的可验证基线，不是实机性能结论。400 kHz I²C 下，两只 AS5600 的事务时间以及每隔一周期的 BMI160 连续读取会占用显著预算。只有实机测得最坏执行时间小于约 700 µs、峰值不超过 900 µs，且长时间无超期，才考虑用 `esp_timer` 仅发送任务通知，将 FOC 提升到 2000 Hz。定时器回调本身不得访问 I²C、SimpleFOC 或控制器。

## 6. 单周期执行顺序

每个 1 ms 基准周期建议按以下顺序执行：

1. 记录释放时间并检查上周期是否超期；
2. 调用两台电机的 `loopFOC()`，刷新编码器并施加上一周期准备好的 q 轴电压；
3. 从已更新的编码器对象取得轮速快照；
4. 到达 500 Hz 分频点时读取 IMU、更新姿态并运行直立内环；
5. 到达 100 Hz 分频点时读取最新命令，运行命令整形与速度外环；
6. 检查快速故障并选择正常或安全输出路径；故障优先于正常输出，并锁存状态；
7. 正常路径运行统一混合与饱和，故障路径强制目标为零；
8. 调用两台电机的 `move(targetVoltage)`，为下一次 `loopFOC()` 准备电压；必要时随后关闭驱动；
9. 按分频发布只含值类型的遥测快照。

这种顺序带来一个明确且可测的约 1 ms“控制计算到 PWM 更新”延迟，优于当前由不定长循环形成的未知延迟。不要为了消除这 1 ms 而绕过 SimpleFOC 直接跨模块写 PWM。

## 7. 跨任务通信与所有权

建议的数据通道如下：

- BLE 回调 → `control_fast`：长度 1 的 `CommandFrame` 静态队列，使用 `xQueueOverwrite()`；帧内含油门、转向、接收时间戳和序号。
- `supervisor` → `control_fast`：粘滞故障位或长度 1 的 `SupervisorStatus`；故障只能由明确的恢复流程清除。
- `control_fast` → `telemetry`：长度 1 的 `TelemetryFrame` 静态队列，消费者只取最新状态，不积压旧日志。
- 任务启动确认：可使用单 notification index；启动完成后不要再把同一个 notification index 同时解释为多种无关事件。

禁止跨任务直接读写以下对象：

- `BLDCMotor`、`BLDCDriver3PWM`、`MagneticSensorI2C`；
- BMI160/Wire/Wire1；
- SimpleFOC `PIDController` 和 `LowPassFilter`；
- 控制器内部积分量和滤波器历史状态。

首版不需要 I²C mutex，也不应依赖 `volatile float`。队列复制的是小型快照，延迟可界定，且避免跨核 C++ 数据竞争。

BLE 的初始化、GATT 事件机、当前 BTDM 预编译配置限制和压力测试方法详见 [BLE 去 Arduino 依赖与 FreeRTOS 隔离方案](ble_esp_idf_migration_plan.md)。这里的去依赖只移除 Arduino BLE C++ 封装；ESP-IDF 管理的 Bluetooth Controller/Host 系统任务仍然存在并固定在 Core 0。

## 8. 控制方案边界

### 8.1 SimpleFOC 内层

保持：

- `TorqueControlType::voltage`；
- `MotionControlType::torque`；
- SimpleFOC 负责编码器角度、电角度换相和 q 轴电压施加。

这不是闭环电流控制，因此“目标转矩”实际仍是电压近似。电池电压、绕组电阻、反电动势和电机温度都会改变相同命令对应的真实转矩。外环整定时必须把这一点作为实机误差来源。

### 8.2 速度外环

状态量为两轮有符号角速度平均值：

```text
wheelSpeed = (M0 * leftVelocity + M1 * rightVelocity) / 2
speedError = wheelSpeed - filteredThrottleTarget
targetPitch = clamp(speedController(speedError), -pitchLimit, +pitchLimit)
```

100 Hz 与 500 Hz 直立内环形成 1:5 的带宽层级。首轮迁移可保留现有参数作为起点，但固定采样率、传感器 ODR 和滤波方式改变后必须重新整定，不能把编译成功视为参数仍然有效。

### 8.3 直立内环

状态量至少包括俯仰角和陀螺仪角速度。首版可继续使用现有 PID 接口；更推荐在验证阶段明确写成角度 P、角速度 D 和小积分项，以避免对噪声角度做数值微分。输出单位统一定义为伏特。

### 8.4 转向和输出饱和

转向先作为低频差分电压，后续若需要更一致的转向响应，再增加左右轮差速闭环。混合器应优先保留平衡所需的共同电压，并按剩余电压余量限制转向：

```text
balanceVoltage = clamp(balanceVoltage, -motorLimit, +motorLimit)
steeringLimit = motorLimit - abs(balanceVoltage)
steeringVoltage = clamp(steeringVoltage, -steeringLimit, +steeringLimit)
leftTarget  = M0 * (balanceVoltage + steeringVoltage)
rightTarget = M1 * (balanceVoltage - steeringVoltage)
```

速度 PID 的目标俯仰角、直立 PID 的电压以及最终电机命令都要分别限幅，并实现积分抗饱和。`driver.voltage_power_supply` 和 `motor.voltage_limit` 应显式配置，不能依赖库默认值。

### 8.5 LQR 的后续边界

在 PID 架构稳定后，可以把状态扩展为 `x = [pitch, pitchRate, wheelPosition, wheelVelocity]`，由 LQR 直接输出共同电压，转向仍作为独立差分通道。进入 LQR 前必须：

1. 获得质量、质心高度、轮半径、等效转动惯量和电机电压到力矩模型；
2. 检查离散模型在目标采样周期下的可控性；
3. 在仿真中说明 Q 对各状态偏差的惩罚、R 对电压消耗和激进程度的惩罚；
4. 将电压饱和、命令斜率、状态估计延迟和跌倒状态纳入仿真；
5. 再进行限压、悬空、系留实机验证。

仿真可稳定不代表电压模式实机一定稳定，LQR 参数不能直接从理想模型搬到车辆。

## 9. 安全状态机

建议状态：

```text
BOOT -> WAIT_POWER -> CALIBRATING -> STANDBY -> BALANCING
                                      ^             |
                                      |             v
                                   RECOVER <------ FAULT
```

关键规则：

- 上电后先将驱动使能置为安全态，再初始化 Serial、ADC、I²C 和传感器。
- 欠压等待和电机对齐必须是显式状态；对齐失败不得继续创建平衡输出。
- 只有任务启动确认、命令为零、姿态处于允许窗口且传感器有效时才允许进入 `BALANCING`。
- 倾角过大、IMU 非有限数、控制任务连续超期或初始化失败应立即置零并禁用电机。
- BLE 命令超过建议的 250 ms 未更新时，油门和转向按斜率回零；超过更长的解除阈值时退出平衡状态。阈值需结合遥控端实际发包周期验证。
- 欠压应采用滤波和去抖，避免电机瞬态电流造成误触发；故障恢复必须要求用户重新解锁，不能电压回升后自动突然启动。
- 驱动使能引脚只由 `control_fast`/安全管理器的同一执行所有者修改，Core 0 监督任务只提交故障请求。

## 10. 分阶段迁移

### 阶段 A：基线与可测性

- 保留当前行为，记录空载循环频率、两路 AS5600 读取时间、BMI160 连续读取时间和整体最坏执行时间。
- 添加 GPIO 脉冲或微秒时间戳统计，验证 BLE 连接、持续写命令时的抖动。
- 记录当前实机能够保持平衡时的 pitch、wheel speed、target pitch 和输出电压，作为回归数据。

### 阶段 B：纯模块拆分

- 已引入固定大小值类型和代码级单一所有权，移除 `config.cpp` 中的可变过程全局量。
- 已按同类聚合拆分解析器、IMU/估计器、两级控制器、现状混合器、电机FOC服务和启动电源监测。
- 统一饱和、安全状态机和运行期欠压仍未实现，必须作为后续独立行为变更。
- 后续先对命令解析、混合器、状态转换和滤波数学编写可在主机运行的单元测试。

### 阶段 C：FreeRTOS 执行壳

- 静态创建 `control_fast`、`supervisor` 和 `telemetry`。
- 建立三个静态通信通道和任务启动确认。
- `loop()` 永久阻塞；所有硬件对象转移给指定任务所有者。
- 先在电机断电或轮子悬空条件下验证周期、栈、队列和故障路径。

### 阶段 D：固定采样率与重新整定

- BMI160 使用连续六轴读取，设置明确 ODR。
- 互补滤波改为基于 `dt` 的时间常数形式，验证后可接入 `encoder_attitude` 中已有的卡尔曼实现。
- 依次整定直立环、速度环、命令滤波和转向；每次只改变一层。

### 阶段 E：性能提升

- 根据实机 WCET 和抖动决定是否把 FOC 提升至 2000 Hz。
- 若 400 kHz I²C 成为瓶颈，优先优化连续读取和总线事务；不要先用跨核 mutex 掩盖总线带宽不足。
- PID 版本稳定并获得模型参数后，再评估 LQR。

## 11. 验收标准

以下是首轮工程目标，必须由实机测量确认：

- `control_fast` 连续运行 10 分钟无丢周期；常规 WCET 小于 700 µs，峰值小于 900 µs。
- BLE 持续收包和遥测开启时，控制周期无连续超期，I²C 无错误或状态破坏。
- 快速故障被控制任务发现后，电机目标在 1 个控制周期内归零并进入禁止状态。
- Core 0 的 100 Hz 监督故障在约 11 ms 内被 Core 1 消费，且故障保持粘滞直到明确复位。
- 所有任务在最长运行路径下保留至少 25% 栈余量，运行时空闲堆不持续下降。
- 命令断流、BLE 断开、欠压、IMU 非法值、任务超期和倾倒均有可重复的台架测试。
- 单元测试覆盖命令边界值、混合饱和、方向符号、状态机禁止转换、毫秒计数回绕和非有限浮点输入。

## 12. 当前基线构建

当前源码在既定 PlatformIO 环境下可成功构建：

- 静态 RAM：41652 / 327680 B（12.7%）；
- Flash：1183993 / 1310720 B（90.3%）。

Flash 余量约 126727 B。新增日志、BLE 特性和诊断字符串时需要持续检查固件尺寸；任务静态栈则主要增加 RAM 占用。该构建结果只证明源码可链接，不代表现有控制周期或实机稳定性已经验证。
