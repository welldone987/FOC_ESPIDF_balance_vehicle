# BLE 去 Arduino 依赖与 FreeRTOS 隔离方案

## 1. 结论

本项目建议采用“Arduino 启动框架 + ESP-IDF 原生 Bluedroid GATT Server API”的过渡架构：

- 主工程暂时保留 `framework = arduino`，避免同时迁移 SimpleFOC、Wire、Serial 和启动入口。
- BLE 模块移除 `Arduino.h`、`BLEDevice.h`、`BLEServer.h`、`BLEUtils.h`、Arduino `String`、`Serial`、`millis()` 和 `delay()`。
- BLE 模块只使用 ESP-IDF 4.4.7 已提供的 `esp_bt_*`、`esp_bluedroid_*`、`esp_ble_gap_*`、`esp_ble_gatts_*`、`esp_timer_get_time()` 和 FreeRTOS 静态队列 API。
- 不创建项目自有的 BLE 轮询任务。Bluetooth Controller、BTU/BTC 等任务由 ESP-IDF 管理；GAP/GATTS 回调只做有界校验和队列投递。
- `control_fast` 继续固定在 Core 1。当前预编译 SDK 已将 Bluetooth Controller 和 Bluedroid 固定在 Core 0，因此 Core 0 的 `supervisor`、`telemetry` 只能作为可被蓝牙抢占的低优先级任务。

这里的“去 Arduino 依赖”是指 BLE 业务实现不依赖 Arduino C++ BLE 封装，并不表示蓝牙协议栈不再使用 FreeRTOS。ESP-IDF 官方说明 `esp_bt_controller_init()` 本身就会分配控制器任务和其他资源。换成原生 API 可以控制边界、生命周期和回调工作量，但不能也不应消除 ESP-IDF 的蓝牙系统任务。

## 2. 版本与官方依据

本方案针对仓库当前锁定环境：

- PlatformIO `espressif32@6.10.0`；
- Arduino-ESP32 `2.0.17`；
- 本机 `esp_idf_version.h` 确认为 ESP-IDF `4.4.7`；
- Wemos LOLIN32 Lite，双核经典 ESP32。

主要官方资料：

- [ESP-IDF 4.4.7 Bluetooth Controller API](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/bluetooth/controller_vhci.html)：控制器初始化、启停、资源释放及调用顺序。
- [ESP-IDF 4.4.7 GATT Server API](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/bluetooth/esp_gatts.html)：GATTS 回调注册、应用注册、静态属性表、服务启动和通知接口。
- [ESP-IDF 4.4.7 GAP BLE API](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/bluetooth/esp_gap_ble.html)：广播数据配置、广播启动和连接参数接口。
- [ESP-IDF 4.4.7 Kconfig Reference](https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/kconfig.html)：Bluetooth Controller/Host 的核绑定、栈和模式配置。
- [ESP-IDF 4.4.7 GATT Server Service Table 官方示例](https://github.com/espressif/esp-idf/tree/v4.4.7/examples/bluetooth/bluedroid/ble/gatt_server_service_table)：BLE-only 初始化、异步广播配置和静态 GATT 属性表的参考流程。

官方示例中的 `esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)` 不能原样复制到当前工程。该示例使用 BLE-only SDK 配置，而本机 Arduino 预编译 SDK 的控制器模式是 BTDM；`BT_CONTROLLER_INIT_CONFIG_DEFAULT()` 的 `.mode` 由预编译配置决定，官方头文件也要求 `esp_bt_controller_enable(mode)` 与初始化配置中的模式一致。

## 3. 当前实现为什么会与 RTOS 重构互相干扰

当前 `src/command_input.cpp` 已移除Arduino `String`和显式动态callback对象，但Arduino BLE包装层仍存在以下边界：

1. `BLEDevice::init()` 隐藏了控制器、Bluedroid、GAP、GATTS 的初始化及一个固定延时，调用方无法表达异步就绪状态。
2. 回调仍使用Arduino BLE类并输出少量串口日志；包装层内部的堆分配、任务和锁竞争不由本项目控制。
3. 回调直接写 `steering`、`throttle` 全局 `float`，而控制任务将位于另一核，形成 C++ 数据竞争；`volatile` 不能修复该问题。
4. 断开回调只改布尔量并重启广播，没有把“断开”作为带时间戳的安全命令传给控制所有者。
5. 全局解析状态会让相邻写请求互相污染，也没有严格处理长度、偏移、prepared write、数值范围和非法字符。

原生 API 的价值不是让蓝牙“没有任务”，而是让应用只保留一个清晰、可测的跨域接口：

```text
Core 0 / ESP-IDF 管理                     Core 1 / 项目管理

BT Controller（系统高优先级）
        │
Bluedroid BTU/BTC
        │ GAP/GATTS callback
        │ 仅校验、复制、xQueueOverwrite()
        ▼
CommandFrame 静态最新值队列 ───────────► control_fast（唯一控制所有者）
        │                                      │
        └─ BleEvent 静态事件队列 ─► supervisor │ 命令超时/断开/故障优先

telemetry（低优先级、可丢帧） ──可选──► esp_ble_gatts_send_indicate()
```

## 4. 任务、核与优先级边界

本机预编译 SDK 的实际配置快照为：

```text
CONFIG_BTDM_CTRL_PINNED_TO_CORE=0
CONFIG_BT_BLUEDROID_PINNED_TO_CORE=0
CONFIG_BTDM_CTRL_MODE_BTDM=y
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_BLE_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=n
CONFIG_BT_BTC_TASK_STACK_SIZE=8192
CONFIG_BT_BTU_TASK_STACK_SIZE=8192
```

ESP-IDF 4.4.7 的 `ESP_TASK_BT_CONTROLLER_PRIO` 为 `configMAX_PRIORITIES - 2`，在当前优先级范围 0～24 下等于 23。这个系统任务会抢占 Core 0 上的普通应用任务，是正常行为。

建议保留以下应用分配：

| 执行域 | 核 | 优先级 | 原则 |
|---|---:|---:|---|
| ESP-IDF Bluetooth Controller/Host | 0 | 框架配置，控制器约 23 | 不修改、不挂起、不由应用看门狗管理 |
| `control_fast` | 1 | 6 | 唯一的电机、I²C、估计器和控制器所有者 |
| `supervisor` | 0 | 4 | 允许被 BLE 抢占；不得承担亚毫秒硬截止期 |
| `telemetry` | 0 | 2 | 可丢帧、可降频、不得反压控制任务 |
| Arduino `loopTask` | 1 | 1 | 完成启动后永久阻塞 |

不要把 `control_fast` 提升到 20 以上与系统任务竞逐，也不要为了“隔离”而修改预编译的 `CONFIG_BT_*_PINNED_TO_CORE`。Core 1 仍可能受到跨核临界区、系统中断和共享内存总线影响，因此 BLE 压力测试仍是实时验收的一部分。

## 5. 目标模块

建议将现有 `command_input.cpp/.h` 中的BLE适配部分替换为：

```text
include/ble_idf_service.h       # 原生 BLE 生命周期和状态查询
src/ble_idf_service.cpp         # Controller、Bluedroid、GAP、GATTS 事件机
include/command_input.h         # 保留现有命令值类型和解析入口
src/command_input.cpp           # 保留无堆定长解析，移除Arduino BLE适配
```

`ble_idf_service.cpp` 不得包含任何 Arduino 头文件。建议公开最小接口：

```cpp
enum class BleServiceState : uint8_t {
  kStopped,
  kStarting,
  kAdvertising,
  kConnected,
  kFault,
};

struct CommandFrame {
  int16_t steering_raw;       // -1500..1500
  int16_t throttle_raw;       // -40..40
  uint32_t received_ms;       // esp_timer_get_time()/1000
  uint32_t sequence;
  bool connected;
};

esp_err_t bleServiceInit(QueueHandle_t command_queue,
                         QueueHandle_t event_queue);
BleServiceState bleServiceState();
```

队列必须在 BLE 初始化前静态创建。`CommandFrame` 队列长度为 1，生产者使用 `xQueueOverwrite()`；控制任务每个 100 Hz 命令处理周期读取最新帧。事件队列只承载连接、断开、协议错误和 BLE 故障等低频事件，队列满时记录计数而不是阻塞回调。`bleServiceState()` 的内部状态使用原子整数读写，不能以普通全局枚举跨核共享。

## 6. 初始化与异步事件机

### 6.1 当前 Arduino 框架内的安全初始化顺序

1. Arduino Core 已在进入 `setup()` 前初始化 NVS；BLE 模块不重复擦除或格式化 NVS。
2. 断言 Bluetooth Controller 状态为 `ESP_BT_CONTROLLER_STATUS_IDLE`，保证没有 `BLEDevice`、`BluetoothSerial` 或其他模块抢先拥有协议栈。
3. 使用 `BT_CONTROLLER_INIT_CONFIG_DEFAULT()` 创建控制器配置。
4. 调用 `esp_bt_controller_init(&config)`。
5. 由于当前预编译配置是 BTDM，调用 `esp_bt_controller_enable(ESP_BT_MODE_BTDM)`，不能擅自改为 BLE。
6. 依次调用 `esp_bluedroid_init()`、`esp_bluedroid_enable()`。
7. 注册 GAP、GATTS 回调，再调用 `esp_ble_gatts_app_register(app_id)`。
8. 在 `ESP_GATTS_REG_EVT` 中设置设备名、配置广播包和扫描响应包，并调用 `esp_ble_gatts_create_attr_tab()`。
9. 等待两个异步广播配置完成事件和 `ESP_GATTS_CREAT_ATTR_TAB_EVT`；属性表成功后启动服务，所有前置条件满足后才开始广播。
10. `bleServiceInit()` 只表示“初始化请求已接受”。是否真正可连接由状态机或 `BleEvent` 报告，禁止用固定 `delay()` 猜测就绪时间。

任何一步失败都进入 `kFault`，发布故障事件，但不在回调中反复自动重建协议栈。车辆控制不依赖 BLE 初始化成功才能进入安全空闲态，但没有有效命令时不得进入运动状态。

### 6.2 为什么第一阶段不释放 Classic BT 内存

当前 SDK 是预编译 BTDM 配置，默认控制器配置的模式不是 BLE-only。因此第一阶段：

- 不调用 `esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)`；
- 不通过 `build_flags` 重定义 `CONFIG_BTDM_CTRL_MODE_*`；
- 不复制并篡改 `sdkconfig.h`；
- 不调用 Arduino `btStart()`，避免 BLE 模块重新依赖 Arduino HAL。

若后续 RAM 预算证明必须回收 Classic BT 内存，应单独立项构建 BLE-only ESP-IDF/Arduino SDK，并完整回归 BLE、SimpleFOC、启动和 OTA。官方说明内存释放不可逆，而且启用模式必须与控制器初始化模式一致。

## 7. GATT 与广播设计

优先采用官方 `gatt_server_service_table` 示例的静态属性表，而不是逐个动态创建 Service/Characteristic：

- 服务 UUID 保持 `6e400001-b5a3-f393-e0a9-e50e24dcca9e`；
- RX 特性 UUID 保持 `6e400002-b5a3-f393-e0a9-e50e24dcca9e`；
- 第一阶段维持 READ/WRITE 行为，避免同时修改手机端协议；
- 使用 `ESP_GATT_AUTO_RSP`，应用回调只处理完整普通写；prepared write 不刷新命令时戳；
- 属性值和广播配置使用静态存储期对象，避免异步 API 尚未完成时对象失效。

ESP-IDF 的 128-bit UUID 字节数组使用低字节在前的表示，不能直接按 UUID 字符串从左到右抄写。迁移时应为 UUID 转换和手机端发现结果增加测试。

广播包最长 31 字节。128-bit Service UUID、Flags 和 UTF-8 设备名“平衡车”不应盲目塞入同一包；建议主广播包放 Flags 与 Service UUID，扫描响应包放完整设备名。只有 GAP 同时报告广播包和扫描响应包配置完成后才启动广播。

## 8. 回调规则

GAP/GATTS 回调运行在 ESP-IDF 蓝牙 Host 的执行上下文，不是项目自有任务。统一遵守：

- 不使用 `Serial`、`printf` 大段格式化或十六进制 dump；
- 不使用 `new/delete`、Arduino `String`、容器扩容或异常；
- 不等待 mutex、队列空间、I²C、UART 或其他任务完成；
- 不调用电机、IMU、SimpleFOC、控制器或安全状态机对象；
- 每次只做固定上限的数据校验、拷贝和非阻塞队列写入；
- `param->write.value` 只在当前回调中读取，需跨任务使用的数据必须复制到值类型帧中；
- 所有错误只增加计数或投递小事件，日志由 `telemetry` 延后输出。

关键事件处理如下：

| 事件 | 回调动作 |
|---|---|
| `ESP_GATTS_CONNECT_EVT` | 保存连接 ID/地址快照，发布 connected 事件，不直接允许车辆运动 |
| `ESP_GATTS_DISCONNECT_EVT` | 立即覆盖一帧 `connected=false, steering=0, throttle=0`，再按官方示例重启广播 |
| `ESP_GATTS_WRITE_EVT` | 校验 handle、`is_prep`、offset、长度和字符，成功后覆盖最新命令；失败时不更新时间戳 |
| `ESP_GATTS_CONGEST_EVT` | 标记拥塞，暂停可选遥测通知；不影响命令接收和控制 |
| GAP 广播完成事件 | 更新状态或发布故障，不做循环重试风暴 |

## 9. 命令协议与安全语义

第一阶段保留手机端的 ASCII 协议 `steering,throttle`，但改用无堆分配的严格整数解析器：

- 总长度设置一个很小的硬上限，例如 16 字节；
- 仅允许一个逗号、可选负号和十进制数字；
- steering 范围 `-1500..1500`，throttle 范围 `-40..40`；
- 拒绝空字段、多逗号、溢出、尾随垃圾和 prepared write；
- BLE 回调只发布原始整数，不做 `maxSteering/maxThrottle` 浮点缩放；
- 缩放、死区、斜率限制和低通滤波由 `control_fast` 的命令输入模块在固定 100 Hz 周期完成。

非法数据不应立即改写上一条有效命令，也不能刷新其时戳。这样持续非法包最终会触发命令超时并安全回零。断开事件则必须立即发布零命令，不能等待超时。32-bit 毫秒时戳约 49.7 天回绕一次，超时比较必须使用无符号差值并编写回绕单元测试。

建议后续协议升级为带版本、序号和校验的固定二进制帧，但这应与手机端同步迁移，不能夹在 BLE API 迁移中一起完成。

## 10. BLE 遥测边界

首轮迁移建议只实现命令接收，不立刻增加通知特性。需要 BLE 遥测时：

- 由现有 `telemetry` 任务以不高于 20 Hz 读取最新快照；
- 只有连接有效、客户端启用 CCCD 且未拥塞时调用 `esp_ble_gatts_send_indicate()`；
- 优先使用无需确认的 notification，控制丢包而不是让旧遥测积压；
- 单连接最多只保留一个待发最新快照；
- BLE 发送失败或拥塞不得传播为控制任务阻塞。

不要额外创建“BLE 遥测任务”，除非实测证明现有 `telemetry` 的职责不可界定。任务越多并不会自动提高实时性，反而增加栈、调度和同步成本。

## 11. 生命周期与恢复

正常运行中不反复 disable/deinit 蓝牙。完整关闭必须按连接断开、停止广播、Bluedroid disable/deinit、Controller disable/deinit 的逆序执行，且只用于明确的系统关机或独立测试路径。

BLE 初始化故障后的第一版恢复策略是保持车辆安全并报告故障，不在现场自动销毁和重建协议栈。原因是控制器 API 存在严格状态顺序，且内存释放不可逆；自动重试容易把偶发错误变成难以复现的生命周期竞争。

## 12. 分阶段实施

### 阶段 A：基线测量

- 在尚未修改 BLE 前记录未连接、已连接、10/50/100 Hz 写命令时 `control_fast` 的周期抖动、WCET、丢周期和空闲堆。
- 记录当前手机端服务发现、读、写、断开重连行为。
- 确认工程中没有同时使用 `BluetoothSerial`、`SimpleBLE` 或 NimBLE。

### 阶段 B：纯解析器和数据通道

- 为现有 `command_input` 中的定长解析器补充无Arduino边界单元测试。
- 静态创建长度 1 的命令队列和低频 BLE 事件队列。
- 先让旧 BLE 回调只投递 `CommandFrame`，移除对全局控制量的直接写入。

### 阶段 C：原生 ESP-IDF BLE

- 用 `ble_idf_service` 替换 Arduino BLE 类。
- 采用静态 GATT 属性表和显式异步状态机。
- 删除 `command_input.cpp` 中剩余的Arduino BLE类和串口回调日志，保留解析与值类型接口。
- 构建后检查 Flash/RAM 变化；去掉 C++ 封装不保证协议栈本体显著缩小。

### 阶段 D：RTOS 合并与实机压力测试

- `control_fast` 固定 Core 1；`supervisor`/`telemetry` 固定 Core 0 且优先级低于系统蓝牙任务。
- 依次执行未连接、持续连接、命令洪泛、反复断连、非法包、最大长度包和 RF 较差环境测试。
- 记录各应用任务 stack high-water mark、最低空闲堆、队列覆盖/丢弃计数、BLE 拥塞计数和控制周期统计。

### 阶段 E：可选的完整 ESP-IDF 化

只有在上述架构稳定后，才评估把整个 PlatformIO 工程切换为 ESP-IDF。那会同时涉及 Arduino 启动、Wire、ADC、Serial、SimpleFOC 兼容层和构建系统，不应作为“避免 BLE 与 RTOS 打架”的前置条件。

## 13. 验收标准

- `src/ble_idf_service.cpp` 和命令解析实现中不存在 Arduino/BLE C++封装头文件与Arduino类型。
- 工程只初始化一次 Bluetooth Controller 和 Bluedroid；重复初始化会被检测并拒绝。
- BLE 回调没有动态分配、串口输出、阻塞等待或硬件访问。
- 连接、写命令、断开和重连功能与当前手机端兼容。
- 断开后 `control_fast` 在下一个控制周期内消费到零命令；非法包不刷新命令时戳。
- 100 Hz 持续写命令并同时输出 20 Hz 遥测时，控制任务连续运行 10 分钟无连续超期；WCET/峰值仍满足 FreeRTOS 重构方案的 700/900 µs 目标。
- BLE 压力测试下应用任务栈至少保留 25% 余量，空闲堆不持续下降。
- 不修改 `FreeRTOSConfig.h`、`sdkconfig`、`sdkconfig.h`，也不通过 `build_flags` 重定义 `CONFIG_BT_*` 或 `CONFIG_FREERTOS_*`。

## 14. 常见陷阱

- 把“移除 Arduino BLE 类”误认为“BLE 不再创建 RTOS 任务”。
- 复制官方 BLE-only 示例后，在当前 BTDM 预编译 SDK 中错误地释放 Classic 内存或以 BLE 模式 enable 控制器。
- 在 GAP/GATTS 回调里打印日志、解析浮点字符串、等待 mutex 或调用控制器。
- 认为 Core 0 只运行本项目的 `supervisor/telemetry`，忽略优先级约 23 的 Bluetooth Controller 任务。
- 用共享 `bool/float` 代替队列；即使单次 32-bit 访问看似原子，也不能建立完整的跨核一致性和时序语义。
- 在 BLE 尚未收到异步完成事件时用固定延时后假定服务已就绪。
- 将遥测 notification 可靠性置于控制实时性之上，导致拥塞时积压或反压。

## 15. 后续学习与验证方向

- 对照 ESP-IDF 官方 GATT service table 示例理解 `REG -> CREATE_ATTR_TAB -> START_SERVICE -> START_ADV` 的异步事件链。
- 学习 BLE connection interval、slave latency 和 MTU 对无线延迟与功耗的影响，但不要把连接间隔当作控制周期基准。
- 用 GPIO 脉冲和微秒时间戳区分理论核隔离、软件调度结果与实机 RF 压力下的最坏抖动。
- 在命令输入稳定后，再评估安全配对、固定二进制协议和 BLE telemetry；每次只引入一个可独立验证的变化。
