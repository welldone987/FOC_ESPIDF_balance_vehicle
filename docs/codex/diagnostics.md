# BLE DIAG与故障维护

普通故障保存在RAM，掉电后丢失；需要保留时由客户端导出。Flash Core dump仅补充panic现场，不用于日常日志，也不因普通故障主动panic。固件不自动擦除NVS或旧dump。

## 启动和停机

boot_step按安全输出、存储、电源/电压、NVS/dump、BLE、可选Wi-Fi、IMU、电机、输出关闭、定时器、完成推进。电机子步骤沿用ErrorPoint数值，BEGIN/OK可定位驱动、零偏、编码器及左右对齐；失败摘要含point、原始码及源码位置。必要失败锁存停机，Wi-Fi失败DEGRADED。BOOT_SUMMARY OK后才放行ARM，启动前ARM不会延后生效。

错误接口为esp_err_t + ErrorInfo。value/threshold/channel分别由valid_fields位0/1/2说明是否有效；comparison=-1表示<=，+1表示>，0表示其他比较。相电流比较采用绝对值，ErrorInfo.value保留有符号电流。ADC四通道0/1是左A/B，2/3是右A/B；重构相的channel=0/1表示左/右C。IMU写失败的channel保存寄存器地址，value保存写入字节；读取失败value保存寄存器地址。句柄创建API只返回空指针时，错误域为application，不伪造SDK原始码。

检测错误后先请求公共禁能，复制最后有效控制现场到独立故障槽，记录首因和次级禁能错误，再锁存故障并清理运行状态。首故障不会被BLE异常、重连或后续错误覆盖。运行电压仍只在启动检查；没有新增连续欠压保护。

## 串口运行诊断

当前初始化后开始独立零速平衡，BLE ARM只授权运动目标；S、断连和超时仍关闭输出，停止后须重新ARM。串口CONTROL_START说明启动模式，CONTROL_CONFIG报告1000/5000/10000us目标周期，CONTROL_LIMITS报告4000/2000us编码器/电流输出年龄上限。BOOT_SUMMARY OK之后，app_main复用其现有任务，每100 ms读取受短临界区保护的定长快照，输出CONTROL_LOOP_ALIVE与首个成功平衡周期CONTROL_BALANCE_ACTIVE；首次平衡成功或故障后返回。启动前已收到停止事件时输出CONTROL_STOPPED并等待重新ARM。不新增诊断任务。首帧完成不代表连续周期预算或闭环稳定性验收。

ControlTask每轮记录通知数量、累计合并释放数、cycle/balance_cycle、balancing/driving、starting、IMU是否更新、阶段与耗时。balancing表示本地平衡使能，driving表示遥控授权，starting表示首次或恢复平衡。首轮控制dt初始化为1000 us、姿态dt初始化为5000 us，first_release=1明确其非实测；后续间隔仍按实际时间计算，零/负/超过10 ms的间隔仍会失败。首帧姿态使用加速度计建基准，倾倒检查不等待滤波从零收敛。初始化打印期间的通知可能合并，notify/skipped会揭示积压，不补算旧周期。

运行故障先禁能、冻结故障计时、记录首因并停止定时器，再一次性打印CONTROL_FAULT、CONTROL_ERROR_FIELDS、CONTROL_TIMING、CONTROL_STAGE、CONTROL_OUTPUT、CONTROL_LAST_VALID及CONTROL_SUMMARY FAIL。次级禁能失败另行打印。高频正常路径只记内存，不执行串口格式化。

output_age（786/0x0312）使用4 ms编码器输出年龄限制和原始编码器开始时间；current_output_age（788/0x0314）独立使用2 ms电流输出年龄限制和ADC开始时间。两种年龄均在BEFORE_PWM、AFTER_PWM检查。编码器/ADC读取耗时各保留独立2ms限制；这些返回后检查不替代底层调用超时。ADC/math/PWM以及编码器、IMU、外环、输出关闭耗时均为us。两种输出年龄故障的ErrorInfo.value/threshold分别填写实测年龄和4000/2000，valid_fields=3，comparison=1；RAM的fault_timing保留故障当轮，fault_control保留此前最后有效轮，first_loop/first_balance保留首个完整周期。RAM schema=3，BLE schema仍为1。

详细串口操作与验收见[串口排查步骤](./serial_output_age_debug.md)。当前没有新增CONTROL_READY连续运行门，也没有实现卡在外设调用内的独立超时关断；这两项仍见[后续运行诊断方案](./runtime_diagnostics_plan.md)。

## 无线契约

主服务及.004 v2命令、.003状态包保持原单位、缩放和applied_sequence语义。.002 READ返回`legacy control unsupported`，WRITE拒绝。新增DIAG UUID：`6e400005-b5a3-f393-e0a9-e50e24dcca9e`。

所有多字节字段使用小端，schema=1；不要将C++结构体内存当作协议。

| NOTIFY偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0 | 1 | schema |
| 1 | 2 | diag16，当前为稳定ErrorPoint字典值 |
| 3 | 4 | event_seq，从1递增，仅表示RAM提交顺序 |
| 7 | 1 | 原始域：0 application、1 ESP-IDF、2 NimBLE、3 SimpleFOC |
| 8 | 1 | flags：bit0 fatal，bit1 first fault |
| 9 | 4 | 有符号原始错误码 |
| 13 | 1 | channel；255表示不适用 |

| READ偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0 | 1 | schema |
| 1 | 1 | bit0启动完成，bit1存在首故障 |
| 2 | 2 | boot_step |
| 4 | 4 | 最后event_seq |
| 8 | 4 | 首故障event_seq，0表示无 |
| 12 | 2 | 首故障diag16 |
| 14 | 1 | 当前事件数 |
| 15 | 1 | 容量16 |
| 16 | 2 | MTU |
| 18 | 1 | 是否连接 |
| 19 | 1 | 是否订阅DIAG |

只有订阅DIAG时发送；每次100 ms callout至多提交一条。首次订阅/重连先重放首故障，再按顺序重放仍保留的历史。首故障可能同时出现在环中，客户端按event_seq去重。提交失败不推进游标，不删除记录。RAM环覆盖会造成序号缺口；Notify成功只代表本地协议栈接收，不代表手机已保存。最后BLE传输异常单独存储，避免通知失败不断产生新通知事件。

字典：[diagnostic_dictionary.json](../../scripts/diagnostic_dictionary.json)。解码工具：[diag_protocol.py](../../scripts/diag_protocol.py)。新增检测点后运行`python scripts/generate_diagnostic_dictionary.py`并验证字典。

## 新版网页读取

[新版遥控台](../平衡车控制界面（新版）.html)连接后读取.005摘要并订阅历史/实时错误事件，不需要ARM。页面保留原.004遥控和.003状态协议，增加首次故障、原始域/有符号码/通道、事件去重与JSON导出。完整字典嵌入HTML，单文件不依赖外部网络；更新字典时同步HTML的DIAG_POINTS，`node --test tests/ble_diagnostics_web.test.cjs`会检查一致性。

控制循环状态根据.003的16位sample_sequence是否变化及状态有效性推断；它与命令applied_sequence不同。600 ms内没有收到状态表示通信回报中断，仍有状态但采样序号不动表示没有观测到新控制快照；均不直接断言具体阻塞位置。BOOT通过不能证明已完成首轮控制。网页只呈现BLE现有字段，不显示未发送的value/threshold/file/line或完整CrashState，也不报告Flash dump存在性。

设备先重放首故障再重放环，页面按event_seq去重并按序排列。每次连接独立记录，避免重启后的序号与旧会话混合；页面保留最近8次连接，每次最多256条事件及独立首故障，断连不清空。导出包含摘要、事件原始十六进制、最近状态、接收时间和最近解码错误。刷新/关闭网页会失去页面内记录，设备复位会清空RAM，需事先导出。时间为客户端接收时间，不是设备事件发生时间。设备摘要只在连接和手动刷新时读取；驾驶期间不执行手动读取，与遥控写操作串行，事件通知持续接收。

本地验证：15项Node协议/生命周期测试通过，除报文、去重、重连与导出外，新增FAULT后不发送S、在途写入拒绝后保留诊断连接、未知写入失败仍断开。收到终态或首故障后停止D/A/S，显式E仍可幂等请求；导出新增lastValidStatus、lastGattError（含命令）、endReason。既有布局已核对桌面与390px手机宽度；手机/实板修复后行为仍为Unverified。

```powershell
python scripts/diag_protocol.py '01 04 02 78 56 34 12 01 03 85 ff ff ff 03'
python tests/diagnostics/test_protocol.py
```

## Flash Core dump

保留现有128 KiB coredump分区；开启Flash保存和`CONFIG_ESP_COREDUMP_FLASH_NO_OVERWRITE=y`。`g_diag_crash`使用`COREDUMP_DRAM_ATTR`保存启动阶段、事件、首故障、BLE原始异常/连接/订阅/MTU与控制快照。没有开启整个DRAM/heap捕获。

旧dump存在时后续panic不会覆盖。空白分区才可保存新的首个dump；损坏旧内容也需要人工判断、提取后清除，本固件不擦除。维护时保留产生dump的ELF及对应构建目录，不要拿当前重新编译的ELF解析旧dump。

```powershell
idf.py -B <matched-build-dir> -p <serial-port> coredump-info
idf.py -B <matched-build-dir> -p <serial-port> coredump-debug
```

上述实板命令本次未执行；清除操作也未执行。128 KiB容量是否足够保存实际任务栈组合仍需实板panic验证。

详细提取、保存、离线GDB查看命令见[RAM与Core dump读取](./ram_coredump_reading.md)。BOOT到控制循环的补齐方案见[运行诊断方案](./runtime_diagnostics_plan.md)，该方案尚未实现。

## Wi-Fi与实板验收

`CONFIG_VEHICLE_WIFI_ENABLED`为真正总开关；关闭后无Wi-Fi初始化、专用任务/栈/队列及控制侧Wi-Fi写入。开关启用时保留TCP调试与WIFI_PS_MIN_MODEM。关闭TCP的旧bool仅是次级开关。

以下均未实板/手机验证：GPIO12/22电路、上电安全态、对齐和禁能、电流相序/量程、GATT缓存刷新、MTU/订阅/重连/历史补发、Wi-Fi共存下最坏控制周期、断连/超时/S/E延迟、输出行为、任务栈水位、Flash dump保存/解码/清除。硬件确认门在当前源码已为true，本次保持原值；该值不构成硬件验证证明。同步I²C/ADC或库对齐中的阻塞仍限制软件关断时延。
