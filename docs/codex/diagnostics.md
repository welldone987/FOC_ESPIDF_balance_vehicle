# 统一错误与故障维护

普通故障保存在RAM，掉电后丢失；需要保留时采集串口或使用调试器读取。Flash Core dump仅补充panic现场，不用于日常日志，也不因普通故障主动panic。固件不自动擦除NVS或旧dump。

## 启动和停机

boot_step按安全输出、存储、电源/电压、NVS/dump、BLE、可选Wi-Fi、IMU、电机、输出关闭、定时器、完成推进。电机子步骤沿用ErrorPoint数值，BEGIN/OK可定位驱动、零偏、编码器及左右对齐；失败摘要含point、原始码及源码位置。必要失败锁存停机，Wi-Fi失败DEGRADED。BOOT_SUMMARY OK后开始独立零速平衡，仅采纳控制就绪后收到的运动目标。

错误接口为esp_err_t + ErrorInfo。value/threshold/channel分别由valid_fields位0/1/2说明是否有效；comparison=-1表示<=，+1表示>，0表示其他比较。相电流比较采用绝对值，ErrorInfo.value保留有符号电流。ADC四通道0/1是左A/B，2/3是右A/B；重构相的channel=0/1表示左/右C。IMU写失败的channel保存寄存器地址，value保存写入字节；读取失败value保存寄存器地址。句柄创建API只返回空指针时，错误域为application，不伪造SDK原始码。

检测错误后先请求公共禁能，复制最后有效控制现场到独立故障槽，记录首因和次级禁能错误，再锁存故障并清理运行状态。首故障不会被BLE异常、重连或后续错误覆盖。运行电压仍只在启动检查；没有新增连续欠压保护。

## 串口运行诊断

初始化后独立零速平衡；BLE只提供运动目标，断连/超时归零但不关闭平衡输出。串口CONTROL_START说明模式，CONTROL_CONFIG报告2000/5000/10000us目标周期，CONTROL_LIMITS报告4000/2000us编码器/电流输出年龄上限。app_main每100ms观察首帧及首次平衡成功或故障，输出CONTROL_LOOP_ALIVE/CONTROL_BALANCE_ACTIVE后返回；不新增诊断任务。首帧完成不代表稳定性验收。

ControlTask每轮记录通知数量、累计合并释放数、cycle/balance_cycle、balancing/driving、starting、IMU是否更新、阶段与耗时。balancing表示本地平衡使能，driving表示遥控授权，starting表示首次或恢复平衡。首轮控制dt初始化为2000 us、姿态dt初始化为5000 us，first_release=1明确其非实测；后续间隔仍按实际时间计算，零/负/超过10 ms的间隔仍会失败。首帧姿态使用加速度计建基准，倾倒检查不等待滤波从零收敛。初始化打印期间的通知可能合并，notify/skipped会揭示积压，不补算旧周期。

运行故障先禁能、冻结故障计时、记录首因并停止定时器，再一次性打印CONTROL_FAULT、CONTROL_ERROR_FIELDS、CONTROL_TIMING、CONTROL_STAGE、CONTROL_OUTPUT、CONTROL_LAST_VALID及CONTROL_SUMMARY FAIL。次级禁能失败另行打印。高频正常路径只记内存，不执行串口格式化。

output_age（786/0x0312）使用4 ms编码器输出年龄限制和原始编码器开始时间；current_output_age（788/0x0314）独立使用2 ms电流输出年龄限制和ADC开始时间。两种年龄均在BEFORE_PWM、AFTER_PWM检查。编码器/ADC读取耗时各保留独立2ms限制；这些返回后检查不替代底层调用超时。ADC/math/PWM以及编码器、IMU、外环、输出关闭耗时均为us。两种输出年龄故障的ErrorInfo.value/threshold分别填写实测年龄和4000/2000，valid_fields=3，comparison=1；RAM的fault_timing保留故障当轮，fault_control保留此前最后有效轮，first_loop/first_balance保留首个完整周期。RAM schema=4；不保留BLE错误包；.007仅通知测量值与时效有效位，不含故障原因。

详细串口操作与验收见[串口排查步骤](./serial_output_age_debug.md)。当前没有新增CONTROL_READY连续运行门，也没有实现卡在外设调用内的独立超时关断；这两项仍见[后续运行诊断方案](./runtime_diagnostics_plan.md)。

## 唯一错误规范

`components/Middlewares/Diagnostics/error_info.hpp`定义ErrorPoint、ErrorDomain和ErrorInfo。BLE初始化及协议栈运行失败同样走ErrorInfo；协议格式错误被丢弃，不产生第二套应用错误码。NimBLE的ATT响应仅为协议栈要求的传输返回值。BLE没有publishFault、独立故障数字映射、错误订阅或重放。

`scripts/diagnostic_dictionary.json`只由`python scripts/generate_diagnostic_dictionary.py`从ErrorPoint生成，供本地查码；不是独立维护的规范。已移除无线包解码脚本和BLE诊断快照。首故障及16条事件仍可通过串口/匹配ELF的Core dump检查。
