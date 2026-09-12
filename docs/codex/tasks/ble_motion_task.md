# BLE控制与单队列遥测

## 实现与协议

- [x] ESP-IDF v6.0.2 / ESP32 / NimBLE；使用安装目录下NimBLE_GATT_Server的GATT注册与订阅处理、blehr的notify_custom、blecsc的主机callout方式。
- [x] 保留BleTask及两级最新命令队列，回调不访问电机；300ms命令时效由ControlTask独立检查，断连/过期目标归零但保持平衡。
- [x] 名称“平衡车”，服务6e400001-b5a3-f393-e0a9-e50e24dcca9e。
- [x] 命令6e400002-b5a3-f393-e0a9-e50e24dcca9e，READ/WRITE，普通带响应写入；载荷X,Y，可带LF或CRLF。X右正，Y前正；整数±100，拒绝多字段、溢出、非法字符与旧D命令。单次写入最多20字节。READ为接口说明。
- [x] BLE按当前控制限速缩放：Y/100×DriveSpeedLimit_rad_s，X/100×YawRateLimit_rad_s；轮速目标为旧版25rad/s的80%=20rad/s，转向目标±2rad/s。
- [x] 遥测6e400007-b5a3-f393-e0a9-e50e24dcca9e，READ/NOTIFY，CCCD由NimBLE管理，100ms主机callout发送。连接句柄/订阅状态只归主机所有，断连/复位清空订阅；未订阅不发送，mbuf拥塞丢帧，不重试积压。
- [x] main在BLE启动前创建唯一长度1遥测队列，ControlTask每周期overwrite，NimBLE和可选Wi-Fi均peek。Wi-Fi关闭仍发布；沿用现有wifi_telemetry::TelemetrySnapshot类型和TCP字段。
- [x] 配套页面为[平衡车控制界面.html](../../平衡车控制界面.html)，50ms串行发送X,Y+LF，200ms操作延迟归零、1000ms操作超时断连，松手/失焦/后台清零；有有效遥测才允许非零目标。

## 20字节遥测格式

全部多字节字段为小端，不直接发送C++结构体。

| 偏移 | 类型 | 内容 |
| --- | --- | --- |
| 0 | uint8 | 格式版本1 |
| 1 | uint8 | bit0有效，其余0；有快照、current_valid、三项有限且样本年龄小于500ms |
| 2 | uint16 | 控制采样序号低16位，允许回绕 |
| 4 | uint32 | 控制采样时刻ms低32位，允许回绕 |
| 8 | float32 | 俯仰角deg |
| 12 | float32 | 左轮车辆坐标角速度rad/s |
| 16 | float32 | 右轮车辆坐标角速度rad/s |

无快照时版本1，其余0；非有限测量编码为0且无效。故障后控制停止发布，最后快照在500ms后变为无效；不是即时故障通知。网页同时跟踪采样序号与时刻，重复包不刷新样本活性；600ms无新样本清除显示和非零目标。收到新样本不会自动恢复之前的摇杆位置。有效位不表示驾驶命令已执行。

旧网页可向.002发送命令，但没有遥测显示和新网页保护；其250ms发送间隔与300ms超时只有50ms裕量，不作为最终配套页面。当前无ARM和无线急停接口。

## 官方源码依据

本机版本C:/esp/v6.0.2/esp-idf，官方同版本路径：

- [NimBLE_GATT_Server](https://github.com/espressif/esp-idf/tree/v6.0.2/examples/bluetooth/ble_get_started/nimble/NimBLE_GATT_Server)
- [blehr](https://github.com/espressif/esp-idf/tree/v6.0.2/examples/bluetooth/nimble/blehr)
- [blecsc](https://github.com/espressif/esp-idf/tree/v6.0.2/examples/bluetooth/nimble/blecsc)

## 验证

- [x] 当前ESP-IDF v6.0.2全项目构建；本次未重跑独立Wi-Fi关闭构建。
- [x] 当前网页Node模拟测试15/15：摇杆、松手、后台、软/硬超时、重连、遥测、诊断字段及序号确认；本次未进行浏览器视觉或蓝牙实测。
- [x] 本地ESP-IDF测试应用编译通过，新增最终每轮±1A限幅与上层不提前裁剪检查；更新与当前P增益、Uq滤波旁路不符的旧断言。
- [ ] 当前环境未找到qemu-system-xtensa，仿真测试未运行；不将idf.py qemu的零退出码视为测试通过。
- [ ] 实板连接、GATT缓存重发现、方向/速度、订阅/取消订阅、断连/快速重连、Wi-Fi共存、控制周期与栈水位。

本次不烧录、不自动驱动电机；构建和软件测试不证明射频、时序或闭环稳定性。

## 本地验证文件

tests/和scripts/均保留在本机并由.gitignore排除，不随Git分发；解析断言位于tests/ble/remote_protocol_checks.cpp，由本地诊断测试应用编译，不再进入生产固件构建。

## 错误诊断协议与参数验收

新增.008 READ/WRITE，沿用ErrorInfo/ErrorPoint/ErrorDomain及16条事件环，不新增诊断任务。READ返回UTF-8单事件文本（最长511字节，支持GATT长读），包含seq、flags、point、code/name、domain、raw、file、line、function、valid、value、threshold、channel、comparison。WRITE为4字节小端seq确认；未确认保持相同事件，连接重建后先重放独立首故障，再遍历保留环。网页按序号去重，环覆盖允许丢失历史；RAM重启丢失。断链后无法即时无线发送原因，重连后读取或查看串口。

错误点沿用原编号并追加：0x60c连接失败、0x60d断开、0x60e命令非法、0x60f命令过期、0x610连接参数更新失败；0x620网页GATT操作超时、0x621网页GATT失败、0x622网页遥测异常。网页domain=application，raw为DOMException.code或0，不伪造HCI原因。NimBLE原始错误码含其错误域偏移，不能只看低字节。断开事件value=连接间隔ms、threshold=监督超时ms、channel=连接延迟事件数；过期事件value/threshold单位us；notify事件value为累计失败数，最多每秒记录一次。

网页每250ms最多读取/确认一条，全部GATT操作与命令串行。200ms未完成即清除旧摇杆并优先排队零指令，1000ms仍未完成才断开；固件300ms指令有效期不变。串口在BleTask每100ms最多输出一条完整事件，ControlTask只复制故障，不格式化日志。

速度20rad/s、速度斜坡20rad/s²、偏航2rad/s、偏航斜坡4rad/s²、俯仰增量±4.8°、Uq上限4.8V；姿态/速度/偏航增益保持。每轮±1A仅在BSP电流环入口裁剪最终Iq请求，不在BLE或控制混合层限幅。电流PI输出单位V，实测电流瞬态超调不由目标限幅直接保证。实机方向、速度、稳定性、实际电流与断联仍为Unverified；不自动烧录。
