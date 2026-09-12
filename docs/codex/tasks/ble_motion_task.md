# BLE控制与单队列遥测

## 实现与协议

- [x] ESP-IDF v6.0.2 / ESP32 / NimBLE；使用安装目录下NimBLE_GATT_Server的GATT注册与订阅处理、blehr的notify_custom、blecsc的主机callout方式。
- [x] 保留BleTask及两级最新命令队列，回调不访问电机；300ms命令时效由ControlTask独立检查，断连/过期目标归零但保持平衡。
- [x] 名称“平衡车”，服务6e400001-b5a3-f393-e0a9-e50e24dcca9e。
- [x] 命令6e400002-b5a3-f393-e0a9-e50e24dcca9e，READ/WRITE，普通带响应写入；载荷X,Y，可带LF或CRLF。X右正，Y前正；整数±100，拒绝多字段、溢出、非法字符与旧D命令。单次写入最多20字节。READ为接口说明。
- [x] BLE按当前控制限速缩放：Y/100×kDriveSpeedLimitRadS，-X/100×kYawRateLimitRadS；不复制旧版差分电压或25rad/s油门映射。
- [x] 遥测6e400007-b5a3-f393-e0a9-e50e24dcca9e，READ/NOTIFY，CCCD由NimBLE管理，100ms主机callout发送。连接句柄/订阅状态只归主机所有，断连/复位清空订阅；未订阅不发送，mbuf拥塞丢帧，不重试积压。
- [x] main在BLE启动前创建唯一长度1遥测队列，ControlTask每周期overwrite，NimBLE和可选Wi-Fi均peek。Wi-Fi关闭仍发布；沿用现有wifi_telemtry::TelemetrySnapshot类型和TCP字段。
- [x] 配套页面为[平衡车控制界面.html](../../平衡车控制界面.html)，50ms串行发送X,Y+LF，200ms写超时断连，松手/失焦/后台清零；有有效遥测才允许非零目标。

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

- [x] ESP-IDF v6.0.2全项目构建及独立Wi-Fi关闭配置构建。
- [x] 网页Node模拟测试13/13：摇杆、松手、后台、写超时、重连、遥测解码及失效；浏览器桌面和390×844布局已检查。
- [x] 固件解析静态断言、遥测字节编码/500ms边界/NaN/Inf检查通过。QEMU全套20/21通过；唯一失败是既有outerLoopAtTwoHundredHzAttitude将P增益固定为0.064×RadToDeg，而当前源码为0.08×RadToDeg。本次未修改控制参数或该断言，不能宣称全套回归通过。
- [ ] 实板连接、GATT缓存重发现、方向/速度、订阅/取消订阅、断连/快速重连、Wi-Fi共存、控制周期与栈水位。

本次不烧录、不自动驱动电机；构建和软件测试不证明射频、时序或闭环稳定性。

## 本地验证文件

tests/和scripts/均保留在本机并由.gitignore排除，不随Git分发；解析断言位于tests/ble/remote_protocol_checks.cpp，由本地诊断测试应用编译，不再进入生产固件构建。
