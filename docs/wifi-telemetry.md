# Wi-Fi TCP遥测

## 连接和配置

ESP32使用STA模式连接路由器/热点，在IPv4端口3333提供单客户端TCP服务。默认50Hz，在 `include/vehicle_config.h` 修改 `telemetry_period_ms`；该周期只控制网络任务，不改变控制循环。电脑与ESP32需要网络可达，客户端按LF（`\n`）累计解析，不以一次recv作为一帧。

本地 `include/wifi_credentials.h` 复用参考工程的配置，并被Git忽略；不要提交凭据。新检出工程可复制 `wifi_credentials.example.h` 并填写SSID和密码。未提供配置仍能构建，但网络任务会记录错误并挂起，车辆控制继续运行。当前STA认证门槛为WPA2，不支持开放热点。

Wi-Fi固定使用原生esp_wifi_set_ps(WIFI_PS_MIN_MODEM)，不提供关闭modem sleep的选项。当前SDK要求Wi-Fi/BLE共存时开启该模式，否则可能在Wi-Fi内部任务中abort并重启；检查调用返回值不能替代正确配置。

网络使用ESP-IDF 4.4.7原生 `esp_wifi` / `esp_event` / `esp_netif` 和lwIP socket，不使用Arduino WiFi/WiFiClient。网络时钟为 `esp_timer_get_time()`；日志为ESP_LOG；两个网络模块通过本地LOG_LOCAL_LEVEL和独立日志标签启用INFO，输出设备IP与连接状态，不修改SDK配置。

## 固定协议

连接后先发送两行元数据，再发送七列CSV：

```text
#balancing_vehicle_tcp,v1
#time_s,pitch_deg,left_velocity_rad_s,right_velocity_rad_s,velocity_difference_rad_s,left_target_v,right_target_v
12.345678,1.8000,2.0000,1.5000,0.5000,0.7000,0.6000
```

| 列 | 单位和定义 |
| --- | --- |
| time_s | 控制周期开始时的64位设备单调时间，秒，6位小数；重启归零 |
| pitch_deg | 本周期实际用于控制的互补滤波倾角，deg |
| left_velocity_rad_s | 与控制器相同方向系数修正后的左轮轴速度，rad/s |
| right_velocity_rad_s | 与控制器相同方向系数修正后的右轮轴速度，rad/s |
| velocity_difference_rad_s | 上述左轮速减右轮速，rad/s；不是车体偏航角速度 |
| left_target_v | 混合、方向系数处理后的左电机目标，V |
| right_target_v | 混合、方向系数处理后的右电机目标，V |

目标电压在下一控制周期由move消费，不是实测输出电压、电流或转矩。倾角与轮速属于同一控制周期的顺序采样，不保证硬件同步。只发送以上7列；内部序号、蓝牙命令、编码器角度不发送。

队列只保留最新状态，不保证完整记录每个控制周期；同连接重复快照不再发送。当前帧部分发送后必须先续传，未进入发送缓存的中间快照可被覆盖。客户端持续EAGAIN约1秒后断开，成功写入socket不等于电脑已收到。重新连接时重发协议头。不建议将这类最新值记录直接用于要求等间隔、无损采样的频谱分析。

## 调度和后续迁移

`beginTelemetry()` 在车辆初始化后、控制循环开始前调用一次，创建长度1静态Queue及Core0/P1/4096 B网络任务，不等待联网。网络任务20ms绝对节拍，超期不追赶；Wi-Fi系统任务和协议栈仍由ESP-IDF管理。

`runVehicleControlOnce()` 末尾调用 `publishTelemetry()`，只构造固定值帧和非阻塞覆盖队列。网络任务不读取传感器、不调用控制器、不访问 `latestTelemetry()` 的共享引用。未来只将单周期入口移至唯一ControlTask，并停止Arduino loop重复执行；无需搬动Wi-Fi生命周期。不要将beginTelemetry重复放入任务循环。控制Task的周期/优先级/栈需要独立设计和实测。

## 分区和构建

Wi-Fi+BLE超出默认1.25MiB应用槽，因此采用框架 `min_spiffs.csv`：双OTA槽各1.875MiB、SPIFFS 128KiB、coredump 64KiB。NVS和otadata位置不变。当前工程未使用文件系统；若设备上另有历史文件系统数据，不能假定更换分区后仍可读取。首次部署应通过串口写入包含新分区表的完整构建；本次未执行烧录。分区保留OTA槽不代表固件已经实现OTA服务。

```powershell
& C:/Users/tgs27/.platformio/penv/Scripts/platformio.exe run -d balancing_vehicle_work -e lolin32_lite
```

2026-09-05目标工具链构建通过：Flash 1,509,933 / 1,966,080 B（76.8%），静态RAM 63,380 / 327,680 B（19.3%）。构建使用Arduino-ESP32 2.0.17内含的ESP-IDF 4.4.7；未修改SDK配置。

## 验证

本次只使用PlatformIO ESP32工具链验证固件编译链接，并静态核对字段映射、非阻塞socket和队列边界。没有执行协议运行测试；部分发送、拥塞超时和重连路径仍需目标板验证。

修复后实机状态为Unverified，先确认双轮自检只执行一次、进入控制循环，且不再出现modem sleep错误或abort。随后需验证：

1. 获取IP后电脑连接3333，检查每行七列、字段符号和设备时间轴。
2. BLE操控与Wi-Fi同时运行，对比控制周期P99/max及BLE命令更新间隔。
3. 停止客户端读取、断网、重连，观察控制是否持续运行且无帧交错。
4. 测量网络栈high-water（按当前ESP-IDF字节语义解释）和最小空闲堆，不能用静态RAM占用推断运行期余量。

现有BLE断连保持最后命令、无命令超时归零的行为未改变；测试时应与主动发送零命令区分。
