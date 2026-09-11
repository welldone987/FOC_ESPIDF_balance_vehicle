# BLE速度/方向任务拆分

## 实现验收

- [x] BleTask：Core0，优先级5，静态4096字节栈；初始化蓝牙、阻塞接收、解析并转换目标。
- [x] NimBLE回调 → 长度1 Incoming静态队列 → BleTask → 长度1 MotionCommand静态队列 → ControlTask；不共享RemoteState。
- [x] main通过独立BleStartup队列等待首次广播结果，成功才创建控制任务。
- [x] 只保留.006 WRITE：D,seq,steering,throttle；百分比±100、uint16序号；去掉ARM/停止/急停及状态/DIAG特征。
- [x] 控制每周期检查原始接收时刻，拒绝就绪前命令；满300ms无效，速度和偏航归零并保持零速平衡。
- [x] 控制侧传感器/倾倒/时序故障锁存禁能；统一ErrorInfo / ErrorPoint，RAM schema=4。
- [x] 两份HTML均简化为连接、摇杆与目标百分比；串行20Hz发送，松手/失焦/后台归零，写入超时断连。
- [x] ADR实际填写区、控制说明和诊断说明同步；模板及只读前缀保持原样。

## 软件验证

ESP-IDF v6.0.2全项目构建通过（本机ccache不可用，采用idf.py --no-ccache build）。网页Node模拟测试8/8通过；QEMU故障注入20/20通过，覆盖命令时效边界、零速平衡、ADC/编码器/IMU/电流输出和故障现场保留。浏览器已检查桌面及390×844手机视口。日志位于build/reports/ble/。

以上不验证真实BLE射频、FreeRTOS端到端调度延迟或物理闭环。GATT写成功仅代表复制入队；两级最新值队列允许跳过中间命令。原始接收时间不因解析和转发刷新。

## 待实板验收（未烧录、未驱动电机）

- [ ] 保护架、限流电源下验证松手、失焦、后台、断连、快速重连和序号回绕；旧网页不能通过旧UUID控制。
- [ ] 测量正常及BLE/Wi-Fi共存时命令接收至控制采用的延迟、过期归零延迟和输出行为；归零保持平衡，不等于关闭公共使能。
- [ ] 检查故障锁存后的禁能延迟与输出行为，恢复连接不能恢复故障停机。
- [ ] 测量BleTask、ControlTask、NimBLE及Wi-Fi栈高水位；检查控制WCET、周期抖动、样本年龄及漏周期。
- [ ] 使用对应ELF核对RAM schema=4及Core dump；固件构建不构成相序、极性和稳定性证明。

ELF检查：BleTask栈4096字节；原始报文/目标/启动结果缓冲分别48/24/40字节（不含队列控制块）；g_diag_crash为1344字节并完整位于Core dump DRAM区域。固件镜像0x13c7b0字节，应用分区余量59%；控制相关7个源文件保持-fno-fast-math。
