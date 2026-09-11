> 当前BLE仅输出速度/方向；旧ARM、状态及DIAG相关内容不再适用。当前实现与验收见[BLE任务拆分](./tasks/ble_motion_task.md)。

# 初始化与诊断实施验收

依据用户提供的 FOC_Diagnostics_BLE_Refactor_Condensed.md。

- [x] ErrorInfo、检测点字典、16条事件、首故障与控制现场。
- [x] 电流、电机、IMU、电源显式错误传播及失败时间戳保护。
- [x] 固定启动步骤、必要失败停止、BLE首次广播门、控制放行门。
- [x] legacy拒绝、v2兼容、DIAG元信息与事件重放。
- [x] 删除诊断任务、Wi-Fi总开关、保留旧Flash Core dump。
- [x] 故障注入与协议测试、Wi-Fi开/关构建、资源比较。
- [x] 更新当前架构和维护文档；实板、手机项目明确标注未验证。

普通错误只进入RAM记录及BLE导出；Flash仅用于panic Core dump。
不改控制数学、参数、频率、保护阈值和引脚；不烧录、不使能硬件确认门。

验证结果见[diagnostics_validation.md](./diagnostics_validation.md)。实板/手机验收仅列出，未执行。
