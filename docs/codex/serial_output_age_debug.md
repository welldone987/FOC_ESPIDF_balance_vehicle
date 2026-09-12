> 当前BLE仅输出速度/方向；旧ARM、状态及DIAG相关内容不再适用。当前实现与验收见[BLE任务拆分](./tasks/ble_motion_task.md)。

# 独立平衡启动与串口排查

本文描述feature分支当前源码，验证产物基于80b777e之后的修复构建。初始化完成后，ControlTask以零速度/零偏航目标开始平衡，不等待BLE连接或ARM。BLE ARM只授权速度和转向目标。

当前BMI160芯片ODR为800Hz，软件IMU读取/姿态环目标200Hz，速度/转向环目标100Hz，电流环请求500Hz。到期IMU和倾倒检查位于编码器之前。编码器到PWM年龄为用户授权的4ms低速候选，电流到PWM年龄保护为2ms；编码器和ADC批次读取耗时仍各限2ms。50度倾倒硬停机门、500Hz新时序与10Hz Wi-Fi服务尚未实板验证。

当前M0/M1控制方向映射均为-1（vehicle_config.hpp），目标电流与轮速/Iq/Uq反馈同步反向；FOC对齐方向和相电流极性未改。启动换向自检属于对齐过程，不由这两个车辆前进符号决定。

## 启动与停止行为

| 场景 | 平衡输出 |
| --- | --- |
| 初始化成功，尚未连接BLE | 尝试零速平衡；倾倒、数据有效性、过流和时效检查必须通过 |
| 首次BLE连接，但未ARM | 原有零速平衡继续，非零遥控目标不生效 |
| BLE ARM成功 | 授权运动目标；已在平衡时不重置平衡状态 |
| 显式S、遥控超时、实际BLE断连 | 关闭输出，清零控制状态；只经新的ARM恢复 |
| 停止后发送D或快速重连 | 保持关闭；停止事件序号跨重连保留 |
| 急停、倾倒、传感器/电流/时序故障 | 禁能锁存，须复位处理 |

独立平衡使能由ControlTask持有，不把BLE状态伪装成active。原20字节无线状态仍表示遥控状态，没有新增平衡使能位；网页的“待授权”不代表电机关闭。

首帧姿态以加速度计角度建基准，后续按实际dt及98ms时间常数计算互补权重alpha=0.098/(0.098+dt_s)；2ms时0.98，5ms时约0.95146。这样初始倾倒不会被从0度起算的滤波器暂时掩盖；首个有效姿态通过倾倒检查后才执行电流输出。

## 复测方法

使用build中的本次固件，保留匹配的ELF并核对启动SHA。本文记录的提交前构建版本字符串为80b777e-dirty，提交后重新构建会更新版本和ELF；不能仅凭版本字符串区分诊断构建；也可以用下述CONTROL_START确认模式。上电后会主动尝试平衡，按现有保护架/限流条件操作。

本次构建标识与结果在文末更新；核对匹配ELF，不使用旧9aaed54d构建代表本次时序。也可通过CONTROL_CONFIG与CONTROL_LIMITS确认参数；重新构建后以新ELF为准。

开电机电源、保持串口连接，首次测试不必连接BLE。预期路径：

```text
CONTROL_START mode=INDEPENDENT_BALANCE; BLE ARM authorizes motion targets
CONTROL_CONFIG current_period_us=2000 attitude_period_us=5000 outer_period_us=10000
CONTROL_LIMITS encoder_output_max_age_us=4000 current_output_max_age_us=2000
BOOT_SUMMARY OK
CONTROL_LOOP_ALIVE first frame completed; first dt is nominal, not measured
CONTROL_TIMING cycle=1 balance_cycle=1 balancing=1 driving=0 starting=1 ...
CONTROL_STAGE stage=COMPLETE current_stage=COMPLETE ...
CONTROL_OUTPUT adc_us=... math_us=... pwm_us=... encoder_age_us=... current_age_us=...
CONTROL_BALANCE_ACTIVE first balance frame completed; remote_active=0; this is not a stability validation
```

这是字段示例，不是实测结果。首次完整平衡周期成功不证明稳定站立或1 kHz预算通过。如果启动前已经接收停止事件，可能出现CONTROL_STOPPED，须新ARM恢复。

若失败，保存从CONTROL_START/BOOT_SUMMARY到CONTROL_SUMMARY的完整内容：

```text
CONTROL_FAULT point=786 code=ESP_ERR_TIMEOUT ... file=... function=runCurrentControl
CONTROL_ERROR_FIELDS valid=0x3 value=... threshold=4000 channel=-1 comparison=1
CONTROL_TIMING cycle=... balance_cycle=... balancing=1 driving=0 starting=... imu_updated=... notify=... skipped=...
CONTROL_STAGE stage=CURRENT current_stage=BEFORE_PWM|AFTER_PWM dt_us=... elapsed_us=... off_us=... encoder_us=... imu_us=... outer_us=...
CONTROL_OUTPUT adc_us=... math_us=... pwm_us=... encoder_age_us=... current_age_us=...
CONTROL_LAST_VALID valid=... seq=... pitch_deg=... left_rad_s=... right_rad_s=... left_target_a=... right_target_a=...
CONTROL_SUMMARY FAIL
```

首次平衡就失败时，可能没有CONTROL_LOOP_ALIVE/CONTROL_BALANCE_ACTIVE。fault_control/CONTROL_LAST_VALID是此前有效周期，fault_timing是失败当轮；前者valid=0是合理的，不代表故障计时没有记录。

## 计时解读

| 观测 | 下一步 |
| --- | --- |
| BEFORE_PWM，encoder_age明显大于current_age | 看编码器、外环耗时和期间抢占；本轮IMU已先于编码器 |
| BEFORE_PWM，adc_us或math_us较大 | 分别定位ADC读取/换算与电流计算 |
| AFTER_PWM，pwm_us明显增加 | 检查PWM写入和该区间调度延迟 |
| point=788 current_output_age | 电流到PWM年龄超过2000us；与786编码器年龄4000us分别处理 |
| balancing=1，driving=0 | 正在尝试独立平衡，未授权遥控运动 |
| starting=1且balance_cycle=1 | 首次尝试平衡，尚无成功输出周期 |
| notify>1或skipped增加 | 通知被合并；首轮可能来自BOOT打印积压，不能据此推断稳态频率 |
| 其它ErrorPoint | 优先处理实际首因，不预设为output_age |

计时单位均为us。未执行阶段保留0；首轮控制dt=1000、姿态dt=5000是名义值，不是实测周期。当前顺序为到期IMU/倾倒检查→编码器→到期外环→ADC→PI→PWM，IMU不再计入随后编码器年龄，但仍计入整帧耗时。两种输出年龄在PWM前后独立检查；786的阈值4000，788的阈值2000，原始采样时间起点不变。4ms是低速候选，不是实际轮速限制，也不是稳定性证明。

用户此前的故障实测为首帧encoder_age=3422us、IMU=892us、encoder=1250us、ADC=562us、math=476us。仅移动IMU后粗估编码器到PWM前约2530us，可能通过4ms候选但仍需新日志验证；其余三个阶段合计2288us，未证明满足1ms电流周期。降到200Hz减少平均IMU负载，首帧仍必须读取有效姿态，不绕过倾倒保护。芯片800Hz最新帧轮询没有FIFO抗混叠，振动/滤波延迟待实测。

## RAM和验证

RAM schema=3，last_timing/fault_timing/first_loop/first_balance随g_diag_crash进入panic Core dump，BLE DIAG schema仍为1。app_main每100 ms观察快照，首次平衡成功或故障后返回；若此前已经停止则等待新ARM，等待期间保留原3584字节main栈。正常高频路径只写定长内存，故障打印在请求禁能和停止定时器后进行。本次没有独立永久监护任务或连续CONTROL_READY验收门。

2026-09-11本次软件验证：

- `idf.py --no-ccache build size`及Wi-Fi关闭配置构建通过。
- QEMU：25 Tests 0 Failures 0 Ignored；包含左右轮双向控制、轮速跨零点和Iq/Uq反馈坐标一致性。网页Node测试15项、Python协议测试6项沿用此前通过结果，本次方向改动未涉及无线协议。
- 新增覆盖：4000us编码器边界、2000us电流边界及PWM前后故障，保持2ms读取耗时限制，不使能/不提交失败PI状态；实际dt滤波；200Hz姿态到100Hz外环；BMI160两个ODR寄存器的800Hz写入。
- 生产ControlTask的IMU→倾倒检查→编码器→外环→电流调用顺序已静态核对。QEMU未运行真实控制任务调度；寄存器fake只模拟就绪状态，不证明真实IMU数据率、滤波和总线时序。
- 图像报告1,299,864B；DRAM 77,591B，IRAM 120,603B（92.01%）。资源与构建不是闭环稳定性证明。
- Wi-Fi启用ELF SHA256：`e8203dc48c922b80c743e2593d38486b3a432590dd4831b3bb1609ef145c5fa9`；启动前缀应为`e8203dc4`。
- Wi-Fi关闭ELF SHA256：`d396848be32486c8dc82eff941486beef8c684fa0c36c3b6f1f13d1c04d5f8f7`。
- 本次未烧录。下述报告保留提交前构建证据，重新构建后须保留新ELF并重新核对SHA。方向修改报告目录：`build/reports/motor_reverse/`；时序及网页/协议验证报告目录：`build/reports/imu_200hz/`。

复测时先核对800Hz芯片ODR对应固件、200Hz姿态周期及4ms/2ms输出限值，保存CONTROL_CONFIG、CONTROL_LIMITS和首次COMPLETE/FAULT全组日志；重点观察实际dt、通知积压、encoder_age/current_age、角度/电流与停止行为。4ms不证明实际轮速受限；真实闭环稳定性、振动混叠、停止延迟和栈水位仍待实板验证。
