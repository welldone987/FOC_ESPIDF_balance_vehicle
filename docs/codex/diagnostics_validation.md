# 初始化、故障诊断与持久化验收

日期：2026-09-11。本文保留初始化与持久化重构的验收数据，对应现已提交的feature分支基线80b777e；未push/flash/erase或运行实板。下面的10组QEMU测试、904字节诊断对象和资源表属于该基线，当前串口诊断修复的验证结果见[独立平衡串口验收](./serial_output_age_debug.md)。

## 改了什么

固定步骤启动，必要失败停止，可选Wi-Fi降级；等待BLE首次广播成功；BOOT_SUMMARY之后才放行控制。删除DiagnosticsTask及其4,096字节静态栈。ErrorInfo携带检测点、esp_err_t、原始域/码、位置、通道、现场及有效位。ADC、电机、IMU、电源错误细分；失败采样不提交反馈和时间状态。GPIO禁能先于故障记录和状态清理；首因独立保存，次级错误不覆盖首因。

固定16条RAM事件、首故障、最后有效/故障快照、最后BLE异常及连接状态进入g_diag_crash。保留原Flash分区，启用旧dump不覆盖，不增加普通错误NVS日志。启动期间电机细分阶段也更新boot_step。

v2命令解析、remote_control授权、状态codec与NimBLE生命周期/GATT处理职责分开。新增DIAG UUID .005，20字节READ和14字节NOTIFY；.003状态包的缩放、单位和sequence语义保持。.002保留UUID，READ说明不支持、WRITE拒绝。订阅后先重放首故障，再重放历史；发送失败不推进游标。

## 实际验证

| 验证 | 执行结果 |
| --- | --- |
| idf.py --version | ESP-IDF v6.0.2 |
| Python主机协议测试 | 6项通过，tests/diagnostics/test_protocol.py |
| C++编译期协议断言 | 生产解析器及remote_control断言通过 |
| QEMU故障注入 | 10组、0失败、0忽略，DIAGNOSTICS_TESTS_PASS |
| Wi-Fi启用 | idf.py --no-ccache -B build_diag_v3 build size，通过 |
| Wi-Fi关闭 | idf.py --no-ccache -B build_diag_wifi_off -D SDKCONFIG=sdkconfig.diag_wifi_off build size，通过 |
| 原始基线 | git archive HEAD的隔离源码，恢复原NO_OVERWRITE=n，ESP-IDF build size通过 |
| 浮点保护 | 编译命令核对Motor四个源文件、IMU、Control、application_tasks均含-fno-fast-math |
| Core dump符号 | g_diag_crash=904字节，完整位于_coredump_dram_start/end范围 |
| Wi-Fi资源裁剪 | 关闭ELF中无wifiTelemetryTask及wifi_telemtry初始化符号 |
| 数学/参数/引脚/分区 | 对比HEAD：PI、SVPWM、balance_controller、vehicle_config、board_pins、partitions.csv未变 |
| 差异格式 | git diff --check通过 |

QEMU编译真实生产源码，只有ADC/I²C/GPIO/驱动库依赖使用测试stub。覆盖四路ADC初始化/读取/转换、零偏平均/噪声、四个实测及两个重构相超限、采样超时、左右驱动/对齐失败、GPIO禁能错误、失败时间戳保护、编码器原始错误锁存、事件环覆盖、首因保护、重放失败/重连、启动/故障门及20字节状态兼容。QEMU是目标模拟测试，不是原生主机C++或硬件验证。

## 资源结果

单位：字节。镜像大小取idf.py size的Total image size，不等同于.bin填充后的文件长度；DRAM/IRAM为链接静态占用，不包含运行时堆峰值和任务栈水位。

| 构建 | 静态DRAM | IRAM | 镜像大小 |
| --- | ---: | ---: | ---: |
| 原始HEAD基线，Wi-Fi启用 | 80,227 | 120,603 | 1,288,132 |
| 重构后，Wi-Fi启用 | 76,959 | 120,603 | 1,295,004 |
| 重构后，Wi-Fi关闭 | 49,111 | 91,959 | 741,008 |

相同Wi-Fi启用条件下：DRAM减少3,268字节，IRAM不变，镜像增加6,872字节。关闭Wi-Fi相对重构启用配置再减少27,848字节DRAM、28,644字节IRAM、553,996字节镜像。

资源变化来自删除诊断任务TCB/4 KiB栈，新增904字节crash对象及小量BLE状态；ControlTask保留8 KiB栈，Wi-Fi启用时保留8 KiB栈，关闭时裁剪其栈、TCB、单元素队列、事件组和服务缓冲。NimBLE继续使用原host任务和100 ms callout。

## 实板和手机边界

全部未验证：GPIO12/22实物、电流相序/量程、上电/对齐/禁能、手机GATT缓存、MTU/订阅/重连/补发、Wi-Fi共存WCET、S/E/断连/超时延迟与输出行为、栈水位、Flash dump保存/匹配ELF解码/清除。

当前源码kCurrentHardwareVerified原本为true，本次未修改；它不证明硬件已经验证。库对齐与同步总线操作仍限制软件故障关断时延。普通故障RAM记录掉电丢失；旧dump不覆盖会阻止新dump保存，需人工维护。Notify提交成功不表示客户端已持久保存。

维护协议与命令见[diagnostics.md](./diagnostics.md)，测试复现见[tests/diagnostics/README.md](../../tests/diagnostics/README.md)。本地构建日志统一位于build/reports/diagnostics/，包括build_diag_v3_final.log、build_diag_wifi_off_final.log、build_diag_baseline_final.log、build_diag_qemu_final.log，未复制日志中的本机Wi-Fi配置到文档。
