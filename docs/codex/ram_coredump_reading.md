# RAM与Core dump读取

依据：当前源码、sdkconfig、ESP-IDF v6.0.2本机命令帮助及官方文档。本文没有执行串口连接、Flash读取、擦除或故障注入。

## 三种数据不要混淆

| 数据 | 当前保存位置 | 当前读取方式 | 保存期限 |
| --- | --- | --- | --- |
| 普通错误、首故障、控制快照 | RAM中的g_diag_crash | BLE .005读取部分摘要/事件；.003读取最近轻量状态 | 复位或掉电丢失 |
| 网页收到的诊断 | 浏览器页面内存 | 新版页面“导出诊断JSON” | 导出文件长期保留；页面刷新会清空页面记录 |
| panic现场 | Flash的128 KiB coredump分区 | idf.py coredump-info / coredump-debug | 掉电保留，配置禁止覆盖旧dump |

普通stopControl保护停机不会调用panic，因此不会自动生成Core dump。COREDUMP_DRAM_ATTR只是声明变量应被收录到panic dump，不会自动把每次RAM变更写入Flash。当前没有普通故障NVS持久化，没有在线串口RAM读取命令，也没有BLE整块内存下载命令。

## 先读RAM，后决定是否需要dump

1. 故障后保持设备供电并避免复位，用新版页面连接，不必开始遥控。
2. 查看启动摘要与首故障；.005订阅会先重放首故障，再重放仍保留的最多16条事件。页面会去重。
3. 查看采样序号是否持续增加。命令序号不动可能只是未驾驶，不能据此判断任务停机。
4. 导出JSON。BLE当前没有传输完整ErrorInfo的code/value/threshold/valid_fields/file/function/line或完整控制快照；application域raw_code=0不表示没有故障。

当前串口固件会在保护停机后自动打印CONTROL_FAULT、CONTROL_ERROR_FIELDS、CONTROL_STAGE和CONTROL_OUTPUT，包含完整首因字段与当轮阶段耗时，详见[串口定位](./serial_output_age_debug.md)。这些输出不需要Core dump；BLE仍只传输原有摘要。要读完整RAM对象，可以在适配好的JTAG/GDB会话中执行下面的p命令。普通USB串口不是任意RAM读取器；当前PANIC_GDBSTUB未启用。经典ESP32默认JTAG引脚与本板电机引脚存在复用，不能未经板图检查直接连接或启用JTAG。不要为获取普通故障记录主动触发panic。

## 读取Flash并保存为离线文件

必须保留产生故障的那次应用ELF、对应源码和构建配置。应用ELF提供符号和类型，dump ELF保存现场，两者作用不同。使用启动日志中的应用ELF SHA核对；重新编译当前代码并不能替代原ELF。

以下在项目根目录运行。`$matchedBuild`和`$serialPort`为示例，须替换为实际匹配目录和串口。不要一边运行Monitor一边占用同一串口。Flash读取工具通常通过下载模式连接，可能复位设备并在结束后重启；应先导出仍需要的RAM证据，并在电机不会意外动作的维护条件下执行。

```powershell
Set-Location 'E:\project-for-code\2026_09_05_FOC_espidf_balance'
& 'C:\Espressif\tools\Microsoft.v6.0.2.PowerShell_profile.ps1'
idf.py --version

$matchedBuild = 'build_diag_v3' # 仅当这是产生故障固件的匹配构建目录
$serialPort = 'COM7'           # 替换为实际串口
$dumpDir = Join-Path 'build/reports/coredump' (Get-Date -Format 'yyyyMMdd-HHmmss')
New-Item -ItemType Directory -Force -Path $dumpDir | Out-Null
$dumpFile = Join-Path $dumpDir 'panic.core.elf'

idf.py -B $matchedBuild -p $serialPort coredump-info --save-core $dumpFile
```

该命令打印崩溃原因、任务/寄存器/调用栈并保留转换后的dump ELF。所有产物放在build/reports/coredump内，不堆在根目录。无有效dump时提取失败并不证明普通控制没有出错。

保存后可断开串口进行离线分析：

```powershell
idf.py -B $matchedBuild coredump-info --core $dumpFile
idf.py -B $matchedBuild coredump-debug --core $dumpFile
```

GDB内查看：

```gdb
set print pretty on
info threads
thread apply all bt
p g_diag_crash
p g_diag_crash.boot_complete
p g_diag_crash.boot_step
p g_diag_crash.first_fault
p g_diag_crash.first_fault.error
p g_diag_crash.fault_control
p g_diag_crash.last_control
p g_diag_crash.fault_timing
p g_diag_crash.last_timing
p g_diag_crash.first_loop
p g_diag_crash.first_balance
p g_diag_crash.events
p g_diag_crash.events
quit
```

优先读first_fault而不是事件环最后一项，后者可能只是次级错误。fault_control是首次致命错误时复制的最后有效控制快照；如果第一轮尚未完成，其valid可能为false、sequence为0。value/threshold/channel分别由valid_fields的bit0/1/2决定是否有意义。浮点现场没有有效位时不能把默认0当实测。

RAM schema=4的fault_timing记录失败当轮阶段和耗时，first_loop/first_balance分别保留首次完整采样周期/平衡周期；其cycle=0表示尚未记录。balancing与driving分别标记平衡使能和遥控目标有效。计时单位为us，未执行阶段的默认0不表示实测耗时为零。旧固件的匹配ELF可能没有这些字段，不应使用新版ELF解释旧dump。

当前仅显式收录g_diag_crash及SDK选择的任务现场，CONFIG_ESP_COREDUMP_CAPTURE_DRAM未开启，不能假设全部堆/全局内存都在dump中。NO_OVERWRITE开启时旧dump可能来自更早的故障；必须核对来源，提取并确认后再单独安排清理。本说明不执行或提供自动擦除流程。

官方依据：[ESP-IDF v6.0.2 ESP32 Core Dump](https://docs.espressif.com/projects/esp-idf/en/v6.0.2/esp32/api-guides/core_dump.html)。本机已核对coredump-info/debug的--core、--save-core参数，以及保存结果为ELF、离线输入格式自动识别的实现。
