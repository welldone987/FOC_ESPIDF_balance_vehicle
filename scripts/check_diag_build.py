from pathlib import Path
import json,subprocess
from elftools.elf.elffile import ELFFile
root=Path.cwd()
with (root/'build_diag_v3/2026_09_05_FOC_espidf_balance.elf').open('rb') as f:
 elf=ELFFile(f);syms=elf.get_section_by_name('.symtab');items=syms.get_symbol_by_name('g_diag_crash');assert items
 symbol=items[0];section=elf.get_section(symbol['st_shndx']);print('g_diag_crash',symbol['st_size'],section.name,hex(symbol['st_value']))
 assert 'dram' in section.name
 start=syms.get_symbol_by_name('_coredump_dram_start')[0]['st_value'];end=syms.get_symbol_by_name('_coredump_dram_end')[0]['st_value']
 assert start <= symbol['st_value'] and symbol['st_value']+symbol['st_size'] <= end
 print('Core dump range contains entire crash state: PASS')
 assert not any('diagnosticsTask' in s.name for s in syms.iter_symbols())
commands=json.loads(Path('build_diag_v3/compile_commands.json').read_text())
for name in ['current_sense.cpp','motor_foc_service.cpp','current_control.cpp','svpwm.cpp','bmi160_attitude.cpp','balance_controller.cpp','application_tasks.cpp']:
 cmd=next(c['command'] for c in commands if c['file'].endswith('/'+name) or c['file'].endswith('\\'+name))
 assert '-fno-fast-math' in cmd,name
print('IEEE guard flags: 7 sources PASS')
for name in ['components/BSP/Board/board_pins.hpp','partitions.csv']:
 old=subprocess.check_output(['git','show','HEAD:'+name]);current=Path(name).read_bytes()
 assert old.replace(b'\r\n',b'\n')==current.replace(b'\r\n',b'\n'),name
print('Board pins / partition unchanged: PASS')

with (root/'build_diag_wifi_off/2026_09_05_FOC_espidf_balance.elf').open('rb') as f:
    elf=ELFFile(f);syms=elf.get_section_by_name('.symtab')
    for symbol in syms.iter_symbols():
        assert 'wifiTelemetryTask' not in symbol.name
        assert not ('wifi_telemtry' in symbol.name and 'initialize' in symbol.name)
    print('Wi-Fi disabled: no telemetry task or service initialization symbol PASS')
