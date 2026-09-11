import json,re
from pathlib import Path
s=Path('components/BSP/Error/error_info.hpp').read_text();body=s.split('enum class ErrorPoint')[1].split('{',1)[1].split('}',1)[0]
d={};value=-1
for item in body.split(','):
 item=item.strip()
 if not item:continue
 parts=item.split('=');name=parts[0].strip();value=int(parts[1],0) if len(parts)==2 else value+1
 assert str(value) not in d
 d[str(value)]=name
Path('scripts/diagnostic_dictionary.json').write_text(json.dumps(d,indent=2)+'\n')
