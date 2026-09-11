"""Decode BLE DIAG packets. Supply hexadecimal bytes; no Bluetooth side effects."""
import argparse
import json
import struct
from pathlib import Path

def decode_event(packet: bytes) -> dict:
    if len(packet) != 14 or packet[0] != 1:
        raise ValueError('DIAG event requires schema 1 and exactly 14 bytes')
    version, point, sequence, domain, flags, raw, channel = struct.unpack('<BHIBBiB', packet)
    if domain > 3 or flags & ~3:
        raise ValueError('unsupported domain or flags')
    return dict(schema=version, diag16=point, event_seq=sequence, domain=domain,
                flags=flags, raw_code=raw, channel=None if channel == 255 else channel)

def decode_metadata(packet: bytes) -> dict:
    if len(packet) != 20 or packet[0] != 1:
        raise ValueError('DIAG metadata requires schema 1 and exactly 20 bytes')
    version, flags, step, last, first, point, count, capacity, mtu, connected, subscribed = struct.unpack('<BBHIIHBBHBB', packet)
    if flags & ~3 or capacity != 16 or count > capacity or connected > 1 or subscribed > 1:
        raise ValueError('invalid DIAG metadata')
    return dict(schema=version, flags=flags, boot_step=step, last_event_seq=last,
                first_fault_seq=first, first_fault_diag16=point, count=count,
                capacity=capacity, mtu=mtu, connected=bool(connected), subscribed=bool(subscribed))

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('hex_packet')
    parser.add_argument('--metadata',action='store_true')
    args=parser.parse_args()
    value=(decode_metadata if args.metadata else decode_event)(bytes.fromhex(args.hex_packet))
    dictionary=json.loads((Path(__file__).with_name('diagnostic_dictionary.json')).read_text())
    key=value.get('diag16',value.get('first_fault_diag16'))
    value['point_name']=dictionary.get(str(key),'unknown')
    print(json.dumps(value,indent=2))
