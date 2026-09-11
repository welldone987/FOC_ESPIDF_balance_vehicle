import json
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from diag_protocol import decode_event,decode_metadata
class ProtocolTests(unittest.TestCase):
    def test_signed_raw_and_channel(self):
        e=decode_event(bytes.fromhex('01 04 02 78 56 34 12 01 03 85 ff ff ff 03'))
        self.assertEqual((e['diag16'],e['event_seq'],e['raw_code'],e['channel']),(0x204,0x12345678,-123,3))
    def test_unspecified_channel(self):
        self.assertIsNone(decode_event(bytes.fromhex('01 04 02 01 00 00 00 01 00 00 00 00 00 ff'))['channel'])
    def test_lengths_versions_and_flags(self):
        for data in [bytes(13),bytes(15),bytes(14),bytes.fromhex('01 04 02 01 00 00 00 04 00 00 00 00 00 ff')]:
            with self.assertRaises(ValueError): decode_event(data)
    def test_metadata(self):
        m=decode_metadata(bytes.fromhex('01 03 0d 00 20 00 00 00 02 00 00 00 04 02 10 10 f7 00 01 01'))
        self.assertEqual(m['first_fault_seq'],2);self.assertEqual(m['mtu'],247);self.assertEqual(m['count'],16)
    def test_bad_metadata(self):
        for data in [bytes(19),bytes(21),bytes(20),bytes.fromhex('01 03 0d 00 20 00 00 00 02 00 00 00 04 02 11 10 f7 00 01 01')]:
            with self.assertRaises(ValueError): decode_metadata(data)
    def test_dictionary(self):
        d=json.loads((Path(__file__).resolve().parents[2]/'scripts/diagnostic_dictionary.json').read_text())
        self.assertEqual(d['516'],'current_raw');self.assertEqual(d['768'],'motor_gate')
        self.assertEqual(len(d),len(set(d.values())))
if __name__=='__main__':unittest.main()
