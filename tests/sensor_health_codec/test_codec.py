import argparse
from dataclasses import replace
from pathlib import Path
import subprocess
import sys
import unittest

p = argparse.ArgumentParser()
p.add_argument('--probe', required=True, type=Path)
p.add_argument('--sdk', required=True, type=Path)
args = p.parse_args()
sys.dont_write_bytecode = True
sys.path[:0] = [str(Path(__file__).resolve().parents[2] / 'python'), str(args.sdk / 'python')]
from hakoniwa_sensor_health import SensorHealth, encode, decode, py_to_pdu_SensorHealth, pdu_to_py_SensorHealth


class CodecTests(unittest.TestCase):
    def setUp(self):
        self.sample = SensorHealth('joint_state', 9007199254740993, 0, 1000000,
                                   2000000, 5, 2, 3, 'sim.truth', '校正')

    def probe(self, mode, data=None, ok=True):
        cmd = [str(args.probe), mode] + ([] if data is None else [data.hex()])
        r = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(r.returncode == 0, ok, r.stderr)
        return bytes.fromhex(r.stdout.strip()) if ok else None

    def test_cross_language_payload_and_outer_pdu(self):
        self.assertEqual(decode(self.probe('encode')), self.sample)
        data = encode(self.sample)
        self.assertEqual(self.probe('decode', data), data)
        self.assertEqual(pdu_to_py_SensorHealth(self.probe('wrap', data)), self.sample)
        self.assertEqual(self.probe('unwrap', py_to_pdu_SensorHealth(self.sample)), data)

    def test_fixed_wire_header_and_zero_time(self):
        h = SensorHealth('x')
        data = encode(h)
        # Independent fixed header: magic/version/reserved/length = 67.
        self.assertEqual(data[:12].hex(), '485348310100000043000000')
        self.assertEqual(len(data), 67)
        self.assertEqual(data[60:].hex(), '01000000000078')
        self.assertEqual(decode(self.probe('decode', data)), h)

    def test_reject_bad_headers_lengths_and_utf8(self):
        good = encode(self.sample)
        bad = [b'', good[:-1], good + b'0', b'BAD!' + good[4:]]
        for offset, value in [(4, 2), (6, 1), (8, 1), (60, 255), (66, 255), (44, 128)]:
            data = bytearray(good)
            data[offset] = value
            bad.append(bytes(data))
        for data in bad:
            with self.subTest(data=data.hex()):
                with self.assertRaises(ValueError): decode(data)
                self.probe('decode', data, ok=False)

    def test_field_bounds_and_all_status_bits(self):
        h = replace(self.sample, sequence=2**64-1, dropped_count=2**64-1,
                    queue_depth=2**32-1, status=127, sensor_id='s'*127,
                    profile_id='p'*127, calibration_id='c'*127)
        self.assertEqual(len(encode(h)), 447)
        self.assertEqual(decode(self.probe('decode', encode(h))), h)
        self.assertLessEqual(len(py_to_pdu_SensorHealth(h)), 1024)
        for changes in [dict(sensor_id=''), dict(profile_id='p'*128),
                        dict(profile_id='a\0b'), dict(status=128), dict(sequence=-1),
                        dict(queue_depth=2**32), dict(source_time_ns=3_000_000)]:
            with self.assertRaises(ValueError): encode(replace(h, **changes))

    def test_standard_sdk_does_not_have_custom_message(self):
        import importlib.util
        self.assertIsNone(importlib.util.find_spec('hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_SensorHealth'))


unittest.main(argv=[sys.argv[0]], verbosity=2)
