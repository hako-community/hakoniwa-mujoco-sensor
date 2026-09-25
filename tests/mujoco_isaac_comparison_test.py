import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from capture_mujoco_static_scene import DEFAULT_CONTRACT, capture
from compare_mujoco_isaac_traces import compare


class S5ComparisonTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.mj = self.root / "mujoco"
        capture(DEFAULT_CONTRACT, self.mj, "s5-test")
        self.rows = [json.loads(s) for s in (self.mj / "common_trace.jsonl").read_text().splitlines()]
        self.fixture = self.root / "isaac_fixture.jsonl"

    def write_fixture(self, rows):
        for row in rows:
            row["identity"]["backend"] = "isaac-fixture"
            row["context"]["capture_provenance"] = "synthetic_fixture"
        self.fixture.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")

    def test_fixture_pass_is_explicitly_not_actual_isaac(self):
        self.write_fixture(copy.deepcopy(self.rows))
        report = compare(DEFAULT_CONTRACT, self.mj / "common_trace.jsonl", self.fixture,
                         fixture=True)
        self.assertTrue(report["pass"])
        self.assertEqual(report["mode"], "synthetic_fixture")
        self.assertEqual(len(report["samples"]), 3)
        output = self.root / "fixture_report.json"
        subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "tools/compare_mujoco_isaac_traces.py"),
                        str(DEFAULT_CONTRACT), str(self.mj / "common_trace.jsonl"),
                        str(self.fixture), "--fixture", "--output", str(output)],
                       check=True, capture_output=True, text=True)
        self.assertTrue(json.loads(output.read_text(encoding="utf-8"))["pass"])
        with self.assertRaises(ValueError):
            compare(DEFAULT_CONTRACT, self.mj / "common_trace.jsonl", self.fixture)

    def test_depth_difference_fails_metric(self):
        rows = copy.deepcopy(self.rows)
        blob = rows[0]["data"]["payload"]["distance_to_image_plane"]
        source = self.mj / "s5_camera-0-distance_to_image_plane.bin"
        depth = np.fromfile(source, dtype="<f4").copy()
        depth[np.isfinite(depth)] += 0.5
        changed = self.root / "changed_depth.bin"
        changed.write_bytes(depth.tobytes())
        blob["uri"] = changed.as_uri()
        blob["sha256"] = hashlib.sha256(changed.read_bytes()).hexdigest()
        self.write_fixture(rows)
        report = compare(DEFAULT_CONTRACT, self.mj / "common_trace.jsonl", self.fixture,
                         fixture=True)
        self.assertFalse(report["pass"])
        self.assertFalse(report["samples"][0]["checks"]["depth_median"])

    def test_contract_mismatch_rejected(self):
        rows = copy.deepcopy(self.rows)
        rows[0]["context"]["contract_sha256"] = "0" * 64
        self.write_fixture(rows)
        with self.assertRaisesRegex(ValueError, "contract hash"):
            compare(DEFAULT_CONTRACT, self.mj / "common_trace.jsonl", self.fixture,
                    fixture=True)


if __name__ == "__main__":
    unittest.main()
