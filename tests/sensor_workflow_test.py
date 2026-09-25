import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from sensor_workflow import capability_gate, fit_validate, write_json, digest
from common_trace import convert


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.descriptor = {"backend": "isaac-export", "capabilities": {"rgb": True, "multipath": False}}
        self.scenario = {"scenario_id": "test", "required_capabilities": {"rgb": True}}
        self.manifest = {"run_id": "r1", "scenario_id": "test", "robot_id": "robot",
                         "backend": "isaac-export", "backend_version": "fixture",
                         "git_commit": "test", "model_sha256": "0" * 64,
                         "profile_id": "truth", "calibration_id": "none", "seed_tree": {"root": 1},
                         "descriptor": self.descriptor, "scenario": self.scenario}
        self.row = {"sensor_id": "camera", "frame_id": "optical", "sequence": 0,
                    "source_time_ns": 10, "scheduled_time_ns": 11, "publish_time_ns": 12, "consume_time_ns": 13,
                    "health": {"status_bits": 1, "drop_count": 0, "queue_depth": 0}, "context": {},
                    "annotators": {"rgb": self.blob("rgb", bytes([0, 1, 255]), [1, 1, 3], "uint8")}}

    def blob(self, name, raw, shape, dtype):
        (self.root / name).write_bytes(raw)
        return {"uri": name, "sha256": hashlib.sha256(raw).hexdigest(), "shape": shape, "dtype": dtype, "unit": "m"}

    def test_gate(self):
        self.assertTrue(capability_gate(self.descriptor, self.scenario)["pass"])
        for policy in ("fail", "skip", "degraded"):
            self.scenario.update(required_capabilities={"multipath": True}, on_missing=policy)
            result = capability_gate(self.descriptor, self.scenario)
            self.assertFalse(result["pass"])
            self.assertEqual(result["decision"], policy)
            self.assertEqual(result["status_bits"], 32)

    def test_unknown_capability(self):
        self.scenario["required_capabilities"] = {"unknown": False}
        self.assertFalse(capability_gate(self.descriptor, self.scenario)["pass"])

    def test_named_fidelity(self):
        self.descriptor["capabilities"]["material"] = "scalar_approximation"
        self.scenario["required_capabilities"] = {"material": "physical"}
        self.assertFalse(capability_gate(self.descriptor, self.scenario)["pass"])

    def signals(self, name, run, residual=2):
        p = self.root / name
        p.write_text("run_id,timestamp_ns,sensor_id,axis,unit,truth,measured\n" +
                     "".join(f"{run},{i},imu,x,m/s2,{i},{i+residual}\n" for i in range(3)), encoding="utf-8")
        return p

    def fit_inputs(self):
        return self.signals("fit.csv", "train"), self.signals("hold.csv", "hold"), {"imu": {"x": {"unit": "m/s2", "max_bias": .01, "max_rmse": .01}}}

    def test_fit_holdout(self):
        train, hold, limits = self.fit_inputs()
        profile, report = fit_validate(train, hold, "p", limits)
        self.assertTrue(report["pass"])
        self.assertEqual(profile["parameters"]["residual_models"][0]["bias"], 2)

    def test_leakage(self):
        train, hold, limits = self.fit_inputs()
        hold = self.signals("hold.csv", "train", 3)
        with self.assertRaises(ValueError): fit_validate(train, hold, "p", limits)

    def test_failed_validation(self):
        train, hold, limits = self.fit_inputs()
        hold = self.signals("hold.csv", "hold", 3)
        self.assertFalse(fit_validate(train, hold, "p", limits)[1]["pass"])

    def test_cli_profile_hash_and_failure(self):
        train, hold, limits = self.fit_inputs()
        write_json(self.root / "limits.json", limits)
        cli = Path(__file__).resolve().parents[1] / "tools/sensor_workflow.py"
        args = [sys.executable, str(cli), "fit", str(train), str(hold), "--thresholds", str(self.root / "limits.json"), "--profile-id", "test"]
        for residual, stem, expected in ((2, "pass", 0), (4, "fail", 2)):
            self.signals("hold.csv", "hold", residual)
            profile, report = self.root / (stem + ".profile.json"), self.root / (stem + ".report.json")
            completed = subprocess.run(args + ["--profile", str(profile), "--report", str(report)], capture_output=True)
            self.assertEqual(completed.returncode, expected, completed.stderr)
            self.assertEqual(profile.exists(), expected == 0)
            if profile.exists():
                self.assertEqual(json.loads(profile.read_text())["fit"]["holdout_report_sha256"], digest(report))

    def test_nonfinite_rejected(self):
        train, hold, limits = self.fit_inputs()
        self.signals("hold.csv", "hold", float("nan"))
        with self.assertRaises(ValueError): fit_validate(train, hold, "p", limits)

    def test_rgbd_and_lidar(self):
        self.row["annotators"]["distance_to_image_plane"] = self.blob("depth", struct.pack("<f", float("inf")), [1, 1], "float32-le")
        lidar = copy.deepcopy(self.row)
        lidar["sensor_id"] = "lidar"
        lidar["annotators"] = {"pointcloud": self.blob("points", struct.pack("<fff", 1, 2, 3), [1, 3], "float32-le")}
        result = convert(self.manifest, [self.row, lidar], self.root)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["health"]["age_ns"], 3)
        self.assertEqual(result[0]["data"]["payload"]["rgb"]["sha256"], digest(self.root / "rgb"))
        lidar["annotators"] = {"pointcloud": self.blob("empty", b"", [0, 3], "float32-le")}
        self.assertEqual(len(convert(self.manifest, [lidar], self.root)), 1)

    def test_corrupt_blob(self):
        (self.root / "rgb").write_bytes(b"bad")
        with self.assertRaises(ValueError): convert(self.manifest, [self.row], self.root)

    def test_causality_and_duplicate(self):
        with self.assertRaises(ValueError): convert(self.manifest, [self.row, self.row], self.root)
        self.row["publish_time_ns"] = 1
        with self.assertRaises(ValueError): convert(self.manifest, [self.row], self.root)

    def test_trace_gate(self):
        self.scenario["required_capabilities"] = {"multipath": True}
        with self.assertRaises(ValueError): convert(self.manifest, [self.row], self.root)

    def test_capture_exporter(self):
        import numpy as np
        from isaac_trace_export import IsaacTraceExporter
        exporter = IsaacTraceExporter(self.root / "export", self.manifest)
        exporter.capture(self.row, {"rgba": np.array([[[0, 1, 2, 255]]], dtype=np.uint8),
                                    "distance_to_image_plane": np.array([[2.0]], dtype="<f4")})
        lidar = dict(self.row, sensor_id="lidar", frame_id="sensor_local")
        exporter.capture(lidar, {"pointcloud": np.array([[1., 2., 3.]], dtype="<f4")})
        exporter.close()
        rows = (exporter.root / "common_trace.jsonl").read_text().splitlines()
        self.assertEqual(len(rows), 2)
        self.assertEqual(json.loads(rows[1])["sensor_id"], "lidar")


if __name__ == "__main__":
    unittest.main(verbosity=2)
