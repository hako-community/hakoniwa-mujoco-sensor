#!/usr/bin/env python3
"""Compare fixed S5 MuJoCo and Isaac common traces, with explicit fixture mode."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import url2pathname

import numpy as np


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_trace(path, contract, contract_hash, backend, fixture=False):
    rows = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()
            if line.strip()]
    expected = {(sensor["sensor_id"], t)
                for sensor in contract["sensors"].values()
                for t in contract["time"]["sample_times_ns"]}
    found = {}
    for row in rows:
        if row["format"] != "common-sensor-trace/v1":
            raise ValueError("not a common sensor trace")
        identity = row["identity"]
        if identity["scenario_id"] != contract["scenario_id"]:
            raise ValueError("scenario_id mismatch")
        if backend == "mujoco" and identity["backend"] != "mujoco":
            raise ValueError("MuJoCo trace has wrong backend")
        if backend == "isaac" and identity["backend"] not in (
                ("isaac-fixture",) if fixture else ("isaac", "isaac-export")):
            raise ValueError("Isaac trace has wrong backend")
        context = row["context"]
        if context.get("contract_sha256") != contract_hash:
            raise ValueError("contract hash mismatch")
        if backend == "isaac" and not fixture and context.get("capture_provenance") != "isaac_annotator":
            raise ValueError("actual comparison requires Isaac annotator provenance")
        times = row["time"]
        acquired = times["source_time_ns"]
        if not (acquired <= times["scheduled_time_ns"] <= times["publish_time_ns"] <=
                times["consume_time_ns"]):
            raise ValueError("non-causal timestamps")
        key = row["sensor_id"], acquired
        if key not in expected or key in found:
            raise ValueError("unexpected or duplicate sensor/time")
        if row["health"]["status_bits"] & 1 == 0:
            raise ValueError("comparison requires valid acquisition")
        sensor = next(s for s in contract["sensors"].values() if s["sensor_id"] == key[0])
        if row["frame_id"] != sensor["frame_id"]:
            raise ValueError("sensor frame mismatch")
        found[key] = row
    if set(found) != expected:
        raise ValueError("trace lacks expected sensor/time samples")
    identities = {(r["identity"]["run_id"], r["identity"]["backend_version"],
                   r["provenance"]["model_sha256"], r["provenance"]["seed_tree"].get("root"))
                  for r in found.values()}
    if len(identities) != 1:
        raise ValueError("trace provenance varies between samples")
    representative = next(iter(found.values()))
    if representative["provenance"]["seed_tree"].get("root") != contract["seed"]:
        raise ValueError("scenario seed mismatch")
    if backend == "mujoco" and representative["provenance"]["model_sha256"] != contract["mujoco"]["mjcf_sha256"]:
        raise ValueError("MuJoCo model hash differs from contract")
    return found


def blob_array(row, name):
    blob = row["data"]["payload"][name]
    parsed = urlparse(blob["uri"])
    if parsed.scheme != "file":
        raise ValueError("trace blob must use a file URI")
    path = Path(url2pathname(parsed.path))
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != blob["sha256"]:
        raise ValueError("trace blob hash mismatch")
    dtype = np.dtype("uint8" if name == "rgb" else "<f4")
    shape = tuple(blob["shape"])
    if len(raw) != int(np.prod(shape)) * dtype.itemsize:
        raise ValueError("trace blob byte count mismatch")
    return np.frombuffer(raw, dtype=dtype).reshape(shape)


def compare(contract_path, mujoco_path, isaac_path, fixture=False):
    contract_path = Path(contract_path)
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    if contract["format"] != "s5-scenario/v1":
        raise ValueError("unsupported S5 contract")
    contract_hash = digest(contract_path)
    mj = read_trace(mujoco_path, contract, contract_hash, "mujoco")
    isaac = read_trace(isaac_path, contract, contract_hash, "isaac", fixture)
    camera, lidar = contract["sensors"]["rgbd"], contract["sensors"]["lidar"]
    limits = contract["comparison"]
    samples = []
    for t in contract["time"]["sample_times_ns"]:
        ca, cb = mj[(camera["sensor_id"], t)], isaac[(camera["sensor_id"], t)]
        la, lb = mj[(lidar["sensor_id"], t)], isaac[(lidar["sensor_id"], t)]
        color_a, color_b = blob_array(ca, "rgb"), blob_array(cb, "rgb")
        depth_a = blob_array(ca, camera["depth_annotator"])
        depth_b = blob_array(cb, camera["depth_annotator"])
        points_a, points_b = blob_array(la, "pointcloud"), blob_array(lb, "pointcloud")
        expected_rgb = (camera["image_height"], camera["image_width"], 3)
        if (color_a.shape != expected_rgb or color_b.shape != expected_rgb or
                depth_a.shape != expected_rgb[:2] or depth_b.shape != expected_rgb[:2] or
                points_a.ndim != 2 or points_b.ndim != 2 or
                points_a.shape[1] != 3 or points_b.shape[1] != 3):
            raise ValueError("array shape differs from fixed contract")
        if (ca["context"].get("depth_definition") != camera["depth_annotator"] or
                cb["context"].get("depth_definition") != camera["depth_annotator"] or
                ca["context"].get("depth_invalid") != camera["depth_invalid"] or
                cb["context"].get("depth_invalid") != camera["depth_invalid"]):
            raise ValueError("depth semantics mismatch")
        focal = camera["image_height"] / (2 * np.tan(np.deg2rad(camera["vertical_fov_deg"]) / 2))
        expected_intrinsics = {"fx": focal, "fy": focal,
                               "cx": (camera["image_width"] - 1) / 2,
                               "cy": (camera["image_height"] - 1) / 2,
                               "width": camera["image_width"], "height": camera["image_height"]}
        for row in (ca, cb):
            intrinsics = row["context"].get("intrinsics", {})
            if any(key not in intrinsics or not np.isclose(intrinsics[key], value, atol=1e-4)
                   for key, value in expected_intrinsics.items()):
                raise ValueError("camera intrinsics differ from fixed contract")
        for depth in (depth_a, depth_b):
            if not np.all(np.isposinf(depth) | ((depth > 0) & (depth <= camera["depth_max_m"]))):
                raise ValueError("depth contains a value outside the fixed valid/invalid convention")
        valid_a = np.isfinite(depth_a) & (depth_a > 0) & (depth_a <= camera["depth_max_m"])
        valid_b = np.isfinite(depth_b) & (depth_b > 0) & (depth_b <= camera["depth_max_m"])
        overlap = valid_a & valid_b
        depth_diff = (float(np.median(np.abs(depth_a[overlap] - depth_b[overlap])))
                      if overlap.any() else None)
        ray_count = lidar["azimuth_count"] * lidar["elevation_count"]
        for row, points in ((la, points_a), (lb, points_b)):
            if (row["context"].get("ray_count") != ray_count or
                    row["context"].get("no_return_count") != ray_count - len(points) or
                    row["context"].get("point_frame") != "sensor_local_xyz_m"):
                raise ValueError("LiDAR ray/no-return/frame contract mismatch")
            ranges = np.linalg.norm(points, axis=1)
            if not (np.all(np.isfinite(points)) and
                    np.all((ranges >= lidar["min_range_m"]) & (ranges <= lidar["max_range_m"]))):
                raise ValueError("LiDAR points contain invalid or out-of-range coordinates")
        ranges_a = np.linalg.norm(points_a, axis=1)
        ranges_b = np.linalg.norm(points_b, axis=1)
        median_a = float(np.median(ranges_a)) if len(ranges_a) else None
        median_b = float(np.median(ranges_b)) if len(ranges_b) else None
        range_diff = abs(median_a - median_b) if median_a is not None and median_b is not None else None
        metrics = {
            "source_time_ns": t,
            "rgb_mean_absolute_difference": float(np.mean(np.abs(color_a.astype(np.int16) - color_b.astype(np.int16)))),
            "depth_valid_fraction_mujoco": float(np.mean(valid_a)),
            "depth_valid_fraction_isaac": float(np.mean(valid_b)),
            "depth_median_abs_difference_m": depth_diff,
            "lidar_return_fraction_mujoco": len(points_a) / ray_count,
            "lidar_return_fraction_isaac": len(points_b) / ray_count,
            "lidar_median_range_mujoco_m": median_a,
            "lidar_median_range_isaac_m": median_b,
            "lidar_median_range_abs_difference_m": range_diff
        }
        checks = {
            "depth_coverage": abs(metrics["depth_valid_fraction_mujoco"] -
                                  metrics["depth_valid_fraction_isaac"]) <= limits["depth_valid_fraction_abs_tolerance"],
            "depth_median": depth_diff is not None and depth_diff <= limits["depth_median_abs_difference_max_m"],
            "lidar_coverage": abs(metrics["lidar_return_fraction_mujoco"] -
                                  metrics["lidar_return_fraction_isaac"]) <= limits["lidar_return_fraction_abs_tolerance"],
            "lidar_median": range_diff is not None and range_diff <= limits["lidar_median_range_abs_difference_max_m"]
        }
        samples.append({"metrics": metrics, "checks": checks, "pass": all(checks.values())})
    return {"format": "s5-comparison-report/v1", "scenario_id": contract["scenario_id"],
            "contract_sha256": contract_hash, "mode": "synthetic_fixture" if fixture else "actual_isaac",
            "mujoco_trace_sha256": digest(mujoco_path), "isaac_trace_sha256": digest(isaac_path),
            "backends": {
                name: {"identity": next(iter(rows.values()))["identity"],
                       "provenance": next(iter(rows.values()))["provenance"]}
                for name, rows in (("mujoco", mj), ("isaac", isaac))},
            "thresholds": limits, "samples": samples, "pass": all(s["pass"] for s in samples),
            "limitations": ["RGB pixel difference is diagnostic because renderers/materials differ.",
                            "LiDAR median range is a distribution metric, not per-ray correspondence."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contract", type=Path)
    parser.add_argument("mujoco_trace", type=Path)
    parser.add_argument("isaac_trace", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("use a new comparison report path")
    result = compare(args.contract, args.mujoco_trace, args.isaac_trace, args.fixture)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"pass": result["pass"], "mode": result["mode"],
                      "report": str(args.output)}))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
