#!/usr/bin/env python3
"""Offline exported-annotator -> common sensor trace adapter; no Isaac dependency.

Export RGB(A) as uint8 raw, depth as little-endian float32 metres, and LiDAR
as little-endian float32 Nx3 in an explicitly declared coordinate frame.
The acquisition layer supplies physics-clock nanoseconds, never wall time.
"""
import argparse
import json
import math
import re
from pathlib import Path
from sensor_workflow import capability_gate, digest, read_json


def integer(value):
    if type(value) is not int or value < 0:
        raise ValueError("nonnegative integer required")
    return value


def convert(manifest, records, root):
    for field in ("run_id", "scenario_id", "robot_id", "backend", "backend_version",
                  "git_commit", "model_sha256", "profile_id", "calibration_id", "seed_tree"):
        if field not in manifest or manifest[field] is None:
            raise ValueError("missing provenance: " + field)
        if field != "seed_tree" and (not isinstance(manifest[field], str) or not manifest[field]):
            raise ValueError("nonempty provenance string required: " + field)
    if not re.fullmatch(r"[0-9a-fA-F]{64}", manifest["model_sha256"]):
        raise ValueError("model_sha256 must be a SHA256 digest")
    if not isinstance(manifest["seed_tree"], dict):
        raise ValueError("seed_tree must be an object")
    for value in manifest["seed_tree"].values():
        integer(value)
    gate = capability_gate(manifest["descriptor"], manifest["scenario"])
    if not gate["pass"]:
        raise ValueError("trace scenario rejected: " + json.dumps(gate["missing"]))
    if manifest["backend"] != manifest["descriptor"]["backend"] or manifest["scenario_id"] != manifest["scenario"]["scenario_id"]:
        raise ValueError("descriptor/scenario identity mismatch")
    previous = {}
    result = []
    formats = {"rgb": ("uint8", 1, 3), "rgba": ("uint8", 1, 4),
               "distance_to_image_plane": ("float32-le", 4, None),
               "distance_to_camera": ("float32-le", 4, None),
               "pointcloud": ("float32-le", 4, 3)}
    for row in records:
        sid, frame = row["sensor_id"], row["frame_id"]
        if not sid or not frame:
            raise ValueError("sensor_id/frame_id required")
        sequence = integer(row["sequence"])
        times = [integer(row[k + "_time_ns"]) for k in ("source", "scheduled", "publish", "consume")]
        if times != sorted(times):
            raise ValueError("non-causal physics timestamps")
        if sid in previous and (sequence <= previous[sid][0] or times[0] < previous[sid][1]):
            raise ValueError("non-monotonic stream")
        previous[sid] = (sequence, times[0])
        blobs = {}
        for name, blob in row["annotators"].items():
            if name not in formats:
                raise ValueError("unsupported annotator: " + name)
            dtype, width, channels = formats[name]
            shape = blob["shape"]
            if not isinstance(shape, list) or any(integer(x) == 0 for x in shape):
                # Empty LiDAR clouds are valid and must not disappear from the trace.
                if name != "pointcloud" or shape != [0, 3]:
                    raise ValueError("invalid blob shape")
            if name == "pointcloud":
                valid_shape = len(shape) == 2 and shape[1] == 3
            elif channels:
                valid_shape = len(shape) == 3 and shape[2] == channels
            else:
                valid_shape = len(shape) == 2
            if not valid_shape or blob["dtype"] != dtype:
                raise ValueError("annotator dtype/shape mismatch")
            path = (root / blob["uri"]).resolve()
            if not path.is_relative_to(root.resolve()):
                raise ValueError("blob must be inside export root")
            if path.stat().st_size != math.prod(shape) * width or digest(path) != blob["sha256"]:
                raise ValueError("blob byte count/hash mismatch")
            if name not in ("rgb", "rgba") and blob.get("unit") != "m":
                raise ValueError("depth/points must be in metres")
            blobs[name] = dict(blob, uri=path.as_uri())
        if not blobs:
            raise ValueError("no sensor data")
        color = blobs.get("rgb", blobs.get("rgba"))
        for name in ("distance_to_image_plane", "distance_to_camera"):
            if color and name in blobs and color["shape"][:2] != blobs[name]["shape"]:
                raise ValueError("RGBD dimensions differ")
        health = row["health"]
        for key in ("status_bits", "drop_count", "queue_depth"):
            integer(health[key])
        result.append({"format": "common-sensor-trace/v1",
                       "identity": {k: manifest[k] for k in ("run_id", "scenario_id", "robot_id", "backend", "backend_version")},
                       "provenance": {k: manifest[k] for k in ("git_commit", "model_sha256", "profile_id", "calibration_id", "seed_tree")},
                       "sensor_id": sid, "frame_id": frame, "sequence": sequence,
                       "time": dict(zip((k + "_time_ns" for k in ("source", "scheduled", "publish", "consume")), times)),
                       "data": {"raw_truth": row.get("raw_truth"), "payload": blobs,
                                "observation": row.get("observation")},
                       "health": dict(health, age_ns=times[3] - times[0]),
                       "context": row["context"]})
    if not result:
        raise ValueError("empty trace")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("manifest", type=Path)
    p.add_argument("records", type=Path, help="export JSONL")
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    try:
        if a.output.exists():
            raise ValueError("use a new trace output path")
        rows = [json.loads(line) for line in a.records.read_text(encoding="utf-8").splitlines() if line.strip()]
        converted = convert(read_json(a.manifest), rows, a.records.parent)
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text("".join(json.dumps(r, allow_nan=False) + "\n" for r in converted), encoding="utf-8")
    except (ValueError, KeyError, TypeError, OSError) as exc:
        p.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    main()
