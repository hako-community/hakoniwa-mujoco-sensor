#!/usr/bin/env python3
"""Capture the fixed S5 static scene with MuJoCo into common-sensor-trace/v1."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from isaac_trace_export import ArrayTraceExporter


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = ROOT / "config/mujoco_isaac_comparison/static_pillar_rgbd_lidar_v1.json"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_contract(path):
    path = Path(path).resolve()
    contract = json.loads(path.read_text(encoding="utf-8"))
    if contract["format"] != "s5-scenario/v1":
        raise ValueError("unsupported scenario contract")
    model_path = (path.parent / contract["mujoco"]["mjcf_path"]).resolve()
    if not model_path.is_relative_to(path.parent) or digest(model_path) != contract["mujoco"]["mjcf_sha256"]:
        raise ValueError("MJCF path/hash differs from the fixed scenario")
    return contract, model_path


def lidar_points(model, data, spec):
    origin = np.asarray(spec["position_world_m"], dtype=np.float64)
    azimuths = np.linspace(spec["azimuth_min_deg"], spec["azimuth_max_deg"],
                           spec["azimuth_count"])
    elevations = np.linspace(spec["elevation_min_deg"], spec["elevation_max_deg"],
                             spec["elevation_count"])
    points = []
    no_return = 0
    geom_id = np.empty(1, dtype=np.int32)
    for elevation in elevations:
        el = math.radians(float(elevation))
        for azimuth in azimuths:
            az = math.radians(float(azimuth))
            direction = np.asarray([math.cos(el) * math.cos(az),
                                    math.cos(el) * math.sin(az), math.sin(el)],
                                   dtype=np.float64)
            distance = mujoco.mj_ray(model, data, origin, direction, None, 1, -1, geom_id)
            if not (spec["min_range_m"] <= distance <= spec["max_range_m"]):
                no_return += 1
                continue
            # The sensor axes are the world axes in this first static scenario.
            points.append(direction * distance)
    return np.asarray(points, dtype="<f4").reshape((-1, 3)), no_return


def capture(contract_path, output, run_id):
    contract, model_path = load_contract(contract_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    clock = contract["time"]
    if round(model.opt.timestep * 1e9) != clock["physics_dt_ns"]:
        raise ValueError("MJCF timestep differs from scenario contract")
    if any(t % clock["physics_dt_ns"] for t in clock["sample_times_ns"]):
        raise ValueError("sample time is not aligned to the physics step")
    camera = contract["sensors"]["rgbd"]
    lidar = contract["sensors"]["lidar"]
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera["sensor_id"])
    if cam_id < 0 or not np.allclose(model.cam_pos[cam_id], camera["position_world_m"]):
        raise ValueError("MJCF camera pose differs from scenario contract")
    if not math.isclose(float(model.cam_fovy[cam_id]), camera["vertical_fov_deg"]):
        raise ValueError("MJCF camera FOV differs from scenario contract")
    rotation = np.asarray(data.cam_xmat[cam_id]).reshape((3, 3))
    optical = camera["optical_axes_world"]
    for local_axis, expected in (([1, 0, 0], optical["right"]),
                                 ([0, -1, 0], optical["down"]),
                                 ([0, 0, -1], optical["forward"])):
        if not np.allclose(rotation @ local_axis, expected, atol=1e-7):
            raise ValueError("MJCF camera orientation differs from scenario contract")
    contract_hash = digest(contract_path)
    commit = subprocess.run(["git", "-c", f"safe.directory={ROOT}", "-C", str(ROOT),
                             "rev-parse", "HEAD"], text=True, capture_output=True,
                            check=True).stdout.strip()
    manifest = {
        "run_id": run_id, "scenario_id": contract["scenario_id"], "robot_id": "static_sensor_rig",
        "backend": "mujoco", "backend_version": mujoco.__version__,
        "git_commit": commit, "model_sha256": digest(model_path),
        "profile_id": "s5.static.truth", "calibration_id": "none",
        "seed_tree": {"root": contract["seed"]},
        "descriptor": {"backend": "mujoco", "capabilities": {
            "rgb": True, "distance_to_image_plane": True, "pointcloud": True}},
        "scenario": contract["capability_scenario"],
        "contract_sha256": contract_hash,
        "capture_provenance": "mujoco_renderer_and_mj_ray"
    }
    exporter = ArrayTraceExporter(output, manifest)
    width, height = camera["image_width"], camera["image_height"]
    renderer = mujoco.Renderer(model, width=width, height=height)
    try:
        fy = height / (2 * math.tan(math.radians(camera["vertical_fov_deg"]) / 2))
        intrinsics = {"fx": fy, "fy": fy, "cx": (width - 1) / 2,
                      "cy": (height - 1) / 2, "width": width, "height": height}
        for sequence, target_ns in enumerate(clock["sample_times_ns"]):
            target_step = target_ns // clock["physics_dt_ns"]
            while round(data.time / model.opt.timestep) < target_step:
                mujoco.mj_step(model, data)
            if abs(round(data.time * 1e9) - target_ns) > 1:
                raise ValueError("MuJoCo physics clock diverged from scenario")
            renderer.disable_depth_rendering()
            renderer.update_scene(data, camera=camera["sensor_id"])
            rgb = np.ascontiguousarray(renderer.render().copy(), dtype=np.uint8)
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera["sensor_id"])
            depth = renderer.render().copy().astype("<f4")
            depth[~np.isfinite(depth) | (depth <= 0) | (depth > camera["depth_max_m"])] = np.inf
            points, no_return = lidar_points(model, data, lidar)
            base = {"sequence": sequence, "source_time_ns": target_ns,
                    "scheduled_time_ns": target_ns, "publish_time_ns": target_ns,
                    "consume_time_ns": target_ns,
                    "health": {"status_bits": 1, "drop_count": 0, "queue_depth": 0}}
            exporter.capture(dict(base, sensor_id=camera["sensor_id"],
                frame_id=camera["frame_id"], raw_truth=None,
                context={"contract_sha256": contract_hash,
                         "capture_provenance": "mujoco_renderer_and_mj_ray",
                         "depth_definition": camera["depth_annotator"],
                         "depth_invalid": camera["depth_invalid"],
                         "intrinsics": intrinsics}),
                {"rgb": rgb, "distance_to_image_plane": depth})
            exporter.capture(dict(base, sensor_id=lidar["sensor_id"],
                frame_id=lidar["frame_id"], raw_truth=None,
                context={"contract_sha256": contract_hash,
                         "capture_provenance": "mujoco_renderer_and_mj_ray",
                         "ray_count": lidar["azimuth_count"] * lidar["elevation_count"],
                         "no_return_count": no_return,
                         "point_frame": "sensor_local_xyz_m"}),
                {"pointcloud": points})
        exporter.close()
    finally:
        renderer.close()
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", default="s5-mujoco-static-pillar-20260925")
    args = parser.parse_args()
    capture(args.contract, args.output, args.run_id)
    print(args.output / "common_trace.jsonl")


if __name__ == "__main__":
    main()
