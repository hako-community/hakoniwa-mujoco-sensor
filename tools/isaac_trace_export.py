"""Capture adapter for already acquired CPU NumPy annotator arrays.

Isaac imports deliberately stay in the caller. Pass Camera.get_rgba() or RGB
annotator data, distance_to_image_plane data, and RTX pointcloud annotator data.
The caller must synchronize capture and provide physics timestamps/coordinate
frame. This module never guesses time, stage units or world-to-sensor transforms.
"""
import hashlib
from pathlib import Path
import re
from common_trace import convert
from sensor_workflow import write_json


class ArrayTraceExporter:
    def __init__(self, directory, manifest):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=False)
        self.manifest = manifest
        self.records = []
        write_json(self.root / "manifest.json", manifest)

    def capture(self, metadata, arrays):
        """arrays: rgb/rgba uint8, metric depth float32, metric local points Nx3.

        metadata includes sensor_id, frame_id, sequence, four *_time_ns fields,
        health and context. Depth invalids (NaN/Inf) are retained in the blob.
        """
        import numpy as np
        sid = metadata["sensor_id"]
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", sid):
            raise ValueError("sensor ID must be filename safe")
        descriptors = {}
        for name, array in arrays.items():
            if name not in ("rgb", "rgba", "distance_to_image_plane", "distance_to_camera", "pointcloud"):
                raise ValueError("unsupported annotator")
            dtype = np.dtype("uint8" if name in ("rgb", "rgba") else "<f4")
            array = np.asarray(array)
            if array.dtype != dtype:
                raise ValueError("explicitly convert annotator dtype/units before capture")
            raw = np.ascontiguousarray(array).tobytes()
            filename = f"{sid}-{metadata['sequence']}-{name}.bin"
            with (self.root / filename).open("xb") as f:
                f.write(raw)
            descriptors[name] = {"uri": filename, "sha256": hashlib.sha256(raw).hexdigest(),
                                 "shape": list(array.shape), "dtype": "uint8" if name in ("rgb", "rgba") else "float32-le"}
            if name not in ("rgb", "rgba"):
                descriptors[name]["unit"] = "m"
        row = dict(metadata, annotators=descriptors)
        # Validate before including the capture in the stream; malformed blobs
        # may remain as unreferenced artifacts for diagnosis.
        convert(self.manifest, self.records + [row], self.root)
        self.records.append(row)

    def close(self):
        import json
        rows = convert(self.manifest, self.records, self.root)
        for filename, data in (("records.jsonl", self.records), ("common_trace.jsonl", rows)):
            (self.root / filename).write_text("".join(json.dumps(r, allow_nan=False) + "\n" for r in data), encoding="utf-8")


class IsaacTraceExporter(ArrayTraceExporter):
    """Backwards-compatible name for callers inside Isaac Sim."""
