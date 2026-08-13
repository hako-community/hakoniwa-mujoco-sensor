#!/usr/bin/env python3
"""
Validate an A-2 sensor manifest against the JSON Schema, and check that its
radars are not quietly less sensitive than the baseline fit (issue #13).

Usage: python3 validate_manifest.py [manifest.json ...]
Default manifest: ../../config/a2/drone-a2-sensors.json

Why the second check exists
---------------------------
RadarSensor::PointsPerScan() is points_per_second / update_rate and does NOT
depend on the angular window. Widening the window therefore spreads the same
rays over more sky, and every target inside it collects proportionally fewer of
them -- a wider radar is a LESS sensitive radar unless points_per_second is
raised to match. The cost is the ratio of SOLID ANGLES, so widening two axes
multiplies rather than adds: 60x20 -> 150x45 deg is 5.6x, not 2.5x.

Nothing fails at load time when this is got wrong. The radar still runs, still
publishes, and simply detects less -- which is indistinguishable from a target
that was not there. So the number is computed here instead of being remembered.
"""

import json
import os
import sys

import jsonschema

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.normpath(os.path.join(HERE, "../../config/a2"))
SCHEMA = os.path.join(CFG, "a2-sensor-manifest.schema.json")
DEFAULT = os.path.join(CFG, "drone-a2-sensors.json")
# The fit every other one is judged against: the shipped baseline radar.
REFERENCE = DEFAULT
# Below this share of the reference angular density, say so loudly.
DENSITY_WARN = 0.8
# Not settable from a manifest -- OutputBinding::update_rate_hz (include/sensor.hpp).
UPDATE_RATE_HZ = 10.0


def window(params):
    """(az_span, el_span) in degrees, mirroring radar_math.hpp WindowOf."""
    a0, a1 = params.get("azimuth_start_deg"), params.get("azimuth_end_deg")
    if a0 is None or a1 is None:
        h = float(params.get("horizontal_fov_deg", 30.0))
        a0, a1 = -0.5 * h, 0.5 * h
    e0, e1 = params.get("elevation_start_deg"), params.get("elevation_end_deg")
    if e0 is None or e1 is None:
        v = float(params.get("vertical_fov_deg", 10.0))
        e0, e1 = -0.5 * v, 0.5 * v
    return float(a1) - float(a0), float(e1) - float(e0)


def radar_density(component):
    """Returns (points_per_scan, az_span, el_span, points per deg^2)."""
    p = component.get("params", {}) or {}
    az, el = window(p)
    per_scan = float(p.get("points_per_second", 1500)) / UPDATE_RATE_HZ
    area = az * el
    return per_scan, az, el, (per_scan / area if area > 0 else 0.0)


def reference_density():
    with open(REFERENCE) as f:
        ref = json.load(f)
    for c in ref.get("components", []):
        if c.get("type") == "radar":
            return radar_density(c)[3]
    return 0.0


def check(manifest, schema, ref_density):
    with open(manifest) as f:
        data = json.load(f)
    name = os.path.basename(manifest)
    try:
        jsonschema.validate(instance=data, schema=schema)
    except jsonschema.ValidationError as e:
        print(f"[FAIL] {name} invalid: {e.message} (at {list(e.path)})")
        return 1

    sensors = [c for c in data["components"] if c.get("kind", "sensor") == "sensor"]
    print(f"[ ok ] {name} valid: {len(sensors)} sensor component(s): "
          + ", ".join(f"{c.get('id', c['type'])}({c['type']}->{c.get('pdu_name', c.get('id'))})"
                      for c in sensors))

    warned = 0
    for c in sensors:
        if c.get("type") != "radar":
            continue
        per_scan, az, el, dens = radar_density(c)
        ratio = dens / ref_density if ref_density > 0 else 1.0
        line = (f"       {c.get('id', 'radar')}: window {az:.0f}x{el:.0f} deg, "
                f"{per_scan:.0f} pts/scan -> {dens:.4f} pts/deg^2 "
                f"({ratio:.2f}x the baseline fit)")
        if ratio < DENSITY_WARN:
            want = float(c["params"].get("points_per_second", 1500)) / ratio
            print(line)
            print(f"[WARN] {name}: {c.get('id', 'radar')} is {1.0 / ratio:.1f}x THINNER than "
                  f"the baseline -- it will detect less, at every range.")
            print(f"       raise points_per_second to ~{want:.0f} to keep the "
                  f"angular density (window widened without matching the point rate)")
            warned += 1
        else:
            print(line)
    return 2 if warned else 0


def main() -> int:
    manifests = sys.argv[1:] or [DEFAULT]
    with open(SCHEMA) as f:
        schema = json.load(f)
    ref = reference_density()
    rc = 0
    for m in manifests:
        r = check(m, schema, ref)
        # An invalid manifest is fatal; a thin one is a warning, not a failure --
        # a deliberately low-rate radar is a legitimate thing to model.
        if r == 1:
            rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
