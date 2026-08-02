#!/usr/bin/env python3
"""
Validate an A-2 sensor manifest against the JSON Schema.

Usage: python3 validate_manifest.py [manifest.json]
Default manifest: ../../config/a2/drone-a2-sensors.json
"""

import json
import os
import sys

import jsonschema

HERE = os.path.dirname(os.path.abspath(__file__))
SCHEMA = os.path.normpath(os.path.join(HERE, "../../config/a2/a2-sensor-manifest.schema.json"))
DEFAULT = os.path.normpath(os.path.join(HERE, "../../config/a2/drone-a2-sensors.json"))


def main() -> int:
    manifest = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    with open(SCHEMA) as f:
        schema = json.load(f)
    with open(manifest) as f:
        data = json.load(f)
    try:
        jsonschema.validate(instance=data, schema=schema)
    except jsonschema.ValidationError as e:
        print(f"[FAIL] {os.path.basename(manifest)} invalid: {e.message} (at {list(e.path)})")
        return 1
    sensors = [c for c in data["components"] if c.get("kind", "sensor") == "sensor"]
    print(f"[ ok ] {os.path.basename(manifest)} valid: "
          f"{len(sensors)} sensor component(s): "
          + ", ".join(f"{c.get('id', c['type'])}({c['type']}->{c.get('pdu_name', c.get('id'))})"
                      for c in sensors))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
