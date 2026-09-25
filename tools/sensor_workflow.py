#!/usr/bin/env python3
"""Offline capability gate and audited scalar sensor calibration (stdlib only)."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

VERSION = "sens014-v1"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def capability_gate(descriptor, scenario):
    capabilities = descriptor["capabilities"]
    required = scenario["required_capabilities"]
    if not isinstance(capabilities, dict) or not isinstance(required, dict):
        raise ValueError("capabilities must be objects")
    for values in (capabilities, required):
        if any(type(v) not in (bool, str) for v in values.values()):
            raise ValueError("capabilities must be boolean or named fidelity levels")
    missing = {k: {"required": v, "actual": capabilities.get(k)}
               for k, v in required.items()
               if k not in capabilities or capabilities[k] != v}
    policy = scenario.get("on_missing", "fail")
    if policy not in ("fail", "skip", "degraded"):
        raise ValueError("on_missing must be fail, skip or degraded")
    return {"format": "sensor-capability-report/v1", "backend": descriptor["backend"],
            "scenario_id": scenario["scenario_id"], "pass": not missing,
            "decision": policy if missing else "pass",
            "status_bits": 32 if missing else 1, "missing": missing}


def load_signals(path):
    groups, runs, previous = {}, set(), {}
    with Path(path).open(encoding="utf-8", newline="") as f:
        rows = csv.DictReader(f)
        required = {"run_id", "timestamp_ns", "sensor_id", "axis", "unit", "truth", "measured"}
        if not required.issubset(rows.fieldnames or []):
            raise ValueError("CSV requires " + ",".join(sorted(required)))
        for r in rows:
            if any(not r[k] for k in required):
                raise ValueError("empty CSV field")
            key = (r["sensor_id"], r["axis"], r["unit"])
            clock_key = (r["run_id"],) + key
            t, truth, measured = int(r["timestamp_ns"]), float(r["truth"]), float(r["measured"])
            if t < 0 or t <= previous.get(clock_key, -1):
                raise ValueError("timestamps must strictly increase per run/axis")
            if not math.isfinite(truth) or not math.isfinite(measured):
                raise ValueError("non-finite signal")
            if "status" in r and int(r["status"]) != 1:
                raise ValueError("fit requires Valid-only samples; curate faults separately")
            previous[clock_key] = t
            runs.add(r["run_id"])
            groups.setdefault(key, []).append(measured - truth)
    if not groups or any(len(v) < 2 for v in groups.values()):
        raise ValueError("at least two samples per axis required")
    return groups, runs


def fit_validate(train, holdout, profile_id, limits):
    training, train_runs = load_signals(train)
    validation, val_runs = load_signals(holdout)
    if digest(train) == digest(holdout) or train_runs & val_runs:
        raise ValueError("fit and holdout must use independent run IDs and logs")
    if set(training) != set(validation):
        raise ValueError("fit and holdout sensor/axis/unit sets differ")
    parameters, results = [], []
    for key, values in sorted(training.items()):
        sensor, axis, unit = key
        bound = limits[sensor][axis]
        if bound["unit"] != unit:
            raise ValueError("threshold unit mismatch")
        for name in ("max_bias", "max_rmse"):
            if not math.isfinite(bound[name]) or bound[name] < 0:
                raise ValueError("invalid validation threshold")
        bias = statistics.mean(values)
        errors = [v - bias for v in validation[key]]
        mean, rmse = statistics.mean(errors), math.sqrt(statistics.mean(v*v for v in errors))
        parameters.append({"sensor_id": sensor, "axis": axis, "unit": unit,
                           "bias": bias, "white_noise_stddev": statistics.pstdev(values)})
        results.append({"sensor_id": sensor, "axis": axis, "unit": unit,
                        "fit_count": len(values), "holdout_count": len(errors),
                        "corrected_bias": mean, "corrected_rmse": rmse,
                        "pass": abs(mean) <= bound["max_bias"] and rmse <= bound["max_rmse"]})
    report = {"format": "sensor-validation-report/v1", "tool_version": VERSION,
              "profile_id": profile_id, "pass": all(x["pass"] for x in results),
              "training": {"sha256": digest(train), "run_ids": sorted(train_runs)},
              "holdout": {"sha256": digest(holdout), "run_ids": sorted(val_runs)},
              "thresholds": limits, "streams": results,
              "limitations": ["Scalar additive residual model only; not Allan/PSD, extrinsic or 6x6 calibration.",
                              "Units must be curated SI quantities; no automatic unit conversion."]}
    profile = {"schema_version": "sens005-v1", "profile_id": profile_id,
               "kind": "fitted", "provenance": "fitted", "units": "SI",
               "basis": "Independent-run holdout of additive scalar residual calibration",
               "parameters": {"residual_models": parameters},
               "fit": {"source_log_sha256": digest(train), "tool_version": VERSION}}
    return profile, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    gate = sub.add_parser("gate")
    gate.add_argument("descriptor")
    gate.add_argument("scenario")
    gate.add_argument("--report", required=True)
    fit = sub.add_parser("fit")
    fit.add_argument("training")
    fit.add_argument("holdout")
    fit.add_argument("--thresholds", required=True)
    fit.add_argument("--profile-id", required=True)
    fit.add_argument("--profile", required=True)
    fit.add_argument("--report", required=True)
    args = parser.parse_args()
    try:
        if args.command == "gate":
            result = capability_gate(read_json(args.descriptor), read_json(args.scenario))
            write_json(args.report, result)
            return 0 if result["pass"] else 2
        # Refuse stale or overwritten inputs/artifacts, including a previous passing profile.
        inputs = {Path(p).resolve() for p in (args.training, args.holdout, args.thresholds)}
        outputs = {Path(p).resolve() for p in (args.profile, args.report)}
        if len(outputs) != 2 or inputs & outputs or any(p.exists() for p in outputs):
            raise ValueError("use distinct, new output paths")
        profile, result = fit_validate(args.training, args.holdout, args.profile_id,
                                       read_json(args.thresholds))
        write_json(args.report, result)
        if result["pass"]:
            profile["fit"]["holdout_report_sha256"] = digest(args.report)
            write_json(args.profile, profile)
        return 0 if result["pass"] else 2
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
