#!/usr/bin/env python3
"""Unit test for the radar-fit layer of daa_common (issue #6).

Backend-free on purpose, in the spirit of the C++ tests: hakopy and the PDU
converter are stubbed, so this runs with a bare interpreter and no hakoniwa
master. What it pins down is everything that is silent when wrong --

  * a radar the manifest fits but the pdudef does not wire is NOT in the fit
    (reading its channel would read some other robot's PDU);
  * with no environment set the fit is the historical single ch19 radar, so
    every existing invocation keeps its exact behaviour;
  * bearings from a yawed mount are rotated into the body frame before they are
    compared with anything else -- a rear radar reporting "dead ahead" in its
    own frame must not read as a head-on threat;
  * the merged scan takes the NEAREST return across radars and the TOTAL count.

Usage: python3 radar_fit_test.py
"""
import math
import os
import struct
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

# --- stubs, installed before daa_common is imported -------------------------
_PAYLOADS = {}      # (robot, channel) -> list of (x, y, z, doppler)


class _FakeCloud:
    def __init__(self, pts):
        self.width = len(pts)
        self.point_step = 16
        self.data = b"".join(struct.pack("<ffff", *p) for p in pts)


def _fake_read(robot, channel, size):
    if (robot, channel) not in _PAYLOADS:
        raise RuntimeError(f"no such channel {channel}")
    pts = _PAYLOADS[(robot, channel)]
    # The real read returns raw bytes; our converter stub decodes them back.
    return struct.pack("<I", len(pts)) + b"".join(
        struct.pack("<ffff", *p) for p in pts)


def _fake_conv(raw):
    n = struct.unpack_from("<I", raw, 0)[0]
    return _FakeCloud([struct.unpack_from("<ffff", raw, 4 + 16 * i)
                       for i in range(n)])


sys.modules["hakopy"] = types.SimpleNamespace(pdu_read=_fake_read)
_pkg = types.ModuleType("hakoniwa_pdu")
_pkg.__path__ = []
sys.modules["hakoniwa_pdu"] = _pkg
for _name in ("hakoniwa_pdu.pdu_msgs", "hakoniwa_pdu.pdu_msgs.sensor_msgs"):
    _m = types.ModuleType(_name)
    _m.__path__ = []
    sys.modules[_name] = _m
_conv = types.ModuleType(
    "hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2")
_conv.pdu_to_py_PointCloud2 = _fake_conv
sys.modules["hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2"] = _conv

sys.path.insert(0, HERE)
import daa_common as dc     # noqa: E402

CFG_A2 = os.path.join(REPO, "config", "a2")
PDUDEF_1 = os.path.join(HERE, "config2", "webavatar-2-radar.json")
PDUDEF_2 = os.path.join(HERE, "config2", "webavatar-2-radar2.json")

_checks = 0
_fails = []


def check(cond, what):
    global _checks
    _checks += 1
    if not cond:
        _fails.append(what)
    print(f"  [{'ok' if cond else 'FAIL'}] {what}")


def close(a, b, tol=1e-6):
    return a is not None and b is not None and abs(a - b) <= tol


def use(pdudef=None, manifest=None, channels=None, stack=None):
    """Reset the cache and set the environment one case at a time."""
    dc._fit_cache.clear()
    dc._missing_channels.clear()
    for var in ("A2_PDUDEF", "A2_MANIFEST", "A2_MANIFEST_Drone",
                "A2_RADAR_CHANNELS"):
        os.environ.pop(var, None)
    # Point the stack descriptor at nothing unless a case supplies one, so a
    # logs/stack.json left behind by a real run cannot steer this test.
    os.environ["A2_STACK_JSON"] = stack or os.path.join(HERE, "logs", "__none__.json")
    if pdudef:
        os.environ["A2_PDUDEF"] = pdudef
    if manifest:
        os.environ["A2_MANIFEST"] = manifest
    if channels:
        os.environ["A2_RADAR_CHANNELS"] = channels


def main():
    print("1) pdudef parsing (the Python twin of pdudef_channels.hpp)")
    ch1 = dc._pdudef_channels(PDUDEF_1, "Drone")
    ch2 = dc._pdudef_channels(PDUDEF_2, "Drone")
    check(ch1.get("radar_points") == (19, 177424),
          f"single-radar pdudef: radar_points -> ch19/177424 (got {ch1.get('radar_points')})")
    check("radar_points_rear" not in ch1,
          "single-radar pdudef declares no rear channel")
    check(ch2.get("radar_points_rear") == (21, 177424),
          f"dual pdudef: radar_points_rear -> ch21 (got {ch2.get('radar_points_rear')})")
    check(dc._pdudef_channels(PDUDEF_2, "Drone1").get("radar_points_rear") == (21, 177424),
          "the rear channel has the SAME number on both robots (radar invariant #11)")
    check(dc._pdudef_channels(PDUDEF_1, "NoSuchRobot") == {},
          "unknown robot -> empty map, not an exception")

    print("2) manifest parsing: what is fitted, and where it looks")
    one = dc._manifest_radars(os.path.join(CFG_A2, "drone-a2-sensors.json"))
    dual = dc._manifest_radars(os.path.join(CFG_A2, "drone-a2-sensors-dual.json"))
    omni = dc._manifest_radars(os.path.join(CFG_A2, "drone-a2-sensors-360.json"))
    check(len(one) == 1 and close(one[0]["az_lo"], -30.0) and close(one[0]["az_hi"], 30.0),
          "baseline manifest: one radar, 60 deg sector -> az [-30,+30]")
    check(len(dual) == 2 and dual[1]["pdu_name"] == "radar_points_rear"
          and close(dual[1]["az_lo"], 150.0) and close(dual[1]["az_hi"], 210.0),
          "dual manifest: explicit azimuth_start/end wins over horizontal_fov -> az [150,210]")
    check(len(omni) == 1 and close(omni[0]["az_hi"] - omni[0]["az_lo"], 360.0),
          "360 manifest: full azimuth")

    print("3) fit resolution (manifest x pdudef), weakest layer first")
    use()
    fit = dc.radar_fit("Drone")
    check(len(fit) == 1 and fit[0].channel == 19 and fit[0].pdu_size == 177424,
          "no A2_PDUDEF/A2_MANIFEST -> the historical single ch19 radar (non-regression)")

    use(pdudef=PDUDEF_2, manifest=os.path.join(CFG_A2, "drone-a2-sensors-dual.json"))
    fit = dc.radar_fit("Drone")
    check([u.channel for u in fit] == [19, 21],
          f"dual manifest + dual pdudef -> ch19 + ch21 (got {[u.channel for u in fit]})")
    check([u.sensor_id for u in fit] == ["front_radar", "rear_radar"],
          "units keep their manifest ids, so a report can name the radar that saw the threat")

    # The case that must not go wrong: the aircraft is FITTED with a rear radar
    # but the master was started on a pdudef that has no channel for it. The
    # bridge refuses to publish it; the fit must refuse to read it.
    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors-dual.json"))
    fit = dc.radar_fit("Drone")
    check([u.channel for u in fit] == [19],
          "rear radar fitted but not wired in the pdudef -> dropped from the fit")

    use(channels="19,21")
    check([u.channel for u in dc.radar_fit("Drone")] == [19, 21],
          "A2_RADAR_CHANNELS overrides everything (escape hatch)")

    # The path that actually runs: the launcher records the stack, the scenario
    # starts in a different shell and picks it up with no environment at all.
    import json as _json
    import tempfile
    stack = os.path.join(tempfile.mkdtemp(), "stack.json")
    with open(stack, "w") as f:
        _json.dump({"pdudef": PDUDEF_2,
                    "manifests": {
                        "Drone": os.path.join(CFG_A2, "drone-a2-sensors-dual.json"),
                        "Drone1": os.path.join(CFG_A2, "drone-a2-sensors.json")}}, f)
    use(stack=stack)
    check([u.channel for u in dc.radar_fit("Drone")] == [19, 21],
          "logs/stack.json alone resolves the fit -- no env needed across shells")
    check([u.channel for u in dc.radar_fit("Drone1")] == [19],
          "and each aircraft gets ITS own fit: Drone1 is forward-only here")
    use(stack=stack, manifest=os.path.join(CFG_A2, "drone-a2-sensors.json"))
    check([u.channel for u in dc.radar_fit("Drone")] == [19],
          "an explicit A2_MANIFEST still beats the recorded stack")

    print("4) coverage arithmetic")
    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors.json"))
    check(close(dc.azimuth_coverage_deg("Drone"), 60.0),
          f"one 60 deg sector -> 60 deg covered (got {dc.azimuth_coverage_deg('Drone')})")
    check(dc.covers_az("Drone", 0.0) is True, "forward radar covers dead ahead")
    check(dc.covers_az("Drone", 180.0) is False,
          "forward radar does NOT cover astern -- the S-3 blind sector, now measured")

    use(pdudef=PDUDEF_2, manifest=os.path.join(CFG_A2, "drone-a2-sensors-dual.json"))
    check(close(dc.azimuth_coverage_deg("Drone"), 120.0),
          f"forward + rear sectors -> 120 deg covered (got {dc.azimuth_coverage_deg('Drone')})")
    check(dc.covers_az("Drone", 180.0) is True,
          "the rear sector closes the blind spot astern")
    check(dc.covers_az("Drone", 90.0) is False,
          "the beam is still not covered by either sector")

    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors-360.json"))
    check(close(dc.azimuth_coverage_deg("Drone"), 360.0),
          "360 manifest -> full coverage, overlap counted once")

    print("5) body-frame bearings: a yawed mount is rotated before use")
    # One return 2 m dead ahead OF THE SENSOR.
    payload = struct.pack("<I", 1) + struct.pack("<ffff", 2.0, 0.0, 0.0, -1.0)
    h0 = dc._hits(payload, 180.0, 15.0, None, None, mount_yaw_deg=0.0)
    h180 = dc._hits(payload, 180.0, 15.0, None, None, mount_yaw_deg=180.0)
    check(len(h0) == 1 and close(h0[0][2], 0.0),
          "yaw 0: bearing unchanged (bit-identical to the old code path)")
    check(len(h180) == 1 and close(abs(h180[0][2]), 180.0),
          f"yaw 180: the same return reads as astern, not head-on (got {h180[0][2]:+.1f} deg)")
    h_win = dc._hits(payload, 15.0, 15.0, None, None, mount_yaw_deg=180.0)
    check(h_win == [],
          "and it is therefore excluded from a +/-15 deg forward window")

    print("6) merging across the fit")
    use(pdudef=PDUDEF_2, manifest=os.path.join(CFG_A2, "drone-a2-sensors-dual.json"))
    _PAYLOADS.clear()
    # front radar: two returns, nearest at 3.0 m; rear radar: one at 1.5 m.
    _PAYLOADS[("Drone", 19)] = [(3.0, 0.0, 0.0, -0.5), (5.0, 0.0, 0.0, -0.5)]
    _PAYLOADS[("Drone", 21)] = [(-1.5, 0.0, 0.0, +0.25)]
    front_only = dc.scan("Drone", az_half=180.0, el_half=15.0)
    best = dc.scan_best("Drone", az_half=180.0, el_half=15.0)
    per = dc.scan_units("Drone", az_half=180.0, el_half=15.0)
    check(close(front_only.rng, 3.0) and front_only.count == 2,
          "single-channel scan() still sees only its own radar: 3.00 m, 2 hits")
    check(close(best.rng, 1.5) and best.source == "rear_radar",
          f"scan_best takes the nearest across radars: {best.rng:.2f} m from {best.source}")
    check(best.count == 3, f"and the total hit count, not one radar's (got {best.count})")
    check(close(per["front_radar"].rng, 3.0) and close(per["rear_radar"].rng, 1.5),
          "the per-radar breakdown keeps both answers")
    check(close(best.doppler, 0.25),
          "doppler and bearing come from the SAME (nearest) return")

    # A forward window must not let the rear radar's returns in.
    fwd = dc.scan_best("Drone", az_half=15.0, el_half=15.0)
    check(close(fwd.rng, 3.0) and fwd.source == "front_radar",
          "a +/-15 deg forward window ignores the rear sector entirely")

    print("7) single-radar equivalence (the non-regression that matters)")
    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors.json"))
    _PAYLOADS.clear()
    _PAYLOADS[("Drone", 19)] = [(2.0, 0.3, 0.0, -1.25), (4.0, 0.0, 0.0, -0.5)]
    a = dc.scan("Drone", az_half=30.0, el_half=15.0)
    b = dc.scan_best("Drone", az_half=30.0, el_half=15.0)
    check((a.count, a.rng, a.doppler, a.az_deg, a.el_deg)
          == (b.count, b.rng, b.doppler, b.az_deg, b.el_deg),
          "on a one-radar fit scan_best() == scan(): every scenario keeps its numbers")

    print("8) a missing channel is skipped, not fatal")
    use(pdudef=PDUDEF_2, manifest=os.path.join(CFG_A2, "drone-a2-sensors-dual.json"))
    _PAYLOADS.clear()
    _PAYLOADS[("Drone", 19)] = [(2.0, 0.0, 0.0, -1.0)]      # ch21 absent -> raises
    s = dc.scan_best("Drone", az_half=180.0, el_half=15.0)
    check(close(s.rng, 2.0) and s.source == "front_radar",
          "the fit lists ch21 but the stack has none: skipped after the first failure")
    check(("Drone", 21) in dc._missing_channels,
          "and remembered, so it is not retried every tick")

    print("9) elevation coverage (#7)")
    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors.json"))
    check(close(dc.elevation_coverage_deg("Drone"), 20.0),
          f"baseline: symmetric 20 deg vertical FOV (got {dc.elevation_coverage_deg('Drone')})")
    check(dc.covers_el("Drone", 0.0) is True and dc.covers_el("Drone", -25.0) is False,
          "covers the horizon, not 25 deg below it")
    u = dc.radar_fit("Drone")[0]
    check(close(u.el_lo, -10.0) and close(u.el_hi, 10.0),
          "vertical_fov_deg -> a window centred on the boresight")

    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors-approach.json"))
    u = dc.radar_fit("Drone")[0]
    check(close(u.el_lo, -35.0) and close(u.el_hi, 10.0),
          "approach manifest: explicit elevation_start/end wins, and is ASYMMETRIC")
    check(close(dc.elevation_coverage_deg("Drone"), 45.0),
          "45 deg of elevation, biased below the horizon")
    check(dc.covers_el("Drone", -30.0) is True and dc.covers_el("Drone", +20.0) is False,
          "it buys downward coverage and gives up sky it does not need")
    # The point rate must grow with the window or the radar gets less sensitive
    # (measured in radar_math_test); the manifest is the place that has to honour it.
    import json as _j
    _base = _j.load(open(os.path.join(CFG_A2, "drone-a2-sensors.json")))
    _app = _j.load(open(os.path.join(CFG_A2, "drone-a2-sensors-approach.json")))
    def _radar_pps(doc):
        for c in doc["components"]:
            if c["type"] == "radar":
                return c["params"]["points_per_second"]
    ratio = _radar_pps(_app) / _radar_pps(_base)
    check(2.1 < ratio < 2.4,
          f"points_per_second scaled by the 45/20 span ratio (got {ratio:.2f}x)")

    print("10) why a target was missed")
    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors.json"))
    check(dc.coverage_gap("Drone", 0.0, 0.0, 5.0) is None,
          "dead ahead, in range -> no gap")
    g = dc.coverage_gap("Drone", 0.0, -25.0, 5.0)
    check(g is not None and g.startswith("elevation"),
          f"25 deg below the horizon -> named as an ELEVATION gap ({g})")
    g = dc.coverage_gap("Drone", -50.0, 0.0, 5.0)
    check(g is not None and g.startswith("azimuth"),
          f"50 deg off the nose -> named as an AZIMUTH gap ({g})")
    g = dc.coverage_gap("Drone", 0.0, 0.0, 40.0)
    check(g is not None and g.startswith("range"),
          f"inside the window but too far -> named as a RANGE gap ({g})")
    g = dc.coverage_gap("Drone", -50.0, -25.0, 5.0)
    check(g is not None and "every window" in g,
          f"outside on both axes -> reported as such ({g})")

    use(pdudef=PDUDEF_1, manifest=os.path.join(CFG_A2, "drone-a2-sensors-approach.json"))
    check(dc.coverage_gap("Drone", 0.0, -25.0, 5.0) is None,
          "the approach fit closes exactly the gap that blocked S-5")

    use(pdudef=PDUDEF_1,
        manifest=os.path.join(CFG_A2, "drone-a2-sensors-approach-wide.json"))
    check(dc.coverage_gap("Drone", -50.0, -25.0, 5.0) is None,
          "the wide approach fit also covers 50 deg off the nose (#13)")
    # The trap #13 actually turned on: a scenario filtering harder than its own
    # radar. Blamed on the sensor, it sends you looking for a coverage problem
    # that is not there -- so the caller's window is named separately.
    g = dc.coverage_gap("Drone", -50.0, -25.0, 5.0, az_half=25.0)
    check(g is not None and "SCENARIO" in g and "azimuth" in g,
          f"a caller narrower than the fit is blamed on the CALLER ({g})")
    g = dc.coverage_gap("Drone", 0.0, -25.0, 5.0, el_half=12.0)
    check(g is not None and "SCENARIO" in g and "elevation" in g,
          f"same on the elevation axis ({g})")
    check(dc.coverage_gap("Drone", -50.0, -25.0, 5.0,
                          az_half=None, el_half=None) is None,
          "no caller window -> the fit alone decides")

    print("11) body-frame bearing of a known target (diagnosis only)")
    # Heading +x, target 4 m ahead and 1 m below.
    az, el, r = dc.body_bearing((0.0, 0.0, 1.2), 0.0, (4.0, 0.0, 0.2))
    check(close(az, 0.0) and close(el, math.degrees(math.atan2(-1.0, 4.0)), 1e-9),
          f"ahead and below -> az 0, el {el:+.1f} deg")
    check(close(r, math.hypot(4.0, 1.0)), "range is the true slant range")
    # Same target, aircraft turned 90 deg left: it is now off the starboard bow.
    az2, _, _ = dc.body_bearing((0.0, 0.0, 1.2), 90.0, (4.0, 0.0, 0.2))
    check(close(az2, -90.0), f"yaw is applied: az {az2:+.0f} deg (target to our right)")

    print(f"\n{_checks - len(_fails)}/{_checks} checks passed")
    for f in _fails:
        print(f"  FAILED: {f}")
    return 1 if _fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
