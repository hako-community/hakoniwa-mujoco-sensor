#!/usr/bin/env python3
"""S-6 crewed aircraft encounter -- the UAS gives way, always.

This is the scaled re-creation of the NEDO DRESS / JRC flight demonstration:
a small UAS and a crewed helicopter approach head-on, and the UAS alone
manoeuvres. There is no rule to select and no bearing test to make -- 国土交通省
航空局 (2016-11-08) concluded that because a crewed aircraft cannot reliably see
or avoid a UA, the UA gives way in every case.

Two things differ from S-1, and both matter:

1. Well Clear is the CREWED volume: h = 2.44 m (ASTM F3442's 2000 ft scaled by
   1/250), not the 1.25 m used between UAs. And only ONE aircraft manoeuvres, so
   the lateral offset has to exceed that on its own.

2. The threat is physically BIGGER. Our radar is a geometric ray caster with no
   RCS model, so target size is what sets detection range. The size is derived
   rather than guessed:

     radar equation      Rmax proportional to sigma^(1/4)
     helicopter ~10 m^2 vs small UA ~0.01 m^2  -> sigma ratio 1000
                                              -> range ratio 1000^0.25 = 5.6x

     a random-ray sampler detects with probability proportional to
     (target cross-section) / R^2, so matching a 5.6x range needs 5.6^2 = 31.6x
     the frontal AREA: the 0.5 x 0.3 m UA box (0.15 m^2) becomes 2.8 x 1.7 m
     (4.76 m^2). That is env_actors_heli.xml.

   Hypothesis under test (Phase 2 finding #1): the ~3 m detection range that
   limited S-1..S-3 is a property of the TARGET, not of the radar, so this
   encounter should be detected several times further out -- and if so, the
   tau >= 3.5 s criterion becomes satisfiable for the first time.

Run AFTER:  bash drone_daasim/two_drone_run.sh crewed
Usage:      python scenario_s6_crewed.py
"""
import math
import os
import time

import daa_common as dc
import daa_metrics
from daa_metrics import EncounterRecorder, RangeTracker, StepReport, tau_mod

DWC = daa_metrics.UAS_VS_MANNED       # h=2.44 m, v=0.30 m, tau=3.5 s
TAU_FIRE = DWC.tau_s
R_FLOOR = DWC.h_m * 1.3               # fail-safe divert distance

H = 0.6
# Only the UAS moves, so its offset alone has to clear the crewed Well Clear
# radius, with margin for the sluggishness of the position controller.
Y_AVOID = 3.0
GOAL_EPS = 0.5
MOVE_SPEED = 1.5
APPROACH_TO = 0.5
APPROACH_TICKS = 20
LANE_TO, CROSS_TO, HOME_TO = 8.0, 14.0, 10.0
AZ_HALF = 25.0

UAS, THREAT = "Drone", "Drone1"
# Speed ratio 1:3 mirrors the flight demo (UAV 50 km/h vs helicopter 150 km/h).
STEP_UAS, STEP_THREAT = 0.45, 1.35
# Start beyond the radar's 20 m range by default, so the FIRST detection is a
# measurement of the effective detection range rather than an artefact of where
# the aircraft happened to be placed. S6_START overrides it.
_S = float(os.environ.get("S6_START", "13.0"))
START = {UAS: -_S, THREAT: +_S}
GOAL = {UAS: _S + 0.5, THREAT: -(_S + 0.5)}
YAW = {UAS: 0.0, THREAT: 180.0}
RIGHT_SIGN = -1                       # UAS heads +x, so its right is -y


def main():
    clients = dc.connect([UAS, THREAT])

    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H)
    time.sleep(1.0)

    print(f"set up head-on: {UAS} at x={START[UAS]} (+x), "
          f"crewed traffic at x={START[THREAT]} (-x) ...", flush=True)
    for name in (UAS, THREAT):
        clients[name].moveToPosition(START[name], 0.0, H, MOVE_SPEED,
                                     yaw_deg=YAW[name], timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, UAS, THREAT)
    sampler.start()

    detected = r_det = None
    recognised = False
    tau_fire = r_fire = None
    fired_by = None
    rng_tr = RangeTracker()
    # The crewed aircraft is not scored -- it never manoeuvres -- but we record
    # whether it saw anything at all, because "the crewed aircraft cannot avoid"
    # is the premise the whole rule rests on.
    threat_saw = False

    print(f"A) close head-on; the UAS alone must give way "
          f"(DWC {DWC.h_m:.2f} m, manoeuvre scored on tau >= {TAU_FIRE:.1f} s) ...", flush=True)
    for k in range(APPROACH_TICKS):
        for name, step in ((THREAT, STEP_THREAT), (UAS, STEP_UAS)):
            p = dc.read_xyz(name)
            px = p[0] if p else START[name]
            s = dc.scan_best(name, az_half=AZ_HALF, el_half=15.0)
            sign = 1.0 if YAW[name] == 0.0 else -1.0
            if name == THREAT:
                if s.rng is not None:
                    threat_saw = True
                clients[name].moveToPosition(px + sign * step, 0.0, H, MOVE_SPEED,
                                             yaw_deg=YAW[name], timeout_sec=APPROACH_TO)
                print(f"  [A{k:02d}] {name}(crewed) x={px:+.2f} sep={enc.min_h:.2f}", flush=True)
                continue

            if s.rng is not None and detected is None:
                detected, r_det = True, s.rng
            closure = rng_tr.update(time.time(), s.rng, s.doppler)
            if s.rng is not None and closure is not None:
                recognised = True
            t = tau_mod(s.rng, closure, DWC.h_m) if s.rng is not None else math.inf
            if tau_fire is None and s.rng is not None:
                if recognised and closure is not None and closure > 0.0:
                    tau_fire, r_fire, fired_by = t, s.rng, "recognition"
                elif s.rng <= R_FLOOR:
                    tau_fire, r_fire, fired_by = t, s.rng, "floor"
            rs = f"{s.rng:.2f}" if s.rng is not None else "--"
            cs = f"{closure:+.2f}" if closure is not None else "--"
            ts = f"{t:.1f}" if t != math.inf else "inf"
            print(f"  [A{k:02d}] {name}(UAS)    x={px:+.2f} r={rs} closure={cs} tau={ts} "
                  f"fire={fired_by} sep={enc.min_h:.2f}", flush=True)
            clients[name].moveToPosition(px + sign * step, 0.0, H, MOVE_SPEED,
                                         yaw_deg=YAW[name], timeout_sec=APPROACH_TO)
        if tau_fire is not None:
            print("  -> UAS has a firing solution; giving way.", flush=True)
            break

    # --- PHASE B: the UAS clears the crewed aircraft's track -----------------
    print(f"B) UAS gives way: offset {Y_AVOID:.1f} m to its right; crewed traffic continues ...",
          flush=True)
    time.sleep(0.6)
    lane_ok = False
    for attempt in range(3):
        p = dc.read_xyz(UAS)
        px = p[0] if p else START[UAS]
        clients[UAS].moveToPosition(px, RIGHT_SIGN * Y_AVOID, H, MOVE_SPEED,
                                    yaw_deg=YAW[UAS], timeout_sec=LANE_TO)
        p = dc.read_xyz(UAS)
        lane_ok = p is not None and abs(abs(p[1]) - Y_AVOID) < 0.5
        print(f"  {UAS} lane y={p[1]:+.2f} (target {RIGHT_SIGN * Y_AVOID:+.2f}) "
              f"ok={lane_ok} attempt={attempt}" if p else "  lane ??", flush=True)
        if lane_ok:
            break
    clients[THREAT].moveToPosition(GOAL[THREAT], 0.0, H, MOVE_SPEED,
                                   yaw_deg=YAW[THREAT], timeout_sec=CROSS_TO)

    # --- PHASE C: hold the lane until the crewed aircraft is past ------------
    print("C) hold the lane until the crewed aircraft has passed ...", flush=True)
    for _ in range(24):
        pu, pt = dc.read_xyz(UAS), dc.read_xyz(THREAT)
        if pu and pt and pt[0] < pu[0] - DWC.h_m:
            break
        time.sleep(0.3)
    clients[UAS].moveToPosition(GOAL[UAS], RIGHT_SIGN * Y_AVOID, H, MOVE_SPEED,
                                yaw_deg=YAW[UAS], timeout_sec=CROSS_TO)

    # --- PHASE D: back onto the planned route --------------------------------
    print("D) UAS returns to its planned route ...", flush=True)
    reached, off_track = {}, {}
    for name in (UAS, THREAT):
        clients[name].moveToPosition(GOAL[name], 0.0, H, MOVE_SPEED,
                                     yaw_deg=YAW[name], timeout_sec=HOME_TO)
        p = dc.read_xyz(name)
        reached[name] = p is not None and abs(p[0] - GOAL[name]) < GOAL_EPS
        off_track[name] = abs(p[1]) if p else math.inf
        ps = f"{p[0]:+.2f},{p[1]:+.2f}" if p else "??"
        print(f"  {name} at ({ps}) goal_x={GOAL[name]:+.2f} reached={reached[name]}", flush=True)

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-6 crewed aircraft (UAS gives way -- 有人機優先)", DWC)

    rep.record(1, bool(detected),
               f"UAS first detection at " + (f"{r_det:.2f} m" if r_det else "none")
               + f" | crewed aircraft saw anything? {'yes' if threat_saw else 'no'}"
               + "  (the rule exists because it cannot be relied on to)")

    rep.record(2, recognised,
               "range + closure available: " + ("yes" if recognised else "no")
               + "  (classification not needed: a crewed aircraft is given way to "
                 "whatever the geometry)")

    in_time = tau_fire is not None and tau_fire >= TAU_FIRE
    rep.record(3, in_time and lane_ok,
               "manoeuvre start r=" + (f"{r_fire:.2f} m, tau {tau_fire:.1f} s "
                                       f">= {TAU_FIRE:.1f}? {'yes' if in_time else 'NO'}, "
                                       f"by {fired_by}" if tau_fire is not None else "never")
               + f" | offset reached: {lane_ok}")

    rep.record(4, enc.well_clear_kept, enc.summary())

    rep.record(5, reached[UAS], f"UAS back on route: {off_track[UAS]:.2f} m off track")

    rep.record(6, all(reached.values()),
               "goals reached: " + ", ".join(f"{n}={reached[n]}" for n in (UAS, THREAT)))

    print(rep.render(), flush=True)
    print(f"[s6] min separation = {enc.min_h:.2f} m "
          f"(crewed Well Clear >= {DWC.h_m} m? {enc.min_h >= DWC.h_m})", flush=True)
    print(f"[s6] DETECTION RANGE vs small UA: {r_det:.2f} m here"
          if r_det else "[s6] DETECTION RANGE: no detection", flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(detect={bool(detected)}, recognise={recognised}, in_time={in_time}, "
          f"well_clear={enc.well_clear_kept}, returned={reached[UAS]})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
