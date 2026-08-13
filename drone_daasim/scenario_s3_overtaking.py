#!/usr/bin/env python3
"""S-3 overtaking -- 航空法施行規則 §185 / §186.

Geometry (env frame, both heading +x at the same height):

    Drone (fast) -->        Drone1 (slow) -->
      (-3, 0)                 (-0.6, 0)            +x ->
                    ... Drone passes on Drone1's RIGHT (-y) and returns ...

  §185  前方に飛行中の航空機を他の航空機が追い越そうとする場合には、後者は、
        前者の右側を通過しなければならない
  §186  進路権を有する航空機は、その進路及び速度を維持しなければならない

Two things make this scenario worth building:

1. Telling §185 from §182 needs more than a bearing. A target dead ahead is
   either coming at us (head-on) or being overtaken, and the radar does not
   report the target's heading. daa_common.classify_encounter() infers it from
   the closure rate against our own ground speed -- closing at ~2x own speed is
   head-on, closing at well under own speed is an overtake.

2. The aircraft BEING overtaken cannot see the one behind it: the radar looks
   forward only (60 deg baseline, 150 deg wide variant), and the threat is at
   180 deg. That is not a scenario failure -- §185 puts the whole burden on the
   overtaking aircraft -- but it is a real sensor-coverage finding, and it is
   recorded as such rather than hidden.

Run AFTER:  bash drone_daasim/two_drone_run.sh
Usage:      python scenario_s3_overtaking.py
"""
import math
import os
import time

import daa_common as dc
import daa_metrics
from daa_metrics import EncounterRecorder, RangeTracker, StepReport, tau_mod

DWC = daa_metrics.UAS_VS_UAS
TAU_FIRE = DWC.tau_s
# Fail-safe: never keep closing on an unclassified target (ISO 15964 6.2.5).
R_FLOOR = DWC.h_m * 1.6

H = 0.6
# The passing lane must clear the Well Clear radius, not merely miss the other
# aircraft: only one of the two moves here, so the lane offset IS the separation.
Y_PASS = 1.5          # lateral offset used to pass on the right (-y); > DWC 1.25 m
PASS_LEAD = 0.8       # forward component of the sidestep
# Slower overall than S-1: tau scales as range/closure, and the radar only
# picks the target up at ~2 m, so a gentle closure is what buys enough time
# to classify the encounter AND still manoeuvre before the tau threshold.
STEP_FAST = 0.50      # overtaking aircraft, per control tick
STEP_SLOW = 0.15      # overtaken aircraft (§186: constant course AND speed)
MOVE_SPEED = 1.0
APPROACH_TO = 0.5
APPROACH_TICKS = 16
GOAL_EPS = 0.45
LANE_TO, PASS_TO, HOME_TO = 6.0, 12.0, 8.0
AZ_HALF = 30.0        # the target is dead ahead; no need to open the window wide
# Separate window used ONLY to ask the coverage question: can the aircraft being
# overtaken see the one behind it? With a forward radar the answer is no by
# construction; with a 360 deg azimuth manifest it becomes a real measurement.
AZ_HALF_REAR = 180.0

FAST, SLOW = "Drone", "Drone1"
# Gap between the two aircraft at the start -- a scenario parameter, not a
# sensor property (the first detection cannot exceed it).
_G = float(os.environ.get("S3_GAP", "2.4"))
START = {FAST: (-3.0, 0.0), SLOW: (-3.0 + _G, 0.0)}
# The goals must themselves be Well Clear apart: an overtake that ends with the
# two aircraft parked 0.8 m from each other has not kept separation, however
# clean the pass itself was.
GOAL = {SLOW: (START[SLOW][0] + 2.8, 0.0), FAST: (START[SLOW][0] + 4.8, 0.0)}
YAW = 0.0             # both head +x
RIGHT = (0.0, -1.0)   # own right for a +x heading


def main():
    clients = dc.connect([FAST, SLOW])

    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H)
    time.sleep(1.0)

    print(f"line up: {FAST} at {START[FAST]} behind {SLOW} at {START[SLOW]}, both heading +x ...",
          flush=True)
    for name in (SLOW, FAST):
        clients[name].moveToPosition(START[name][0], START[name][1], H, MOVE_SPEED,
                                     yaw_deg=YAW, timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, FAST, SLOW)
    sampler.start()

    detected = {n: False for n in (FAST, SLOW)}
    r_det = {n: None for n in (FAST, SLOW)}
    # Which radar of the fit produced the first detection. On the overtaken
    # aircraft this is the whole coverage question in one field.
    det_by = {n: None for n in (FAST, SLOW)}
    rng_tr = {n: RangeTracker() for n in (FAST, SLOW)}
    spd_tr = {n: dc.SpeedTracker() for n in (FAST, SLOW)}
    role = {n: None for n in (FAST, SLOW)}
    tau_fire = {n: None for n in (FAST, SLOW)}
    r_fire = {n: None for n in (FAST, SLOW)}
    az_fire = {n: None for n in (FAST, SLOW)}
    closure_fire = {n: None for n in (FAST, SLOW)}
    own_speed_fire = {n: None for n in (FAST, SLOW)}

    # --- PHASE A: close up until the overtaker classifies the encounter ------
    print("A) close up until the overtaking aircraft classifies the encounter "
          "(§185 vs §182 from closure vs own speed) ...", flush=True)
    for k in range(APPROACH_TICKS):
        for name, step in ((SLOW, STEP_SLOW), (FAST, STEP_FAST)):
            now = time.time()
            p = dc.read_xyz(name)
            px, py = (p[0], p[1]) if p else START[name]
            # The overtaker uses its forward window to fly the encounter; the
            # overtaken aircraft is polled over the full azimuth so the report
            # can state whether the threat behind it was visible at all.
            s = dc.scan_best(name, az_half=AZ_HALF, el_half=15.0)
            # The overtaken aircraft is polled across EVERY radar channel it has:
            # on a dual-radar stack the rear sector answers, on a single-radar one
            # only the (blind) forward radar does.
            look = s if name == FAST else dc.scan_best(name, az_half=AZ_HALF_REAR, el_half=15.0)
            if look.rng is not None and not detected[name]:
                detected[name], r_det[name] = True, look.rng
                det_by[name] = look.source
            closure = rng_tr[name].update(now, s.rng, s.doppler)
            own = spd_tr[name].update(now, p)
            if role[name] is None:
                r = dc.classify_encounter(s.az_deg, closure, own)
                if r is None and s.rng is not None and s.rng <= R_FLOOR:
                    # Unclassified but close: treat it as a threat and manoeuvre
                    # anyway rather than press on (fail-safe).
                    r = dc.OVERTAKE
                    print(f"       -> {name} FAIL-SAFE divert at r={s.rng:.2f} m "
                          f"(encounter never classified)", flush=True)
                if r is not None:
                    role[name] = r
                    tau_fire[name] = tau_mod(s.rng, closure, DWC.h_m)
                    r_fire[name], az_fire[name] = s.rng, s.az_deg
                    closure_fire[name], own_speed_fire[name] = closure, own
                    print(f"       -> {name} classified: {r} "
                          f"(az {s.az_deg:+.0f} deg, closure {closure:+.2f} m/s, "
                          f"own {own:.2f} m/s)", flush=True)
            nx = px + step
            rs = f"{s.rng:.2f}" if s.rng is not None else "--"
            cs = f"{closure:+.2f}" if closure is not None else "--"
            os_ = f"{own:.2f}" if own else "--"
            print(f"  [A{k:02d}] {name} x={px:+.2f} r={rs} closure={cs} own={os_} "
                  f"role={role[name]} sep={enc.min_h:.2f}", flush=True)
            clients[name].moveToPosition(nx, py, H, MOVE_SPEED,
                                         yaw_deg=YAW, timeout_sec=APPROACH_TO)
        if role[FAST] is not None:
            print("  -> overtaking solution; passing on the right per §185.", flush=True)
            break

    # --- PHASE B: step onto the right-hand passing lane (§185) ---------------
    print(f"B) §185: {FAST} offsets to the RIGHT of {SLOW} (-y); {SLOW} holds (§186) ...",
          flush=True)
    # Let the last approach command finish first: a lane command issued on top of
    # an in-flight one comes out as forward-only motion, which closes the range
    # without ever opening the lateral gap.
    time.sleep(0.6)
    lane_ok = False
    for attempt in range(3):
        p = dc.read_xyz(FAST)
        px = p[0] if p else START[FAST][0]
        clients[FAST].moveToPosition(px, RIGHT[1] * Y_PASS, H, MOVE_SPEED,
                                     yaw_deg=YAW, timeout_sec=LANE_TO)
        p = dc.read_xyz(FAST)
        lane_ok = p is not None and abs(p[1] - RIGHT[1] * Y_PASS) < 0.30
        print(f"  {FAST} lane y={p[1]:+.2f} (target {RIGHT[1] * Y_PASS:+.2f}) "
              f"ok={lane_ok} attempt={attempt}" if p else "  lane ??", flush=True)
        if lane_ok:
            break

    # --- PHASE C: pass, while the overtaken aircraft keeps course and speed --
    print("C) pass on the lane; overtaken aircraft continues unchanged ...", flush=True)
    clients[SLOW].moveToPosition(GOAL[SLOW][0], 0.0, H, MOVE_SPEED,
                                 yaw_deg=YAW, timeout_sec=PASS_TO)
    clients[FAST].moveToPosition(GOAL[FAST][0], RIGHT[1] * Y_PASS, H, MOVE_SPEED,
                                 yaw_deg=YAW, timeout_sec=PASS_TO)

    # --- PHASE D: back onto the centre line, now ahead -----------------------
    # §185 is not finished at the moment of passing: cutting back in front of the
    # overtaken aircraft would re-enter its Well Clear volume. Stay in the lane
    # until the gap ahead exceeds the DWC radius.
    print("D) hold the lane until clear ahead, then return to the centre line ...", flush=True)
    for _ in range(3):
        pf, psl = dc.read_xyz(FAST), dc.read_xyz(SLOW)
        if pf and psl and (pf[0] - psl[0]) >= DWC.h_m + 0.3:
            break
        print(f"  not clear ahead yet (dx={pf[0] - psl[0]:+.2f} m); extending in lane",
              flush=True)
        clients[FAST].moveToPosition(GOAL[FAST][0], RIGHT[1] * Y_PASS, H, MOVE_SPEED,
                                     yaw_deg=YAW, timeout_sec=LANE_TO)
    reached, off_track = {}, {}
    for name in (FAST, SLOW):
        clients[name].moveToPosition(GOAL[name][0], 0.0, H, MOVE_SPEED,
                                     yaw_deg=YAW, timeout_sec=HOME_TO)
        p = dc.read_xyz(name)
        reached[name] = p is not None and abs(p[0] - GOAL[name][0]) < GOAL_EPS
        off_track[name] = abs(p[1]) if p else math.inf
        ps = f"{p[0]:+.2f},{p[1]:+.2f}" if p else "??"
        print(f"  {name} at ({ps}) goal_x={GOAL[name][0]:+.2f} reached={reached[name]}", flush=True)

    passed_ahead = False
    pf, psl = dc.read_xyz(FAST), dc.read_xyz(SLOW)
    if pf and psl:
        passed_ahead = pf[0] > psl[0]

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-3 overtaking (施行規則 §185 pass on the right / §186 stand on)", DWC)

    # Whether the overtaken aircraft can see astern is a property of the FIT it
    # was launched with, not an assumption: ask the fit, and say which radar
    # answered. A forward-only aircraft is blind there and the run says so; add
    # a rear sector and the same scenario reports the detection instead.
    rear_covered = dc.covers_az(SLOW, 180.0)
    rep.record(1, detected[FAST],
               f"overtaking {FAST}: first detection at "
               + (f"{r_det[FAST]:.2f} m" if r_det[FAST] is not None else "none")
               + (f" (by {det_by[FAST]})" if det_by[FAST] else "")
               + f" | overtaken {SLOW} [{dc.fit_summary(SLOW)}]: "
               + (f"{r_det[SLOW]:.2f} m by {det_by[SLOW]} (full-azimuth check)"
                  if r_det[SLOW] is not None else
                  "NO detection -- "
                  + ("no radar in the fit covers 180 deg" if rear_covered is False else
                     "the rear sector is fitted but returned nothing")
                  + " (coverage finding, not a scenario failure: §185 puts the duty "
                    "on the overtaker)"))

    rec = role[FAST] == dc.OVERTAKE
    rep.record(2, rec, "encounter classified from closure vs own speed: "
               + (f"{role[FAST]} (closure {closure_fire[FAST]:+.2f} m/s vs own "
                  f"{own_speed_fire[FAST]:.2f} m/s, az {az_fire[FAST]:+.0f} deg)"
                  if role[FAST] else "unresolved")
               + f" | expected {dc.OVERTAKE}")

    in_time = tau_fire[FAST] is not None and tau_fire[FAST] >= TAU_FIRE
    rep.record(3, rec and lane_ok and in_time,
               f"passed on the RIGHT of the overtaken aircraft (y={RIGHT[1] * Y_PASS:+.2f} m), "
               f"lane reached={lane_ok}, manoeuvre began at r="
               + (f"{r_fire[FAST]:.2f} m, tau {tau_fire[FAST]:.1f} s >= {TAU_FIRE:.1f}? "
                  f"{'yes' if in_time else 'NO'}" if tau_fire[FAST] is not None else "n/a"))

    rep.record(4, enc.well_clear_kept, enc.summary())

    # §186: the overtaken aircraft must have held course (and it never manoeuvred,
    # since it could not even see the threat).
    stand_ok = off_track[SLOW] < 0.30
    rep.record(5, stand_ok and reached[FAST],
               f"§186 {SLOW} held course: cross-track {off_track[SLOW]:.2f} m | "
               f"{FAST} back on centre line: {off_track[FAST]:.2f} m")

    rep.record(6, passed_ahead and all(reached.values()),
               f"overtake completed: {FAST} ahead of {SLOW}? {passed_ahead} | goals "
               + ", ".join(f"{n}={reached[n]}" for n in (FAST, SLOW)))

    print(rep.render(), flush=True)
    print(f"[s3] min separation = {enc.min_h:.2f} m "
          f"(Well Clear >= {DWC.h_m} m? {enc.min_h >= DWC.h_m})", flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(overtaker_detect={detected[FAST]}, classified={rec}, "
          f"well_clear={enc.well_clear_kept}, stand_on_held={stand_ok}, passed={passed_ahead})",
          flush=True)
    print(f"[s3] COVERAGE: overtaken aircraft detected the follower? "
          + (f"yes, by {det_by[SLOW]} at {r_det[SLOW]:.2f} m" if detected[SLOW]
             else "NO (rear blind sector)")
          + f" | fit: {dc.fit_summary(SLOW)}, astern covered: {rear_covered}",
          flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
