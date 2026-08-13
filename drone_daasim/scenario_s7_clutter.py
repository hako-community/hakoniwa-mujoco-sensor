#!/usr/bin/env python3
"""S-7 static + dynamic objects together -- ISO 15964 4.6.

  4.6  The detection and avoidance system shall include static objects and
       dynamic objects. Static objects refer to objects that are stationary to
       the ground, such as buildings, trees, towers, poles, and wires. Dynamic
       objects include birds and aircraft.

Every scenario so far ran over an empty field, where the only radar return was
the other aircraft. This one runs inside the walled room (floor, four walls and
a pillar) and asks the question that makes a real radar hard: with the aircraft
buried in wall clutter, can it still be found and avoided?

The discriminator is Doppler. Structure that is stationary with respect to the
ground returns ~0 Doppler; an approaching aircraft does not. daa_common.scan()
takes `min_abs_doppler`, which is the classic moving-target filter, and it is
precisely the problem the JRC sea trials describe: "the received signal contains
reflections from waves, terrain and buildings besides the threat".

The run reports BOTH views of the same radar frame:
  unfiltered -> the nearest return, whatever it is (usually a wall)
  filtered   -> the nearest MOVING return (the other aircraft)
and checks the filtered range against the true inter-aircraft distance, so the
claim "we found the aircraft, not a wall" is measured rather than asserted.

Run AFTER:  bash drone_daasim/two_drone_run.sh room
Usage:      python scenario_s7_clutter.py
"""
import math
import os
import time

import daa_common as dc
import daa_metrics
from daa_metrics import EncounterRecorder, RangeTracker, StepReport, tau_mod

DWC = daa_metrics.UAS_VS_UAS
TAU_FIRE = DWC.tau_s
R_FLOOR = DWC.h_m * 1.6

# The room is 10 x 8 m with a pillar, so the encounter has to stay compact.
H = 0.8
Y_AVOID = 1.0
STEP = 0.4
MOVE_SPEED = 1.0
APPROACH_TO = 0.5
APPROACH_TICKS = 14
GOAL_EPS = 0.45
LANE_TO, CROSS_TO, HOME_TO = 6.0, 10.0, 8.0
AZ_HALF, EL_HALF = 15.0, 15.0
# Doppler magnitude above which a return counts as a moving target. The aircraft
# close at a few tenths of a m/s, walls sit at 0.
MIN_DOPPLER = 0.10
# Keep clear of the room's walls (they are at x = +/-4 m in the env frame).
WALL_X = 4.0
WALL_CLEARANCE = 0.6

_S = float(os.environ.get("S7_START", "2.6"))
DRONES = {
    "Drone":  dict(face=+1, veer=-1, start=-_S, goal=+(_S + 0.2), yaw=0.0),
    "Drone1": dict(face=-1, veer=+1, start=+_S, goal=-(_S + 0.2), yaw=180.0),
}


def main():
    clients = dc.connect(DRONES)

    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H)
    time.sleep(1.0)

    print("line up head-on inside the walled room ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["start"], 0.0, H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, "Drone", "Drone1")
    sampler.start()

    detected = {n: False for n in DRONES}
    r_det = {n: None for n in DRONES}
    tau_fire = {n: None for n in DRONES}
    r_fire = {n: None for n in DRONES}
    track = {n: RangeTracker() for n in DRONES}
    # Evidence that the filter is doing something: how often the unfiltered
    # nearest return was NOT the aircraft.
    clutter_frames = {n: 0 for n in DRONES}
    total_frames = {n: 0 for n in DRONES}
    range_err = {n: [] for n in DRONES}
    # Returns seen vs returns that survived the moving-target filter. This is the
    # direct measure of how much static structure the radar was looking at --
    # independent of whether a wall happened to be nearer than the aircraft.
    raw_hits = {n: 0 for n in DRONES}
    mov_hits = {n: 0 for n in DRONES}

    print(f"A) close head-on inside clutter; moving-target filter at "
          f"|doppler| >= {MIN_DOPPLER} m/s ...", flush=True)
    for k in range(APPROACH_TICKS):
        for name, d in DRONES.items():
            other = "Drone1" if name == "Drone" else "Drone"
            p, po = dc.read_xyz(name), dc.read_xyz(other)
            px = p[0] if p else d["start"]
            true_sep = math.dist(p, po) if (p and po) else None

            raw = dc.scan_best(name, az_half=AZ_HALF, el_half=EL_HALF)
            mov = dc.scan_best(name, az_half=AZ_HALF, el_half=EL_HALF,
                               min_abs_doppler=MIN_DOPPLER)
            total_frames[name] += 1
            raw_hits[name] += raw.count
            mov_hits[name] += mov.count
            # "the nearest thing in front of us is not the aircraft"
            if (raw.rng is not None and true_sep is not None
                    and abs(raw.rng - true_sep) > 0.5):
                clutter_frames[name] += 1
            if mov.rng is not None and true_sep is not None:
                range_err[name].append(abs(mov.rng - true_sep))

            if mov.rng is not None and not detected[name]:
                detected[name], r_det[name] = True, mov.rng
            closure = track[name].update(time.time(), mov.rng, mov.doppler)
            t = tau_mod(mov.rng, closure, DWC.h_m) if mov.rng is not None else math.inf
            if tau_fire[name] is None and mov.rng is not None:
                if closure is not None and closure > 0.0:
                    tau_fire[name], r_fire[name] = t, mov.rng
                elif mov.rng <= R_FLOOR:
                    tau_fire[name], r_fire[name] = t, mov.rng

            rr = f"{raw.rng:.2f}" if raw.rng is not None else "--"
            mr = f"{mov.rng:.2f}" if mov.rng is not None else "--"
            ts = f"{true_sep:.2f}" if true_sep is not None else "--"
            print(f"  [A{k:02d}] {name} x={px:+.2f} nearest={rr}({raw.count} hits) "
                  f"moving={mr}({mov.count} hits) true={ts} "
                  f"fire={tau_fire[name] is not None} sep={enc.min_h:.2f}", flush=True)
            clients[name].moveToPosition(px + d["face"] * STEP, 0.0, H, MOVE_SPEED,
                                         yaw_deg=d["yaw"], timeout_sec=APPROACH_TO)
        if all(tau_fire[n] is not None for n in DRONES):
            print("  -> both aircraft found the MOVING target; diverting (§182).", flush=True)
            break

    # --- PHASE B: sidestep, staying clear of the walls -----------------------
    print("B) sidestep onto own-right lanes, inside the room ...", flush=True)
    time.sleep(0.6)
    lane_ok = {}
    for attempt in range(2):
        for name, d in DRONES.items():
            if lane_ok.get(name):
                continue
            p = dc.read_xyz(name)
            px = p[0] if p else d["start"]
            clients[name].moveToPosition(px, d["veer"] * Y_AVOID, H, MOVE_SPEED,
                                         yaw_deg=d["yaw"], timeout_sec=LANE_TO)
            p = dc.read_xyz(name)
            lane_ok[name] = p is not None and abs(abs(p[1]) - Y_AVOID) < 0.3
            print(f"  {name} lane y={p[1]:+.2f} ok={lane_ok[name]}" if p else "  ??",
                  flush=True)

    # --- PHASE C / D ---------------------------------------------------------
    print("C) cross on separated lanes ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"], d["veer"] * Y_AVOID, H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=CROSS_TO)
    print("D) recenter ...", flush=True)
    reached, wall_ok = {}, True
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"], 0.0, H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
        p = dc.read_xyz(name)
        reached[name] = p is not None and abs(p[0] - d["goal"]) < GOAL_EPS
        if p:
            wall_ok = wall_ok and abs(p[0]) <= WALL_X - WALL_CLEARANCE
        ps = f"{p[0]:+.2f},{p[1]:+.2f}" if p else "??"
        print(f"  {name} at ({ps}) reached={reached[name]}", flush=True)

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-7 static + dynamic objects (ISO 15964 4.6, walled room)", DWC)

    det = all(detected.values())
    clutter_pct = {n: (100.0 * clutter_frames[n] / max(1, total_frames[n])) for n in DRONES}
    rejected = {n: (100.0 * (raw_hits[n] - mov_hits[n]) / raw_hits[n]) if raw_hits[n] else 0.0
                for n in DRONES}
    rep.record(1, det, "first MOVING-target detection at " + ", ".join(
        f"{n}={r_det[n]:.2f} m" if r_det[n] is not None else f"{n}=none" for n in DRONES)
        + " | returns rejected as static clutter: " + ", ".join(
        f"{n}={raw_hits[n] - mov_hits[n]}/{raw_hits[n]} ({rejected[n]:.0f}%)" for n in DRONES)
        + " | frames whose nearest unfiltered return was NOT the aircraft: "
        + ", ".join(f"{n}={clutter_pct[n]:.0f}%" for n in DRONES))

    # The filtered range must agree with the truth, otherwise we locked onto a wall.
    worst = {n: (max(range_err[n]) if range_err[n] else math.inf) for n in DRONES}
    rec = all(worst[n] < 0.8 for n in DRONES)
    rep.record(2, rec, "filtered range vs true separation, worst error: " + ", ".join(
        f"{n}={worst[n]:.2f} m" if worst[n] != math.inf else f"{n}=no samples"
        for n in DRONES) + "  (< 0.8 m means the track is the aircraft, not a wall)")

    in_time = all(tau_fire[n] is not None and tau_fire[n] >= TAU_FIRE for n in DRONES)
    rep.record(3, in_time and all(lane_ok.values()),
               "manoeuvre start " + ", ".join(
                   f"{n}=(r {r_fire[n]:.2f} m, tau {tau_fire[n]:.1f} s >= {TAU_FIRE:.1f}? "
                   f"{'yes' if tau_fire[n] >= TAU_FIRE else 'NO'})"
                   if tau_fire[n] is not None else f"{n}=never" for n in DRONES)
               + f" | lanes reached: {all(lane_ok.values())}")

    rep.record(4, enc.well_clear_kept, enc.summary())

    rep.record(5, wall_ok,
               f"stayed clear of the static structure: |x| <= {WALL_X - WALL_CLEARANCE} m "
               f"for both aircraft = {wall_ok}")

    rep.record(6, all(reached.values()),
               "goals reached: " + ", ".join(f"{n}={reached[n]}" for n in DRONES))

    print(rep.render(), flush=True)
    print(f"[s7] min separation = {enc.min_h:.2f} m "
          f"(Well Clear >= {DWC.h_m} m? {enc.min_h >= DWC.h_m})", flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(moving_target_detect={det}, track_is_aircraft={rec}, "
          f"well_clear={enc.well_clear_kept}, clear_of_structure={wall_ok})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
