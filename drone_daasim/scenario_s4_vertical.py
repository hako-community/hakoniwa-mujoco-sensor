#!/usr/bin/env python3
"""S-4 vertical avoidance -- ISO 15964 3.9 (climb / descent as manoeuvres).

S-1 solved a head-on encounter in the horizontal plane. This one solves the same
encounter in the vertical, and in doing so exercises the part of the Well Clear
definition that the horizontal scenarios never touch: loss of Well Clear needs
horizontal AND vertical AND tau to be violated together (DO-365 form). Two
aircraft may therefore pass directly over one another -- horizontal separation
near zero -- and still be well clear, provided the vertical gap holds.

That is the result this scenario is built to show:

    horizontal separation  ->  small (they cross the same ground track)
    vertical separation    ->  >= 0.30 m (the crewed/UA vertical DWC)
    LoDWC                  ->  no

Rule basis: 施行規則 §182 tells head-on traffic to turn right but says nothing
about the vertical; ISO 15964 3.9 lists climb and descent among the manoeuvres a
DAA system may use. The split is made deterministic the same way the horizontal
one is -- by a fixed convention, here "the aircraft heading +x climbs".

Run AFTER:  bash drone_daasim/two_drone_run.sh noground
Usage:      python scenario_s4_vertical.py
"""
import math
import os
import time

import daa_common as dc
import daa_metrics
from daa_metrics import EncounterRecorder, RangeTracker, StepReport, tau_mod

DWC = daa_metrics.UAS_VS_UAS          # h=1.25 m, v=0.30 m, tau=3.5 s
TAU_FIRE = DWC.tau_s
R_FLOOR = DWC.h_m * 1.6

# Cruise altitude sits high enough that one aircraft can descend a metre without
# reaching the ground.
H_CRUISE = 2.0
DZ = 1.2                              # vertical manoeuvre, each aircraft
GOAL_EPS = 0.45
STEP = 0.45
MOVE_SPEED = 1.0
APPROACH_TO = 0.5
APPROACH_TICKS = 16
CLIMB_TO, CROSS_TO, HOME_TO = 8.0, 12.0, 10.0
AZ_HALF, EL_HALF = 20.0, 15.0

_S = float(os.environ.get("S4_START", "6.0"))
# climb=+1 means this aircraft goes up; the convention keys off the heading so
# both aircraft reach the same decision independently.
DRONES = {
    "Drone":  dict(face=+1, climb=+1, start=-_S, goal=+(_S + 0.2), yaw=0.0),
    "Drone1": dict(face=-1, climb=-1, start=+_S, goal=-(_S + 0.2), yaw=180.0),
}


def main():
    clients = dc.connect(DRONES)

    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H_CRUISE)
    time.sleep(1.0)

    print(f"line up head-on at z={H_CRUISE} m ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["start"], 0.0, H_CRUISE, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, "Drone", "Drone1")
    sampler.start()

    detected = {n: False for n in DRONES}
    r_det = {n: None for n in DRONES}
    recognised = {n: False for n in DRONES}
    tau_fire = {n: None for n in DRONES}
    r_fire = {n: None for n in DRONES}
    el_fire = {n: None for n in DRONES}
    track = {n: RangeTracker() for n in DRONES}

    print("A) close head-on until both recognise the other ...", flush=True)
    for k in range(APPROACH_TICKS):
        for name, d in DRONES.items():
            p = dc.read_xyz(name)
            px = p[0] if p else d["start"]
            s = dc.scan(name, az_half=AZ_HALF, el_half=EL_HALF)
            if s.rng is not None and not detected[name]:
                detected[name], r_det[name] = True, s.rng
            closure = track[name].update(time.time(), s.rng, s.doppler)
            if s.rng is not None and closure is not None:
                recognised[name] = True
            t = tau_mod(s.rng, closure, DWC.h_m) if s.rng is not None else math.inf
            if tau_fire[name] is None and s.rng is not None:
                if recognised[name] and closure is not None and closure > 0.0:
                    tau_fire[name], r_fire[name], el_fire[name] = t, s.rng, s.el_deg
                elif s.rng <= R_FLOOR:
                    tau_fire[name], r_fire[name], el_fire[name] = t, s.rng, s.el_deg
            rs = f"{s.rng:.2f}" if s.rng is not None else "--"
            es = f"{s.el_deg:+.0f}" if s.el_deg is not None else "--"
            ts = f"{t:.1f}" if t != math.inf else "inf"
            print(f"  [A{k:02d}] {name} x={px:+.2f} r={rs} el={es} tau={ts} "
                  f"fire={tau_fire[name] is not None} h_sep={enc.min_h:.2f}", flush=True)
            clients[name].moveToPosition(px + d["face"] * STEP, 0.0, H_CRUISE, MOVE_SPEED,
                                         yaw_deg=d["yaw"], timeout_sec=APPROACH_TO)
        if all(tau_fire[n] is not None for n in DRONES):
            print("  -> both have a firing solution; separating vertically.", flush=True)
            break

    # --- PHASE B: vertical split --------------------------------------------
    print(f"B) vertical manoeuvre: Drone climbs +{DZ} m, Drone1 descends -{DZ} m ...", flush=True)
    time.sleep(0.6)
    z_target = {n: H_CRUISE + d["climb"] * DZ for n, d in DRONES.items()}
    level_ok = {}
    for attempt in range(2):
        for name, d in DRONES.items():
            if level_ok.get(name):
                continue
            p = dc.read_xyz(name)
            px = p[0] if p else d["start"]
            clients[name].moveToPosition(px, 0.0, z_target[name], MOVE_SPEED,
                                         yaw_deg=d["yaw"], timeout_sec=CLIMB_TO)
            p = dc.read_xyz(name)
            level_ok[name] = p is not None and abs(p[2] - z_target[name]) < 0.35
            print(f"  {name} z={p[2]:+.2f} (target {z_target[name]:+.2f}) "
                  f"ok={level_ok[name]} attempt={attempt}" if p else "  z ??", flush=True)
    vertical_split = all(level_ok.values())

    # --- PHASE C: cross while vertically separated ---------------------------
    print("C) cross on separated levels (same ground track) ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"], 0.0, z_target[name], MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=CROSS_TO)

    # --- PHASE D: back to the cruise level -----------------------------------
    print("D) return to the cruise level ...", flush=True)
    reached, z_err = {}, {}
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"], 0.0, H_CRUISE, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
        p = dc.read_xyz(name)
        reached[name] = p is not None and abs(p[0] - d["goal"]) < GOAL_EPS
        z_err[name] = abs(p[2] - H_CRUISE) if p else math.inf
        ps = f"{p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f}" if p else "??"
        print(f"  {name} at ({ps}) goal_x={d['goal']:+.2f} reached={reached[name]}", flush=True)

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-4 vertical avoidance (ISO 15964 3.9: climb / descent)", DWC)

    det = all(detected.values())
    rep.record(1, det, "first detection at " + ", ".join(
        f"{n}={r_det[n]:.2f} m" if r_det[n] is not None else f"{n}=none" for n in DRONES))

    rec = all(recognised.values())
    rep.record(2, rec, "range + closure + elevation for " + ", ".join(
        f"{n}={'yes' if recognised[n] else 'no'} (el {el_fire[n]:+.0f} deg)"
        if el_fire[n] is not None else f"{n}=no" for n in DRONES))

    in_time = all(tau_fire[n] is not None and tau_fire[n] >= TAU_FIRE for n in DRONES)
    rep.record(3, in_time and vertical_split, "vertical split reached: " + ", ".join(
        f"{n}->{z_target[n]:+.2f} m {'ok' if level_ok.get(n) else 'NO'}" for n in DRONES)
        + " | start " + ", ".join(
        f"{n}=(r {r_fire[n]:.2f} m, tau {tau_fire[n]:.1f} s >= {TAU_FIRE:.1f}? "
        f"{'yes' if tau_fire[n] >= TAU_FIRE else 'NO'})"
        if tau_fire[n] is not None else f"{n}=never" for n in DRONES))

    # The whole point: horizontal separation is allowed to collapse here, because
    # the vertical gap is what keeps the pair well clear.
    vert_ok = enc.v_at_cpa >= DWC.v_m
    rep.record(4, enc.well_clear_kept and vert_ok,
               enc.summary() + f" | vertical gap at CPA {enc.v_at_cpa:.2f} m "
               f">= {DWC.v_m:.2f} m? {'yes' if vert_ok else 'NO'}"
               + (" -- horizontal separation collapsed as intended; the vertical gap "
                  "is what kept them clear" if enc.min_h < DWC.h_m else ""))

    ret = all(reached.values())
    rep.record(5, ret and all(z < 0.35 for z in z_err.values()),
               "back at cruise level: " + ", ".join(f"{n}={z_err[n]:.2f} m off" for n in DRONES))

    rep.record(6, ret, "goals reached: " + ", ".join(f"{n}={reached[n]}" for n in DRONES))

    print(rep.render(), flush=True)
    print(f"[s4] CPA horizontal {enc.min_h:.2f} m / vertical at CPA {enc.v_at_cpa:.2f} m "
          f"(min vertical seen {enc.min_v:.2f} m)", flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(detect={det}, recognise={rec}, split={vertical_split}, "
          f"well_clear={enc.well_clear_kept}, vertical_gap_ok={vert_ok})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
