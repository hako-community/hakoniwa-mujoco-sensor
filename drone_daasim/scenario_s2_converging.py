#!/usr/bin/env python3
"""S-2 converging (90 deg crossing) -- 航空法施行規則 §181 / §186.

Geometry (env frame, both at the same height):

        Drone1  goal (0,+2.2)
                  ^  +y
                  |
    Drone ------->+------->  goal (+2.2, 0)   Drone flies +x from (-2, 0)
        (-2,0)    |
                  |
               (0,-2)  Drone1 flies +y

Why this scenario is the interesting one: unlike S-1 the two aircraft must
behave DIFFERENTLY, and each one works that out from its own radar alone.

  §181  飛行中の同順位の無人航空機相互間にあつては、他の無人航空機を右側に
        見る航空機が進路を譲らなければならない
  §186  進路権を有する航空機は、その進路及び速度を維持しなければならない

Radar bearing decides the roles (daa_common.role_from_bearing):
  Drone  heads +x and sees Drone1 at about -45 deg (its RIGHT) -> GIVE WAY
  Drone1 heads +y and sees Drone  at about +45 deg (its LEFT)  -> STAND ON

The give-way manoeuvre is a lateral offset to own right, held until the other
aircraft has passed, then the route is resumed -- the same phase structure that
S-1 proved, so the new part really is only the asymmetry.

Run AFTER:  bash drone_daasim/two_drone_run.sh
Usage:      python scenario_s2_converging.py
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

H = 0.6
# Only the give-way aircraft moves here, so this offset IS the separation --
# it has to clear the Well Clear radius, not merely miss the other aircraft.
Y_AVOID = 1.5        # lateral offset of the give-way manoeuvre; > DWC 1.25 m
STEP = 0.35          # gentler than S-1: a slower closure buys tau at the same range
MOVE_SPEED = 1.0
APPROACH_TO = 0.5
APPROACH_TICKS = 14
GOAL_EPS = 0.40
LANE_TO, CROSS_TO, HOME_TO = 6.0, 10.0, 8.0
# The target sits near +/-45 deg on a 90 deg collision course, so the search
# window must be opened well past the +/-15 deg used for the head-on case.
# Needs the wide manifest: A2_MANIFEST=config/a2/drone-a2-sensors-wide.json
AZ_HALF = 75.0

# track = unit ground track; start/goal on that track. yaw follows the track.
# Distance from the crossing point at which each aircraft starts. This is a
# SCENARIO parameter: the first detection can never exceed the initial spacing.
_S = float(os.environ.get("S2_START", "2.0"))
DRONES = {
    "Drone":  dict(track=(1.0, 0.0), start=(-_S, 0.0), goal=(+(_S + 0.2), 0.0)),
    "Drone1": dict(track=(0.0, 1.0), start=(0.0, -_S), goal=(0.0, +(_S + 0.2))),
}
CROSSING = (0.0, 0.0)   # where the two tracks intersect

for _d in DRONES.values():
    _d["yaw"] = dc.yaw_for_heading(*_d["track"])
    # own right = track rotated -90 deg
    _d["right"] = (_d["track"][1], -_d["track"][0])


def main():
    clients = dc.connect(DRONES)

    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H)
    time.sleep(1.0)

    print("set up the crossing: Drone -> (-2,0) hdg +x, Drone1 -> (0,-2) hdg +y ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["start"][0], d["start"][1], H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, "Drone", "Drone1")
    sampler.start()

    detected = {n: False for n in DRONES}
    r_det = {n: None for n in DRONES}
    recognised = {n: False for n in DRONES}
    role = {n: None for n in DRONES}
    az_at_role = {n: None for n in DRONES}
    tau_fire = {n: None for n in DRONES}
    r_fire = {n: None for n in DRONES}
    track = {n: RangeTracker() for n in DRONES}

    # --- PHASE A: converge until BOTH have assigned themselves a rule --------
    print("A) converge until each aircraft resolves its role from radar bearing ...", flush=True)
    for k in range(APPROACH_TICKS):
        for name, d in DRONES.items():
            p = dc.read_xyz(name)
            px, py = (p[0], p[1]) if p else d["start"]
            s = dc.scan(name, az_half=AZ_HALF, el_half=15.0)
            if s.rng is not None and not detected[name]:
                detected[name], r_det[name] = True, s.rng
            closure = track[name].update(time.time(), s.rng, s.doppler)
            if s.rng is not None and closure is not None:
                recognised[name] = True
            t = tau_mod(s.rng, closure, DWC.h_m) if s.rng is not None else math.inf
            if role[name] is None and recognised[name] and closure is not None and closure > 0.0:
                role[name] = dc.role_from_bearing(s.az_deg, closing=True)
                az_at_role[name] = s.az_deg
                tau_fire[name], r_fire[name] = t, s.rng
                print(f"       -> {name} role = {role[name]} (bearing {s.az_deg:+.0f} deg)", flush=True)
            nx, ny = px + d["track"][0] * STEP, py + d["track"][1] * STEP
            azs = f"{s.az_deg:+.0f}" if s.az_deg is not None else "--"
            rs = f"{s.rng:.2f}" if s.rng is not None else "--"
            print(f"  [A{k:02d}] {name} p=({px:+.2f},{py:+.2f}) r={rs} az={azs} "
                  f"role={role[name]} sep={enc.min_h:.2f}", flush=True)
            clients[name].moveToPosition(nx, ny, H, MOVE_SPEED,
                                         yaw_deg=d["yaw"], timeout_sec=APPROACH_TO)
        if all(role[n] is not None for n in DRONES):
            print("  -> both roles resolved; applying §181.", flush=True)
            break

    giver = [n for n in DRONES if role[n] == dc.GIVE_WAY]
    stander = [n for n in DRONES if role[n] == dc.STAND_ON]

    # --- PHASE B: the give-way aircraft stands clear of the other's track ----
    # Turning right is not enough in a crossing encounter. The two tracks are
    # perpendicular, so a purely lateral step does not increase the distance to
    # the OTHER aircraft's track -- that distance is set by how far along its own
    # track the give-way aircraft has already come. It therefore holds short of
    # the crossing point by more than the Well Clear radius, offset to its own
    # right, and waits: hovering and deceleration are manoeuvres in their own
    # right (ISO 15964 3.9), and this is what "passing astern" reduces to when
    # the aircraft can stop.
    print(f"B) §181 give way: {giver} holds clear of the crossing; "
          f"§186 stand on: {stander} continues ...", flush=True)
    stand_x0 = {n: dc.read_xyz(n) for n in stander}
    time.sleep(0.6)   # let the last approach command finish before overriding it
    hold_back = DWC.h_m + 0.4
    for name in giver:
        d = DRONES[name]
        tx = CROSSING[0] - d["track"][0] * hold_back + d["right"][0] * Y_AVOID
        ty = CROSSING[1] - d["track"][1] * hold_back + d["right"][1] * Y_AVOID
        print(f"  {name} holds at ({tx:+.2f},{ty:+.2f}) "
              f"= {hold_back:.2f} m short of the crossing, {Y_AVOID:.2f} m to its right",
              flush=True)
        clients[name].moveToPosition(tx, ty, H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=LANE_TO)
    for name in stander:
        d = DRONES[name]
        clients[name].moveToPosition(d["goal"][0], d["goal"][1], H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=CROSS_TO)

    # --- PHASE C: resume only once the stand-on aircraft is past -------------
    print("C) give-way aircraft waits for the crossing to clear, then resumes ...", flush=True)
    for _ in range(20):
        clear = True
        for name in stander:
            p, d = dc.read_xyz(name), DRONES[name]
            along = (p[0] * d["track"][0] + p[1] * d["track"][1]) if p else -9.9
            clear = clear and along >= hold_back
        if clear:
            break
        time.sleep(0.3)
    print(f"  crossing clear; {giver} resumes", flush=True)
    for name in giver:
        d = DRONES[name]
        clients[name].moveToPosition(d["goal"][0], d["goal"][1], H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=CROSS_TO)

    # --- PHASE D: settle on the goal -----------------------------------------
    reached, off_track = {}, {}
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"][0], d["goal"][1], H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
        p = dc.read_xyz(name)
        if p:
            # cross-track error relative to the intended straight line
            cx = abs(p[0] * d["track"][1] - p[1] * d["track"][0])
            reached[name] = math.hypot(p[0] - d["goal"][0], p[1] - d["goal"][1]) < GOAL_EPS
        else:
            cx, reached[name] = math.inf, False
        off_track[name] = cx
        ps = f"{p[0]:+.2f},{p[1]:+.2f}" if p else "??"
        print(f"  {name} at ({ps}) goal=({d['goal'][0]:+.2f},{d['goal'][1]:+.2f}) "
              f"reached={reached[name]}", flush=True)

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-2 converging 90deg (施行規則 §181 give way / §186 stand on)", DWC)

    det = all(detected.values())
    rep.record(1, det, "first detection at " + ", ".join(
        f"{n}={r_det[n]:.2f} m" if r_det[n] is not None else f"{n}=none" for n in DRONES))

    rec = all(recognised.values())
    rep.record(2, rec, "range + closure + bearing for " + ", ".join(
        f"{n}={'yes' if recognised[n] else 'no'}" for n in DRONES)
        + "  (target classification / trajectory prediction: not implemented)")

    # Rule compliance is the point of S-2: exactly one give-way, one stand-on,
    # and each role has to follow from the measured bearing.
    roles_ok = len(giver) == 1 and len(stander) == 1
    # The tau deadline applies to the aircraft that has to ACT. §186 forbids the
    # stand-on aircraft from manoeuvring at all, so scoring it on "did you start
    # your manoeuvre in time" would be self-contradictory; its own resolution
    # time is still printed, just not gated on.
    in_time = all(tau_fire[n] is not None and tau_fire[n] >= TAU_FIRE for n in giver)
    rep.record(3, roles_ok and in_time, "roles from bearing: " + ", ".join(
        f"{n}={role[n]} @ az {az_at_role[n]:+.0f} deg, tau {tau_fire[n]:.1f} s "
        f">= {TAU_FIRE:.1f}? {'yes' if tau_fire[n] and tau_fire[n] >= TAU_FIRE else 'NO'}"
        if role[n] else f"{n}=unresolved" for n in DRONES))

    rep.record(4, enc.well_clear_kept, enc.summary())

    # §186: the stand-on aircraft must have held its course.
    stand_ok = True
    stand_note = []
    for n in stander:
        p0, p1 = stand_x0.get(n), dc.read_xyz(n)
        d = DRONES[n]
        dev = abs(p1[0] * d["track"][1] - p1[1] * d["track"][0]) if p1 else math.inf
        stand_ok = stand_ok and dev < 0.30
        stand_note.append(f"{n} cross-track {dev:.2f} m")
    rep.record(5, all(reached.values()) and stand_ok,
               "§186 stand-on held course: " + (", ".join(stand_note) or "n/a")
               + " | back on route: " + ", ".join(f"{n}={off_track[n]:.2f} m" for n in DRONES))

    rep.record(6, all(reached.values()),
               "goals reached: " + ", ".join(f"{n}={reached[n]}" for n in DRONES))

    print(rep.render(), flush=True)
    print(f"[s2] min separation = {enc.min_h:.2f} m "
          f"(Well Clear >= {DWC.h_m} m? {enc.min_h >= DWC.h_m})", flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(detect={det}, recognise={rec}, roles={roles_ok}, "
          f"well_clear={enc.well_clear_kept}, stand_on_held={stand_ok})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
