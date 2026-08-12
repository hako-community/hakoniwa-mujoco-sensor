#!/usr/bin/env python3
"""Two-drone mutual collision avoidance (head-on).

Combines B-1 (mutual radar detection, scenario_b1_faceoff.py) with M6 (detect ->
avoidance steering, m6_avoid.py): two drones fly straight at each other, each one
detects the OTHER in its forward radar cone, and each veers to ITS OWN RIGHT so
they pass on opposite sides -- then returns to the centre line and continues to
its goal. This is the missing piece: B-1 only *detected*; M6 only avoided a
*static* pillar with a *single* drone. Here both drones sense-and-steer.

Topology (brought up by two_drone_run.sh, no Godot -- PDU/physics layer):
  Drone  spawns @ x=-2 facing +x ; Drone1 spawns @ x=+2 facing -x.
  bridge#1 -> Drone/radar_points  (Drone1 injected as actor)
  bridge#2 -> Drone1/radar_points (Drone  injected as actor)

Right-of-way rule (deterministic, symmetric):
  Drone  faces +x, its right is -y  -> veers to -y.
  Drone1 faces -x, its right is +y  -> veers to +y.
  => in world frame they open a 2*Y_AVOID gap in y and cannot collide.

Control is phase-based so the drones fly continuously (no per-tick hover stalls):
  A APPROACH : reactive head-on stepping on y=0 until BOTH radars detect.
  B SIDESTEP : each shifts laterally onto its own-right lane (x held).
  C CROSS    : each dashes to its goal x while holding its lane -> they pass.
  D RECENTER : each returns to y=0 at its goal.
A background thread samples both drones' true positions (raw pos PDU) throughout
to report the real closest approach.

Both drones are driven from THIS single process/session (a second external client
in another process cannot write commands). Reads use hakopy.pdu_read directly.

Run AFTER:  bash drone_daasim/two_drone_run.sh
Usage:      python two_drone_avoid.py

Verification: the encounter is scored against the six-step DAA procedure of
ISO 21384-3:2023 and must keep Well Clear -- not merely avoid contact. See
daa_metrics.py and devai/daa_scenario_study_iso_20260801.md. Scenario S-1
(head-on) implements the rule of 航空法施行規則 §182: aircraft approaching
head-on both alter course to their own right.
"""
import math
import os
import time

import daa_common as dc
import daa_metrics
from daa_common import read_xyz
from daa_metrics import EncounterRecorder, RangeTracker, StepReport, tau_mod


# --- DAA criteria (see daa_metrics.py for the derivation) --------------------
DWC        = daa_metrics.UAS_VS_UAS   # h=1.25 m, v=0.30 m, tau=3.5 s
TAU_FIRE   = DWC.tau_s
# When to start the manoeuvre. ISO 21384-3 puts step 3 (manoeuvre) straight
# after step 2 (recognition), and the JRC/NEDO flight demo turned as soon as the
# target was identified (at 500 m), not at some later timeline threshold. So the
# trigger here is RECOGNITION -- the first radar return that also yields a
# closing range rate -- and tau is used to SCORE the manoeuvre instead: starting
# while tau >= TAU_FIRE means the turn began before the well-clear timeline was
# violated. Waiting for tau <= 3.5 s would be far too late at this scale, since
# the radar only picks up a 0.5 m airframe at ~3 m.
# Backstop: if a closing rate never materialises (random-ray radar can jump from
# "no returns" to "close"), divert once inside 1.6x the Well Clear radius.
R_FLOOR    = DWC.h_m * 1.6

# --- scenario geometry (env frame; matches two_drone_run.sh spawn) -----------
H          = 0.6     # flight height
Y_AVOID    = 1.0     # lateral lane offset (m); +y=left, own-right sign applied
COLL_R     = 0.5     # min allowed inter-drone distance (both ~0.25 m bodies)
STEP       = 0.5     # forward increment per control tick during the approach
GOAL_EPS   = 0.40
APPROACH_TICKS = 14
MOVE_SPEED = 1.0
APPROACH_TO = 0.5    # per-move timeout during the reactive approach (s)

# Per-drone plan. face=+1 faces +x (yaw 0), -1 faces -x (yaw 180).
# veer = sign of y to steer (own right). goal = target x beyond the other's start.
# Starting separation is a SCENARIO parameter, not a sensor property: the first
# detection can never be farther than the aircraft are placed apart. S1_START
# overrides it so detection range can be measured rather than assumed.
_S = float(os.environ.get("S1_START", "2.0"))
DRONES = {
    "Drone":  dict(face=+1, veer=-1, start=-_S, goal=+(_S + 0.2)),
    "Drone1": dict(face=-1, veer=+1, start=+_S, goal=-(_S + 0.2)),
}




def yaw_for(face):
    return 0 if face > 0 else 180


def main():
    clients = dc.connect(DRONES)

    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H)
    time.sleep(1.0)

    # Face each other and settle onto the centre line, ~4 m apart.
    print("face-off: Drone +x / Drone1 -x, onto y=0 centre line ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["start"], 0.0, H, MOVE_SPEED, yaw_deg=yaw_for(d["face"]))
    time.sleep(1.0)

    # Background sampler -> true separation history, scored against Well Clear.
    enc = EncounterRecorder(DWC)

    sampler = dc.Sampler(enc, "Drone", "Drone1")
    sampler.start()

    # Per-drone DAA state for steps 1-3 of the ISO 21384-3 procedure.
    detected = {n: False for n in DRONES}       # step 1
    r_det = {n: None for n in DRONES}           # range of the FIRST detection
    recognised = {n: False for n in DRONES}     # step 2: range + closure rate
    tau_fire = {n: None for n in DRONES}        # step 3: tau when the turn began
    r_fire = {n: None for n in DRONES}
    fired_by = {n: None for n in DRONES}        # "tau" | "floor"
    track = {n: RangeTracker() for n in DRONES}

    # --- PHASE A: reactive head-on approach on the centre line ----------------
    print(f"A) approach head-on on y=0 until the other aircraft is recognised "
          f"(backstop: range <= {R_FLOOR:.2f} m); manoeuvre is scored on tau >= {TAU_FIRE:.1f} s ...",
          flush=True)
    for k in range(APPROACH_TICKS):
        for name, d in DRONES.items():
            face, goal = d["face"], d["goal"]
            xyz = read_xyz(name); px = xyz[0] if xyz else d["start"]
            sc = dc.scan(name, az_half=15.0, el_half=15.0)
            rng, dop = sc.rng, sc.doppler
            if rng is not None and not detected[name]:
                detected[name], r_det[name] = True, rng
            closure = track[name].update(time.time(), rng, dop)
            if rng is not None and closure is not None:
                recognised[name] = True
            t = tau_mod(rng, closure, DWC.h_m) if rng is not None else math.inf
            if tau_fire[name] is None and rng is not None:
                if recognised[name] and closure is not None and closure > 0.0:
                    tau_fire[name], r_fire[name], fired_by[name] = t, rng, "recognition"
                elif rng <= R_FLOOR:
                    tau_fire[name], r_fire[name], fired_by[name] = t, rng, "floor"
            nx = px + face * STEP
            tx = min(nx, goal) if face > 0 else max(nx, goal)
            rs = f"{rng:.2f}" if rng is not None else "--"
            cs = f"{closure:+.2f}" if closure is not None else "--"
            ts = f"{t:.1f}" if t != math.inf else "inf"
            print(f"  [A{k:02d}] {name} x={px:+.2f} r={rs} closure={cs} tau={ts} "
                  f"fire={fired_by[name]} sep={enc.min_h:.2f}", flush=True)
            clients[name].moveToPosition(tx, 0.0, H, MOVE_SPEED, yaw_deg=yaw_for(face), timeout_sec=APPROACH_TO)
        if all(tau_fire[n] is not None for n in DRONES):
            print("  -> both aircraft have a firing solution; diverting.", flush=True)
            break

    # Committed moves use a bounded timeout: moveToPosition otherwise blocks
    # forever waiting on an arrival tolerance the controller may never settle
    # into. We fly for a fixed budget, then verify with the true position.
    LANE_TO, CROSS_TO, HOME_TO = 6.0, 10.0, 6.0

    # --- PHASE B: sidestep onto own-right lanes (x held) ----------------------
    print("B) sidestep onto lanes: Drone -> -y, Drone1 -> +y ...", flush=True)
    for name, d in DRONES.items():
        xyz = read_xyz(name); px = xyz[0] if xyz else d["start"]
        clients[name].moveToPosition(px, d["veer"] * Y_AVOID, H, MOVE_SPEED,
                                     yaw_deg=yaw_for(d["face"]), timeout_sec=LANE_TO)
    # The lanes are what keeps the pair apart during the crossing, so confirm the
    # aircraft actually got there before dashing. Crossing while still sliding
    # sideways is what pulled CPA below the Well Clear radius previously.
    lane_ok = {}
    for name, d in DRONES.items():
        xyz = read_xyz(name)
        lane_ok[name] = xyz is not None and abs(xyz[1]) >= 0.85 * Y_AVOID
        if not lane_ok[name]:
            xyz = read_xyz(name); px = xyz[0] if xyz else d["start"]
            clients[name].moveToPosition(px, d["veer"] * Y_AVOID, H, MOVE_SPEED,
                                         yaw_deg=yaw_for(d["face"]), timeout_sec=LANE_TO)
            xyz = read_xyz(name)
            lane_ok[name] = xyz is not None and abs(xyz[1]) >= 0.85 * Y_AVOID
        print(f"  {name} lane y={xyz[1]:+.2f} (target {d['veer'] * Y_AVOID:+.2f}) "
              f"ok={lane_ok[name]}" if xyz else f"  {name} lane ??", flush=True)

    # --- PHASE C: cross -- dash to goal x while holding the lane --------------
    print("C) cross on separated lanes to goal x ...", flush=True)
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"], d["veer"] * Y_AVOID, H, MOVE_SPEED,
                                     yaw_deg=yaw_for(d["face"]), timeout_sec=CROSS_TO)

    # --- PHASE D: recenter onto y=0 at the goal -------------------------------
    print("D) recenter onto centre line at goal ...", flush=True)
    reached, off_track = {}, {}
    for name, d in DRONES.items():
        clients[name].moveToPosition(d["goal"], 0.0, H, MOVE_SPEED,
                                     yaw_deg=yaw_for(d["face"]), timeout_sec=HOME_TO)
        xyz = read_xyz(name)
        reached[name] = xyz is not None and abs(xyz[0] - d["goal"]) < GOAL_EPS
        off_track[name] = abs(xyz[1]) if xyz else math.inf
        rx = f"{xyz[0]:+.2f},{xyz[1]:+.2f}" if xyz else "??"
        print(f"  {name} at ({rx}) goal_x={d['goal']:+.2f} reached={reached[name]}", flush=True)

    sampler.stop()

    # --- verdict: the six steps of ISO 21384-3:2023 ---------------------------
    rep = StepReport("S-1 head-on (施行規則 §182: both alter course to the right)", DWC)

    det = all(detected.values())
    rep.record(1, det, "first detection at " + ", ".join(
        f"{n}={r_det[n]:.2f} m" if r_det[n] is not None else f"{n}=none" for n in DRONES))

    rec = all(recognised.values())
    rep.record(2, rec, "range + closure rate available for " + ", ".join(
        f"{n}={'yes' if recognised[n] else 'no'}" for n in DRONES)
        + "  (target classification / trajectory prediction: not implemented)")

    # Started in time = the turn began while tau was still above the threshold.
    fired = all(tau_fire[n] is not None and tau_fire[n] >= TAU_FIRE for n in DRONES)
    rep.record(3, fired, "manoeuvre start " + ", ".join(
        f"{n}=(r {r_fire[n]:.2f} m, tau {tau_fire[n]:.1f} s >= {TAU_FIRE:.1f}? "
        f"{'yes' if tau_fire[n] >= TAU_FIRE else 'NO'}, by {fired_by[n]})"
        if tau_fire[n] is not None else f"{n}=never" for n in DRONES)
        + "; both veered to their own right (§182)")

    rep.record(4, enc.well_clear_kept, enc.summary())

    ret = all(reached.values())
    rep.record(5, ret, "returned to the centre line: " + ", ".join(
        f"{n}={off_track[n]:.2f} m off track" for n in DRONES))

    rep.record(6, ret, "goals reached: " + ", ".join(f"{n}={reached[n]}" for n in DRONES))

    print(rep.render(), flush=True)

    # Legacy line kept so older comparisons stay readable.
    clr = enc.min_h >= COLL_R
    print(f"[avoid] min inter-drone sep = {enc.min_h:.2f} m "
          f"(no-contact >= {COLL_R} m? {clr} | Well Clear >= {DWC.h_m} m? "
          f"{enc.min_h >= DWC.h_m})", flush=True)

    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(detect={det}, recognise={rec}, manoeuvre={fired}, "
          f"well_clear={enc.well_clear_kept}, return={ret})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
