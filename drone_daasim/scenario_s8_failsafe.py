#!/usr/bin/env python3
"""S-8 degraded sensor / fail-safe -- ISO 15964 6.2.5, 4.2 d) e) f).

  4.2 d) fault detection and isolation: monitoring the operation of the obstacle
         avoidance system and finding out the faults that can cause anomalies
      e) fault indication: conveying the fault message timely
      f) fault correction: correcting the fault without affecting operation
  6.2.5 fail safe: the system can operate safely on the failure of any single
         component

Every previous scenario assumed both radars work. Here one of them stops mid
encounter and the question becomes whether the ENCOUNTER still ends safely.

Fault injection is real, not simulated in the scenario logic: the bridge process
that publishes Drone's radar is SIGSTOPped, so the PDU simply stops changing.
That is what a hung sensor looks like from the flight software's side -- the last
frame stays in shared memory forever, so a "no data" check would never fire and
freshness has to be judged from the CONTENT (daa_common.StaleWatch).

This scenario is also what forced the radar to timestamp its scans. Judging
freshness by content alone is unsound: a scan that detects nothing is
byte-identical to the previous empty scan, so a perfectly healthy sensor looking
at empty sky reads as "stale". Every scan now carries a monotonic stamp
(radar_sensor.cpp / radar_point_cloud.hpp), which makes "the payload stopped
changing" mean exactly "the sensor stopped producing".

Expected behaviour:
  Drone  (faulty)  detects the fault, gives up on avoidance and holds -- an
                   aircraft that cannot see must not keep flying at traffic.
  Drone1 (healthy) sees Drone and resolves the whole encounter on its own, so
                   Well Clear survives a single-component failure.
  Then SIGCONT restores the sensor and the recovery is verified (4.2 f).

Run AFTER:  bash drone_daasim/two_drone_run.sh noground
Usage:      python scenario_s8_failsafe.py
"""
import math
import os
import signal
import time

import daa_common as dc
import daa_metrics
from daa_metrics import EncounterRecorder, RangeTracker, StepReport, tau_mod

DWC = daa_metrics.UAS_VS_UAS
TAU_FIRE = DWC.tau_s

FAULTY, HEALTHY = "Drone", "Drone1"
H = 0.8
# Only the healthy aircraft manoeuvres, so its offset alone is the separation.
Y_AVOID = 1.6
STEP = 0.45
MOVE_SPEED = 1.0
APPROACH_TO = 0.5
TICKS = 18
LANE_TO, CROSS_TO, HOME_TO = 6.0, 12.0, 8.0
AZ_HALF, EL_HALF = 20.0, 15.0
GOAL_EPS = 0.5
STALE_TIMEOUT = 1.2                   # declare the sensor faulty after this
FAULT_AT_TICK = 2                     # inject once the encounter is under way

_S = float(os.environ.get("S8_START", "6.0"))
PLAN = {
    FAULTY:  dict(face=+1, start=-_S, goal=+(_S + 0.2), yaw=0.0, veer=-1),
    HEALTHY: dict(face=-1, start=+_S, goal=-(_S + 0.2), yaw=180.0, veer=+1),
}
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def bridge_pid(robot):
    path = os.path.join(LOG_DIR, f"bridge_{robot}.pid")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        try:
            return int(f.read().strip())
        except ValueError:
            return None


def main():
    pid = bridge_pid(FAULTY)
    if pid is None:
        print(f"MISSING {LOG_DIR}/bridge_{FAULTY}.pid -- re-run two_drone_run.sh", flush=True)
        return 2

    clients = dc.connect(PLAN)
    print("takeoff both ...", flush=True)
    for c in clients.values():
        c.takeoff(H)
    time.sleep(1.0)

    print("line up head-on ...", flush=True)
    for name, d in PLAN.items():
        clients[name].moveToPosition(d["start"], 0.0, H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, FAULTY, HEALTHY)
    sampler.start()

    watch = dc.StaleWatch(STALE_TIMEOUT)
    fault_injected_at = None
    fault_declared_at = None
    contingency = False
    hold_x = None

    detected_h = False
    r_det_h = None
    tau_fire_h = r_fire_h = None
    rng_tr = RangeTracker()

    print(f"A) approach; radar of {FAULTY} will be stopped at tick {FAULT_AT_TICK} ...",
          flush=True)
    for k in range(TICKS):
        # --- inject the fault ------------------------------------------------
        if k == FAULT_AT_TICK and fault_injected_at is None:
            os.kill(pid, signal.SIGSTOP)
            fault_injected_at = time.time()
            print(f"  *** FAULT INJECTED: SIGSTOP on the {FAULTY} radar bridge "
                  f"(pid {pid}) ***", flush=True)

        # --- faulty aircraft: monitor its own sensor -------------------------
        now = time.time()
        stale = watch.update(now, dc.radar_hash(FAULTY))
        if stale and not contingency:
            contingency = True
            fault_declared_at = now
            p = dc.read_xyz(FAULTY)
            hold_x = p[0] if p else PLAN[FAULTY]["start"]
            lat = (fault_declared_at - fault_injected_at) if fault_injected_at else float("nan")
            print(f"  *** SENSOR FAULT DECLARED on {FAULTY} after {lat:.2f} s "
                  f"-> contingency: hold position at x={hold_x:+.2f} ***", flush=True)

        pf = dc.read_xyz(FAULTY)
        pfx = pf[0] if pf else PLAN[FAULTY]["start"]
        if contingency:
            # An aircraft that cannot see must not keep closing on traffic.
            clients[FAULTY].moveToPosition(hold_x, 0.0, H, MOVE_SPEED,
                                           yaw_deg=PLAN[FAULTY]["yaw"], timeout_sec=APPROACH_TO)
        else:
            clients[FAULTY].moveToPosition(pfx + PLAN[FAULTY]["face"] * STEP, 0.0, H,
                                           MOVE_SPEED, yaw_deg=PLAN[FAULTY]["yaw"],
                                           timeout_sec=APPROACH_TO)

        # --- healthy aircraft: normal DAA ------------------------------------
        ph = dc.read_xyz(HEALTHY)
        phx = ph[0] if ph else PLAN[HEALTHY]["start"]
        s = dc.scan(HEALTHY, az_half=AZ_HALF, el_half=EL_HALF)
        if s.rng is not None and not detected_h:
            detected_h, r_det_h = True, s.rng
        closure = rng_tr.update(time.time(), s.rng, s.doppler)
        t = tau_mod(s.rng, closure, DWC.h_m) if s.rng is not None else math.inf
        if tau_fire_h is None and s.rng is not None and closure is not None and closure > 0.0:
            tau_fire_h, r_fire_h = t, s.rng
        rs = f"{s.rng:.2f}" if s.rng is not None else "--"
        print(f"  [A{k:02d}] {FAULTY} x={pfx:+.2f} stale={stale} contingency={contingency} | "
              f"{HEALTHY} x={phx:+.2f} r={rs} tau={t if t == math.inf else round(t, 1)} "
              f"sep={enc.min_h:.2f}", flush=True)
        clients[HEALTHY].moveToPosition(phx + PLAN[HEALTHY]["face"] * STEP, 0.0, H,
                                        MOVE_SPEED, yaw_deg=PLAN[HEALTHY]["yaw"],
                                        timeout_sec=APPROACH_TO)

        if contingency and tau_fire_h is not None:
            print("  -> faulty aircraft holding; healthy aircraft resolves the encounter alone.",
                  flush=True)
            break

    # --- PHASE B: the healthy aircraft avoids on its own ---------------------
    print(f"B) {HEALTHY} avoids alone: offset {Y_AVOID} m to its right ...", flush=True)
    time.sleep(0.6)
    lane_ok = False
    for attempt in range(3):
        ph = dc.read_xyz(HEALTHY)
        phx = ph[0] if ph else PLAN[HEALTHY]["start"]
        clients[HEALTHY].moveToPosition(phx, PLAN[HEALTHY]["veer"] * Y_AVOID, H, MOVE_SPEED,
                                        yaw_deg=PLAN[HEALTHY]["yaw"], timeout_sec=LANE_TO)
        ph = dc.read_xyz(HEALTHY)
        lane_ok = ph is not None and abs(abs(ph[1]) - Y_AVOID) < 0.35
        print(f"  {HEALTHY} lane y={ph[1]:+.2f} ok={lane_ok} attempt={attempt}"
              if ph else "  ??", flush=True)
        if lane_ok:
            break

    print("C) pass on the lane while the faulty aircraft holds ...", flush=True)
    clients[HEALTHY].moveToPosition(PLAN[HEALTHY]["goal"], PLAN[HEALTHY]["veer"] * Y_AVOID,
                                    H, MOVE_SPEED, yaw_deg=PLAN[HEALTHY]["yaw"],
                                    timeout_sec=CROSS_TO)

    # --- fault correction (ISO 15964 4.2 f) ----------------------------------
    print("D) restore the sensor (SIGCONT) and verify recovery ...", flush=True)
    os.kill(pid, signal.SIGCONT)
    recovered = False
    recover_t0 = time.time()
    for _ in range(30):
        if not watch.update(time.time(), dc.radar_hash(FAULTY)):
            recovered = True
            break
        time.sleep(0.2)
    recover_dt = time.time() - recover_t0
    print(f"  radar of {FAULTY} recovered: {recovered} ({recover_dt:.2f} s)", flush=True)

    # --- resume --------------------------------------------------------------
    reached = {}
    for name, d in PLAN.items():
        clients[name].moveToPosition(d["goal"], 0.0, H, MOVE_SPEED,
                                     yaw_deg=d["yaw"], timeout_sec=HOME_TO)
        p = dc.read_xyz(name)
        reached[name] = p is not None and abs(p[0] - d["goal"]) < GOAL_EPS
        ps = f"{p[0]:+.2f},{p[1]:+.2f}" if p else "??"
        print(f"  {name} at ({ps}) reached={reached[name]}", flush=True)

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-8 degraded sensor / fail-safe (ISO 15964 6.2.5, 4.2 d-f)", DWC)

    latency = (fault_declared_at - fault_injected_at) \
        if (fault_declared_at and fault_injected_at) else None

    rep.record(1, detected_h,
               f"healthy {HEALTHY} first detection at "
               + (f"{r_det_h:.2f} m" if r_det_h else "none")
               + f" | faulty {FAULTY}: radar stopped at tick {FAULT_AT_TICK} (SIGSTOP)")

    rep.record(2, latency is not None and latency <= STALE_TIMEOUT + 1.0,
               "fault detected from payload staleness in "
               + (f"{latency:.2f} s (timeout {STALE_TIMEOUT} s)" if latency else "NOT DETECTED")
               + "  -- the PDU still reads back fine; what stops is the scan timestamp advancing")

    rep.record(3, contingency and lane_ok,
               f"contingency on {FAULTY}: hold at x={hold_x:+.2f} (stopped closing) | "
               f"{HEALTHY} resolved the encounter alone, lane reached={lane_ok}"
               + (f", start r={r_fire_h:.2f} m, tau {tau_fire_h:.1f} s"
                  if tau_fire_h is not None else ""))

    rep.record(4, enc.well_clear_kept,
               enc.summary() + " -- kept by a single working sensor (6.2.5 fail safe)")

    rep.record(5, recovered,
               f"fault correction: sensor restored and publishing again in {recover_dt:.2f} s")

    rep.record(6, all(reached.values()),
               "goals reached after recovery: " + ", ".join(
                   f"{n}={reached[n]}" for n in PLAN))

    print(rep.render(), flush=True)
    print(f"[s8] min separation = {enc.min_h:.2f} m "
          f"(Well Clear >= {DWC.h_m} m? {enc.min_h >= DWC.h_m})", flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(fault_detected={latency is not None}, contingency={contingency}, "
          f"well_clear={enc.well_clear_kept}, recovered={recovered})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
