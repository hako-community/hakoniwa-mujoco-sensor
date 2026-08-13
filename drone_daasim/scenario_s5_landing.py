#!/usr/bin/env python3
"""S-5 landing priority -- 施行規則 §183 / §184, ISO 15964 6.2.4 (landing scenario).

  §183  着陸のため最終進入の経路にある航空機及び着陸操作を行つている航空機は、
        飛行中の航空機に対して進路権を有する
  §184  着陸のため進入している航空機相互間にあつては、低い高度にある航空機が
        進路権を有する

Geometry (env frame):

              z
    Drone  ---+---------->  cruising at z=1.2 through the approach path
              |  \\
              |    \\  Drone1 on final approach, descending 2.2 m -> 0.2 m
              |      \\
        ------+--------\\------ landing point (0, 0)

What makes this scenario different from S-2: the roles are NOT decided by
bearing. §183 hands priority to the landing aircraft whatever the geometry, so
the cruising aircraft has to work out that the traffic it sees is *landing*.
It does that from the radar alone: relative altitude = range * sin(elevation),
differentiated over a window (daa_metrics.SlopeTracker). A steady negative slope
means a descending aircraft, which under §183 owns the right of way.

The give-way manoeuvre is to hold short of the approach path -- ISO 15964 3.9
counts hovering as a manoeuvre -- and the resumption is gated on an ONBOARD
observation (the traffic is now well below our level), which is step 4 of the
ISO 21384-3 procedure done with the sensor rather than with ground truth.

§184 is not exercised here: it only applies between two aircraft that are both
on approach, and only one aircraft lands in this scenario.

Elevation coverage is the binding constraint here (issue #7). A symmetric
vertical FOV of 2*theta follows a height difference of only r*tan(theta) -- with
the shipped 20 deg that is 0.18r -- so the traffic leaves the window exactly when
it matters: close in and below, on the pad. Every miss is therefore attributed
(daa_common.coverage_gap) to azimuth, elevation or range rather than left blank,
and the onboard confirmation records WHICH source answered, because a fallback
onto ground truth is not an onboard check however green it prints.

In a tighter approach (S5_START=4.0) it is AZIMUTH that binds first, which is
issue #13 and the reason for the -approach-wide fit.

  bash two_drone_run.sh noground                                 # shipped 60x20 deg
  A2_MANIFEST=../config/a2/drone-a2-sensors-approach.json \
    bash two_drone_run.sh noground                               # 60x45, el -35..+10 (#7)
  A2_MANIFEST=../config/a2/drone-a2-sensors-approach-wide.json \
    bash two_drone_run.sh noground                               # 150x45 (#13)

Run AFTER:  bash drone_daasim/two_drone_run.sh noground
Usage:      python scenario_s5_landing.py
"""
import math
import os
import time

import daa_common as dc
import daa_metrics
from daa_metrics import EncounterRecorder, RangeTracker, SlopeTracker, StepReport, tau_mod

DWC = daa_metrics.UAS_VS_UAS
TAU_FIRE = DWC.tau_s

CRUISE, TRANSIT = "Drone", "Drone1"   # Drone cruises; Drone1 lands
H_CRUISE = 1.2
H_APPROACH_TOP = 2.2
H_TOUCHDOWN = 0.2
LANDING_PT = (0.0, 0.0)

_S = float(os.environ.get("S5_START", "7.0"))
CRUISE_START, CRUISE_GOAL = -_S, +_S
APPROACH_START_Y = -6.0
APPROACH_LEGS = 5                     # descent steps from the top of the approach

STEP = 0.45
MOVE_SPEED = 1.0
APPROACH_TO = 0.6
TICKS = 16
HOLD_TO, CROSS_TO, HOME_TO = 8.0, 14.0, 10.0
# No angular filter of our own, on either axis (#7 elevation, #13 azimuth).
# The radar's window IS the limit here. A scenario window narrower than the fit
# throws away exactly the returns a wider fit was installed to produce, and the
# loss is indistinguishable from a sensor that could not see: with AZ_HALF=25
# against a 150 deg radar, the traffic converging at 40-50 deg off the nose was
# discarded by this file, not missed by the sensor.
AZ_HALF = None
EL_HALF = None
GOAL_EPS = 0.5

# A descending target is one whose relative altitude is dropping at least this
# fast; below it the reading is indistinguishable from noise in the elevation.
DESCENT_MPS = 0.05
# Resume once the landing traffic is this far below our level (onboard check).
CLEAR_BELOW_M = 0.7


def main():
    clients = dc.connect([CRUISE, TRANSIT])

    print("takeoff both ...", flush=True)
    clients[CRUISE].takeoff(H_CRUISE)
    clients[TRANSIT].takeoff(H_APPROACH_TOP)
    time.sleep(1.0)

    print(f"line up: {CRUISE} cruising +x at z={H_CRUISE}, "
          f"{TRANSIT} at the top of the approach ...", flush=True)
    clients[CRUISE].moveToPosition(CRUISE_START, 0.0, H_CRUISE, MOVE_SPEED,
                                   yaw_deg=0.0, timeout_sec=HOME_TO)
    clients[TRANSIT].moveToPosition(LANDING_PT[0], APPROACH_START_Y, H_APPROACH_TOP,
                                    MOVE_SPEED, yaw_deg=90.0, timeout_sec=HOME_TO)
    time.sleep(1.0)

    enc = EncounterRecorder(DWC)
    sampler = dc.Sampler(enc, CRUISE, TRANSIT)
    sampler.start()

    detected = False
    r_det = None
    rng_tr = RangeTracker()
    alt_tr = SlopeTracker()
    rel_alt = None
    descent_rate = None
    landing_traffic = False
    tau_fire = r_fire = None
    # #7 instrumentation. A miss on a radar is silent -- nothing comes back --
    # and that is the same signal whether the target was out of range, off to
    # the side, or below the elevation window. Comparing the TRUE bearing with
    # the fit's coverage says which, so the run produces evidence about the
    # sensor instead of a blank. Used for reporting only; no decision reads it.
    gaps = {}                 # reason -> how many ticks it explained the miss
    el_seen_min = None        # lowest elevation at which the target was tracked
    el_true_min = None        # lowest elevation the target actually reached

    # Descent profile for the landing aircraft: a straight approach to the pad.
    legs = [(APPROACH_START_Y + (0.0 - APPROACH_START_Y) * (i + 1) / APPROACH_LEGS,
             H_APPROACH_TOP + (H_TOUCHDOWN - H_APPROACH_TOP) * (i + 1) / APPROACH_LEGS)
            for i in range(APPROACH_LEGS)]
    leg_i = 0

    print("A) cruise on; identify the descending traffic (§183) ...", flush=True)
    for k in range(TICKS):
        # --- landing aircraft: flies its approach, never deviates -------------
        if leg_i < len(legs):
            ly, lz = legs[leg_i]
            clients[TRANSIT].moveToPosition(LANDING_PT[0], ly, lz, MOVE_SPEED,
                                            yaw_deg=90.0, timeout_sec=APPROACH_TO)
            leg_i += 1

        # --- cruising aircraft: sense, classify, decide ----------------------
        p = dc.read_xyz(CRUISE)
        px = p[0] if p else CRUISE_START
        s = dc.scan_best(CRUISE, az_half=AZ_HALF, el_half=EL_HALF)
        now = time.time()
        pt_now = dc.read_xyz(TRANSIT)
        # Where the radar SHOULD have been looking, from truth. Diagnosis only.
        t_az, t_el, t_r = dc.body_bearing(p, 0.0, pt_now)
        if t_el is not None:
            el_true_min = t_el if el_true_min is None else min(el_true_min, t_el)
        if s.rng is not None:
            if not detected:
                detected, r_det = True, s.rng
            rel_alt = s.rng * math.sin(math.radians(s.el_deg))
            descent_rate = alt_tr.update(now, rel_alt)
            el_seen_min = s.el_deg if el_seen_min is None else min(el_seen_min, s.el_deg)
        else:
            why = dc.coverage_gap(CRUISE, t_az, t_el, t_r, AZ_HALF, EL_HALF)
            if why:
                gaps[why] = gaps.get(why, 0) + 1
        closure = rng_tr.update(now, s.rng, s.doppler)
        if (not landing_traffic and descent_rate is not None
                and descent_rate <= -DESCENT_MPS and s.rng is not None):
            landing_traffic = True
            tau_fire = tau_mod(s.rng, closure, DWC.h_m)
            r_fire = s.rng
            print(f"       -> {CRUISE}: traffic is DESCENDING "
                  f"({descent_rate:+.2f} m/s, rel alt {rel_alt:+.2f} m) "
                  f"-> landing traffic has right of way (§183)", flush=True)
        ra = f"{rel_alt:+.2f}" if rel_alt is not None else "--"
        dr = f"{descent_rate:+.2f}" if descent_rate is not None else "--"
        rs = f"{s.rng:.2f}" if s.rng is not None else "--"
        pts = f"{pt_now[1]:+.2f},z={pt_now[2]:+.2f}" if pt_now else "??"
        miss = "" if s.rng is not None else \
            f" MISS[{dc.coverage_gap(CRUISE, t_az, t_el, t_r, AZ_HALF, EL_HALF) or 'in coverage, no return'}]"
        true_b = f" true(az{t_az:+.0f},el{t_el:+.0f})" if t_az is not None else ""
        print(f"  [A{k:02d}] {CRUISE} x={px:+.2f} r={rs} relalt={ra} rate={dr} "
              f"landing={landing_traffic} | {TRANSIT} y={pts} sep={enc.min_h:.2f}"
              f"{true_b}{miss}", flush=True)
        if landing_traffic:
            print("  -> giving way; holding short of the approach path.", flush=True)
            break
        clients[CRUISE].moveToPosition(px + STEP, 0.0, H_CRUISE, MOVE_SPEED,
                                       yaw_deg=0.0, timeout_sec=APPROACH_TO)

    # --- PHASE B: hold short of the approach path (§183) ---------------------
    hold_x = LANDING_PT[0] - (DWC.h_m + 0.4)
    print(f"B) §183 give way: hold at x={hold_x:+.2f} while the landing aircraft "
          f"completes its approach ...", flush=True)
    time.sleep(0.6)
    p = dc.read_xyz(CRUISE)
    px = p[0] if p else CRUISE_START
    clients[CRUISE].moveToPosition(min(px, hold_x), 0.0, H_CRUISE, MOVE_SPEED,
                                   yaw_deg=0.0, timeout_sec=HOLD_TO)

    # landing aircraft completes the approach and touches down
    for ly, lz in legs[leg_i:]:
        clients[TRANSIT].moveToPosition(LANDING_PT[0], ly, lz, MOVE_SPEED,
                                        yaw_deg=90.0, timeout_sec=APPROACH_TO)
    clients[TRANSIT].moveToPosition(LANDING_PT[0], LANDING_PT[1], H_TOUCHDOWN,
                                    MOVE_SPEED, yaw_deg=90.0, timeout_sec=HOME_TO)

    # --- PHASE C: onboard confirmation, then resume --------------------------
    print("C) confirm from our OWN sensor that the traffic is below us, then resume ...",
          flush=True)
    # Step 4 of ISO 21384-3 is supposed to be done WITH THE SENSOR, so which
    # source answered is itself a result. `confirmed_by` records it rather than
    # letting the fallback pass as an onboard check (#7): on the shipped 20 deg
    # window the traffic is 30-odd degrees below us by the time it has landed,
    # which is outside the window at any range, and the fallback is then reading
    # ground truth -- something no aircraft has.
    confirmed_below = False
    confirmed_by = None
    check_gap = None
    for _ in range(24):
        s = dc.scan_best(CRUISE, az_half=AZ_HALF, el_half=EL_HALF)
        _, e_true, _ = dc.body_bearing(dc.read_xyz(CRUISE), 0.0, dc.read_xyz(TRANSIT))
        if e_true is not None:
            el_true_min = e_true if el_true_min is None else min(el_true_min, e_true)
        if s.rng is not None:
            el_seen_min = s.el_deg if el_seen_min is None else min(el_seen_min, s.el_deg)
            ra = s.rng * math.sin(math.radians(s.el_deg))
            if ra <= -CLEAR_BELOW_M:
                confirmed_below, confirmed_by = True, "radar"
                print(f"  onboard check: traffic at {ra:+.2f} m relative altitude "
                      f"(el {s.el_deg:+.0f} deg, <= -{CLEAR_BELOW_M}) "
                      f"-> approach path clear", flush=True)
                break
        time.sleep(0.3)
    if not confirmed_below:
        pc_now, pt_now = dc.read_xyz(CRUISE), dc.read_xyz(TRANSIT)
        c_az, c_el, c_r = dc.body_bearing(pc_now, 0.0, pt_now)
        check_gap = dc.coverage_gap(CRUISE, c_az, c_el, c_r, AZ_HALF, EL_HALF)
        # Fall back on the geometric fact that the pad is below the cruise level.
        if pt_now and (H_CRUISE - pt_now[2]) >= CLEAR_BELOW_M:
            confirmed_below, confirmed_by = True, "ground truth (NOT onboard)"
            print(f"  radar could not confirm: {check_gap or 'no return'} "
                  f"-- falling back on ground truth: traffic is "
                  f"{H_CRUISE - pt_now[2]:.2f} m below the cruise level", flush=True)

    # --- can we still SEE the traffic we are holding for? (#7) ---------------
    # The confirmation above is a single instant, and it is won by checking
    # early -- while still far enough away that a shallow elevation window
    # reaches the pad. Holding is not an instant: the aircraft sits short of the
    # approach path for as long as the landing takes, and the geometry gets
    # WORSE the whole time (same height difference, shrinking range, so the
    # bearing dives). Whether the traffic is still tracked at the hold point is
    # the question that decides if the give-way manoeuvre can be monitored at
    # all, and it is a pure coverage measurement -- reported, not scored, in the
    # same spirit as S-3's rear blind sector.
    print("C2) still holding -- is the traffic we gave way to still tracked? ...", flush=True)
    hold_track = False
    hold_gap = None
    hold_el = None
    for _ in range(8):
        s = dc.scan_best(CRUISE, az_half=AZ_HALF, el_half=EL_HALF)
        pc_now, pt_now = dc.read_xyz(CRUISE), dc.read_xyz(TRANSIT)
        h_az, hold_el, h_r = dc.body_bearing(pc_now, 0.0, pt_now)
        if s.rng is not None:
            hold_track, hold_gap = True, None
            el_seen_min = s.el_deg if el_seen_min is None else min(el_seen_min, s.el_deg)
            break
        hold_gap = dc.coverage_gap(CRUISE, h_az, hold_el, h_r, AZ_HALF, EL_HALF)
        if hold_el is not None:
            el_true_min = hold_el if el_true_min is None else min(el_true_min, hold_el)
        time.sleep(0.25)
    print(f"  traffic at el {hold_el:+.0f} deg from the hold point: "
          + ("still tracked" if hold_track
             else f"LOST -- {hold_gap or 'no return'}")
          if hold_el is not None else "  ??", flush=True)

    print("D) resume the cruise ...", flush=True)
    clients[CRUISE].moveToPosition(CRUISE_GOAL, 0.0, H_CRUISE, MOVE_SPEED,
                                   yaw_deg=0.0, timeout_sec=CROSS_TO)

    pc, pt = dc.read_xyz(CRUISE), dc.read_xyz(TRANSIT)
    cruise_ok = pc is not None and abs(pc[0] - CRUISE_GOAL) < GOAL_EPS
    landed = pt is not None and pt[2] <= H_TOUCHDOWN + 0.35
    approach_straight = pt is not None and abs(pt[0] - LANDING_PT[0]) < 0.35
    print(f"  {CRUISE} at x={pc[0]:+.2f} reached={cruise_ok}" if pc else "  ??", flush=True)
    print(f"  {TRANSIT} touchdown z={pt[2]:+.2f} landed={landed} "
          f"lateral error {abs(pt[0] - LANDING_PT[0]):.2f} m" if pt else "  ??", flush=True)

    sampler.stop()

    # --- verdict --------------------------------------------------------------
    rep = StepReport("S-5 landing priority (施行規則 §183; §184 not exercised)", DWC)

    rep.record(1, detected, "cruising aircraft first detection at "
               + (f"{r_det:.2f} m" if r_det else "none")
               + f" | fit: {dc.fit_summary(CRUISE)}"
               + (" | misses explained by: "
                  + ", ".join(f"{r} x{n}" for r, n in
                              sorted(gaps.items(), key=lambda kv: -kv[1]))
                  if gaps else ""))

    rep.record(2, landing_traffic,
               "traffic classified as LANDING from the radar elevation channel: "
               + (f"descent {descent_rate:+.2f} m/s (threshold -{DESCENT_MPS})"
                  if descent_rate is not None else "no descent measured"))

    rep.record(3, landing_traffic and tau_fire is not None,
               f"gave way by holding short at x={hold_x:+.2f} m "
               + (f"(decision at r={r_fire:.2f} m, tau {tau_fire:.1f} s)"
                  if tau_fire is not None else "")
               + " -- §183 grants priority to the landing aircraft regardless of bearing")

    rep.record(4, enc.well_clear_kept and confirmed_below,
               enc.summary() + " | approach path confirmed clear by: "
               + (confirmed_by or "nothing")
               + (f" -- the radar's elevation window could not reach it "
                  f"({check_gap})" if confirmed_by and "ground" in confirmed_by
                  else ""))

    rep.record(5, landed and approach_straight,
               f"landing aircraft completed its approach undisturbed: touchdown z="
               + (f"{pt[2]:.2f} m, lateral error {abs(pt[0] - LANDING_PT[0]):.2f} m"
                  if pt else "??"))

    rep.record(6, cruise_ok, f"cruising aircraft resumed and reached its goal: {cruise_ok}")

    print(rep.render(), flush=True)
    # The encounter resolves vertically once the traffic is on the pad, so the
    # horizontal CPA on its own is not the measure of safety here -- state both.
    print(f"[s5] CPA horizontal {enc.min_h:.2f} m / vertical at CPA {enc.v_at_cpa:.2f} m "
          f"-> Well Clear kept: {enc.well_clear_kept}"
          + ("  (the cruise passes ABOVE the landed aircraft; the vertical gap is what "
             "keeps it clear)" if enc.min_h < DWC.h_m else ""), flush=True)
    # #7: the elevation budget of this run, stated as numbers. `el reached` is
    # how far below the horizon the traffic actually went; `el tracked` is how
    # far the radar followed it. The gap between them is the coverage shortfall.
    el_cov = dc.elevation_coverage_deg(CRUISE)
    el_floor = min((u.el_lo for u in dc.radar_fit(CRUISE) if u.el_lo is not None),
                   default=None)
    # How much elevation was left over. Zero means the encounter was flown on
    # the window boundary and any tighter geometry loses the target outright.
    margin = None if (el_floor is None or el_true_min is None) else el_true_min - el_floor
    print(f"[s5] ELEVATION: window "
          + ("/".join(f"[{u.el_lo:+.0f},{u.el_hi:+.0f}]" for u in dc.radar_fit(CRUISE))
             if dc.radar_fit(CRUISE)[0].el_lo is not None else "unknown")
          + (f" = {el_cov:.0f} deg" if el_cov is not None else "")
          + f" | target reached el "
          + (f"{el_true_min:+.0f} deg" if el_true_min is not None else "--")
          + (f" (margin to the window floor: {margin:+.1f} deg)"
             if margin is not None else "")
          + " | radar tracked it down to "
          + (f"{el_seen_min:+.0f} deg" if el_seen_min is not None else "--")
          + f" | onboard confirmation: {confirmed_by or 'none'}"
          + " | tracked from the hold point: "
          + ("yes" if hold_track else f"NO ({hold_gap or 'no return'})"), flush=True)
    ok = rep.passed
    print(f"RESULT: {'PASS' if ok else 'FAIL'} "
          f"(detect={detected}, landing_identified={landing_traffic}, "
          f"well_clear={enc.well_clear_kept}, landed={landed}, resumed={cruise_ok}, "
          f"confirmed_by={confirmed_by})", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
