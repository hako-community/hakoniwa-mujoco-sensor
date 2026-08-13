#!/usr/bin/env python3
"""Read-only probe: what the radar returns, binned by elevation (issue #7).

Widening the elevation window is supposed to cost ground clutter. That claim is
worth measuring rather than repeating, because the amount depends on three
things a manifest cannot see: how high the aircraft is, whether the sensing
world even HAS a floor (two_drone_run.sh noground -- the default -- does not),
and whether the moving-target filter already removes it.

The ground is static, so it comes back at Doppler ~0. The split printed here is
therefore the split that matters operationally: returns a moving-target filter
would keep, versus returns it would throw away.

  bash two_drone_run.sh ground          # or: noground
  python probe_elevation.py [robot] [seconds]

Nothing is written and no aircraft is commanded, so this is safe to run
alongside a scenario.
"""
import os
import sys
import time
from collections import Counter

import hakopy
import daa_common as dc

STATIC_DOPPLER = 0.05      # |v| below this reads as static structure
BIN_DEG = 10.0


def main():
    robot = sys.argv[1] if len(sys.argv) > 1 else "Drone"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    if not hakopy.init_for_external():
        print("init_for_external() failed -- is the stack up?", flush=True)
        return 2

    print(f"[probe] {robot}: {dc.fit_summary(robot)}", flush=True)
    el_cov = dc.elevation_coverage_deg(robot)
    print(f"[probe] elevation coverage: "
          + ("/".join(f"[{u.el_lo:+.0f},{u.el_hi:+.0f}]" for u in dc.radar_fit(robot))
             if dc.radar_fit(robot)[0].el_lo is not None else "unknown")
          + (f" = {el_cov:.0f} deg" if el_cov is not None else ""), flush=True)

    bins = Counter()
    static_bins = Counter()
    frames = 0
    total = static = 0
    nearest_static = None
    # Own ground speed, because the static fraction is meaningless without it:
    # ground ahead of a MOVING radar has a closing Doppler of v*cos(el) and stops
    # looking static. A "the target filter removes all of it" result taken from a
    # hovering aircraft would not carry over to one that is flying.
    spd = dc.SpeedTracker()
    own_max = 0.0
    t_end = time.time() + secs
    while time.time() < t_end:
        v = spd.update(time.time(), dc.read_xyz(robot))
        if v is not None:
            own_max = max(own_max, v)
        # No angular filter at all: the point is to see the whole window,
        # including the part a scenario would normally narrow away.
        per = dc.scan_units(robot, az_half=None, el_half=None)
        got = False
        for u in dc.radar_fit(robot):
            raw = dc._read_channel(robot, u.channel, u.pdu_size)
            if raw is None:
                continue
            got = True
            for r, v, az, el in dc._hits(raw, None, None, None, None,
                                         u.mount_yaw_deg):
                b = int(el // BIN_DEG) * int(BIN_DEG)
                bins[b] += 1
                total += 1
                if abs(v) < STATIC_DOPPLER:
                    static_bins[b] += 1
                    static += 1
                    if nearest_static is None or r < nearest_static:
                        nearest_static = r
        if got:
            frames += 1
        time.sleep(0.1)

    if frames == 0:
        print("[probe] no radar frames -- is the bridge publishing?", flush=True)
        return 1

    print(f"[probe] own ground speed during the sample: up to {own_max:.2f} m/s",
          flush=True)
    print(f"[probe] {frames} frames, {total} returns "
          f"({total / frames:.0f}/frame), of which {static} static "
          f"({100.0 * static / total:.0f}%)" if total else
          f"[probe] {frames} frames, no returns", flush=True)
    if nearest_static is not None:
        print(f"[probe] nearest static return: {nearest_static:.2f} m", flush=True)
    print("[probe] elevation band   returns/frame   static", flush=True)
    for b in sorted(bins):
        n, s = bins[b], static_bins[b]
        print(f"[probe]  [{b:+4.0f},{b + BIN_DEG:+4.0f})  "
              f"{n / frames:12.1f}   {100.0 * s / n:5.0f}%", flush=True)
    # The number the issue is really about: what a wider window costs in
    # returns that are not aircraft.
    print(f"[probe] MOVING returns/frame (what a target filter keeps): "
          f"{(total - static) / frames:.1f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
