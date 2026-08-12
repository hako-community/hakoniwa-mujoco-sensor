"""DAA (Detect And Avoid) metrics shared by every collision-avoidance scenario.

Why this module exists
----------------------
Until now a scenario passed when the two drones simply did not touch
(`min_sep >= 0.5 m`). That says nothing about whether the encounter was *safe*
by the standards the project is aligning with. This module replaces
"did not collide" with "did not lose Well Clear", and reports the encounter
against the six-step DAA procedure of ISO 21384-3:2023 rather than as three
booleans.

Design basis: devai/daa_scenario_study_iso_20260801.md

Well Clear
    No published standard defines a Well Clear volume for UAS-to-UAS
    encounters -- ASTM F3442 covers only "UAS avoids crewed aircraft" and
    explicitly defers UA-to-UA. So the values below are *derived*, and the
    derivation is stated here rather than hidden in a constant:

      UAS vs crewed : ASTM F3442 (2000 ft / 250 ft) scaled by the length ratio
                      of this simulation, 1/250          -> 2.44 m / 0.30 m
      UAS vs UAS    : five times the half-width of our airframe (0.25 m),
                      keeping the same 8:1 horizontal:vertical aspect ratio
                      as the ASTM "hockey puck"          -> 1.25 m / 0.30 m

    Similarity of this simulation to the NEDO/JRC flight demonstrations
    (5 km detection, 500 m recognition, 200 km/h closure):
      length 1/250, speed 1/25, time 1/10   (L/V = T is satisfied)
    which is why DO-365's 35 s modified-tau threshold appears here as 3.5 s.

Loss of Well Clear (LoDWC) follows the DO-365 form: all three of horizontal,
vertical and modified tau must be inside their thresholds simultaneously.
"""

from __future__ import annotations

import math
import threading

# --- similarity of this simulation to full-scale flight tests ---------------
SCALE_LENGTH = 1.0 / 250.0
SCALE_SPEED = 1.0 / 25.0
SCALE_TIME = 1.0 / 10.0


class WellClear:
    """A Well Clear volume plus the modified-tau threshold that goes with it."""

    def __init__(self, name, h_m, v_m, tau_s, basis):
        self.name = name
        self.h_m = h_m          # horizontal radius
        self.v_m = v_m          # vertical half-height
        self.tau_s = tau_s      # modified tau threshold
        self.basis = basis      # where the numbers come from (printed in reports)

    def __repr__(self):
        return f"WellClear({self.name}: h={self.h_m} m, v={self.v_m} m, tau={self.tau_s} s)"


UAS_VS_UAS = WellClear(
    "UAS-vs-UAS", 1.25, 0.30, 3.5,
    "5x airframe half-width, 8:1 aspect (no published UA-to-UA standard); "
    "tau = DO-365 35 s x time scale 1/10")

UAS_VS_MANNED = WellClear(
    "UAS-vs-crewed", 2.44, 0.30, 3.5,
    "ASTM F3442 2000 ft / 250 ft x length scale 1/250; "
    "tau = DO-365 35 s x time scale 1/10")


def tau_mod(range_m, closure_mps, dmod_m):
    """DO-365 modified tau: time to penetrate a protection disk of radius dmod.

    `closure_mps` is positive while closing. Returns +inf when not closing or
    when already inside the disk, so callers can compare with `<=` safely.
    """
    if closure_mps is None or closure_mps <= 1e-3 or range_m is None:
        return math.inf
    if range_m <= dmod_m:
        return 0.0
    return (range_m * range_m - dmod_m * dmod_m) / (range_m * closure_mps)


class RangeTracker:
    """Range rate from range measurements over a time window.

    The radar's Doppler channel gives an instantaneous closure rate, but our
    scenarios step the aircraft forward in discrete commanded moves, so Doppler
    reads ~0 during the pauses between them, and the range itself is noisy
    because a Monte-Carlo ray sampler lands on a different part of the target
    each frame.

    Differencing consecutive frames is therefore useless for the case that
    matters most: an overtake closes at a small fraction of the aircraft's own
    speed, so the per-frame range change is buried in the sampling noise and its
    SIGN flips. The rate is measured over a window instead, which is long enough
    that the real trend dominates. Doppler still wins when it is unambiguous.

    The window is a trade: long enough to beat the range noise of a Monte-Carlo
    radar, short enough that classifying the encounter does not cost a whole
    control tick of warning time (which it did at 2 s -- the manoeuvre then
    started below the tau threshold).
    """

    def __init__(self, alpha=0.5, window_s=1.2):
        self.alpha = alpha
        self.window_s = window_s
        self._hist = []            # [(t, range)] within the window
        self.closure = None        # m/s, positive while closing

    def update(self, t, range_m, doppler_mps=None):
        if range_m is None:
            return self.closure
        self._hist.append((t, range_m))
        cutoff = t - self.window_s
        while len(self._hist) > 2 and self._hist[0][0] < cutoff:
            self._hist.pop(0)
        measured = None
        if len(self._hist) >= 2:
            t0, r0 = self._hist[0]
            dt = t - t0
            if dt > 0.2:                       # need a real baseline
                measured = (r0 - range_m) / dt  # + while closing
        # Radar Doppler sign convention: negative = approaching.
        if doppler_mps is not None and abs(doppler_mps) > 0.15:
            measured = -doppler_mps
        if measured is None:
            return self.closure
        self.closure = measured if self.closure is None \
            else self.alpha * measured + (1.0 - self.alpha) * self.closure
        return self.closure


class SlopeTracker:
    """Windowed rate of change of any scalar -- e.g. the target's relative
    altitude, which is how a descending (landing) aircraft is told from one in
    level flight using nothing but the radar's elevation channel."""

    def __init__(self, alpha=0.5, window_s=1.2):
        self.alpha, self.window_s = alpha, window_s
        self._hist = []
        self.slope = None

    def update(self, t, value):
        if value is None:
            return self.slope
        self._hist.append((t, value))
        cutoff = t - self.window_s
        while len(self._hist) > 2 and self._hist[0][0] < cutoff:
            self._hist.pop(0)
        if len(self._hist) < 2:
            return self.slope
        t0, v0 = self._hist[0]
        dt = t - t0
        if dt <= 0.2:
            return self.slope
        measured = (value - v0) / dt
        self.slope = measured if self.slope is None \
            else self.alpha * measured + (1.0 - self.alpha) * self.slope
        return self.slope


class EncounterRecorder:
    """Samples true positions of two aircraft and scores the encounter.

    Thread-safe: the sampling loop usually runs in a background thread while
    the scenario drives the aircraft from the main thread.
    """

    def __init__(self, dwc=UAS_VS_UAS):
        self.dwc = dwc
        self._lock = threading.Lock()
        self.samples = 0
        self.min_h = math.inf       # horizontal separation at CPA
        self.v_at_cpa = math.inf    # vertical separation at that same instant
        self.t_cpa = None
        self.min_v = math.inf
        self.lodwc = False          # all three DO-365 conditions held at once
        self.lodwc_t = None
        self._prev = None           # (t, horizontal range)

    def sample(self, t, a, b):
        """`a`, `b` are (x, y, z) tuples in the env frame. None entries ignored."""
        if a is None or b is None:
            return
        h = math.hypot(a[0] - b[0], a[1] - b[1])
        v = abs(a[2] - b[2])
        with self._lock:
            self.samples += 1
            if h < self.min_h:
                self.min_h, self.v_at_cpa, self.t_cpa = h, v, t
            self.min_v = min(self.min_v, v)
            closure = None
            if self._prev is not None:
                dt = t - self._prev[0]
                if dt > 1e-3:
                    closure = (self._prev[1] - h) / dt
            self._prev = (t, h)
            if (not self.lodwc and h < self.dwc.h_m and v < self.dwc.v_m
                    and tau_mod(h, closure, self.dwc.h_m) <= self.dwc.tau_s):
                self.lodwc, self.lodwc_t = True, t

    @property
    def well_clear_kept(self):
        return not self.lodwc

    def summary(self):
        return (f"CPA h={self.min_h:.2f} m (DWC {self.dwc.h_m:.2f} m), "
                f"v={self.v_at_cpa:.2f} m (DWC {self.dwc.v_m:.2f} m), "
                f"LoDWC={'YES' if self.lodwc else 'no'}, samples={self.samples}")


class StepReport:
    """The six-step DAA procedure of ISO 21384-3:2023 as a scored checklist."""

    STEPS = [
        (1, "detection", "対象物の探知"),
        (2, "recognition", "ターゲットの認識"),
        (3, "manoeuvre", "回避機動"),
        (4, "check manoeuvre", "回避結果の確認"),
        (5, "return to route", "元ルートへの復帰"),
        (6, "fly the route", "元ルートでの飛行"),
    ]

    def __init__(self, scenario, dwc=UAS_VS_UAS):
        self.scenario = scenario
        self.dwc = dwc
        self._rows = {}     # step number -> (verdict, observation)

    def record(self, step, ok, observation):
        self._rows[step] = (bool(ok), observation)

    @property
    def passed(self):
        return bool(self._rows) and all(ok for ok, _ in self._rows.values())

    def render(self):
        w = max(len(o) for _, (_, o) in self._rows.items()) if self._rows else 10
        out = [
            "",
            f"=== DAA report: {self.scenario} ===",
            f"Well Clear: {self.dwc.name}  h={self.dwc.h_m:.2f} m  v={self.dwc.v_m:.2f} m  "
            f"tau={self.dwc.tau_s:.1f} s",
            f"  basis: {self.dwc.basis}",
            "",
            f"{'#':<3}{'step (ISO 21384-3)':<28}{'verdict':<9}observation",
            f"{'-'*3:<3}{'-'*27:<28}{'-'*8:<9}{'-' * min(w, 60)}",
        ]
        for num, en, ja in self.STEPS:
            if num not in self._rows:
                out.append(f"{num:<3}{en + ' / ' + ja:<28}{'SKIP':<9}(not exercised)")
                continue
            ok, obs = self._rows[num]
            out.append(f"{num:<3}{en + ' / ' + ja:<28}{'PASS' if ok else 'FAIL':<9}{obs}")
        out.append("")
        return "\n".join(out)
