"""Shared plumbing for the DAA scenarios (S-1 head-on, S-2 converging, S-3 overtaking).

Everything here is scenario-agnostic: PDU access, the radar read, and the
right-of-way rule selector of 航空法施行規則 §180-187. The scoring lives in
daa_metrics.py; the encounter geometry lives in each scenario script.

Sensing is expressed against the aircraft's RADAR FIT -- every radar it carries,
discovered from the manifest and the pdudef rather than hard-coded (issue #6).
A scenario asks "what is the nearest threat in this body-frame window", and the
answer comes from whichever radars can see into that window. The same scenario
therefore runs unchanged on a one-radar aircraft and on a forward+rear pair, and
gains the rear coverage automatically when it is there.

Design basis: devai/daa_scenario_study_iso_20260801.md
"""

from __future__ import annotations

import json
import math
import os
import struct
import threading
import time
from collections import namedtuple

import hakopy
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
SENSOR_REPO = os.path.dirname(HERE)
CFG = os.path.join(HERE, "config2", "webavatar-2-radar.json")

RADAR_CH, RADAR_SIZE = 19, 177424
# Second radar channel, present only when the stack was brought up with
# A2_DUAL_RADAR=1. Reading it when it does not exist simply yields nothing.
RADAR_REAR_CH = 21
# Channels that this stack does not have. Reading a channel that is absent from
# the pdudef raises, so the first failure is remembered and the channel is then
# skipped -- that is what lets one scenario run unchanged against both the
# single-radar and the dual-radar configuration.
_missing_channels = set()
POS_CH, POS_SIZE = 1, 72                       # geometry_msgs/Twist
POS_X_OFF, POS_Y_OFF, POS_Z_OFF = 24, 32, 40   # 9 doubles; linear x,y,z at index 3,4,5
POS_YAW_OFF = 64                               # angular.z, radians

# Radar returns in the sensor's ROS frame: x forward, y LEFT, z up.
# So azimuth = atan2(y, x) is POSITIVE when the target is on our LEFT.
#
# `source` names the radar the nearest return came from. With one radar it is
# always the same string and can be ignored; with several it is the difference
# between "something is behind us" and "the rear radar sees something".
Scan = namedtuple("Scan", "count rng doppler az_deg el_deg source",
                  defaults=(None,))
EMPTY_SCAN = Scan(0, None, 0.0, None, None, None)


# --- the radar fit ----------------------------------------------------------
# Which radars this aircraft carries, where they point, and where each one
# publishes. Discovered rather than declared twice: the manifest already says
# what is fitted and the pdudef already says which channel each PDU goes to --
# the same two files the C++ bridge resolves its own wiring from (#5).
#
# az_lo/az_hi are the azimuth window in the BODY frame (the mount yaw is already
# folded in), so they are directly comparable with a bearing reported by any
# other radar on the same aircraft. They may run past +/-180 (a rear sector is
# 150..210), which is what keeps the interval contiguous.
#
# el_lo/el_hi are the elevation window (issue #7). A mount has no pitch, so
# elevation is the same in the sensor and body frames. It is NOT symmetric in
# general: the sector that matters for approach traffic lies below the horizon,
# so an approach radar is placed at e.g. -35..+10 rather than +/-22.5.
RadarUnit = namedtuple("RadarUnit",
                       "sensor_id pdu_name channel pdu_size mount_yaw_deg "
                       "az_lo az_hi el_lo el_hi range_m")

# The wiring every launcher used before any of this was configurable. Kept as
# the floor of the resolution order so a stack brought up without A2_PDUDEF or a
# manifest behaves exactly as it always did.
DEFAULT_UNIT = RadarUnit("front_radar", "radar_points", RADAR_CH, RADAR_SIZE,
                         0.0, -30.0, 30.0, -10.0, 10.0, 20.0)

_fit_cache = {}


def _pdudef_channels(path, robot):
    """org_name -> (channel_id, pdu_size) for one robot.

    The Python twin of runtime/pdudef_channels.hpp, including its ordering:
    readers first, then writers, so a name declared on both sides takes the
    writer's width -- that is the entry a publisher must honour.
    """
    out = {}
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return out
    for r in doc.get("robots", []):
        if r.get("name") != robot:
            continue
        for key in ("shm_pdu_readers", "shm_pdu_writers"):
            for p in r.get(key, []) or []:
                name, ch = p.get("org_name"), p.get("channel_id")
                if name is None or ch is None:
                    continue
                out[name] = (int(ch), int(p.get("pdu_size", 0)))
        break
    return out


def _manifest_radars(path):
    """The radar components of an A-2 manifest, in declaration order."""
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return []
    out = []
    for c in doc.get("components", []):
        if c.get("type") != "radar":
            continue
        params = c.get("params", {}) or {}
        mount = c.get("mount", {}) or {}
        yaw = float(mount.get("yaw_deg", 0.0))
        # The sampler's own rule: an explicit azimuth_start/end window if both
        # are given, otherwise a window centred on the boresight
        # (radar_math.hpp MakeWindow). Restating it here rather than guessing
        # keeps the coverage report honest for the rear-sector manifests.
        lo, hi = params.get("azimuth_start_deg"), params.get("azimuth_end_deg")
        if lo is None or hi is None:
            h = float(params.get("horizontal_fov_deg", 30.0))
            lo, hi = -0.5 * h, 0.5 * h
        # Elevation follows the same rule (#7). The mount carries no pitch, so
        # nothing is added here -- unlike azimuth, which picks up the mount yaw.
        elo, ehi = params.get("elevation_start_deg"), params.get("elevation_end_deg")
        if elo is None or ehi is None:
            v = float(params.get("vertical_fov_deg", 10.0))
            elo, ehi = -0.5 * v, 0.5 * v
        out.append(dict(sensor_id=c.get("id", "radar"),
                        pdu_name=c.get("pdu_name", "radar_scan"),
                        mount_yaw_deg=yaw,
                        az_lo=float(lo) + yaw, az_hi=float(hi) + yaw,
                        el_lo=float(elo), el_hi=float(ehi),
                        range_m=float(params.get("range", 20.0))))
    return out


def _stack_descriptor():
    """What the launcher recorded about the running stack, or {}.

    A scenario is normally started from a different shell than the launcher --
    demo_all.sh runs the two as separate commands -- so the launcher cannot pass
    the manifest in the environment. It writes it to logs/stack.json instead,
    next to the bridge PID files S-8 already relies on. Stale or absent, the
    caller falls back to the baseline manifest, which is the configuration every
    scenario assumed before any of this was discoverable.
    """
    path = os.environ.get("A2_STACK_JSON") or os.path.join(HERE, "logs", "stack.json")
    try:
        with open(path) as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return {}
    return doc if isinstance(doc, dict) else {}


def _pdudef_path():
    """The pdudef the master was started with: env, then the launcher's record."""
    p = os.environ.get("A2_PDUDEF")
    if p and os.path.exists(p):
        return p
    p = _stack_descriptor().get("pdudef")
    if p and os.path.exists(p):
        return p
    return CFG


def _manifest_path(robot):
    """Manifest for one robot: env override, the launcher's record, then the default.

    Each aircraft may carry a different fit, which is why the record is per
    robot. A2_MANIFEST alone still describes both -- the common case, and what
    every existing invocation sets.
    """
    for var in (f"A2_MANIFEST_{robot}", "A2_MANIFEST"):
        p = os.environ.get(var)
        if p and os.path.exists(p):
            return p
    p = (_stack_descriptor().get("manifests") or {}).get(robot)
    if p and os.path.exists(p):
        return p
    fallback = os.path.join(SENSOR_REPO, "config", "a2", "drone-a2-sensors.json")
    return fallback if os.path.exists(fallback) else None


def radar_fit(robot):
    """Every radar `robot` carries, as a tuple of RadarUnit.

    Resolution mirrors sensor_bridge_multi's three layers, weakest first:
      1) the built-in single forward radar on ch19
      2) the manifest (what is fitted) crossed with the pdudef (where it goes)
      3) A2_RADAR_CHANNELS="19,21" -- an explicit escape hatch

    A radar that the manifest fits but the pdudef gives no channel is dropped:
    it is exactly the sensor the bridge refuses to publish, so reading a channel
    for it would read someone else's PDU.
    """
    if robot in _fit_cache:
        return _fit_cache[robot]

    override = os.environ.get("A2_RADAR_CHANNELS")
    if override:
        units = []
        for i, tok in enumerate(override.split(",")):
            tok = tok.strip()
            if not tok:
                continue
            units.append(DEFAULT_UNIT._replace(
                sensor_id=f"radar{i}", channel=int(tok),
                az_lo=None, az_hi=None, el_lo=None, el_hi=None))
        if units:
            _fit_cache[robot] = tuple(units)
            return _fit_cache[robot]

    declared = _pdudef_channels(_pdudef_path(), robot)
    # The primary radar is "radar_points" in the pdudef but "radar_scan" in the
    # manifest (RadarSensor's default PDU name). The bridge bridges the two; so
    # must we, or the fit loses the radar every scenario depends on.
    if "radar_points" in declared and "radar_scan" not in declared:
        declared["radar_scan"] = declared["radar_points"]

    manifest = _manifest_path(robot)
    fitted = _manifest_radars(manifest) if manifest else []

    units = []
    for r in fitted:
        spec = declared.get(r["pdu_name"])
        if spec is None:
            # Fitted but unwired. Silent by design in the bridge; here it means
            # "this aircraft cannot use that radar", which the fit must reflect.
            continue
        ch, size = spec
        units.append(RadarUnit(r["sensor_id"], r["pdu_name"], ch,
                               size or RADAR_SIZE, r["mount_yaw_deg"],
                               r["az_lo"], r["az_hi"], r["el_lo"], r["el_hi"],
                               r["range_m"]))
    if not units:
        units = [DEFAULT_UNIT]
    _fit_cache[robot] = tuple(units)
    return _fit_cache[robot]


def fit_summary(robot):
    """One line naming the fit and the sky it covers. For the run logs."""
    units = radar_fit(robot)
    parts = []
    for u in units:
        w = ("az [--]" if u.az_lo is None
             else f"az [{u.az_lo:+.0f},{u.az_hi:+.0f}]")
        if u.el_lo is not None:
            w += f" el [{u.el_lo:+.0f},{u.el_hi:+.0f}]"
        parts.append(f"{u.sensor_id} ch{u.channel} {w}")
    cov = azimuth_coverage_deg(robot)
    return f"{len(units)} radar(s): " + " + ".join(parts) + \
           (f" | {cov:.0f} deg of 360 covered" if cov is not None else "")


def azimuth_coverage_deg(robot):
    """Total azimuth covered by the fit, overlaps counted once. None if unknown."""
    spans = [(u.az_lo, u.az_hi) for u in radar_fit(robot) if u.az_lo is not None]
    if not spans:
        return None
    # Normalise each window onto [0,360) and merge; a window may wrap (150..210
    # does not, but -30..30 does), so split the wrapping ones in two.
    segs = []
    for lo, hi in spans:
        span = hi - lo
        if span >= 360.0:
            return 360.0
        lo %= 360.0
        hi = lo + span
        if hi > 360.0:
            segs.append((lo, 360.0))
            segs.append((0.0, hi - 360.0))
        else:
            segs.append((lo, hi))
    segs.sort()
    total, cur_lo, cur_hi = 0.0, segs[0][0], segs[0][1]
    for lo, hi in segs[1:]:
        if lo > cur_hi:
            total += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    return total + (cur_hi - cur_lo)


def covers_az(robot, az_deg):
    """Can any radar on `robot` look at this body-frame bearing? None if unknown."""
    known = False
    for u in radar_fit(robot):
        if u.az_lo is None:
            continue
        known = True
        d = (az_deg - u.az_lo) % 360.0
        if d <= (u.az_hi - u.az_lo):
            return True
    return False if known else None


def _unit_sees(u, az_deg, el_deg, rng_m):
    """Is this bearing inside one radar's window and range?"""
    if u.az_lo is None or u.el_lo is None:
        return None
    if (az_deg - u.az_lo) % 360.0 > (u.az_hi - u.az_lo):
        return False
    if not (u.el_lo <= el_deg <= u.el_hi):
        return False
    return rng_m is None or rng_m <= u.range_m


def covers_el(robot, el_deg):
    """Can any radar on `robot` look this far above/below the horizon? (#7)

    Elevation is where the shipped fit is thinnest: a symmetric 20 deg FOV
    follows a height difference of only r*tan(10) ~= 0.18r, so traffic that is
    both close and below -- an aircraft on final approach, exactly the case
    S-5 has to confirm -- falls out of the window.
    """
    known = False
    for u in radar_fit(robot):
        if u.el_lo is None:
            continue
        known = True
        if u.el_lo <= el_deg <= u.el_hi:
            return True
    return False if known else None


def elevation_coverage_deg(robot):
    """Total elevation covered by the fit, overlaps counted once. None if unknown.

    Reported alongside the azimuth figure so a fit is described by the solid
    angle it actually watches rather than by "60 degrees", which only ever
    named the azimuth half of it.
    """
    spans = sorted((u.el_lo, u.el_hi) for u in radar_fit(robot)
                   if u.el_lo is not None)
    if not spans:
        return None
    total, cur_lo, cur_hi = 0.0, spans[0][0], spans[0][1]
    for lo, hi in spans[1:]:
        if lo > cur_hi:
            total += cur_hi - cur_lo
            cur_lo, cur_hi = lo, hi
        else:
            cur_hi = max(cur_hi, hi)
    return total + (cur_hi - cur_lo)


def body_bearing(own_xyz, own_yaw_deg, target_xyz):
    """(azimuth, elevation, range) of a known target in our body frame.

    Uses TRUE positions, so it is a diagnostic, never an input to a decision:
    it answers "where should the radar have been looking", which is what turns
    a silent miss into evidence. Sign conventions match the radar's own ROS
    frame -- azimuth positive to the LEFT, elevation positive UP.
    """
    if own_xyz is None or target_xyz is None:
        return (None, None, None)
    dx = target_xyz[0] - own_xyz[0]
    dy = target_xyz[1] - own_xyz[1]
    dz = target_xyz[2] - own_xyz[2]
    yaw = math.radians(own_yaw_deg)
    # Rotate the world offset into the body frame (yaw only; the aircraft is
    # level in every scenario here).
    fx = dx * math.cos(yaw) + dy * math.sin(yaw)
    fy = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return (math.degrees(math.atan2(fy, fx)),
            math.degrees(math.atan2(dz, math.hypot(fx, fy))),
            math.sqrt(dx * dx + dy * dy + dz * dz))


def coverage_gap(robot, az_deg, el_deg, rng_m=None, az_half=None, el_half=None):
    """Why the target at this bearing cannot be seen -- or None if it can.

    A miss has several causes that look identical from the PDU (nothing came
    back), and they call for different answers: outside the azimuth sector is a
    case for another radar, outside the elevation window is a case for a wider
    or tilted one, beyond range is a case for more power. Naming which one
    applies is the whole difference between "the radar did not see it" and
    "the radar was never pointed at it".

    `az_half`/`el_half` are the window the CALLER is applying on top of the
    radar's own. Passing them matters: a scenario that filters harder than its
    radar sees is blind by its own hand, and without this the gap would be
    blamed on the sensor -- or, worse, reported as "in coverage" while the
    returns were being thrown away one layer up (#13).
    """
    if az_deg is None or el_deg is None:
        return None
    if az_half is not None and abs(az_deg) > az_half:
        return (f"azimuth {az_deg:+.0f} deg outside the SCENARIO's own "
                f"+/-{az_half:.0f} deg window (the radar may well see it)")
    if el_half is not None and abs(el_deg) > el_half:
        return (f"elevation {el_deg:+.0f} deg outside the SCENARIO's own "
                f"+/-{el_half:.0f} deg window (the radar may well see it)")
    units = [u for u in radar_fit(robot) if u.az_lo is not None and u.el_lo is not None]
    if not units:
        return None                       # coverage unknown; claim nothing
    if any(_unit_sees(u, az_deg, el_deg, rng_m) for u in units):
        return None
    if not any(_unit_sees(u, az_deg, el_deg, None) for u in units):
        # Not a range problem: no window contains the bearing at all. Say which
        # axis is at fault -- if azimuth is fine somewhere, elevation is.
        if covers_az(robot, az_deg):
            return (f"elevation {el_deg:+.0f} deg outside "
                    + " / ".join(f"[{u.el_lo:+.0f},{u.el_hi:+.0f}]" for u in units))
        if covers_el(robot, el_deg):
            return (f"azimuth {az_deg:+.0f} deg outside "
                    + " / ".join(f"[{u.az_lo:+.0f},{u.az_hi:+.0f}]" for u in units))
        return f"bearing az {az_deg:+.0f} el {el_deg:+.0f} deg outside every window"
    return f"range {rng_m:.2f} m beyond {max(u.range_m for u in units):.1f} m"


def _read_channel(robot, channel, size):
    """Raw PDU bytes, or None if this stack has no such channel."""
    if (robot, channel) in _missing_channels:
        return None
    try:
        raw = hakopy.pdu_read(robot, channel, size)
    except RuntimeError:
        _missing_channels.add((robot, channel))
        return None
    return raw or None


def _hits(raw, az_half, el_half, r_max, min_abs_doppler, mount_yaw_deg=0.0):
    """Returns inside an angular window, as (range, doppler, az, el).

    The window is expressed in ANGLES (not a box around the boresight) so it can
    be opened past 45 deg for the crossing scenarios, where the target sits far
    off boresight, and it is a BODY-frame window: a radar bolted on at a mount
    yaw reports bearings relative to its own boresight, and those have to be
    rotated before they can be compared with another radar's on the same
    aircraft, or with a right-of-way rule that speaks of "our right".

    `min_abs_doppler` keeps only MOVING returns. Static structure comes back at
    Doppler ~0, so this is the classic moving-target filter, and it is what makes
    an aircraft findable inside wall clutter (S-7). The JRC sea trials describe
    exactly this problem: the received signal carries reflections from waves,
    terrain and buildings alongside the threat.

    `az_half`/`el_half` of None means "do not narrow what the radar reports".
    That matters once a window stops being symmetric (#7): a scenario asking for
    |el| <= 12 would throw away precisely the downward returns an approach radar
    was fitted to get, and the loss would look exactly like a sensor that could
    not see.
    """
    pc = pdu_to_py_PointCloud2(bytes(raw))
    data = bytes(pc.data)
    step = pc.point_step or 16
    n = min(int(pc.width), len(data) // step) if pc.width else len(data) // step
    out = []
    for i in range(n):
        x, y, z, v = struct.unpack_from("<ffff", data, i * step)
        r = math.sqrt(x * x + y * y + z * z)
        if r < 0.05 or (r_max is not None and r > r_max):
            continue
        if min_abs_doppler is not None and abs(v) < min_abs_doppler:
            continue    # static clutter
        az = math.degrees(math.atan2(y, x))
        if mount_yaw_deg:
            # Fold to (-180,180] so a rear mount does not push bearings out of
            # range. Skipped entirely at yaw 0 -- the current fits -- so this
            # cannot perturb a number any existing scenario reads.
            az = (az + mount_yaw_deg + 180.0) % 360.0 - 180.0
        el = math.degrees(math.atan2(z, math.hypot(x, y)))
        if (az_half is None or abs(az) <= az_half) and \
           (el_half is None or abs(el) <= el_half):
            out.append((r, v, az, el))
    return out


def scan(robot, az_half=15.0, el_half=15.0, r_max=None, min_abs_doppler=None,
         channel=RADAR_CH):
    """Nearest return from ONE radar channel, with its bearing.

    Returns EMPTY_SCAN when nothing qualifies. `doppler` is negative while the
    target is closing (radar convention); `az_deg` is positive when the target
    is on our LEFT. Prefer scan_best(), which asks the whole fit -- this is the
    single-channel primitive, kept for probes that must name a channel.
    """
    raw = _read_channel(robot, channel, RADAR_SIZE)
    if raw is None:
        return EMPTY_SCAN
    hits = _hits(raw, az_half, el_half, r_max, min_abs_doppler)
    if not hits:
        return EMPTY_SCAN
    hits.sort()
    r, v, az, el = hits[0]
    return Scan(len(hits), r, v, az, el, f"ch{channel}")


def scan_units(robot, units=None, az_half=15.0, el_half=15.0, r_max=None,
               min_abs_doppler=None):
    """What each radar of the fit sees, keyed by sensor id.

    The per-sensor breakdown is what lets a scenario say WHICH radar found the
    threat -- the difference between "the aircraft is blind astern" and "the
    rear radar picked the follower up at 2.1 m".
    """
    out = {}
    for u in (radar_fit(robot) if units is None else units):
        raw = _read_channel(robot, u.channel, u.pdu_size)
        if raw is None:
            out[u.sensor_id] = EMPTY_SCAN
            continue
        hits = _hits(raw, az_half, el_half, r_max, min_abs_doppler,
                     u.mount_yaw_deg)
        if not hits:
            out[u.sensor_id] = EMPTY_SCAN._replace(source=u.sensor_id)
            continue
        hits.sort()
        r, v, az, el = hits[0]
        out[u.sensor_id] = Scan(len(hits), r, v, az, el, u.sensor_id)
    return out


def scan_best(robot, channels=None, units=None, **kw):
    """Nearest threat in a body-frame window, across EVERY radar the aircraft has.

    This is the sensing call the avoidance logic should make. An aircraft may
    carry one radar or several (forward sector + rear sector), and the decision
    it has to take -- is something closing, and on which side -- does not depend
    on which transceiver saw it. The window is in the body frame, so each radar
    contributes only what genuinely falls inside it; a rear sector adds returns
    at 180 deg and changes nothing about a +/-15 deg forward look.

    With a single-radar fit this is identical to scan() on ch19, which is what
    makes it safe to use everywhere. `channels` still forces an explicit channel
    list for probes; `units` forces a subset of the fit (S-8 uses it to keep
    sensing on the radars that have not failed).
    """
    if channels is not None:
        units = tuple(DEFAULT_UNIT._replace(sensor_id=f"ch{c}", channel=c,
                                            az_lo=None, az_hi=None,
                                            el_lo=None, el_hi=None)
                      for c in channels)
    best = EMPTY_SCAN
    total = 0
    for s in scan_units(robot, units=units, **kw).values():
        total += s.count
        if s.rng is not None and (best.rng is None or s.rng < best.rng):
            best = s
    return best if best.rng is None else best._replace(count=total)


def radar_hash(robot, channel=RADAR_CH, size=RADAR_SIZE):
    """Fingerprint of one radar's raw payload.

    A sensor that has stopped producing data does not disappear from shared
    memory -- the last frame it wrote just stays there forever. Freshness has to
    be judged by whether the CONTENT changes, which is what this supports
    (ISO 15964 4.2 d: fault detection and isolation).
    """
    raw = _read_channel(robot, channel, size)
    if raw is None:
        return None
    return hash(bytes(raw))


class StaleWatch:
    """Declares a sensor faulty once its output has stopped changing.

    `timeout_s` has to be longer than the sensor's own frame interval (the bridge
    publishes at 20 Hz) but short enough to leave time to act on the fault.
    """

    def __init__(self, timeout_s=1.5):
        self.timeout_s = timeout_s
        self._last_change = None
        self._last_hash = None
        self.stale = False
        self.stale_since = None

    def update(self, t, fingerprint):
        if fingerprint is None:
            return self.stale
        if self._last_hash is None or fingerprint != self._last_hash:
            self._last_hash, self._last_change = fingerprint, t
            if self.stale:
                self.stale, self.stale_since = False, None   # recovered
            return self.stale
        if not self.stale and (t - self._last_change) >= self.timeout_s:
            self.stale, self.stale_since = True, t
        return self.stale

    @property
    def age(self):
        return None if self._last_change is None else self._last_change


class FitWatch:
    """Health of the whole radar fit, one StaleWatch per radar.

    ISO 15964 6.2.5 asks that the system still operate safely when any SINGLE
    component fails. That is only a meaningful test if losing one radar is told
    apart from losing them all: an aircraft with a forward and a rear radar that
    loses the rear one is DEGRADED (it has lost coverage astern) but not blind,
    and the right response is to keep flying the encounter on what is left --
    not to declare a contingency it does not need. Watching only the primary
    radar, as before, could see neither distinction.

    With a single-radar fit `blind` is exactly the old StaleWatch verdict.
    """

    def __init__(self, timeout_s=1.5):
        self.timeout_s = timeout_s
        self._watch = {}
        self.fresh = ()          # sensor ids still producing
        self.stale = ()          # sensor ids that have stopped

    def update(self, t, robot):
        fresh, stale = [], []
        for u in radar_fit(robot):
            w = self._watch.get(u.sensor_id)
            if w is None:
                w = self._watch[u.sensor_id] = StaleWatch(self.timeout_s)
            if w.update(t, radar_hash(robot, u.channel, u.pdu_size)):
                stale.append(u.sensor_id)
            else:
                fresh.append(u.sensor_id)
        self.fresh, self.stale = tuple(fresh), tuple(stale)
        return self

    @property
    def blind(self):
        """No radar is producing -- the aircraft cannot see at all."""
        return bool(self.stale) and not self.fresh

    @property
    def degraded(self):
        """Some radars lost, some still working."""
        return bool(self.stale) and bool(self.fresh)

    def healthy_units(self, robot):
        """The subset of the fit still worth reading (all of it, if none failed)."""
        return tuple(u for u in radar_fit(robot) if u.sensor_id in self.fresh)

    def summary(self):
        if not self.stale:
            return f"all {len(self.fresh)} radar(s) healthy"
        return (f"{len(self.stale)} of {len(self.stale) + len(self.fresh)} "
                f"radar(s) stale: {', '.join(self.stale)}"
                + (f" | still sensing on: {', '.join(self.fresh)}" if self.fresh
                   else " | BLIND"))


def read_xyz(robot):
    """True (x, y, z) from the pos PDU via a raw, thread-safe read. None if empty."""
    raw = hakopy.pdu_read(robot, POS_CH, POS_SIZE)
    if not raw or len(raw) < POS_SIZE:
        return None
    b = bytes(raw)
    return (struct.unpack_from("<d", b, POS_X_OFF)[0],
            struct.unpack_from("<d", b, POS_Y_OFF)[0],
            struct.unpack_from("<d", b, POS_Z_OFF)[0])


def read_yaw_deg(robot):
    """True yaw in degrees from the pos PDU. None if empty.

    Needed for diagnosis, not for flying: the drone service decides a move is
    complete only once x, y, z AND YAW have all been within tolerance for 100
    consecutive control steps, so a yaw that never satisfies it leaves the
    aircraft stuck in MOVING with no outward sign (#10).
    """
    raw = hakopy.pdu_read(robot, POS_CH, POS_SIZE)
    if not raw or len(raw) < POS_SIZE:
        return None
    return math.degrees(struct.unpack_from("<d", bytes(raw), POS_YAW_OFF)[0])


def connect(names):
    """Arm every drone from THIS process -- a client in another process cannot
    write commands (see the B-3 notes).

    Also states each aircraft's radar fit. A scenario's numbers only mean
    something against the sensors that produced them, and the fit is now decided
    by the manifest the stack was launched with rather than by the scenario, so
    the log has to say which one it got.
    """
    import hakoniwa_pdu.apps.drone.hakosim as hakosim
    clients = {}
    for name in names:
        c = hakosim.MultirotorClient(CFG, name)
        c.confirmConnection()
        c.enableApiControl(True)
        c.armDisarm(True)
        clients[name] = c
        print(f"[fit] {name}: {fit_summary(name)}", flush=True)
    return clients


class Sampler(threading.Thread):
    """Feeds true positions of two aircraft into an EncounterRecorder."""

    def __init__(self, recorder, a, b, period=0.03):
        super().__init__(daemon=True)
        self.rec, self.a, self.b, self.period = recorder, a, b, period
        # NOT `_stop`: threading.Thread already owns that name internally and
        # shadowing it breaks join().
        self._halt = threading.Event()

    def run(self):
        while not self._halt.is_set():
            self.rec.sample(time.time(), read_xyz(self.a), read_xyz(self.b))
            time.sleep(self.period)

    def stop(self):
        self._halt.set()
        self.join(timeout=1.0)


# --- right-of-way rules: 航空法施行規則 §181, §182, §185, §186 ----------------
HEAD_ON = "head-on §182"          # both alter course to the right
GIVE_WAY = "give way §181"        # sees the other on its RIGHT -> yields
STAND_ON = "stand on §181/§186"   # sees the other on its LEFT -> holds course and speed
OVERTAKE = "overtake §185"        # passes on the RIGHT of the aircraft ahead

HEAD_ON_HALF_DEG = 15.0           # "正面またはこれに近い角度" -- within this is head-on


def role_from_bearing(az_deg, closing, overtaking=False):
    """Pick the rule that applies, from what our own radar sees.

    This needs only the bearing of the target in our body frame, which is what
    makes it implementable: §181 asks whether the other aircraft is on our
    right, not what its heading is.
    """
    if az_deg is None or not closing:
        return None
    if overtaking:
        return OVERTAKE
    if abs(az_deg) <= HEAD_ON_HALF_DEG:
        return HEAD_ON
    return GIVE_WAY if az_deg < 0.0 else STAND_ON


def classify_encounter(az_deg, closure, own_speed):
    """Same as role_from_bearing, but tells head-on from overtaking.

    A target dead ahead is §182 (head-on) or §185 (overtaking) depending on
    which way it is travelling, and the radar does not measure the target's
    heading. The closure rate does the work instead: closing at roughly twice
    our own speed means the other aircraft is coming at us, while a target we
    are creeping up on closes at much less than our own speed. This is the
    cheapest form of the "recognition" step that ISO 15964 3.8 asks for.
    """
    if az_deg is None or closure is None or closure <= 0.0:
        return None
    if abs(az_deg) > HEAD_ON_HALF_DEG:
        return GIVE_WAY if az_deg < 0.0 else STAND_ON
    # The natural threshold is our OWN speed, not an arbitrary fraction of it:
    #   same direction  -> the target recedes as we advance -> closure < own
    #   opposing        -> both speeds add                  -> closure > own
    # 0.9 leaves margin for the noise in a Monte-Carlo range measurement.
    if own_speed and own_speed > 0.05 and closure < 0.9 * own_speed:
        return OVERTAKE
    return HEAD_ON


class SpeedTracker:
    """Own ground speed from successive true positions."""

    def __init__(self, alpha=0.5):
        self.alpha, self._prev, self.speed = alpha, None, None

    def update(self, t, p):
        if p is None:
            return self.speed
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                v = math.hypot(p[0] - self._prev[1][0], p[1] - self._prev[1][1]) / dt
                self.speed = v if self.speed is None else \
                    self.alpha * v + (1.0 - self.alpha) * self.speed
        self._prev = (t, p)
        return self.speed


Flight = namedtuple("Flight", "ok arrived elapsed err start end attempts cancels")


def cancel_move(client, timeout_s=3.0):
    """Return a stuck aircraft to HOVERING so it will accept commands again.

    The drone service is a state machine (drone-pro drone_service_api.hpp): a
    move is accepted ONLY in HOVERING, and MOVING is left only when the move is
    judged complete. If that judgement never fires, the aircraft keeps its last
    setpoint and every later command is dropped without a word -- the scenario
    goes on issuing legs to an aircraft that will never fly them again.

    The service does provide a way out: while MOVING it watches for the move
    PDU's `request` flag going false and treats that as a cancel, which takes it
    back to HOVERING. That is what this writes. hakosim has no wrapper for it
    because its own API never needs one -- it assumes every move completes.

    Returns True once the cancel is acknowledged (the service writes `result`
    back), False on timeout.
    """
    from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_pytype_HakoDroneCmdMove import HakoDroneCmdMove
    from hakoniwa_pdu.pdu_msgs.hako_msgs.pdu_conv_HakoDroneCmdMove import (
        py_to_pdu_HakoDroneCmdMove, pdu_to_py_HakoDroneCmdMove)

    name = client.get_vehicle_name(None)
    cmd = HakoDroneCmdMove()
    cmd.header.request = 0
    cmd.header.result = 0
    cmd.header.result_code = 0
    if not client.pdu_manager.flush_pdu_raw_data_nowait(
            name, 'drone_cmd_move', py_to_pdu_HakoDroneCmdMove(cmd)):
        return False

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        client.pdu_manager.run_nowait()
        raw = client.pdu_manager.read_pdu_raw_data(name, 'drone_cmd_move')
        if raw:
            got = pdu_to_py_HakoDroneCmdMove(raw)
            if got.header.result == 1:
                # Consume the acknowledgement, exactly as hakosim._wait_res does,
                # so the next move's wait does not mistake it for its own.
                got.header.result = 0
                client.pdu_manager.flush_pdu_raw_data_nowait(
                    name, 'drone_cmd_move', py_to_pdu_HakoDroneCmdMove(got))
                return True
        time.sleep(0.1)
    return False


def fly_to(client, robot, x, y, z, speed, yaw_deg, timeout_sec, tol=0.35,
           retries=0, settle_s=0.0, log=None):
    """moveToPosition, then CHECK -- and say what actually happened.

    `moveToPosition` returns False when its own arrival wait times out, and the
    scenarios were discarding that value: a leg that never completed then read
    exactly like one that did, and the failure only surfaced several phases
    later as "the aircraft is not where it should be". Worse, the arrival wait
    polls once a SECOND (hakosim._wait_res), so a leg can be reported late by up
    to a second of wall clock even when the aircraft is already there.

    So: fly, then compare the TRUE position against the target, and return both
    facts. `arrived` is the one worth branching on -- it is measured, whereas
    `ok` is only what the command service claimed. `retries` re-issues the same
    leg while it has not arrived, which is what a real controller would do and
    what turns a slow leg into a slower one instead of a failed scenario.

    A plain re-issue is not enough on its own: an aircraft the service still
    believes to be MOVING ignores new commands entirely, so the retry has to
    CANCEL first (see cancel_move) and only then command again. Retrying without
    the cancel measurably does nothing -- two identical attempts leave the
    aircraft on the same metre.
    """
    p0 = read_xyz(robot)
    t0 = time.time()
    ok = False
    attempts = cancels = 0
    for i in range(max(1, retries + 1)):
        if i > 0 and cancel_move(client):
            cancels += 1
        attempts += 1
        ok = client.moveToPosition(x, y, z, speed, yaw_deg=yaw_deg,
                                   timeout_sec=timeout_sec)
        if settle_s > 0.0:
            time.sleep(settle_s)
        p = read_xyz(robot)
        if p is not None and math.dist(p, (x, y, z)) <= tol:
            break
    p1 = read_xyz(robot)
    err = math.dist(p1, (x, y, z)) if p1 is not None else float("inf")
    f = Flight(bool(ok), err <= tol, time.time() - t0, err, p0, p1, attempts, cancels)
    if log:
        ps = f"({p1[0]:+.2f},{p1[1]:+.2f},{p1[2]:+.2f})" if p1 else "??"
        yw = read_yaw_deg(robot)
        ys = f" yaw={yw:+.2f}" if yw is not None else ""
        print(f"  [fly] {log} {robot} -> ({x:+.2f},{y:+.2f},{z:+.2f}) "
              f"cmd_ok={f.ok} arrived={f.arrived} err={f.err:.2f} m "
              f"in {f.elapsed:.1f}s x{f.attempts}"
              + (f" (cancel x{f.cancels})" if f.cancels else "")
              + f" at {ps}{ys}", flush=True)
    return f


def yaw_for_heading(dx, dy):
    """Yaw in degrees for a ground track (dx, dy). +x is yaw 0, +y is yaw 90."""
    return math.degrees(math.atan2(dy, dx))
