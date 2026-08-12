"""Shared plumbing for the DAA scenarios (S-1 head-on, S-2 converging, S-3 overtaking).

Everything here is scenario-agnostic: PDU access, the forward-cone radar read,
and the right-of-way rule selector of 航空法施行規則 §180-187. The scoring lives
in daa_metrics.py; the encounter geometry lives in each scenario script.

Design basis: devai/daa_scenario_study_iso_20260801.md
"""

from __future__ import annotations

import math
import os
import struct
import threading
import time
from collections import namedtuple

import hakopy
from hakoniwa_pdu.pdu_msgs.sensor_msgs.pdu_conv_PointCloud2 import pdu_to_py_PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
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

# Radar returns in the sensor's ROS frame: x forward, y LEFT, z up.
# So azimuth = atan2(y, x) is POSITIVE when the target is on our LEFT.
Scan = namedtuple("Scan", "count rng doppler az_deg el_deg")
EMPTY_SCAN = Scan(0, None, 0.0, None, None)


def scan(robot, az_half=15.0, el_half=15.0, r_max=None, min_abs_doppler=None,
         channel=RADAR_CH):
    """Nearest return inside an angular window, with its bearing.

    The window is expressed in ANGLES (not a box around the boresight) so it can
    be opened past 45 deg for the crossing scenarios, where the target sits far
    off boresight. Returns EMPTY_SCAN when nothing qualifies. `doppler` is
    negative while the target is closing (radar convention); `az_deg` is
    positive when the target is on our LEFT.

    `min_abs_doppler` keeps only MOVING returns. Static structure comes back at
    Doppler ~0, so this is the classic moving-target filter, and it is what makes
    an aircraft findable inside wall clutter (S-7). The JRC sea trials describe
    exactly this problem: the received signal carries reflections from waves,
    terrain and buildings alongside the threat.
    """
    if (robot, channel) in _missing_channels:
        return EMPTY_SCAN
    try:
        raw = hakopy.pdu_read(robot, channel, RADAR_SIZE)
    except RuntimeError:
        _missing_channels.add((robot, channel))
        return EMPTY_SCAN
    if not raw:
        return EMPTY_SCAN
    pc = pdu_to_py_PointCloud2(bytes(raw))
    data = bytes(pc.data)
    step = pc.point_step or 16
    n = min(int(pc.width), len(data) // step) if pc.width else len(data) // step
    hits = []
    for i in range(n):
        x, y, z, v = struct.unpack_from("<ffff", data, i * step)
        r = math.sqrt(x * x + y * y + z * z)
        if r < 0.05 or (r_max is not None and r > r_max):
            continue
        if min_abs_doppler is not None and abs(v) < min_abs_doppler:
            continue    # static clutter
        az = math.degrees(math.atan2(y, x))
        el = math.degrees(math.atan2(z, math.hypot(x, y)))
        if abs(az) <= az_half and abs(el) <= el_half:
            hits.append((r, v, az, el))
    if not hits:
        return EMPTY_SCAN
    hits.sort()
    r, v, az, el = hits[0]
    return Scan(len(hits), r, v, az, el)


def scan_best(robot, channels=(RADAR_CH, RADAR_REAR_CH), **kw):
    """Nearest return across several radar channels.

    An aircraft may carry more than one radar (forward sector + rear sector), and
    the avoidance logic wants the nearest threat regardless of which sensor saw
    it. Channels that do not exist read back empty and are skipped, so the same
    call works on a single-radar stack.
    """
    best = EMPTY_SCAN
    total = 0
    for ch in channels:
        s = scan(robot, channel=ch, **kw)
        if s.rng is None:
            continue
        total += s.count
        if best.rng is None or s.rng < best.rng:
            best = s
    return best if best.rng is None else best._replace(count=total)


def radar_hash(robot):
    """Fingerprint of the raw radar payload.

    A sensor that has stopped producing data does not disappear from shared
    memory -- the last frame it wrote just stays there forever. Freshness has to
    be judged by whether the CONTENT changes, which is what this supports
    (ISO 15964 4.2 d: fault detection and isolation).
    """
    raw = hakopy.pdu_read(robot, RADAR_CH, RADAR_SIZE)
    if not raw:
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


def read_xyz(robot):
    """True (x, y, z) from the pos PDU via a raw, thread-safe read. None if empty."""
    raw = hakopy.pdu_read(robot, POS_CH, POS_SIZE)
    if not raw or len(raw) < POS_SIZE:
        return None
    b = bytes(raw)
    return (struct.unpack_from("<d", b, POS_X_OFF)[0],
            struct.unpack_from("<d", b, POS_Y_OFF)[0],
            struct.unpack_from("<d", b, POS_Z_OFF)[0])


def connect(names):
    """Arm every drone from THIS process -- a client in another process cannot
    write commands (see the B-3 notes)."""
    import hakoniwa_pdu.apps.drone.hakosim as hakosim
    clients = {}
    for name in names:
        c = hakosim.MultirotorClient(CFG, name)
        c.confirmConnection()
        c.enableApiControl(True)
        c.armDisarm(True)
        clients[name] = c
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


def yaw_for_heading(dx, dy):
    """Yaw in degrees for a ground track (dx, dy). +x is yaw 0, +y is yaw 90."""
    return math.degrees(math.atan2(dy, dx))
