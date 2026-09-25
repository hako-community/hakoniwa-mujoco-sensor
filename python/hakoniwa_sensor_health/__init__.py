"""SENS-006 v1 codec owned by hakoniwa-mujoco-sensor.

The outer transport is the existing std_msgs/UInt8MultiArray PDU.
No generated hako_msgs/SensorHealth modules are imported or installed.
"""
from dataclasses import dataclass
import struct

SCHEMA = 'hakoniwa.sensor-health/v1'
PDU_TYPE = 'std_msgs/UInt8MultiArray'
HEADER = struct.Struct('<4sHHIQqqqIQIHHH')
MAX_TEXT = 127
MAX_SIZE = HEADER.size + 3 * MAX_TEXT


@dataclass
class SensorHealth:
    sensor_id: str = ''
    sequence: int = 0
    source_time_ns: int = 0
    scheduled_time_ns: int = 0
    publish_time_ns: int = 0
    status: int = 1
    dropped_count: int = 0
    queue_depth: int = 0
    profile_id: str = ''
    calibration_id: str = ''


def _validate(h):
    texts = [s.encode('utf-8') for s in (h.sensor_id, h.profile_id, h.calibration_id)]
    if not texts[0] or any(len(t) > MAX_TEXT or b'\0' in t for t in texts):
        raise ValueError('SensorHealth strings must be UTF-8, NUL-free, <=127 bytes')
    for name, bits in (('sequence', 64), ('dropped_count', 64), ('queue_depth', 32),
                       ('status', 7), ('source_time_ns', 63),
                       ('scheduled_time_ns', 63), ('publish_time_ns', 63)):
        value = getattr(h, name)
        if type(value) is not int or not 0 <= value < 1 << bits:
            raise ValueError('SensorHealth field: ' + name)
    if h.scheduled_time_ns < h.source_time_ns or h.publish_time_ns < h.source_time_ns:
        raise ValueError('SensorHealth time order')
    return texts


def encode(h):
    texts = _validate(h)
    return HEADER.pack(b'HSH1', 1, 0, HEADER.size + sum(map(len, texts)),
                       h.sequence, h.source_time_ns, h.scheduled_time_ns,
                       h.publish_time_ns, h.status, h.dropped_count, h.queue_depth,
                       *map(len, texts)) + b''.join(texts)


def decode(data):
    data = bytes(data)
    if not HEADER.size <= len(data) <= MAX_SIZE:
        raise ValueError('SensorHealth length')
    magic, version, reserved, size, seq, source, scheduled, published, status, drops, queue, *lengths = HEADER.unpack_from(data)
    if (magic != b'HSH1' or version != 1 or reserved != 0 or size != len(data)
            or HEADER.size + sum(lengths) != size):
        raise ValueError('SensorHealth magic/version/reserved/length')
    texts, at = [], HEADER.size
    for length in lengths:
        texts.append(data[at:at + length].decode('utf-8'))
        at += length
    h = SensorHealth(texts[0], seq, source, scheduled, published, status, drops,
                     queue, texts[1], texts[2])
    _validate(h)
    return h


def pdu_to_py_SensorHealth(raw):
    from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_UInt8MultiArray import pdu_to_py_UInt8MultiArray
    try:
        msg = pdu_to_py_UInt8MultiArray(raw)
        if msg.layout.dim or msg.layout.data_offset != 0:
            raise ValueError('SensorHealth requires empty MultiArrayLayout')
        return decode(msg.data)
    except (IndexError, struct.error, OverflowError) as exc:
        raise ValueError('Invalid SensorHealth PDU') from exc


def py_to_pdu_SensorHealth(h):
    from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_pytype_UInt8MultiArray import UInt8MultiArray
    from hakoniwa_pdu.pdu_msgs.std_msgs.pdu_conv_UInt8MultiArray import py_to_pdu_UInt8MultiArray
    msg = UInt8MultiArray()
    msg.data = list(encode(h))
    raw = py_to_pdu_UInt8MultiArray(msg)
    if len(raw) > 1024:
        raise ValueError('SensorHealth PDU exceeds 1024-byte channel')
    return raw
