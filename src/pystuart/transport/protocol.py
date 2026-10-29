"""
Wire protocol codec shared by every PyStuart transport (USB CDC, UART, BLE).

Frame layout (design document, section 8.1):

    offset  size  field
    0       2     sync 0xAA55
    2       1     len    payload size in bytes
    3       1     type   packet type, see the TYPE_* constants
    4       1     seq    sequence number, uint8 mod 256
    5       4     ts32   sender timestamp, uint32 LE, microseconds
    9       N     payload
    9+N     2     CRC16-CCITT, uint16 LE

The CRC (poly 0x1021, init 0xFFFF, CCITT-FALSE) covers every byte after
the sync marker. A receiver may therefore scan a raw byte stream for
sync and validate each candidate frame independently; on failure it
resumes scanning right after the failed sync marker.

Payload layouts implemented here:

    CMD_ACT    6 x uint16 LE encoder counts + uint8 flags
    IMU_BATCH  K x (int16 LE x 6) accel/gyro samples, 1 <= K <= 21
    SRV_FB     6 x (uint16 pos, uint16 load, uint8 temp), all LE
    PWR_MON    uint32 V_mV, int32 I_mA, uint8 relay, uint8 estop, LE

MARK / CONFIG / DIAG / BATTERY frames carry opaque payloads and go
through the generic encode_frame/decode_frame pair.
"""
import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

SYNC = b"\xaa\x55"
CRC_INIT = 0xFFFF
HEADER_LEN = 7
TRAILER_LEN = 2
MIN_FRAME = len(SYNC) + HEADER_LEN + TRAILER_LEN
MAX_PAYLOAD = 255
ENCODER_MAX = 4095
MAX_IMU_SAMPLES = MAX_PAYLOAD // 12

TYPE_IMU_BATCH = 0x01
TYPE_CONFIG_ACK = 0x02
TYPE_BATTERY = 0x03
TYPE_MARK = 0x04
TYPE_DIAG = 0x05
TYPE_CMD_ACT = 0x06
TYPE_PWR_MON = 0x07
TYPE_SRV_FB = 0x08

FLAG_CMD_ENABLE = 0x01
FLAG_CMD_TORQUE_OFF = 0x02


class ProtocolError(ValueError):
    """Raised on malformed frames or out-of-range payload values."""


def crc16_ccitt(data: bytes, crc: int = CRC_INIT) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    type: int
    seq: int
    ts32: int
    payload: bytes


def encode_frame(frame_type: int, payload: bytes = b"", seq: int = 0, ts32: int = 0) -> bytes:
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError("payload exceeds %d bytes" % MAX_PAYLOAD)
    frame = SYNC + struct.pack(
        "<BBBI", len(payload), frame_type, seq & 0xFF, ts32 & 0xFFFFFFFF
    ) + payload
    return frame + struct.pack("<H", crc16_ccitt(frame[len(SYNC):]))


def decode_frame(raw: bytes) -> Frame:
    raw = bytes(raw)
    if len(raw) < MIN_FRAME:
        raise ProtocolError("frame too short: %d bytes" % len(raw))
    if raw[:2] != SYNC:
        raise ProtocolError("bad sync marker")
    payload_len = raw[2]
    if len(raw) != MIN_FRAME + payload_len:
        raise ProtocolError(
            "length mismatch: %d bytes, header announces %d" % (len(raw), payload_len)
        )
    (crc,) = struct.unpack("<H", raw[-2:])
    if crc16_ccitt(raw[2:-2]) != crc:
        raise ProtocolError("CRC mismatch")
    frame_type, seq, ts32 = struct.unpack("<BBI", raw[3:9])
    return Frame(frame_type, seq, ts32, raw[9:-2])


class FrameParser:
    """Incremental decoder for a raw byte stream.

    feed() accepts arbitrary chunks and returns every complete valid
    frame they completed. Garbage before sync, split frames and frames
    with a bad CRC are all survived: the parser rescans after the
    failed sync marker.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> List[Frame]:
        self._buf += data
        frames = []
        while True:
            frame = self._extract()
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _extract(self) -> Optional[Frame]:
        while True:
            start = self._buf.find(SYNC)
            if start < 0:
                self._buf.clear()
                return None
            if start:
                del self._buf[:start]
            if len(self._buf) < MIN_FRAME:
                return None
            total = MIN_FRAME + self._buf[2]
            if len(self._buf) < total:
                return None
            candidate = bytes(self._buf[:total])
            try:
                frame = decode_frame(candidate)
            except ProtocolError:
                del self._buf[:2]
                continue
            del self._buf[:total]
            return frame


def encode_cmd_act(counts: Sequence[int], flags: int = 0) -> bytes:
    counts = tuple(counts)
    if len(counts) != 6:
        raise ProtocolError("CMD_ACT carries exactly 6 encoder counts")
    if any(not 0 <= c <= ENCODER_MAX for c in counts):
        raise ProtocolError("encoder counts must be within 0..%d" % ENCODER_MAX)
    return struct.pack("<6HB", *counts, flags & 0xFF)


def decode_cmd_act(payload: bytes) -> Tuple[Tuple[int, ...], int]:
    if len(payload) != 13:
        raise ProtocolError("CMD_ACT payload must be 13 bytes")
    values = struct.unpack("<6HB", payload)
    return values[:6], values[6]


def encode_imu_batch(samples: Sequence[Sequence[int]]) -> bytes:
    if not samples or len(samples) > MAX_IMU_SAMPLES:
        raise ProtocolError("IMU_BATCH holds 1..%d samples" % MAX_IMU_SAMPLES)
    flat = []
    for sample in samples:
        if len(sample) != 6:
            raise ProtocolError("each IMU sample carries 6 int16 values")
        flat.extend(sample)
    return struct.pack("<%dh" % len(flat), *flat)


def decode_imu_batch(payload: bytes) -> List[Tuple[int, ...]]:
    if not payload or len(payload) % 12:
        raise ProtocolError("IMU_BATCH payload must be a non-empty multiple of 12 bytes")
    values = struct.unpack("<%dh" % (len(payload) // 2), payload)
    return [tuple(values[i:i + 6]) for i in range(0, len(values), 6)]


def encode_srv_fb(positions: Sequence[int], loads: Sequence[int], temps: Sequence[int]) -> bytes:
    positions, loads, temps = tuple(positions), tuple(loads), tuple(temps)
    if len(positions) != 6 or len(loads) != 6 or len(temps) != 6:
        raise ProtocolError("SRV_FB carries state of exactly 6 servos")
    if any(not 0 <= p <= ENCODER_MAX for p in positions):
        raise ProtocolError("servo positions must be within 0..%d" % ENCODER_MAX)
    flat = []
    for pos, load, temp in zip(positions, loads, temps):
        flat.extend((pos, load, temp))
    return struct.pack("<" + "HHB" * 6, *flat)


def decode_srv_fb(payload: bytes) -> List[Tuple[int, int, int]]:
    if len(payload) != 30:
        raise ProtocolError("SRV_FB payload must be 30 bytes")
    values = struct.unpack("<" + "HHB" * 6, payload)
    return [tuple(values[i:i + 3]) for i in range(0, 18, 3)]


def encode_pwr_mon(v_mv: int, i_ma: int, relay: int = 0, estop: int = 0) -> bytes:
    return struct.pack("<IiBB", v_mv, i_ma, relay & 0xFF, estop & 0xFF)


def decode_pwr_mon(payload: bytes) -> Tuple[int, int, int, int]:
    if len(payload) != 10:
        raise ProtocolError("PWR_MON payload must be 10 bytes")
    return struct.unpack("<IiBB", payload)
