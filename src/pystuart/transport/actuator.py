"""
CMD_ACT actuator writer for the Bridge servo bus.

Turns IK horn angles into calibrated encoder counts and ships them as
CMD_ACT frames at the caller's cadence (50-200 Hz per design doc,
section 8.3). Sequence numbers and the ts32 sender clock are handled
here. poll() collects upstream SRV_FB / PWR_MON replies so the same
object serves the diagnostic loop.
"""
import time
from typing import List, Sequence, Tuple

from .protocol import (
    FLAG_CMD_ENABLE, FLAG_CMD_TORQUE_OFF, TYPE_CMD_ACT, TYPE_PWR_MON,
    TYPE_SRV_FB, Frame, decode_pwr_mon, decode_srv_fb, encode_cmd_act,
)
from .serial_link import SerialLink
from .servo_map import ServoCalibration


def _monotonic_ts32() -> int:
    return int(time.monotonic_ns() // 1000) & 0xFFFFFFFF


class ActuatorWriter:
    def __init__(self, link: SerialLink, calibration: ServoCalibration):
        calibration.validate()
        self._link = link
        self._calibration = calibration
        self._seq = 0
        self.last_counts = calibration.alpha_to_counts([0.0] * 6)

    def send_targets(self, alpha_rad: Sequence[float], enable: bool = True) -> Frame:
        counts = self._calibration.alpha_to_counts(alpha_rad)
        flags = FLAG_CMD_ENABLE if enable else 0
        frame = Frame(TYPE_CMD_ACT, self._seq, _monotonic_ts32(), encode_cmd_act(counts, flags))
        self._seq = (self._seq + 1) % 256
        self._link.send_frame(frame)
        self.last_counts = counts
        return frame

    def torque_off(self) -> Frame:
        """Release torque immediately, holding the last commanded counts."""
        frame = Frame(TYPE_CMD_ACT, self._seq, _monotonic_ts32(),
                      encode_cmd_act(self.last_counts, FLAG_CMD_TORQUE_OFF))
        self._seq = (self._seq + 1) % 256
        self._link.send_frame(frame)
        return frame

    def poll(self) -> List[Tuple[str, tuple]]:
        """Drain upstream feedback frames: ("srv_fb", state) / ("pwr_mon", state)."""
        out = []
        for frame in self._link.recv_frames():
            if frame.type == TYPE_SRV_FB:
                out.append(("srv_fb", decode_srv_fb(frame.payload)))
            elif frame.type == TYPE_PWR_MON:
                out.append(("pwr_mon", decode_pwr_mon(frame.payload)))
        return out

    def close(self) -> None:
        self._link.close()

    def __enter__(self) -> "ActuatorWriter":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
