"""
Tests for the CMD_ACT actuator writer.

Runs over pyserial's loop:// link: every frame the writer sends comes
straight back, so the tests decode the real wire bytes through the
shared protocol codec - no mocking of the serial or framing layers.
"""
import math

import pytest

from pystuart.transport.actuator import ActuatorWriter
from pystuart.transport.protocol import (
    FLAG_CMD_ENABLE, FLAG_CMD_TORQUE_OFF, TYPE_CMD_ACT, TYPE_PWR_MON,
    TYPE_SRV_FB, decode_cmd_act, decode_frame, encode_frame, encode_pwr_mon,
    encode_srv_fb,
)
from pystuart.transport.serial_link import SerialLink
from pystuart.transport.servo_map import ServoCalibration


@pytest.fixture
def writer():
    # loop:// emulates baud timing and holds 4096 unread bytes: high
    # baudrate keeps writes fast, like the CDC link it stands in for.
    with ActuatorWriter(SerialLink("loop://", baudrate=921600), ServoCalibration()) as actuator:
        yield actuator


def read_wire(link, timeout=0.2):
    link._serial.timeout = timeout
    return link._serial.read(4096)


class TestSendTargets:
    def test_neutral_targets_on_the_wire(self, writer):
        writer.send_targets([0.0] * 6)
        frame = decode_frame(read_wire(writer._link))
        assert frame.type == TYPE_CMD_ACT
        assert frame.seq == 0
        counts, flags = decode_cmd_act(frame.payload)
        assert counts == (2048,) * 6
        assert flags == FLAG_CMD_ENABLE

    def test_counts_follow_calibration(self, writer):
        writer.send_targets([math.radians(10.0)] * 6)
        frame = decode_frame(read_wire(writer._link))
        counts, _ = decode_cmd_act(frame.payload)
        assert counts == (2162,) * 6  # 2048 + round(10 deg * 4096/360)
        assert writer.last_counts == counts

    def test_sequence_numbers_increment_mod_256(self, writer):
        """Stream like a rig would: drain the echo while sending."""
        seqs = []
        for chunk in range(3):
            for _ in range(100):
                writer.send_targets([0.0] * 6)
            buf = read_wire(writer._link)
            while len(buf) >= 11:
                total = 11 + buf[2]
                if len(buf) < total:
                    break
                seqs.append(decode_frame(buf[:total]).seq)
                buf = buf[total:]
        assert seqs == [s % 256 for s in range(300)]

    def test_disable_flag_clears_enable(self, writer):
        writer.send_targets([0.0] * 6, enable=False)
        frame = decode_frame(read_wire(writer._link))
        _, flags = decode_cmd_act(frame.payload)
        assert flags == 0

    def test_nan_alpha_rejected_before_sending(self, writer):
        with pytest.raises(ValueError, match="NaN"):
            writer.send_targets([float("nan")] * 6)
        assert read_wire(writer._link, timeout=0.05) == b""


class TestTorqueOff:
    def test_flag_and_held_counts(self, writer):
        writer.send_targets([math.radians(5.0)] * 6)
        read_wire(writer._link)
        writer.torque_off()
        frame = decode_frame(read_wire(writer._link))
        counts, flags = decode_cmd_act(frame.payload)
        assert flags == FLAG_CMD_TORQUE_OFF
        assert counts == (2105,) * 6  # 2048 + round(5 deg * 4096/360), held


class TestPoll:
    def test_srv_fb_roundtrip(self, writer):
        writer._link.send_raw(encode_frame(
            TYPE_SRV_FB, encode_srv_fb([2048] * 6, [10] * 6, [35] * 6)))
        feedback = writer.poll()
        assert feedback == [("srv_fb", [(2048, 10, 35)] * 6)]

    def test_pwr_mon_roundtrip(self, writer):
        writer._link.send_raw(encode_frame(
            TYPE_PWR_MON, encode_pwr_mon(12000, -350, relay=1, estop=0)))
        feedback = writer.poll()
        assert feedback == [("pwr_mon", (12000, -350, 1, 0))]

    def test_unknown_types_are_dropped(self, writer):
        writer._link.send_raw(encode_frame(0x7F, b"\x01\x02"))
        assert writer.poll() == []
