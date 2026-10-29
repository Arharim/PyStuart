"""
Golden tests for the pystuart transport protocol.

The vectors in tests/golden/protocol_vectors.json freeze the exact wire
bytes agreed with the Bridge/Node firmware: any deviation on either
side turns these red. The vectors are generated with the codec itself,
which makes them blind to symmetric bugs - so the CRC is anchored to
the published CCITT-FALSE known answer AND to the stdlib's independent
binascii.crc_hqx implementation, and one frame is hand-assembled from
the spec table byte by byte.
"""
import binascii
import json
from pathlib import Path

import pytest

from pystuart.transport.mock import make_link
from pystuart.transport.protocol import (
    FLAG_CMD_ENABLE, Frame, FrameParser, ProtocolError, TYPE_CMD_ACT,
    TYPE_IMU_BATCH, TYPE_MARK, decode_cmd_act, decode_frame,
    decode_imu_batch, decode_pwr_mon, decode_srv_fb, encode_cmd_act,
    encode_frame, encode_imu_batch, encode_pwr_mon, encode_srv_fb,
    crc16_ccitt,
)

GOLDEN = Path(__file__).parent / "golden" / "protocol_vectors.json"
VECTORS = json.loads(GOLDEN.read_text(encoding="utf-8"))["vectors"]
TYPE_BY_NAME = {v["name"]: v["type_id"] for v in VECTORS if "type_id" in v}


def test_crc16_ccitt_known_answer():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_crc_matches_independent_stdlib_implementation():
    raw = encode_frame(
        TYPE_CMD_ACT, encode_cmd_act([2048, 1024, 3072, 512, 4095, 0]), 7, 1000
    )
    assert binascii.crc_hqx(b"123456789", 0xFFFF) == 0x29B1
    assert binascii.crc_hqx(raw[2:-2], 0xFFFF) == int.from_bytes(raw[-2:], "little")


def test_cmd_act_frame_matches_hand_assembled_spec_bytes():
    payload = (
        bytes([0x00, 0x08])
        + bytes([0x00, 0x04])
        + bytes([0x00, 0x0C])
        + bytes([0x00, 0x02])
        + bytes([0xFF, 0x0F])
        + bytes([0x00, 0x00])
        + bytes([0x01])
    )
    ts = bytes([1000 & 0xFF, (1000 >> 8) & 0xFF, (1000 >> 16) & 0xFF, (1000 >> 24) & 0xFF])
    header = bytes([0xAA, 0x55, len(payload), TYPE_CMD_ACT, 7]) + ts
    expected = header + payload + binascii.crc_hqx(header[2:] + payload, 0xFFFF).to_bytes(2, "little")
    assert encode_frame(TYPE_CMD_ACT, payload, seq=7, ts32=1000) == expected
    assert expected.hex() == "aa550d0607e803000000080004000c0002ff0f000001f2d5"


@pytest.mark.parametrize("vector", VECTORS, ids=lambda v: v["name"])
def test_golden_vectors(vector):
    if "hex" not in vector:
        return
    raw = bytes.fromhex(vector["hex"])
    if not vector["valid"]:
        with pytest.raises(ProtocolError):
            decode_frame(raw)
        return
    frame = decode_frame(raw)
    assert frame.type == vector["type_id"]
    assert frame.seq == vector["seq"]
    assert frame.ts32 == vector["ts32"]
    assert frame.payload == bytes.fromhex(vector["payload_hex"])
    assert encode_frame(frame.type, frame.payload, frame.seq, frame.ts32) == raw


@pytest.mark.parametrize("vector", VECTORS, ids=lambda v: v["name"])
def test_golden_stream(vector):
    if "stream_hex" not in vector:
        return
    parser = FrameParser()
    raw = bytes.fromhex(vector["stream_hex"]) + b"\x00" * 200
    frames = parser.feed(raw[:5]) + parser.feed(raw[5:])
    expected = [TYPE_BY_NAME[name] for name in vector["expect_frames"]]
    assert [f.type for f in frames] == expected


class TestFrameCodec:
    def test_seq_wraps_mod_256(self):
        frame = decode_frame(encode_frame(TYPE_CMD_ACT, b"", seq=300, ts32=1))
        assert frame.seq == 44

    def test_payload_too_large_rejected(self):
        with pytest.raises(ProtocolError):
            encode_frame(TYPE_MARK, bytes(256))

    def test_empty_payload_roundtrip(self):
        raw = encode_frame(TYPE_MARK)
        assert len(raw) == 11
        assert decode_frame(raw).payload == b""

    def test_short_input_rejected(self):
        with pytest.raises(ProtocolError):
            decode_frame(b"\xaa\x55\x00")


class TestTypedPayloads:
    def test_cmd_act_roundtrip_and_range(self):
        counts, flags = decode_cmd_act(
            encode_cmd_act([1, 2, 3, 4, 5, 6], FLAG_CMD_ENABLE))
        assert counts == (1, 2, 3, 4, 5, 6)
        assert flags == FLAG_CMD_ENABLE
        with pytest.raises(ProtocolError):
            encode_cmd_act([0, 0, 0, 0, 0, 4096])
        with pytest.raises(ProtocolError):
            encode_cmd_act([0, 0, 0, 0, 0])

    def test_imu_batch_roundtrip_and_limits(self):
        samples = [(1, -2, 3, -4, 5, -6)] * 21
        assert len(decode_imu_batch(encode_imu_batch(samples))) == 21
        with pytest.raises(ProtocolError):
            encode_imu_batch([(1, -2, 3, -4, 5, -6)] * 22)
        with pytest.raises(ProtocolError):
            encode_imu_batch([(1, -2, 3, -4, 5)])

    def test_srv_fb_roundtrip_and_range(self):
        state = decode_srv_fb(encode_srv_fb(
            [0, 512, 1024, 2048, 3000, 4095], [10, 20, 30, 40, 50, 60], [35] * 6))
        assert state[0] == (0, 10, 35)
        assert state[5] == (4095, 60, 35)
        with pytest.raises(ProtocolError):
            encode_srv_fb([0] * 6, [10] * 6, [35] * 5)

    def test_pwr_mon_negative_current(self):
        v_mv, i_ma, relay, estop = decode_pwr_mon(encode_pwr_mon(12000, -350, 1, 0))
        assert (v_mv, i_ma, relay, estop) == (12000, -350, 1, 0)


class TestParserResync:
    def test_garbage_prefix_and_split_feed(self):
        good = encode_frame(TYPE_CMD_ACT, encode_cmd_act([1, 2, 3, 4, 5, 6]), 9, 5000)
        parser = FrameParser()
        assert parser.feed(b"\x00\xde\xad\xbe\xef" + good[:4]) == []
        frames = parser.feed(good[4:])
        assert len(frames) == 1
        assert frames[0].seq == 9

    def test_corrupt_frame_does_not_poison_stream(self):
        good = encode_frame(TYPE_CMD_ACT, encode_cmd_act([1, 2, 3, 4, 5, 6]), 1, 1)
        bad = good[:-1] + bytes([good[-1] ^ 0xFF])
        parser = FrameParser()
        frames = parser.feed(bad + good)
        assert len(frames) == 1
        assert frames[0] == decode_frame(good)

    def test_false_sync_inside_payload(self):
        payload = b"\xaa\x55\x03\x06" + bytes(9)
        raw = encode_frame(TYPE_CMD_ACT, payload, 2, 2)
        parser = FrameParser()
        frames = parser.feed(raw)
        assert len(frames) == 1
        assert frames[0].payload == payload


class TestMockLink:
    def test_bidirectional_delivery(self):
        a, b = make_link()
        frame_a = Frame(TYPE_CMD_ACT, 1, 100, encode_cmd_act([1] * 6))
        frame_b = Frame(TYPE_IMU_BATCH, 2, 200, encode_imu_batch([(0, 0, 0, 0, 0, 0)]))
        a.send_frame(frame_a)
        b.send_frame(frame_b)
        assert b.recv_frames() == [frame_a]
        assert a.recv_frames() == [frame_b]
        assert b.recv_frames() == []

    def test_raw_bytes_survive_the_link(self):
        a, b = make_link()
        raw = encode_frame(TYPE_CMD_ACT, encode_cmd_act([4095] * 6), 7, 7)
        a.send_raw(b"noise " + raw)
        frames = b.recv_frames()
        assert frames == [decode_frame(raw)]

    def test_multiple_frames_one_chunk(self):
        a, b = make_link()
        for seq in range(5):
            a.send_frame(Frame(TYPE_MARK, seq, seq, b""))
        received = b.recv_frames()
        assert [f.seq for f in received] == list(range(5))
