"""
Tests for the serial link backend.

These run against pyserial's loop:// handler: a genuine serial object
with in-memory loopback. That exercises the real write/read/parser path
without inventing a fake serial class; the only untested part is the OS
layer of a physical COM port.
"""
import pytest

from pystuart.transport.protocol import TYPE_MARK, Frame, decode_frame, encode_frame
from pystuart.transport.serial_link import SerialLink


@pytest.fixture
def link():
    with SerialLink("loop://") as serial_link:
        yield serial_link


def test_send_frame_comes_back_decoded(link):
    frame = Frame(TYPE_MARK, 5, 1234, b"\x01\x02")
    link.send_frame(frame)
    received = link.recv_frames()
    assert received == [frame]


def test_frames_are_wire_identical(link):
    frame = Frame(TYPE_MARK, 1, 1, b"payload")
    link.send_frame(frame)
    link._serial.timeout = 0.2
    raw = link._serial.read(4096)
    assert raw == encode_frame(frame.type, frame.payload, frame.seq, frame.ts32)
    assert decode_frame(raw) == frame


def test_recv_on_silent_port_returns_empty(link):
    assert link.recv_frames() == []


def test_many_frames_one_drain(link):
    sent = [Frame(TYPE_MARK, seq, seq, b"") for seq in range(10)]
    for frame in sent:
        link.send_frame(frame)
    assert link.recv_frames() == sent


def test_context_manager_closes():
    serial_link = SerialLink("loop://")
    assert serial_link.is_open
    with serial_link:
        pass
    assert not serial_link.is_open
