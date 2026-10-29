"""
In-memory transport pair for tests and Bridge simulation.

Two MockEndpoint objects wired back to back: bytes handed to one
endpoint's send_* calls land in the peer's inbox. No threads, no
timing, delivery is immediate. recv_frames() drains and decodes the
inbox with FrameParser, mirroring how a serial backend consumes its
stream. Future serial/BLE backends implement the same send/recv
surface, so protocol code stays transport-agnostic.
"""
from typing import List

from .protocol import Frame, FrameParser, encode_frame


class MockEndpoint:
    def __init__(self):
        self._peer = None
        self._inbox = bytearray()
        self._parser = FrameParser()

    def send_frame(self, frame: Frame) -> None:
        self.send_raw(encode_frame(frame.type, frame.payload, frame.seq, frame.ts32))

    def send_raw(self, data: bytes) -> None:
        self._peer._inbox += data

    def recv_frames(self) -> List[Frame]:
        frames = self._parser.feed(bytes(self._inbox))
        self._inbox.clear()
        return frames


def make_link():
    """Return (endpoint_a, endpoint_b) wired to each other."""
    a = MockEndpoint()
    b = MockEndpoint()
    a._peer = b
    b._peer = a
    return a, b
