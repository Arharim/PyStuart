"""
Serial link backend for the Bridge board (USB CDC / UART).

Wraps pyserial and reuses the shared frame parser, so the wire path is
byte-identical to the mock link. Any pyserial port spec works, including
URL handlers: a real COM port ("COM7") or the in-memory "loop://" used
by the tests. USB CDC ignores baudrate; it is kept for plain UART rigs.
"""
from typing import List

import serial

from .protocol import Frame, FrameParser, encode_frame


class SerialLink:
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.05, **kwargs):
        self._serial = serial.serial_for_url(port, baudrate=baudrate, timeout=timeout, **kwargs)
        self._parser = FrameParser()

    def send_raw(self, data: bytes) -> int:
        return self._serial.write(data)

    def send_frame(self, frame: Frame) -> int:
        return self.send_raw(encode_frame(frame.type, frame.payload, frame.seq, frame.ts32))

    def recv_frames(self) -> List[Frame]:
        data = self._serial.read(4096)
        return self._parser.feed(data) if data else []

    def close(self) -> None:
        self._serial.close()

    @property
    def is_open(self) -> bool:
        return self._serial.is_open

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
