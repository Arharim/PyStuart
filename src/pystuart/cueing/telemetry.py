"""
Telemetry sources feeding the washout chain.

A source yields TelemetrySample objects: body-frame specific force with
gravity removed (m/s^2) and angular rates (rad/s), whatever the game
can provide. Pacing is the caller's job (GUI timer, replay script);
sources only produce the next sample in order.

Implemented: CSV replay (offline analysis, dataset reruns) and the
Assetto Corsa UDP physics feed. The AC physics page carries no angular
rates - the rotational channel starts at zero and yaw rate can be
derived from the graphics page later; sims like rF2 report real rates.
"""
import csv
import socket
import struct
import time
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

CSV_COLUMNS = ("t_s", "fx_mps2", "fy_mps2", "fz_mps2", "p_radps", "q_radps", "r_radps")


@dataclass(frozen=True)
class TelemetrySample:
    t: float
    force: Tuple[float, float, float]
    rate: Tuple[float, float, float]


def load_csv_samples(path) -> List[TelemetrySample]:
    """Read a telemetry CSV with the t_s / fx..fz / p..r schema."""
    samples: List[TelemetrySample] = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        missing = [c for c in CSV_COLUMNS if c not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("telemetry CSV is missing columns: %s" % ", ".join(missing))
        for row_num, row in enumerate(reader, start=2):
            try:
                samples.append(TelemetrySample(
                    t=float(row["t_s"]),
                    force=(float(row["fx_mps2"]), float(row["fy_mps2"]), float(row["fz_mps2"])),
                    rate=(float(row["p_radps"]), float(row["q_radps"]), float(row["r_radps"])),
                ))
            except (TypeError, ValueError) as e:
                raise ValueError("row %d: %s" % (row_num, e)) from e
    if not samples:
        raise ValueError("telemetry CSV holds no samples")
    return samples


class CsvTelemetrySource:
    """Replays a recorded telemetry CSV sample by sample."""

    def __init__(self, path):
        self.samples = load_csv_samples(path)
        self._index = 0

    def __iter__(self) -> Iterator[TelemetrySample]:
        self._index = 0
        return self

    def __next__(self) -> TelemetrySample:
        if self._index >= len(self.samples):
            raise StopIteration
        sample = self.samples[self._index]
        self._index += 1
        return sample

    def duration_s(self) -> float:
        return self.samples[-1].t - self.samples[0].t


AC_HANDSHAKE_PHYSICS = b"\x03\x00\x00"
AC_HANDSHAKE_STOP = b"\x04"
AC_PHYSICS_PACKET_SIZE = 328


class AcUdpTelemetrySource:
    """
    Assetto Corsa car-physics UDP feed (documented SPageFilePhysics).

    start() performs the handshake (identifier 3 on the game's UDP
    port, default 9996) and next_sample() pulls 328-byte physics pages.
    Only the fields the washout needs are parsed: accG at offset 44,
    velocity at 32, speed at 28.
    """

    def __init__(self, remote_host: str = "127.0.0.1", remote_port: int = 9996, timeout: float = 1.0):
        self._remote = (remote_host, remote_port)
        self._timeout = timeout
        self._sock = None
        self.packets_received = 0

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(self._timeout)
        self._sock.sendto(AC_HANDSHAKE_PHYSICS, self._remote)

    def stop(self) -> None:
        if self._sock is not None:
            self._sock.sendto(AC_HANDSHAKE_STOP, self._remote)
            self._sock.close()
            self._sock = None

    def next_sample(self) -> Optional[TelemetrySample]:
        if self._sock is None:
            raise RuntimeError("call start() before next_sample()")
        deadline = time.monotonic() + self._timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._sock.settimeout(remaining)
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                return None
            except ConnectionResetError:
                # Windows: ICMP port-unreachable from a dead peer surfaces
                # as a reset on the next recv; keep waiting until deadline.
                continue
            if len(data) != AC_PHYSICS_PACKET_SIZE:
                continue
            packet_id, speed_kmh, velocity, accg = self.parse_physics(data)
            self.packets_received += 1
            return TelemetrySample(
                t=time.monotonic(),  # physics pages carry no timestamp; arrival time
                force=accg,
                rate=(0.0, 0.0, 0.0),
            )

    @staticmethod
    def parse_physics(buf: bytes) -> Tuple[int, float, Tuple[float, float, float], Tuple[float, float, float]]:
        if len(buf) != AC_PHYSICS_PACKET_SIZE:
            raise ValueError("physics page must be %d bytes, got %d" % (AC_PHYSICS_PACKET_SIZE, len(buf)))
        (packet_id,) = struct.unpack_from("<i", buf, 0)
        (speed_kmh,) = struct.unpack_from("<f", buf, 28)
        velocity = struct.unpack_from("<3f", buf, 32)
        accg = struct.unpack_from("<3f", buf, 44)
        return packet_id, speed_kmh, velocity, accg
