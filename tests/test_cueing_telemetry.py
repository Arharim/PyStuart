"""
Tests for the telemetry sources.

The AC physics packet in these tests is assembled by hand at the
offsets of the publicly documented SPageFilePhysics layout - the same
table the parser implements - and the UDP test talks to a real local
socket playing the game. First live capture from the game itself
should be frozen as an extra golden fixture.
"""
import socket
import struct

import pytest

from pystuart.cueing.telemetry import (
    AC_PHYSICS_PACKET_SIZE, AcUdpTelemetrySource, CsvTelemetrySource,
    TelemetrySample, load_csv_samples,
)

CSV_HEADER = "t_s,fx_mps2,fy_mps2,fz_mps2,p_radps,q_radps,r_radps\n"


def write_csv(tmp_path, lines):
    path = tmp_path / "telemetry.csv"
    path.write_text(CSV_HEADER + "\n".join(lines) + "\n", encoding="utf-8")
    return path


class TestCsvSource:
    def test_load_and_replay(self, tmp_path):
        path = write_csv(tmp_path, [
            "0.0,1.0,0.0,9.81,0.0,0.0,0.0",
            "0.05,2.0,-1.0,9.81,0.1,0.2,0.3",
            "0.10,0.0,0.0,9.81,0.0,0.0,0.0",
        ])
        source = CsvTelemetrySource(path)
        replayed = list(source)
        assert len(replayed) == 3
        assert replayed[1].t == pytest.approx(0.05)
        assert replayed[1].force == pytest.approx((2.0, -1.0, 9.81))
        assert replayed[1].rate == pytest.approx((0.1, 0.2, 0.3))
        assert source.duration_s() == pytest.approx(0.10)

    def test_second_replay_restarts(self, tmp_path):
        path = write_csv(tmp_path, [
            "0.0,1.0,0.0,9.81,0.0,0.0,0.0",
            "0.05,2.0,0.0,9.81,0.0,0.0,0.0",
        ])
        source = CsvTelemetrySource(path)
        assert len(list(source)) == 2
        assert len(list(source)) == 2

    def test_rejects_missing_column(self, tmp_path):
        path = tmp_path / "bad.csv"
        path.write_text("t_s,fx_mps2\n1.0,2.0\n", encoding="utf-8")
        with pytest.raises(ValueError, match="missing columns"):
            load_csv_samples(path)

    def test_rejects_broken_row(self, tmp_path):
        path = write_csv(tmp_path, [
            "0.0,1.0,0.0,9.81,0.0,0.0,0.0",
            "0.05,not_a_number,0.0,9.81,0.0,0.0,0.0",
        ])
        with pytest.raises(ValueError, match="row 3"):
            load_csv_samples(path)

    def test_rejects_empty(self, tmp_path):
        path = write_csv(tmp_path, [])
        with pytest.raises(ValueError, match="no samples"):
            load_csv_samples(path)


def build_physics_page(packet_id, speed_kmh, velocity, accg):
    """Assemble a 328-byte SPageFilePhysics at the documented offsets."""
    buf = bytearray(AC_PHYSICS_PACKET_SIZE)
    struct.pack_into("<i", buf, 0, packet_id)
    struct.pack_into("<f", buf, 4, 0.5)   # gas
    struct.pack_into("<f", buf, 8, 0.25)  # brake
    struct.pack_into("<f", buf, 12, 34.5)  # fuel
    struct.pack_into("<i", buf, 16, 3)    # gear
    struct.pack_into("<f", buf, 20, 6000.0)  # rpms
    struct.pack_into("<f", buf, 24, -0.1)  # steer
    struct.pack_into("<f", buf, 28, speed_kmh)
    struct.pack_into("<3f", buf, 32, *velocity)
    struct.pack_into("<3f", buf, 44, *accg)
    return bytes(buf)


class TestAcPhysicsParsing:
    def test_parse_documented_offsets(self):
        page = build_physics_page(
            packet_id=4242, speed_kmh=137.5,
            velocity=(12.3, -4.5, 0.6),
            accg=(2.5, -11.0, 9.81),
        )
        packet_id, speed_kmh, velocity, accg = AcUdpTelemetrySource.parse_physics(page)
        assert packet_id == 4242
        assert speed_kmh == pytest.approx(137.5)
        assert velocity == pytest.approx((12.3, -4.5, 0.6))
        assert accg == pytest.approx((2.5, -11.0, 9.81))

    def test_rejects_wrong_size(self):
        with pytest.raises(ValueError, match="328"):
            AcUdpTelemetrySource.parse_physics(b"\x00" * 100)


class TestAcUdpLoopback:
    def test_handshake_stream_and_stop(self):
        game = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        game.bind(("127.0.0.1", 0))
        game.settimeout(2.0)
        game_port = game.getsockname()[1]

        source = AcUdpTelemetrySource(remote_port=game_port, timeout=1.0)
        source.start()
        try:
            handshake, addr = game.recvfrom(64)
            assert handshake == b"\x03\x00\x00"

            page = build_physics_page(7, 88.0, (1.0, 2.0, 3.0), (0.25, -0.5, 9.81))
            game.sendto(page, addr)

            sample = source.next_sample()
            assert sample is not None
            assert sample.force == pytest.approx((0.25, -0.5, 9.81))
            assert sample.rate == (0.0, 0.0, 0.0)
            assert source.packets_received == 1

            game.sendto(b"\x00" * 10, addr)
            good = build_physics_page(8, 90.0, (1.0, 2.0, 3.0), (0.5, 0.5, 9.81))
            game.sendto(good, addr)

            sample = source.next_sample()
            assert sample is not None
            assert sample.force == pytest.approx((0.5, 0.5, 9.81))
            assert source.packets_received == 2

            source.stop()
            stop, _ = game.recvfrom(64)
            assert stop == b"\x04"
        finally:
            source.stop()
            game.close()

    def test_timeout_returns_none(self):
        game = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        game.bind(("127.0.0.1", 0))
        game_port = game.getsockname()[1]
        game.close()
        source = AcUdpTelemetrySource(remote_port=game_port, timeout=0.05)
        source.start()
        try:
            assert source.next_sample() is None
        finally:
            source.stop()

    def test_next_sample_before_start_raises(self):
        source = AcUdpTelemetrySource()
        with pytest.raises(RuntimeError, match="start"):
            source.next_sample()
