"""
Headless runtime: telemetry -> washout -> IK -> servo bus, no GUI.

The rig-side service shape from the design document (section 11):
pull samples from a telemetry source, push them through the washout
chain, run IK on the resulting pose and stream CMD_ACT targets to the
Bridge over serial. Without --serial the pose stream still prints, so
the rig behaves as an offline simulator.

    python -m pystuart.apps.headless ac --serial COM7
    python -m pystuart.apps.headless csv recording.csv --stride 50

Pose lines go to stdout, progress and the final summary to stderr, so
the stream can be piped or recorded directly. On exit torque is
released unconditionally.
"""
import argparse
import math
import sys
import time

from ..core.config import Geometry, load_geometry
from ..core.platform import StewartPlatform
from ..cueing.telemetry import AcUdpTelemetrySource, CsvTelemetrySource
from ..cueing.washout import WashoutConfig, WashoutFilter, load_washout
from ..transport.actuator import ActuatorWriter, PoseSender
from ..transport.serial_link import SerialLink
from ..transport.servo_map import ServoCalibration, load_servo_calibration


def csv_sink(pose, sample):
    r = pose.rotation_rad
    print("%f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f" % (
        sample.t, pose.translation_mm[0], pose.translation_mm[1], pose.translation_mm[2],
        math.degrees(r[0]), math.degrees(r[1]), math.degrees(r[2]),
    ))


def _sender_stats(sender) -> dict:
    if sender is None:
        return {"servos_sent": 0, "invalid_poses": 0}
    return {"servos_sent": sender.sent, "invalid_poses": sender.skipped}


def run_csv(source: CsvTelemetrySource, washout: WashoutFilter, sink,
            pacing: bool = True, stride: int = 1,
            sender: PoseSender = None) -> dict:
    """Replay a CSV source; pacing follows the sample time column."""
    samples = 0
    origin = None
    for n, sample in enumerate(source):
        if pacing:
            if origin is None:
                origin = time.monotonic() - sample.t
            delay = origin + sample.t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        pose = washout.step(sample.force, sample.rate)
        if sender is not None:
            sender.send(pose.translation_mm, pose.rotation_rad)
        if n % stride == 0:
            sink(pose, sample)
        samples += 1
    stats = {"samples": samples}
    stats.update(_sender_stats(sender))
    return stats


def run_ac(source: AcUdpTelemetrySource, washout: WashoutFilter, sink,
           stride: int = 1, max_misses: int = 5,
           sender: PoseSender = None) -> dict:
    """Pull from a live AC feed; stop after max_misses empty polls."""
    samples = 0
    misses = 0
    timed_out = False
    while misses < max_misses:
        sample = source.next_sample()
        if sample is None:
            misses += 1
            continue
        misses = 0
        pose = washout.step(sample.force, sample.rate)
        if sender is not None:
            sender.send(pose.translation_mm, pose.rotation_rad)
        if samples % stride == 0:
            sink(pose, sample)
        samples += 1
    if misses >= max_misses:
        timed_out = True
    stats = {"samples": samples, "timed_out": timed_out}
    stats.update(_sender_stats(sender))
    return stats


def make_washout(config_path) -> WashoutFilter:
    if config_path:
        return WashoutFilter(load_washout(config_path))
    return WashoutFilter(WashoutConfig(dt=0.005))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="pystuart-headless",
        description="Telemetry -> washout -> pose stream without the GUI.",
    )
    sub = parser.add_subparsers(dest="source", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--config", default=None, metavar="YAML",
                        help="rig config with washout, geometry and servo sections "
                             "(built-in defaults if omitted)")
    common.add_argument("--stride", type=int, default=1, metavar="N",
                        help="print every Nth pose (default: all)")
    common.add_argument("--serial", default=None, metavar="PORT",
                        help="stream CMD_ACT targets to the Bridge on this port (e.g. COM7)")
    common.add_argument("--rate", type=int, default=100, metavar="HZ",
                        help="CMD_ACT send rate with --serial (default: 100 Hz)")

    ac = sub.add_parser("ac", parents=[common], help="live Assetto Corsa UDP feed")
    ac.add_argument("--host", default="127.0.0.1")
    ac.add_argument("--port", type=int, default=9996)
    ac.add_argument("--timeout", type=float, default=1.0)

    csvp = sub.add_parser("csv", parents=[common], help="replay a recorded telemetry CSV")
    csvp.add_argument("file", metavar="CSV")
    csvp.add_argument("--no-pacing", action="store_true",
                      help="run as fast as possible instead of real time")

    args = parser.parse_args(argv)
    washout = make_washout(args.config)
    if args.stride < 1:
        parser.error("--stride must be >= 1")
    if args.rate < 1:
        parser.error("--rate must be >= 1")

    actuator = None
    if args.serial:
        geometry = load_geometry(args.config) if args.config else Geometry.desktop()
        calibration = (load_servo_calibration(args.config) if args.config
                       else ServoCalibration())
        actuator = ActuatorWriter(SerialLink(args.serial), calibration)
        sender = PoseSender(actuator, StewartPlatform(geometry), rate_hz=args.rate)
    else:
        sender = None

    print("t_s,tx_mm,ty_mm,tz_mm,roll_deg,pitch_deg,yaw_deg", file=sys.stderr)

    try:
        if args.source == "csv":
            stats = run_csv(
                CsvTelemetrySource(args.file), washout, csv_sink,
                pacing=not args.no_pacing, stride=args.stride, sender=sender,
            )
        else:
            feed = AcUdpTelemetrySource(args.host, args.port, timeout=args.timeout)
            feed.start()
            try:
                stats = run_ac(feed, washout, csv_sink, stride=args.stride,
                               sender=sender)
            finally:
                feed.stop()
    finally:
        if actuator is not None:
            actuator.torque_off()
            actuator.close()

    print("processed %d samples, %d servo frames, %d poses skipped%s" % (
        stats["samples"], stats["servos_sent"], stats["invalid_poses"],
        ", feed timed out" if stats.get("timed_out") else "",
    ), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
