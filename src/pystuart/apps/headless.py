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
from ..transport.actuator import ActuatorWriter
from ..transport.serial_link import SerialLink
from ..transport.servo_map import ServoCalibration, load_servo_calibration


def csv_sink(pose, sample):
    r = pose.rotation_rad
    print("%f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f" % (
        sample.t, pose.translation_mm[0], pose.translation_mm[1], pose.translation_mm[2],
        math.degrees(r[0]), math.degrees(r[1]), math.degrees(r[2]),
    ))


def _maybe_send(actuator, ik_model, pose, state, send_interval):
    """Throttled IK + CMD_ACT send; returns (sent, skipped)."""
    if actuator is None:
        return 0, 0
    now = time.monotonic()
    if now - state["last_send"] < send_interval:
        return 0, 0
    state["last_send"] = now
    ik_model.update_pose(pose.translation_mm, pose.rotation_rad)
    if not ik_model.is_valid_pose():
        return 0, 1
    actuator.send_targets(ik_model.get_alpha_angles())
    return 1, 0


def run_csv(source: CsvTelemetrySource, washout: WashoutFilter, sink,
            pacing: bool = True, stride: int = 1,
            actuator: ActuatorWriter = None, ik_model: StewartPlatform = None,
            send_interval: float = 0.0) -> dict:
    """Replay a CSV source; pacing follows the sample time column."""
    samples = sent = invalid = 0
    origin = None
    send_state = {"last_send": float("-inf")}
    for n, sample in enumerate(source):
        if pacing:
            if origin is None:
                origin = time.monotonic() - sample.t
            delay = origin + sample.t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        pose = washout.step(sample.force, sample.rate)
        s, i = _maybe_send(actuator, ik_model, pose, send_state, send_interval)
        sent += s
        invalid += i
        if n % stride == 0:
            sink(pose, sample)
        samples += 1
    return {"samples": samples, "servos_sent": sent, "invalid_poses": invalid}


def run_ac(source: AcUdpTelemetrySource, washout: WashoutFilter, sink,
           stride: int = 1, max_misses: int = 5,
           actuator: ActuatorWriter = None, ik_model: StewartPlatform = None,
           send_interval: float = 0.0) -> dict:
    """Pull from a live AC feed; stop after max_misses empty polls."""
    samples = sent = invalid = 0
    misses = 0
    timed_out = False
    send_state = {"last_send": float("-inf")}
    while misses < max_misses:
        sample = source.next_sample()
        if sample is None:
            misses += 1
            continue
        misses = 0
        pose = washout.step(sample.force, sample.rate)
        s, i = _maybe_send(actuator, ik_model, pose, send_state, send_interval)
        sent += s
        invalid += i
        if samples % stride == 0:
            sink(pose, sample)
        samples += 1
    if misses >= max_misses:
        timed_out = True
    return {"samples": samples, "servos_sent": sent,
            "invalid_poses": invalid, "timed_out": timed_out}


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

    actuator = ik_model = None
    if args.serial:
        geometry = load_geometry(args.config) if args.config else Geometry.desktop()
        calibration = (load_servo_calibration(args.config) if args.config
                       else ServoCalibration())
        ik_model = StewartPlatform(geometry)
        actuator = ActuatorWriter(SerialLink(args.serial), calibration)

    print("t_s,tx_mm,ty_mm,tz_mm,roll_deg,pitch_deg,yaw_deg", file=sys.stderr)

    try:
        if args.source == "csv":
            stats = run_csv(
                CsvTelemetrySource(args.file), washout, csv_sink,
                pacing=not args.no_pacing, stride=args.stride,
                actuator=actuator, ik_model=ik_model,
                send_interval=1.0 / args.rate if actuator else 0.0,
            )
        else:
            feed = AcUdpTelemetrySource(args.host, args.port, timeout=args.timeout)
            feed.start()
            try:
                stats = run_ac(feed, washout, csv_sink, stride=args.stride,
                               actuator=actuator, ik_model=ik_model,
                               send_interval=1.0 / args.rate if actuator else 0.0)
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
