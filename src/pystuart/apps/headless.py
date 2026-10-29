"""
Headless runtime: telemetry -> washout -> pose stream, no GUI.

The rig-side service shape from the design document (section 11):
pull samples from a telemetry source, push them through the washout
chain, hand poses to a sink. Today the sink prints CSV lines; the
serial CMD_ACT writer joins once the counts calibration exists.

    python -m pystuart.apps.headless ac
    python -m pystuart.apps.headless csv recording.csv --stride 50

Pose lines go to stdout, progress and the final summary to stderr, so
the stream can be piped or recorded directly.
"""
import argparse
import math
import sys
import time

from ..cueing.telemetry import AcUdpTelemetrySource, CsvTelemetrySource
from ..cueing.washout import WashoutConfig, WashoutFilter, load_washout


def csv_sink(pose, sample):
    r = pose.rotation_rad
    print("%f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f" % (
        sample.t, pose.translation_mm[0], pose.translation_mm[1], pose.translation_mm[2],
        math.degrees(r[0]), math.degrees(r[1]), math.degrees(r[2]),
    ))


def run_csv(source: CsvTelemetrySource, washout: WashoutFilter, sink,
            pacing: bool = True, stride: int = 1) -> dict:
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
        if n % stride == 0:
            sink(pose, sample)
        samples += 1
    return {"samples": samples}


def run_ac(source: AcUdpTelemetrySource, washout: WashoutFilter, sink,
           stride: int = 1, max_misses: int = 5) -> dict:
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
        if samples % stride == 0:
            sink(pose, sample)
        samples += 1
    if misses >= max_misses:
        timed_out = True
    return {"samples": samples, "timed_out": timed_out}


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
                        help="rig config with a washout section (built-in defaults if omitted)")
    common.add_argument("--stride", type=int, default=1, metavar="N",
                        help="print every Nth pose (default: all)")

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

    print("t_s,tx_mm,ty_mm,tz_mm,roll_deg,pitch_deg,yaw_deg", file=sys.stderr)

    if args.source == "csv":
        stats = run_csv(
            CsvTelemetrySource(args.file), washout, csv_sink,
            pacing=not args.no_pacing, stride=args.stride,
        )
    else:
        feed = AcUdpTelemetrySource(args.host, args.port, timeout=args.timeout)
        feed.start()
        try:
            stats = run_ac(feed, washout, csv_sink, stride=args.stride)
        finally:
            feed.stop()

    print("processed %d samples%s" % (
        stats["samples"], ", feed timed out" if stats.get("timed_out") else "",
    ), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
