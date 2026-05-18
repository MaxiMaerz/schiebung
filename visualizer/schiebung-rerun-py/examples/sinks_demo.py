#!/usr/bin/env python3
"""
Manual demo for the new ``sinks=[...]`` argument on ``RerunBufferTree``.

This file exercises the two sink kinds that are awkward to assert in CI:

* ``GrpcSink`` — needs a live viewer at the other end to do anything visible.
* ``Stdout``  — writes the raw rrd byte stream to file descriptor 1, which
  would clobber pytest's output.

The CI-safe sink modes (``FileSink``, ``BinaryStream``, multi-sink fan-out,
and all the conflict / error paths) are covered by ``tests/test_rerun_buffer.py``.

Run modes
---------

1.  GrpcSink (default) — fan out to a viewer and an .rrd file in one recording.
    Open a viewer first, then run the script::

        rerun &                       # in another terminal
        python examples/sinks_demo.py

    You should see a "satellite" sphere orbit a "planet" sphere live, and a
    ``/tmp/schiebung_sinks_demo.rrd`` written to disk that you can replay::

        rerun /tmp/schiebung_sinks_demo.rrd

2.  Stdout — pipe the rrd byte stream into a viewer in one command::

        python examples/sinks_demo.py --stdout | rerun -

    Same orbit, but streamed over stdout instead of a file/gRPC sink.
"""

import argparse
import math
import time
from pathlib import Path

import rerun as rr

from schiebung_rerun import (
    BinaryStream,
    FileSink,
    GrpcSink,
    RerunBufferTree,
    StampedIsometry,
    Stdout,
    TransformType,
)

APP_ID = "schiebung_sinks_demo"
RECORDING_ID = "sinks_demo"
NUM_STEPS = 200


def stream_orbit(tree: RerunBufferTree, *, sleep: float = 0.01) -> None:
    """Stream a 200-step satellite orbit through `tree.buffer`."""
    for i in range(NUM_STEPS):
        t_secs = i * 0.05
        angle = t_secs * 2.0 * math.pi / 5.0
        transform = StampedIsometry(
            [math.cos(angle), math.sin(angle), 0.0],
            [0.0, 0.0, 0.0, 1.0],
            int(t_secs * 1_000_000_000),
        )
        tree.buffer.update("planet", "satellite", transform, TransformType.Dynamic)
        if sleep:
            time.sleep(sleep)


def _log_static_geometry(rec: rr.RecordingStream) -> None:
    """Pin spheres to the planet/satellite frames so the orbit is visible."""
    rr.log(
        "planet",
        rr.Ellipsoids3D(half_sizes=[[0.25, 0.25, 0.25]], colors=[[50, 100, 200]],
                        fill_mode=rr.components.FillMode.Solid),
        rr.CoordinateFrame("planet"),
        static=True,
        recording=rec,
    )
    rr.log(
        "satellite",
        rr.Ellipsoids3D(half_sizes=[[0.08, 0.08, 0.08]], colors=[[200, 200, 200]],
                        fill_mode=rr.components.FillMode.Solid),
        rr.CoordinateFrame("satellite"),
        static=True,
        recording=rec,
    )


def run_grpc_demo(rrd_path: Path) -> None:
    """GrpcSink + FileSink multi-sink: viewer AND an .rrd file."""
    print(
        "Make sure a Rerun viewer is already running "
        "(`rerun` in another terminal).",
    )
    print(f"Streaming to GrpcSink() and FileSink({rrd_path!s}) ...")

    # We also keep a BinaryStream so we can print the final byte count at the
    # end — proves the multi-sink really fans out to all three destinations.
    bs = BinaryStream()
    tree = RerunBufferTree(
        APP_ID, RECORDING_ID, "stable_time", True,
        sinks=[GrpcSink(), FileSink(str(rrd_path)), bs],
    )

    # Static geometry needs its own rerun.RecordingStream because BufferTree
    # only logs transforms. We use rr.connect_grpc() to share the same viewer.
    geom_rec = rr.RecordingStream(APP_ID, recording_id=f"{RECORDING_ID}_geom")
    geom_rec.connect_grpc()
    _log_static_geometry(geom_rec)

    stream_orbit(tree)

    data = bs.read()
    del tree  # drop the recording so the FileSink finalises the .rrd
    print(f"Streamed {NUM_STEPS} transforms. "
          f"BinaryStream captured {len(data)} bytes. "
          f"File: {rrd_path} ({rrd_path.stat().st_size} bytes).")
    print(f"Replay with: rerun {rrd_path}")


def run_stdout_demo() -> None:
    """Stdout sink: pipe rrd bytes to stdout for `... | rerun -`."""
    # Suppress any rerun-sdk diagnostics that would also land on stdout.
    import sys
    sys.stderr.write("Streaming to stdout — pipe me into `rerun -`.\n")

    tree = RerunBufferTree(
        APP_ID, RECORDING_ID, "stable_time", True,
        sinks=[Stdout()],
    )
    stream_orbit(tree, sleep=0.0)  # don't sleep — let `rerun -` keep up
    del tree


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stdout", action="store_true",
                    help="Run the Stdout-sink demo instead of the GrpcSink demo.")
    ap.add_argument("--rrd-path", default="/tmp/schiebung_sinks_demo.rrd",
                    help="Path for the FileSink output (default: %(default)s).")
    args = ap.parse_args()

    if args.stdout:
        run_stdout_demo()
    else:
        run_grpc_demo(Path(args.rrd_path))


if __name__ == "__main__":
    main()
