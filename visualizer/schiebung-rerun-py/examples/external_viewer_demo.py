#!/usr/bin/env python3
"""
External-viewer demo for the `spawn` flag.

`RerunBufferTree` normally spawns its own Rerun viewer. Here we instead spawn the
viewer via the rerun SDK (`rr.init(spawn=True)`) and pass `spawn=False` so the
buffer tree *connects* to that already-running viewer over gRPC rather than
launching a second one. You can also point it at an arbitrary endpoint with
`connect_addr="rerun+http://<host>:<port>/proxy"`.

Run it:
    python examples/external_viewer_demo.py

Exactly one Rerun viewer window should open, showing a "satellite" sphere
orbiting a "planet" sphere. The orbit comes from transforms streamed through the
`spawn=False` buffer tree; the spheres are pinned to the `planet` / `satellite`
frames so they move with the transform graph.
"""

import math
import time

import rerun as rr

from schiebung_rerun import RerunBufferTree, StampedIsometry, TransformType

APP_ID = "external_viewer_demo"
RECORDING_ID = "ext_demo"


def main():
    # 1. Spawn the viewer ourselves (this is the "external" viewer).
    rr.init(APP_ID, recording_id=RECORDING_ID, spawn=True)

    # Geometry pinned to the transform frames. Entities tagged with a
    # CoordinateFrame are placed at that frame in the transform graph, so the
    # satellite sphere follows the transforms streamed below.
    rr.log(
        "planet",
        rr.Ellipsoids3D(half_sizes=[[0.25, 0.25, 0.25]], colors=[[50, 100, 200]],
                        fill_mode=rr.components.FillMode.Solid),
        rr.CoordinateFrame("planet"),
        static=True,
    )
    rr.log(
        "satellite",
        rr.Ellipsoids3D(half_sizes=[[0.08, 0.08, 0.08]], colors=[[200, 200, 200]],
                        fill_mode=rr.components.FillMode.Solid),
        rr.CoordinateFrame("satellite"),
        static=True,
    )

    # 2. Buffer tree connects to that viewer instead of spawning another one.
    tree = RerunBufferTree(
        APP_ID,
        RECORDING_ID,
        "stable_time",
        True,          # publish_static_transforms
        spawn=False,   # <-- the flag under test: do NOT spawn, connect instead
        # connect_addr="rerun+http://127.0.0.1:9876/proxy",  # equivalent explicit form
    )

    num_steps = 200
    for i in range(num_steps):
        t_secs = i * 0.05
        angle = t_secs * 2.0 * math.pi / 5.0
        transform = StampedIsometry(
            [math.cos(angle), math.sin(angle), 0.0],
            [0.0, 0.0, 0.0, 1.0],
            int(t_secs * 1_000_000_000),
        )
        tree.buffer.update("planet", "satellite", transform, TransformType.Dynamic)
        time.sleep(0.01)

    print(f"Streamed {num_steps} transforms to the externally-spawned viewer.")
    print("Look for the 'satellite' sphere orbiting the 'planet' sphere in the Rerun window.")


if __name__ == "__main__":
    main()
