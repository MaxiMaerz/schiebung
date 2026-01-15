#!/usr/bin/env python3
"""
Sun-Earth-Moon Demo Example

This example demonstrates:
1. Creating a RerunBufferTree with Rerun visualization
2. Animating orbital mechanics with dynamic transforms
3. Querying transforms between frames to calculate distances

Equivalent to the Rust example at:
  schiebung/visualizer/schiebung-rerun-rs/examples/demo.rs
"""

import math
import rerun as rr

from schiebung_rerun import (
    RerunBufferTree,
    StampedIsometry,
    TransformType,
)


def main():
    # Create a RerunBufferTree with Rerun visualization
    tree = RerunBufferTree(
        "sun_earth_moon",       # application_id
        "demo_id",              # recording_id
        "stable_time",          # timeline
        True,                   # publish_static_transforms
    )

    # Get the recording stream to log visual elements
    rec = rr.RecordingStream(
        application_id="sun_earth_moon",
        recording_id="demo_id"
    )
    rec.serve_grpc()
    rec.spawn()

    # Log the Sun
    rec.log(
        "Sun",
        rr.Ellipsoids3D(
            half_sizes=[[0.15, 0.15, 0.15]],
            colors=[[255, 200, 0]],
            fill_mode=rr.components.FillMode.Solid,
        ),
        rr.CoordinateFrame("Sun"),
        static=True,
    )

    # Log the Earth
    rec.log(
        "Earth",
        rr.Ellipsoids3D(
            half_sizes=[[0.08, 0.08, 0.08]],
            colors=[[50, 100, 200]],
            fill_mode=rr.components.FillMode.Solid,
        ),
        rr.CoordinateFrame("Earth"),
        static=True,
    )

    # Log the Moon
    rec.log(
        "Moon",
        rr.Ellipsoids3D(
            half_sizes=[[0.04, 0.04, 0.04]],
            colors=[[180, 180, 180]],
            fill_mode=rr.components.FillMode.Solid,
        ),
        rr.CoordinateFrame("Moon"),
        static=True,
    )

    # Orbital parameters
    earth_orbit_radius = 2.0
    moon_orbit_radius = 0.5
    num_steps = 1000
    earth_period = 10.0
    moon_period = 2.0

    for i in range(num_steps):
        time_secs = i * 0.01
        time_ns = int(time_secs * 1_000_000_000)

        # Earth orbit around Sun
        earth_angle = (time_secs / earth_period) * 2.0 * math.pi
        earth_x = earth_orbit_radius * math.cos(earth_angle)
        earth_y = earth_orbit_radius * math.sin(earth_angle)

        earth_transform = StampedIsometry(
            [earth_x, earth_y, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            time_ns,
        )
        tree.buffer.update("Sun", "Earth", earth_transform, TransformType.Dynamic)

        # Moon orbit around Earth
        moon_angle = (time_secs / moon_period) * 2.0 * math.pi
        moon_x = moon_orbit_radius * math.cos(moon_angle)
        moon_y = moon_orbit_radius * math.sin(moon_angle)

        moon_transform = StampedIsometry(
            [moon_x, moon_y, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            time_ns,
        )
        tree.buffer.update("Earth", "Moon", moon_transform, TransformType.Dynamic)

        # Query transform from Moon to Sun and calculate distance
        transform = tree.buffer.lookup_latest_transform("Moon", "Sun")
        if transform is not None:
            translation = transform.translation()
            distance = math.sqrt(
                translation[0]**2 + translation[1]**2 + translation[2]**2
            )

            rec.set_time("stable_time", timestamp=time_secs)
            rec.log(
                "Moon/distance",
                rr.Arrows3D(
                    vectors=[translation],
                    labels=[f"{distance:.2f}"],
                ),
                rr.CoordinateFrame("Moon"),
            )

    print(f"Animated {num_steps} orbital steps")
    print("Check the Rerun viewer to see the Sun-Earth-Moon system!")


if __name__ == "__main__":
    main()
