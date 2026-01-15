#!/usr/bin/env python3
"""
URDF Demo Example

This example demonstrates:
1. Creating a RerunBufferTree with Rerun visualization
2. Loading a URDF robot model using UrdfLoader
3. Animating a joint rotation over time

Equivalent to the Rust example at:
  schiebung/visualizer/schiebung-rerun-rs/examples/urdf_demo.rs
"""

import argparse
import math
import os
from pathlib import Path
import rerun as rr

from schiebung_rerun import (
    RerunBufferTree,
    StampedIsometry,
    TransformType,
    UrdfLoader,
)


def get_default_urdf_path() -> Path:
    """Get the default URDF path relative to this script."""
    # examples/ -> schiebung-rerun-py/ -> visualizer/ -> schiebung/
    script_dir = Path(__file__).parent
    resources_dir = script_dir.parent.parent.parent / "resources"
    return resources_dir / "test_robot.urdf"


def parse_args():
    parser = argparse.ArgumentParser(
        description="URDF Demo - Visualize a URDF robot model with animated joints in Rerun"
    )
    parser.add_argument(
        "urdf_path",
        type=str,
        nargs="?",
        default=str(get_default_urdf_path()),
        help="Path to the URDF file (default: resources/test_robot.urdf)",
    )
    return parser.parse_args()


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    """
    Convert euler angles (roll, pitch, yaw) to quaternion [x, y, z, w].

    Uses the ZYX convention (yaw first, then pitch, then roll).
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [x, y, z, w]


def quaternion_multiply(q1: list[float], q2: list[float]) -> list[float]:
    """
    Multiply two quaternions q1 * q2.

    Quaternions are in [x, y, z, w] format.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def main():
    args = parse_args()
    urdf_path = args.urdf_path

    print(f"Loading URDF from {urdf_path}")

    # Create a RerunBufferTree with Rerun visualization
    # Args: application_id, recording_id, timeline, publish_static_transforms
    # Setting publish_static_transforms=False since Rerun's URDF loader handles those
    tree = RerunBufferTree(
        "urdf_demo",           # application_id
        "urdf_demo_session",   # recording_id
        "stable_time",         # timeline
        False,                 # publish_static_transforms
    )

    # Load URDF into the buffer
    loader = UrdfLoader()
    loader.load_into_buffer(str(urdf_path), tree.buffer)

    rec = rr.RecordingStream(application_id="urdf_demo", recording_id="urdf_demo_session")
    rec.serve_grpc()
    rec.spawn()
    rec.log_file_from_path(str(urdf_path), static=True)

    # Define all dynamic (revolute) joints from the URDF with their initial transforms
    # Each entry: (parent_link, child_link, xyz, rpy)
    dynamic_joints = [
        ("base_link", "shoulder_link", [0.0, 0.0, 0.1273], [0.0, 0.0, 0.0]),
        ("shoulder_link", "upper_arm_link", [0.0, 0.220941, 0.0], [0.0, 1.57079632679, 0.0]),
        ("upper_arm_link", "forearm_link", [0.0, -0.1719, 0.612], [0.0, 0.0, 0.0]),
        ("forearm_link", "wrist_1_link", [0.0, 0.0, 0.5723], [0.0, 1.57079632679, 0.0]),
        ("wrist_1_link", "wrist_2_link", [0.0, 0.1149, 0.0], [0.0, 0.0, 0.0]),
        ("wrist_2_link", "wrist_3_link", [0.0, 0.0, 0.1157], [0.0, 0.0, 0.0]),
    ]

    # Animation parameters
    num_steps = 360
    duration = 5.0  # seconds

    for step in range(num_steps):
        time_secs = step * (duration / num_steps)
        time_ns = int(time_secs * 1_000_000_000)  # Convert to nanoseconds
        angle = (time_secs / duration) * 2.0 * math.pi

        for joint_idx, (parent, child, xyz, rpy) in enumerate(dynamic_joints):
            # Apply a phase-shifted sinusoidal rotation to each joint
            joint_angle = math.sin(angle + joint_idx * 0.5) * 0.5

            # Determine rotation axis from URDF (simplified: use Z for pan joints, Y for lift/elbow)
            if joint_idx == 0:  # shoulder_pan: Z axis
                axis_rotation = (0.0, 0.0, joint_angle)
            elif joint_idx == 4:  # wrist_2: Z axis
                axis_rotation = (0.0, 0.0, joint_angle)
            else:  # shoulder_lift, elbow, wrist_1, wrist_3: Y axis
                axis_rotation = (0.0, joint_angle, 0.0)

            # Combine base rotation from URDF with joint rotation
            base_rotation = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            joint_rotation = quaternion_from_euler(*axis_rotation)
            combined = quaternion_multiply(base_rotation, joint_rotation)

            transform = StampedIsometry(list(xyz), combined, time_ns)
            tree.buffer.update(parent, child, transform, TransformType.Dynamic)

    print(f"Animated {num_steps} steps over {duration} seconds")
    print("Check the Rerun viewer to see the visualization!")


if __name__ == "__main__":
    main()
