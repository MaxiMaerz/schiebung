#!/usr/bin/env python3
"""
Example demonstrating Python observer callbacks with schiebung BufferTree.

This shows how to register Python functions as observers to receive
transform updates without needing separate packages like schiebung-rerun-py.
"""

from schiebung import BufferTree, StampedIsometry, TransformType
import time


def simple_observer(from_frame, to_frame, transform, kind):
    """A simple observer that prints transform updates."""
    trans = transform.translation()
    print(f"Transform update: {from_frame} -> {to_frame}")
    print(f"  Translation: [{trans[0]:.2f}, {trans[1]:.2f}, {trans[2]:.2f}]")
    print(f"  Type: {kind}")
    print()


class TransformLogger:
    """An observer class that logs all transforms."""

    def __init__(self, name):
        self.name = name
        self.count = 0

    def __call__(self, from_frame, to_frame, transform, kind):
        self.count += 1
        print(f"[{self.name}] Logged transform #{self.count}: {from_frame} -> {to_frame}")


def main():
    print("=" * 60)
    print("Python Observer Demo")
    print("=" * 60)
    print()

    # Create a buffer tree
    buf = BufferTree()

    # Register a simple function observer
    print("1. Registering simple function observer...")
    buf.register_observer(simple_observer)
    print()

    # Register a class-based observer
    print("2. Registering class-based observer...")
    logger = TransformLogger("MyLogger")
    buf.register_observer(logger)
    print()

    # Add some transforms - observers will be called for each
    print("3. Adding transforms (observers will be notified)...")
    print("-" * 60)

    # Static transform: world -> map
    t1 = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    buf.update("world", "map", t1, TransformType.Static)

    # Static transform: map -> odom
    t2 = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    buf.update("map", "odom", t2, TransformType.Static)

    # Dynamic transform: odom -> base_link (simulating robot movement)
    print("4. Adding dynamic transforms (simulating robot movement)...")
    print("-" * 60)
    for i in range(3):
        x = float(i)
        t = StampedIsometry([x, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], float(i))
        buf.update("odom", "base_link", t, TransformType.Dynamic)
        time.sleep(0.1)

    print("=" * 60)
    print(f"Total transforms logged by {logger.name}: {logger.count}")
    print("=" * 60)
    print()

    # Demonstrate registering observer after transforms exist
    print("5. Registering new observer (will receive all existing transforms)...")
    print("-" * 60)

    late_observer_count = [0]  # Use list to allow modification in closure

    def late_observer(from_frame, to_frame, transform, kind):
        late_observer_count[0] += 1

    buf.register_observer(late_observer)
    print(f"Late observer received {late_observer_count[0]} existing transforms")
    print()

    print("Demo complete!")


if __name__ == "__main__":
    main()
