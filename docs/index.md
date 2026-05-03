# Schiebung

Full [documentation](https://maximaerz.github.io/schiebung/) · API reference: [docs.rs/schiebung](https://docs.rs/schiebung) · [docs.rs/schiebung-rerun](https://docs.rs/schiebung-rerun)

Schiebung is German for "shift" — as in shifting between coordinate frames.

Schiebung is an in-memory transform graph. You declare named frames, push time-stamped rigid-body transforms (isometries) between them, and query any frame's pose relative to any other at any time. Missing samples are interpolated (lerp/slerp); the graph is enforced to be a forest so chains are unambiguous.

Useful anywhere you have multiple coordinate systems that move relative to each other:

- Multi-sensor fusion (LiDAR / camera / IMU rigs, surveying)
- Simulation, games, physics engines
- Motion capture, animation pipelines, articulated CAD
- AR/VR (HMD, controllers, world anchors)
- Robotics, where it serves as a lightweight, ROS-free alternative to [TF2](http://wiki.ros.org/tf) when you don't want to pull in the full ROS stack

Inspiration also from the Rust ROS-1 port [rosrust_tf](https://github.com/arjo129/rustros_tf).

[demo.webm](https://github.com/user-attachments/assets/5c167e6c-ca6a-4a40-af11-8d94ad14fd95)

In the video you can see a very basic example utilizing the TransformBuffer in combination with the rerun visualizer:
We publish the bodies once (Sun, Earth and Moon) and just update the transforms in their respective frames:

- earth circles around the sun
- moon circles around the earth
- The distance between the moon and sun is calculated by a simple frame lookup (Moon -> Sun)

This makes a complex problem trivial. As long as the TransformBuffer is updated, all derived transforms can be calculated at any timestamp (using lerp/slerp), without the caller needing to know anything about the actual chain — only the start and end frames.

[demo_urdf.webm](https://github.com/user-attachments/assets/6f412500-7a5e-43f2-892f-d194e0504573)

The URDF demo loads a 6-DOF arm via the URDF loader, animates each joint, and places a small static cube in the workspace. After every batch joint update we look up `wrist_3_link → target_cube` and log the resulting vector as an arrow with the distance as a label — showing how arbitrary frame-to-frame queries fall out of the buffer for free once the transforms are populated.

## Motivation

ROS [TF2](http://wiki.ros.org/tf) is the canonical implementation of this idea, battle-tested and the right choice if you're already in the ROS ecosystem. Outside of ROS — embedded systems, simulation, AR/VR, custom calibration pipelines — pulling in the full ROS stack just to chain coordinate transforms is a heavy ask. Schiebung is what you reach for when you want the abstraction without the framework.

What you get over rolling your own:

- Sub-microsecond lookups (40–80 ns measured per query)
- Memory safe (Rust); pure-Rust dependency footprint
- Optional features — pull in just the core, or also a [Rerun](https://rerun.io) visualizer / a [Cap'n Proto](https://capnproto.org) + [Zenoh](https://zenoh.io) server
- First-class Python bindings via PyO3, with NumPy interop

## Design goals

- As fast as possible: sub-microsecond lookups and updates
- Framework-agnostic: usable from any application that wants a transform graph, not tied to ROS
- All features are optional: depending on what you need, you can pull in just the core or also the visualizer / server layers
- Minimal, scalable, fast client/server: [Cap'n Proto](https://capnproto.org) + [Zenoh](https://zenoh.io) for over-the-wire transport
- Easy to use: a URDF loader is shipped for the robotics case, a simple [Rerun](https://rerun.io) observer for visualization

## Status

This library is still under development and the API is not considered stable yet and might change.

- The core library is fairly well tested, the Python bindings work.
- The Rerun implementation is a rather shallow wrapper around the core and should be "ok".
- For the server and the full implementation (server + rerun) there is quite some work left.

## Usage

The hosted documentation has runnable examples for each layer:

- [Core](https://maximaerz.github.io/schiebung/schiebung-core/rust/) — Rust and Python implementation of the buffer only, no Rerun or Zenoh dependencies.
- [Comms](https://maximaerz.github.io/schiebung/schiebung-comms/) — Rust and Python implementation of the client/server using Cap'n Proto and Zenoh.
- [Visualizer](https://maximaerz.github.io/schiebung/schiebung-visualizer/rust/) — Rust and Python implementation of the visualizer using Rerun.

Crate-level READMEs ([core](core/schiebung-core-rs/README.md), [rerun](visualizer/schiebung-rerun-rs/README.md)) and the [`examples/`](visualizer/schiebung-rerun-rs/examples/) directory have shorter snippets to copy.
