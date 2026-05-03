# schiebung

Fast, memory-safe in-memory transform graph with time-aware lookups.

`schiebung` stores time-stamped isometries between named frames and lets you query any frame's pose relative to any other by chaining the edges. It supports lerp/slerp interpolation when the requested timestamp falls between samples, batched updates, and a URDF loader for robotics use.

Useful anywhere you have multiple coordinate systems that move relative to each other — multi-sensor fusion, simulation, motion capture, AR/VR, robotics. The closest analogue is ROS [TF2](http://wiki.ros.org/tf); `schiebung` is for projects that want the abstraction without pulling in ROS.

- **Documentation:** <https://maximaerz.github.io/schiebung/>
- **Repository:** <https://github.com/MaxiMaerz/schiebung>
- **Rust API guide:** <https://maximaerz.github.io/schiebung/schiebung-core/rust/>

## At a glance

```rust
use schiebung::{BufferTree, StampedIsometry, TransformType, TransformUpdate};

let mut buffer = BufferTree::new();

buffer.update(&[
    TransformUpdate::new(
        "world",
        "robot_base",
        StampedIsometry::from_secs([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0),
        TransformType::Static,
    ),
])?;

let tf = buffer.lookup_latest_transform("world", "robot_base")?;
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Companion crates

- [`schiebung-rerun`](https://crates.io/crates/schiebung-rerun) — visualize the transform graph live in [Rerun](https://rerun.io).

## License

MIT
