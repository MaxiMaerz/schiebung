# schiebung

Fast, memory-safe transform buffer for robotics — a ROS-agnostic alternative to TF2.

`schiebung` stores time-stamped isometries between named frames in a graph and lets you look up any transform between two frames by chaining the edges. It supports interpolation (lerp/slerp) when the requested timestamp falls between samples, batched updates, and a URDF loader.

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
