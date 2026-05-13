# Schiebung Rerun (Rust)

Rust library for visualizing Schiebung transforms using [Rerun](https://rerun.io/).

> **API reference:** the full type-by-type API is on [docs.rs/schiebung-rerun](https://docs.rs/schiebung-rerun). This page is a quick-start; use the rustdoc reference for exact signatures.

## Overview

This crate provides a `RerunObserver` that can be attached to a `BufferTree` to automatically log all transform updates to Rerun for visualization.

Transform logging is intentionally minimal and tuned for Rerun's batcher:

* Every buffer batch becomes at most two `send_columns` calls — one for dynamic transforms under the `tf` entity, one for static transforms under `tf_static` (matching the ROS / Rerun 0.32+ convention).
* The parent/child relationship of each edge is carried in the `parent_frame` / `child_frame` data columns of Rerun's `Transform3D` archetype (named frames), not in the entity-path string, so the 3D viewer still builds the full transform graph from the collapsed paths.
* Dynamic rows land on the user-supplied timeline at their original per-row stamps. Static rows are sent with no time index (Rerun-static semantics); the observer keeps an internal snapshot of every static frame ever seen and re-sends the full set on each static-touching update so deltas never wipe earlier statics from the collapsed entity.

Trade-off vs. the older `transforms/{from}->{to}` layout: the entity panel no longer breaks transforms out per edge, but the 3D positioning is unchanged.

## Example

```rust
use schiebung::BufferTree;
use schiebung_rerun::RerunObserver;
use rerun::RecordingStream;

fn main() {
    // Create a Rerun recording
    let rec = rerun::RecordingStreamBuilder::new("my_app")
        .spawn()
        .unwrap();

    // Create observer and attach to buffer
    let observer = RerunObserver::new(rec, true, "stable_time".to_string());
    let mut buffer = BufferTree::new();
    buffer.register_observer(Box::new(observer));

    // All updates will now be logged to Rerun
    buffer.update("world", "robot", transform, TransformType::Static);
}
```

## RerunBufferTree

We also provide a `RerunBufferTree` which is a drop-in replacement for `BufferTree`. This is most likely of little use for a implementation in Rust, but it is useful for Python bindings where we can not just mix and match.
