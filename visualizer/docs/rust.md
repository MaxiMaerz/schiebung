# Schiebung Rerun (Rust)

Rust library for visualizing Schiebung transforms using [Rerun](https://rerun.io/).

## Overview

This crate provides a `RerunObserver` that can be attached to a `BufferTree` to automatically log all transform updates to Rerun for visualization.

Currently the logging of Transforms is rather simple:

* from is the parent frame
* to is the child frame
* All is under a flat namespace /transforms/{from}->{to}
* We use the provided timeline to set the timestamp for each transform

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
