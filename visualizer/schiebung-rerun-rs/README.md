# schiebung-rerun

[Rerun](https://rerun.io) visualization adapter for the [`schiebung`](https://crates.io/crates/schiebung) transform buffer.

`RerunObserver` implements `BufferObserver` and bulk-logs every batched buffer update to a Rerun recording stream via the columnar `send_columns` API. All dynamic transforms land on a single `tf` entity and all static transforms on a single `tf_static` entity (ROS / Rerun 0.32+ convention) — so a full transform-graph snapshot (e.g. a multi-sensor rig or a robot pose) becomes at most two columnar writes per batch rather than N row-oriented logs. The parent/child relationship of each edge is carried as named-frames data on the `Transform3D` archetype, so collapsing the entity paths does not change the transform graph the 3D viewer builds.

- **Documentation:** <https://maximaerz.github.io/schiebung/>
- **Repository:** <https://github.com/MaxiMaerz/schiebung>
- **Rerun API guide:** <https://maximaerz.github.io/schiebung/schiebung-visualizer/rust/>

## At a glance

```rust,no_run
use rerun::RecordingStreamBuilder;
use schiebung::BufferTree;
use schiebung_rerun::RerunObserver;

let rec = RecordingStreamBuilder::new("my_app").spawn()?;
let mut buffer = BufferTree::new();

let observer = RerunObserver::new(rec.clone(), true, "stable_time".to_string());
buffer.register_observer(Box::new(observer));

// Every `buffer.update(&[...])` now also gets logged to Rerun.
# Ok::<(), Box<dyn std::error::Error>>(())
```

## Examples

See the [`examples/`](https://github.com/MaxiMaerz/schiebung/tree/main/visualizer/schiebung-rerun-rs/examples) directory in the repo for a Sun-Earth-Moon demo and a URDF-loaded 6-DOF arm demo.

## License

MIT
