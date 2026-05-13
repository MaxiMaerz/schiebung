#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

use std::collections::HashMap;
use std::sync::Mutex;

use rerun::{RecordingStream, TimeColumn};
use schiebung::{BufferObserver, TransformType, TransformUpdate};

/// Entity path that carries every dynamic transform. Matches ROS / rerun
/// 0.32+ convention.
const DYNAMIC_ENTITY_PATH: &str = "tf";
/// Entity path that carries every static transform. Matches ROS / rerun
/// 0.32+ convention.
const STATIC_ENTITY_PATH: &str = "tf_static";

/// Observer that logs transforms to a Rerun recording stream.
///
/// Every `on_update` call from the buffer turns into at most two
/// `send_columns` calls: one columnar batch of all dynamic rows under the
/// `tf` entity on the configured timeline, and one batch of all static rows
/// under the `tf_static` entity with no time index. The parent/child relationship
/// of each edge is carried in the `parent_frame` / `child_frame` data columns
/// of rerun's `Transform3D` archetype (named frames) rather than in the
/// entity-path string, so collapsing to two fixed paths does not change the
/// transform graph rerun builds for the 3D view — only the entity-panel
/// tree no longer breaks transforms out per edge.
///
/// Static transforms use rerun's static-logging semantics ("latest write
/// wins per entity"), so the observer keeps an internal snapshot of every
/// static frame it has ever seen and re-sends the full set whenever any
/// static-touching batch arrives. Otherwise a delta with one new static
/// would overwrite all previously logged statics on the collapsed entity.
///
/// If the model (e.g. a URDF) is loaded via rerun the `publish_static_transforms`
/// flag should be set to false; otherwise the static transforms will be logged
/// twice.
///
/// # Example
///
/// ```no_run
/// use rerun::RecordingStreamBuilder;
/// use schiebung::BufferTree;
/// use schiebung_rerun::RerunObserver;
///
/// let rec = RecordingStreamBuilder::new("my_app").spawn()?;
/// let mut buffer = BufferTree::new();
///
/// // Every subsequent buffer.update(&[...]) is bulk-logged to Rerun on
/// // the "stable_time" timeline.
/// buffer.register_observer(Box::new(RerunObserver::new(
///     rec,
///     /* publish_static_transforms = */ true,
///     "stable_time".to_string(),
/// )));
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub struct RerunObserver {
    rec: RecordingStream,
    publish_static_transforms: bool,
    timeline: String,
    /// Every static transform ever seen, keyed by (parent, child). Re-sent
    /// in full on every static-touching `on_update` because rerun-static
    /// replaces all component values on the entity on each write.
    static_state: Mutex<HashMap<(String, String), Row>>,
}

impl RerunObserver {
    /// Build a new `RerunObserver`.
    ///
    /// - `rec` is the destination [`RecordingStream`] (typically created via
    ///   [`rerun::RecordingStreamBuilder`]).
    /// - `publish_static_transforms` controls whether static edges are forwarded
    ///   to Rerun. Set to `false` when another producer already populates Rerun's
    ///   transform tree with the same static frames — for example when calling
    ///   [`rerun::RecordingStream::log_file_from_path`] on a URDF, which loads
    ///   every link's static offset itself. Forwarding the same static edges
    ///   from the buffer in that case results in duplicated frames.
    /// - `timeline` is the name of the Rerun timeline to attach dynamic
    ///   transform timestamps to (e.g. `"stable_time"`). Static transforms
    ///   ignore this and are sent without a time index.
    pub fn new(rec: RecordingStream, publish_static_transforms: bool, timeline: String) -> Self {
        RerunObserver {
            rec,
            publish_static_transforms,
            timeline,
            static_state: Mutex::new(HashMap::new()),
        }
    }
}

/// Single row of columnar data we collect before sending. Strings are owned
/// so rows can be cached in the static-state snapshot across `on_update`
/// calls.
#[derive(Clone)]
struct Row {
    parent: String,
    child: String,
    translation: [f32; 3],
    quaternion: [f32; 4],
    /// Stamp in nanoseconds since unix epoch. Unused for static entries.
    stamp_ns: i64,
}

fn row_from(update: &TransformUpdate) -> Row {
    let t = update.stamped_isometry.translation();
    let r = update.stamped_isometry.rotation();
    Row {
        parent: update.from.clone(),
        child: update.to.clone(),
        translation: [t[0] as f32, t[1] as f32, t[2] as f32],
        quaternion: [r[0] as f32, r[1] as f32, r[2] as f32, r[3] as f32],
        stamp_ns: update.stamped_isometry.stamp(),
    }
}

impl BufferObserver for RerunObserver {
    fn on_update(&self, updates: &[TransformUpdate]) {
        let mut dynamic_rows: Vec<Row> = Vec::new();
        let mut static_updates: Vec<Row> = Vec::new();

        for update in updates {
            match update.kind {
                TransformType::Dynamic => dynamic_rows.push(row_from(update)),
                TransformType::Static => {
                    if self.publish_static_transforms {
                        static_updates.push(row_from(update));
                    }
                }
            }
        }

        if !dynamic_rows.is_empty() {
            self.send_dynamic(DYNAMIC_ENTITY_PATH, &dynamic_rows);
        }

        if !static_updates.is_empty() {
            // Merge deltas into state and snapshot under a single lock, then
            // release before calling rerun so the batcher never blocks on us.
            let snapshot: Vec<Row> = {
                let mut state = self.static_state.lock().unwrap();
                for row in static_updates {
                    state.insert((row.parent.clone(), row.child.clone()), row);
                }
                state.values().cloned().collect()
            };
            self.send_static(STATIC_ENTITY_PATH, &snapshot);
        }
    }
}

impl RerunObserver {
    fn send_dynamic(&self, entity_path: &str, rows: &[Row]) {
        let stamps: Vec<i64> = rows.iter().map(|r| r.stamp_ns).collect();
        let time_column =
            TimeColumn::new_timestamp_nanos_since_epoch(self.timeline.as_str(), stamps);

        if let Some((tf_columns, frame_columns)) = build_columns(rows) {
            self.rec
                .send_columns(
                    entity_path.to_owned(),
                    [time_column],
                    tf_columns.chain(frame_columns),
                )
                .ok();
        }
    }

    fn send_static(&self, entity_path: &str, rows: &[Row]) {
        // Static transforms have no time index — pass an empty timeline list.
        if let Some((tf_columns, frame_columns)) = build_columns(rows) {
            self.rec
                .send_columns(
                    entity_path.to_owned(),
                    std::iter::empty::<TimeColumn>(),
                    tf_columns.chain(frame_columns),
                )
                .ok();
        }
    }
}

/// Build the Transform3D and CoordinateFrame component columns for a batch
/// of rows. Returns `None` if rerun's columnar serialization fails for
/// either archetype (logged via `re_log` inside rerun).
#[allow(clippy::type_complexity)]
fn build_columns(
    rows: &[Row],
) -> Option<(
    impl Iterator<Item = rerun::SerializedComponentColumn>,
    impl Iterator<Item = rerun::SerializedComponentColumn>,
)> {
    let translations: Vec<[f32; 3]> = rows.iter().map(|r| r.translation).collect();
    let quaternions: Vec<rerun::Quaternion> = rows
        .iter()
        .map(|r| rerun::Quaternion::from_xyzw(r.quaternion))
        .collect();
    let parents: Vec<String> = rows.iter().map(|r| r.parent.clone()).collect();
    let children: Vec<String> = rows.iter().map(|r| r.child.clone()).collect();

    let tf_columns = rerun::archetypes::Transform3D::update_fields()
        .with_many_translation(translations)
        .with_many_quaternion(quaternions)
        .with_many_parent_frame(parents.clone())
        .with_many_child_frame(children.clone())
        .columns_of_unit_batches()
        .ok()?;

    let frame_columns = rerun::archetypes::CoordinateFrame::update_fields()
        .with_many_frame(children)
        .columns_of_unit_batches()
        .ok()?;

    Some((tf_columns, frame_columns))
}
