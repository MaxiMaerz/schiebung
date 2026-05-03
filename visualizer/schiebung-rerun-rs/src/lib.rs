use std::collections::HashMap;

use rerun::{RecordingStream, TimeColumn};
use schiebung::{BufferObserver, TransformType, TransformUpdate};

/// Observer that logs transforms to a Rerun recording stream.
///
/// Each `on_update` call from the buffer becomes one or more `send_columns`
/// calls into rerun — one per distinct entity path in the batch — using
/// rerun's columnar bulk-write API. Static and dynamic transforms live on
/// separate entity-path namespaces (`static_transforms/{to}` and
/// `transforms/{from}->{to}`) so they never end up in the same group.
///
/// If the model (e.g. a URDF) is loaded via rerun the `publish_static_transforms`
/// flag should be set to false; otherwise the static transforms will be logged
/// twice.
pub struct RerunObserver {
    rec: RecordingStream,
    publish_static_transforms: bool,
    timeline: String,
}

impl RerunObserver {
    pub fn new(rec: RecordingStream, publish_static_transforms: bool, timeline: String) -> Self {
        RerunObserver {
            rec,
            publish_static_transforms,
            timeline,
        }
    }
}

/// Single row of columnar data we collect per entity path before sending.
struct Row<'a> {
    parent: &'a str,
    child: &'a str,
    translation: [f32; 3],
    quaternion: [f32; 4],
    /// Stamp in nanoseconds since unix epoch. Unused for static entries.
    stamp_ns: i64,
}

fn row_from(update: &TransformUpdate) -> Row<'_> {
    let t = update.stamped_isometry.translation();
    let r = update.stamped_isometry.rotation();
    Row {
        parent: &update.from,
        child: &update.to,
        translation: [t[0] as f32, t[1] as f32, t[2] as f32],
        quaternion: [r[0] as f32, r[1] as f32, r[2] as f32, r[3] as f32],
        stamp_ns: update.stamped_isometry.stamp(),
    }
}

impl BufferObserver for RerunObserver {
    fn on_update(&self, updates: &[TransformUpdate]) {
        // Partition updates by entity path. Dynamic and static use separate
        // path prefixes so a single (kind, entity_path) key isn't necessary.
        let mut dynamic_groups: HashMap<String, Vec<Row<'_>>> = HashMap::new();
        let mut static_groups: HashMap<String, Vec<Row<'_>>> = HashMap::new();

        for update in updates {
            let row = row_from(update);
            match update.kind {
                TransformType::Dynamic => {
                    let path = format!("transforms/{}->{}", row.parent, row.child);
                    dynamic_groups.entry(path).or_default().push(row);
                }
                TransformType::Static => {
                    if !self.publish_static_transforms {
                        continue;
                    }
                    let path = format!("static_transforms/{}", row.child);
                    static_groups.entry(path).or_default().push(row);
                }
            }
        }

        for (entity_path, rows) in dynamic_groups {
            self.send_dynamic(&entity_path, &rows);
        }
        for (entity_path, rows) in static_groups {
            self.send_static(&entity_path, &rows);
        }
    }
}

impl RerunObserver {
    fn send_dynamic(&self, entity_path: &str, rows: &[Row<'_>]) {
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

    fn send_static(&self, entity_path: &str, rows: &[Row<'_>]) {
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

/// Build the Transform3D and CoordinateFrame component columns for one
/// entity-path group. Returns `None` if rerun's columnar serialization fails
/// for either archetype (logged via `re_log` inside rerun).
#[allow(clippy::type_complexity)]
fn build_columns(
    rows: &[Row<'_>],
) -> Option<(
    impl Iterator<Item = rerun::SerializedComponentColumn>,
    impl Iterator<Item = rerun::SerializedComponentColumn>,
)> {
    let translations: Vec<[f32; 3]> = rows.iter().map(|r| r.translation).collect();
    let quaternions: Vec<rerun::Quaternion> = rows
        .iter()
        .map(|r| rerun::Quaternion::from_xyzw(r.quaternion))
        .collect();
    let parents: Vec<String> = rows.iter().map(|r| r.parent.to_owned()).collect();
    let children: Vec<String> = rows.iter().map(|r| r.child.to_owned()).collect();

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
