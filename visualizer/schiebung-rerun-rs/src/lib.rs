use rerun::RecordingStream;
use schiebung::{BufferObserver, StampedIsometry};

/// Observer that logs transforms to a Rerun recording stream
/// If the model (e.g. a URDF) is laoded via rerun the publish_static_transforms flag should be set to false
/// Otherwise the static transforms will be logged twice.
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

impl BufferObserver for RerunObserver {
    fn on_update(
        &self,
        from: &str,
        to: &str,
        transform: &StampedIsometry,
        kind: schiebung::TransformType,
    ) {
        let t = transform.translation();
        let r = transform.rotation();

        let arch = rerun::archetypes::Transform3D::from_translation([
            t[0] as f32,
            t[1] as f32,
            t[2] as f32,
        ])
        .with_rotation(rerun::Quaternion::from_xyzw([
            r[0] as f32,
            r[1] as f32,
            r[2] as f32,
            r[3] as f32,
        ]))
        .with_parent_frame(from)
        .with_child_frame(to);

        let coord_frame = rerun::CoordinateFrame::new(to);

        match kind {
            schiebung::TransformType::Static => {
                if self.publish_static_transforms {
                    self.rec
                        .set_timestamp_secs_since_epoch(&*self.timeline, transform.stamp_secs());
                    self.rec
                        .log_static(
                            format!("static_transforms/{}", to),
                            &[&arch as &dyn rerun::AsComponents, &coord_frame],
                        )
                        .ok();
                }
            }
            schiebung::TransformType::Dynamic => {
                self.rec
                    .set_timestamp_secs_since_epoch(&*self.timeline, transform.stamp_secs());
                self.rec
                    .log(
                        format!("transforms/{}->{}", from, to),
                        &[&arch as &dyn rerun::AsComponents, &coord_frame],
                    )
                    .ok();
            }
        }
    }
}
