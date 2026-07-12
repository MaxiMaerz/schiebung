//! Conversions between schiebung core types and ROS 2 messages.

use crate::msgs::ros::builtin_interfaces::Time;
use crate::msgs::ros::geometry_msgs::{Quaternion, Transform, TransformStamped, Vector3};
use crate::msgs::ros::std_msgs::Header;
use crate::msgs::ros::tf2_msgs::TFMessage;
use schiebung::types::{StampedIsometry, TransformType, TransformUpdate};

/// Convert nanoseconds since the Unix epoch into a ROS `builtin_interfaces/Time`.
///
/// Uses euclidean division so negative timestamps follow the ROS convention
/// (`-1.7s == {sec: -2, nanosec: 3e8}`, `nanosec` always in `[0, 1e9)`).
pub fn ns_to_time(ns: i64) -> Time {
    Time {
        sec: ns.div_euclid(1_000_000_000) as i32,
        nanosec: ns.rem_euclid(1_000_000_000) as u32,
    }
}

/// Convert a ROS `builtin_interfaces/Time` into nanoseconds since the Unix epoch.
pub fn time_to_ns(time: &Time) -> i64 {
    time.sec as i64 * 1_000_000_000 + time.nanosec as i64
}

/// Build a `geometry_msgs/TransformStamped` from a schiebung transform.
///
/// `from` becomes `header.frame_id` (the reference frame), `to` becomes
/// `child_frame_id` — matching the tf2 convention.
pub fn stamped_isometry_to_transform_stamped(
    from: &str,
    to: &str,
    stamped_isometry: &StampedIsometry,
) -> TransformStamped {
    let translation = stamped_isometry.translation();
    let rotation = stamped_isometry.rotation();
    TransformStamped {
        header: Header {
            stamp: ns_to_time(stamped_isometry.stamp()),
            frame_id: from.to_string(),
        },
        child_frame_id: to.to_string(),
        transform: Transform {
            translation: Vector3 {
                x: translation[0],
                y: translation[1],
                z: translation[2],
            },
            rotation: Quaternion {
                x: rotation[0],
                y: rotation[1],
                z: rotation[2],
                w: rotation[3],
            },
        },
    }
}

/// Convert a `geometry_msgs/TransformStamped` into a schiebung [`StampedIsometry`].
pub fn transform_stamped_to_stamped_isometry(ts: &TransformStamped) -> StampedIsometry {
    StampedIsometry::new(
        [
            ts.transform.translation.x,
            ts.transform.translation.y,
            ts.transform.translation.z,
        ],
        [
            ts.transform.rotation.x,
            ts.transform.rotation.y,
            ts.transform.rotation.z,
            ts.transform.rotation.w,
        ],
        time_to_ns(&ts.header.stamp),
    )
}

/// Convert a `geometry_msgs/TransformStamped` into a schiebung [`TransformUpdate`].
pub fn transform_stamped_to_update(ts: &TransformStamped, kind: TransformType) -> TransformUpdate {
    TransformUpdate::new(
        ts.header.frame_id.clone(),
        ts.child_frame_id.clone(),
        transform_stamped_to_stamped_isometry(ts),
        kind,
    )
}

/// Convert a `tf2_msgs/TFMessage` into a batch of schiebung [`TransformUpdate`]s.
pub fn tf_message_to_updates(msg: &TFMessage, kind: TransformType) -> Vec<TransformUpdate> {
    msg.transforms
        .iter()
        .map(|ts| transform_stamped_to_update(ts, kind))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ns_time_roundtrip() {
        for ns in [
            0i64,
            1,
            999_999_999,
            1_000_000_000,
            1_700_000_000_123_456_789,
            -1,
            -1_700_000_000,
        ] {
            assert_eq!(time_to_ns(&ns_to_time(ns)), ns, "roundtrip failed for {ns}");
        }
    }

    #[test]
    fn test_negative_time_ros_convention() {
        // -1.7s == {sec: -2, nanosec: 3e8} per builtin_interfaces/Time docs
        let t = ns_to_time(-1_700_000_000);
        assert_eq!(t.sec, -2);
        assert_eq!(t.nanosec, 300_000_000);
    }

    #[test]
    fn test_nanosec_always_in_range() {
        for ns in [-1i64, -999_999_999, 999_999_999, 1_000_000_001] {
            let t = ns_to_time(ns);
            assert!(t.nanosec < 1_000_000_000);
        }
    }

    #[test]
    fn test_transform_stamped_roundtrip() {
        let iso = StampedIsometry::new(
            [1.0, 2.0, 3.0],
            [0.0, 0.0, 0.7071067811865476, 0.7071067811865476],
            1_700_000_000_000_000_000,
        );
        let ts = stamped_isometry_to_transform_stamped("world", "base", &iso);
        assert_eq!(ts.header.frame_id, "world");
        assert_eq!(ts.child_frame_id, "base");

        let roundtripped = transform_stamped_to_stamped_isometry(&ts);
        assert_eq!(roundtripped.stamp(), iso.stamp());
        assert_eq!(roundtripped.translation(), iso.translation());
        let (a, b) = (roundtripped.rotation(), iso.rotation());
        for i in 0..4 {
            assert!((a[i] - b[i]).abs() < 1e-12);
        }
    }

    #[test]
    fn test_tf_message_to_updates() {
        let iso = StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 42);
        let msg = TFMessage {
            transforms: vec![
                stamped_isometry_to_transform_stamped("world", "a", &iso),
                stamped_isometry_to_transform_stamped("a", "b", &iso),
            ],
        };
        let updates = tf_message_to_updates(&msg, TransformType::Static);
        assert_eq!(updates.len(), 2);
        assert_eq!(updates[0].from, "world");
        assert_eq!(updates[0].to, "a");
        assert_eq!(updates[1].from, "a");
        assert_eq!(updates[1].to, "b");
        assert!(updates
            .iter()
            .all(|u| matches!(u.kind, TransformType::Static)));
    }
}
