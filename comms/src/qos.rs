//! QoS profiles for the transform topics.
//!
//! These mirror the profiles `tf2_ros` uses in ROS 2, so a schiebung endpoint
//! and a native ROS 2 node match each other on the wire. Getting these wrong
//! does not surface as an error -- endpoints simply fail to match, and messages
//! are silently dropped.

use hiroz::qos::{QosDurability, QosHistory, QosProfile, QosReliability};
use std::num::NonZeroUsize;

/// Depth of the `/tf` history cache, matching `tf2_ros::TransformBroadcaster`.
const TF_DEPTH: usize = 100;

/// QoS for `/tf`, matching `tf2_ros::TransformBroadcaster`: reliable, volatile,
/// keep-last-100.
///
/// Volatile (rather than transient local) is deliberate: dynamic transforms are
/// a time series, so replaying a stale cached sample to a late joiner would feed
/// the buffer a transform that has long since been superseded.
pub fn tf_qos() -> QosProfile {
    QosProfile {
        reliability: QosReliability::Reliable,
        durability: QosDurability::Volatile,
        history: QosHistory::KeepLast(
            NonZeroUsize::new(TF_DEPTH).expect("TF_DEPTH is a non-zero constant"),
        ),
        ..Default::default()
    }
}

/// QoS for `/tf_static`, matching `tf2_ros::StaticTransformBroadcaster`:
/// reliable, transient local, keep-last-1.
///
/// Transient local is what lets a server that starts *after* a client published
/// its static transforms still receive them, out of the publisher's cache. The
/// client republishes its whole accumulated static set in a single message, so a
/// keep-last-1 cache still carries every static transform.
pub fn tf_static_qos() -> QosProfile {
    QosProfile {
        reliability: QosReliability::Reliable,
        durability: QosDurability::TransientLocal,
        history: QosHistory::KeepLast(NonZeroUsize::new(1).expect("1 is non-zero")),
        ..Default::default()
    }
}
