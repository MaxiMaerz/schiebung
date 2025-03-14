use std::sync::{Arc, Mutex};

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use rclrs::*;
use schiebung_core::types::{StampedIsometry, TransformType};
use schiebung_core::{BufferTree, TfError};
use tf2_msgs::msg::TFMessage;

/// TF Buffer listening to tf and tf_static using the Buffer core implementation of schiebung
///
/// If you your system requires a lot of TF access it might be better to use the client server architecture
/// otherwise each Buffer instance will listen to the tf topics and contain the full TF tree in memory.
///
/// The executor will NOT spin automatically but must be spun by the user
pub struct RosBuffer {
    buffer: Arc<Mutex<BufferTree>>,
    _node: Arc<Node>,
    _tf_subscriber: Arc<Subscription<TFMessage>>,
    _static_tf_subscriber: Arc<Subscription<TFMessage>>,
}

impl RosBuffer {
    pub fn new(executor: &Executor) -> Result<Self, Box<dyn std::error::Error>> {
        let node = executor.create_node("tf_relay")?;
        let buffer = Arc::new(Mutex::new(BufferTree::new()));
        let buffer_clone = Arc::clone(&buffer);
        let tf_subscriber = node.create_subscription::<TFMessage, _>(
            "/tf",
            QOS_PROFILE_DEFAULT,
            move |msg: TFMessage| {
                for transform in msg.transforms {
                    let stamp = transform.header.stamp.sec as f64
                        + (transform.header.stamp.nanosec as f64) * 1e-9;
                    let source = transform.header.frame_id;
                    let target = transform.child_frame_id;
                    let trans = Translation3::new(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    );
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        transform.transform.rotation.w,
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                    ));
                    let isometry = Isometry3::from_parts(trans, rot);
                    let stamped_transform = StampedIsometry {
                        isometry: isometry,
                        stamp: stamp,
                    };
                    buffer_clone
                        .lock()
                        .unwrap()
                        .update(source, target, stamped_transform, TransformType::Dynamic)
                        .unwrap();
                }
            },
        )?;
        let buffer_clone = Arc::clone(&buffer);
        let static_tf_subscriber = node.create_subscription::<TFMessage, _>(
            "/tf_static",
            QoSProfile {
                history: QoSHistoryPolicy::KeepLast { depth: 1 },
                reliability: QoSReliabilityPolicy::Reliable,
                durability: QoSDurabilityPolicy::TransientLocal,
                deadline: QoSDuration::Infinite,
                lifespan: QoSDuration::Infinite,
                liveliness: QoSLivelinessPolicy::Automatic,
                liveliness_lease_duration: QoSDuration::Infinite,
                avoid_ros_namespace_conventions: false,
            },
            move |msg: TFMessage| {
                for transform in msg.transforms {
                    let stamp = transform.header.stamp.sec as f64
                        + (transform.header.stamp.nanosec as f64) * 1e-9;
                    let source = transform.header.frame_id;
                    let target = transform.child_frame_id;
                    let trans = Translation3::new(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    );
                    let rot = UnitQuaternion::new_normalize(Quaternion::new(
                        transform.transform.rotation.w,
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                    ));
                    let isometry = Isometry3::from_parts(trans, rot);
                    let stamped_transform = StampedIsometry {
                        isometry: isometry,
                        stamp: stamp,
                    };
                    buffer_clone
                        .lock()
                        .unwrap()
                        .update(source, target, stamped_transform, TransformType::Static)
                        .unwrap();
                }
            },
        )?;
        Ok(Self {
            buffer: Arc::clone(&buffer),
            _node: node,
            _tf_subscriber: tf_subscriber,
            _static_tf_subscriber: static_tf_subscriber,
        })
    }

    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        stamp: f64,
    ) -> Result<StampedIsometry, TfError> {
        self.buffer
            .lock()
            .unwrap()
            .lookup_transform(source.to_string(), target.to_string(), stamp)
    }
    pub fn lookup_latest_transform(
        &self,
        from: &str,
        to: &str,
    ) -> Result<StampedIsometry, TfError> {
        self.buffer
            .lock()
            .unwrap()
            .lookup_latest_transform(source.to_string(), target.to_string())
    }
    pub fn visualize_buffer(&self) {
        self.buffer.lock().unwrap().visualize();
    }
}
