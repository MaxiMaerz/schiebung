use nalgebra::{Quaternion, Translation3, UnitQuaternion};
use rclrs::*;
use schiebung_client::PublisherClient;
use schiebung_core::types::TransformType;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
use tf2_msgs::msg::TFMessage;
use log::{info, error};

use schiebung_ros2::RosBuffer;


/// This node relays the TF data from the ROS2 master to the schiebung server.
pub struct TfRelay {
    _tf_subscriber: Arc<Subscription<TFMessage>>,
    _static_tf_subscriber: Arc<Subscription<TFMessage>>,
    tf_data: Arc<Mutex<Option<TFMessage>>>,
    static_tf_data: Arc<Mutex<Option<TFMessage>>>,
    republisher: PublisherClient,
    node: Arc<Node>,
}

impl TfRelay {
    fn new(executor: &Executor) -> Result<Self, Box<dyn std::error::Error>> {
        let node = executor.create_node("simple_subscription")?;
        let tf_data = Arc::new(Mutex::new(None));
        let static_tf_data = Arc::new(Mutex::new(None));
        let mut_tf_data = Arc::clone(&tf_data);
        let mut_static_tf_data = Arc::clone(&static_tf_data);
        let _tf_subscriber = node.create_subscription::<TFMessage, _>(
            "/tf",
            QOS_PROFILE_DEFAULT,
            move |msg: TFMessage| {
                *mut_tf_data.lock().unwrap() = Some(msg);
            },
        )?;
        let _static_tf_subscriber = node.create_subscription::<TFMessage, _>(
            "/tf_static",
            QoSProfile{
                history: QoSHistoryPolicy::KeepLast {depth: 1},
                reliability: QoSReliabilityPolicy::Reliable,
                durability: QoSDurabilityPolicy::TransientLocal,
                deadline: QoSDuration::Infinite,
                lifespan: QoSDuration::Infinite,
                liveliness: QoSLivelinessPolicy::Automatic,
                liveliness_lease_duration: QoSDuration::Infinite,
                avoid_ros_namespace_conventions: false,
            },
            move |msg: TFMessage| {
                *mut_static_tf_data.lock().unwrap() = Some(msg);
            },
        )?;
        let republisher = PublisherClient::new()?;
        Ok(Self {
            _tf_subscriber: _tf_subscriber,
            _static_tf_subscriber: _static_tf_subscriber,
            tf_data: tf_data,
            static_tf_data: static_tf_data,
            republisher: republisher,
            node: node,
        })
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let subscription = Arc::new(TfRelay::new(&executor)?);
    env_logger::Builder::new().filter(None, log::LevelFilter::Info).init();

    info!("Waiting for tf data to become available");
    loop {
        executor.spin(SpinOptions::spin_once());
        // Wait for events
        let res = WaitSet::new_for_node(&subscription.node)?.wait(Some(Duration::from_secs(5)));
        match res {
            Ok(_res) => {
                // Process dynamic TF data
                if let Some(tf_msg) = subscription.tf_data.lock().unwrap().take() {
                    for msg in tf_msg.transforms {
                        let trans = Translation3::new(
                            msg.transform.translation.x,
                            msg.transform.translation.y,
                            msg.transform.translation.z,
                        );
                        let rot = UnitQuaternion::new_normalize(Quaternion::new(
                            msg.transform.rotation.w,
                            msg.transform.rotation.x,
                            msg.transform.rotation.y,
                            msg.transform.rotation.z,
                        ));
                        let stamp =
                            msg.header.stamp.sec as f64 + (msg.header.stamp.nanosec as f64) * 1e-9;
                        subscription.republisher.send_transform(
                            &msg.header.frame_id,
                            &msg.child_frame_id,
                            trans,
                            rot,
                            stamp,
                            TransformType::Dynamic,
                        );
                    }
                }

                // Process static TF data
                if let Some(static_tf_msg) = subscription.static_tf_data.lock().unwrap().take() {
                    for msg in static_tf_msg.transforms {
                        let trans = Translation3::new(
                            msg.transform.translation.x,
                            msg.transform.translation.y,
                            msg.transform.translation.z,
                        );
                        let rot = UnitQuaternion::new_normalize(Quaternion::new(
                            msg.transform.rotation.w,
                            msg.transform.rotation.x,
                            msg.transform.rotation.y,
                            msg.transform.rotation.z,
                        ));
                        let stamp =
                            msg.header.stamp.sec as f64 + (msg.header.stamp.nanosec as f64) * 1e-9;
                        subscription.republisher.send_transform(
                            &msg.header.frame_id,
                            &msg.child_frame_id,
                            trans,
                            rot,
                            stamp,
                            TransformType::Static,
                        );
                    }
                }
            }
            Err(_e) => {
                error!("No TF data!");
                continue;
            }
        }
    }
}
