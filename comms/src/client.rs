use crate::config::{
    ZenohConfig, GET_TRANSFORMS_SERVICE, LOOKUP_TRANSFORM_SERVICE, TF_STATIC_TOPIC, TF_TOPIC,
};
use crate::conversions::{
    ns_to_time, stamped_isometry_to_transform_stamped, tf_message_to_updates,
    transform_stamped_to_stamped_isometry,
};
use crate::error::CommsError;
use crate::msgs::ros::geometry_msgs::TransformStamped;
use crate::msgs::ros::schiebung_msgs::srv::{GetTransforms, LookupTransform};
use crate::msgs::ros::schiebung_msgs::{GetTransformsRequest, LookupTransformRequest};
use crate::msgs::ros::tf2_msgs::TFMessage;
use crate::qos::{tf_qos, tf_static_qos};
use hiroz::msg::ZMessage;
use hiroz::node::ZNode;
use hiroz::pubsub::ZPub;
use hiroz::service::ZClient;
use hiroz::Builder;
use schiebung::types::{TransformType, TransformUpdate};
use std::collections::HashMap;
use std::sync::Mutex;
use std::time::Duration;

/// Publisher for `tf2_msgs/TFMessage`, used for both `/tf` and `/tf_static`
/// (they differ only in QoS).
type TfPub = ZPub<TFMessage, <TFMessage as ZMessage>::Serdes>;

/// How long to wait for a service reply before giving up.
const LOOKUP_TIMEOUT: Duration = Duration::from_secs(10);

/// Client for publishing transforms and requesting lookups from the server
///
/// Publishes dynamic transforms on `/tf` and static transforms on
/// `/tf_static` (transient local, mirroring `tf2_ros::StaticTransformBroadcaster`),
/// and calls the `schiebung_msgs/LookupTransform` service for lookups.
pub struct TransformClient {
    tf_pub: TfPub,
    static_pub: TfPub,
    /// Accumulated static transforms, republished as one message on every
    /// change so a keep-last-1 transient-local cache still delivers the full
    /// set to late joiners (tf2_ros StaticTransformBroadcaster semantics).
    static_cache: Mutex<HashMap<(String, String), TransformStamped>>,
    lookup_client: ZClient<LookupTransform>,
    get_transforms_client: ZClient<GetTransforms>,
    // Keeps the node (and its liveliness tokens in the ROS graph) alive.
    _node: ZNode,
}

impl TransformClient {
    /// Create a new transform client with default zenoh config (peer mode, multicast discovery).
    pub async fn new() -> Result<Self, CommsError> {
        Self::with_config(ZenohConfig::default()).await
    }

    /// Create a new transform client with an explicit zenoh config.
    ///
    /// Use this to pin the client to known endpoints (e.g. for tests or deployments
    /// where multicast discovery is unavailable).
    pub async fn with_config(config: ZenohConfig) -> Result<Self, CommsError> {
        let ctx = config.to_zcontext()?;

        // Unique per process *and* per client instance, so several clients in
        // one process (or several processes on one host) get distinct ROS
        // node names.
        static CLIENT_COUNTER: std::sync::atomic::AtomicUsize =
            std::sync::atomic::AtomicUsize::new(0);
        let name = format!(
            "schiebung_client_{}_{}",
            std::process::id(),
            CLIENT_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        );
        let node = ctx.create_node(name.as_str()).build()?;

        let tf_pub = node
            .create_pub::<TFMessage>(TF_TOPIC)
            .with_qos(tf_qos())
            .build()?;
        let static_pub = node
            .create_pub::<TFMessage>(TF_STATIC_TOPIC)
            .with_qos(tf_static_qos())
            .build()?;

        let lookup_client = node
            .create_client::<LookupTransform>(LOOKUP_TRANSFORM_SERVICE)
            .build()?;
        let get_transforms_client = node
            .create_client::<GetTransforms>(GET_TRANSFORMS_SERVICE)
            .build()?;

        Ok(TransformClient {
            tf_pub,
            static_pub,
            static_cache: Mutex::new(HashMap::new()),
            lookup_client,
            get_transforms_client,
            _node: node,
        })
    }

    /// Send a new transform to the server
    ///
    /// Dynamic transforms are published on `/tf`; static transforms on
    /// `/tf_static` with transient-local durability so late-joining
    /// subscribers (including a server started afterwards) still receive them.
    pub async fn send_transform(
        &self,
        from: &str,
        to: &str,
        stamped_isometry: schiebung::types::StampedIsometry,
        kind: TransformType,
    ) -> Result<(), CommsError> {
        let transform = stamped_isometry_to_transform_stamped(from, to, &stamped_isometry);
        match kind {
            TransformType::Dynamic => {
                let msg = TFMessage {
                    transforms: vec![transform],
                };
                self.tf_pub.async_publish(&msg).await?;
            }
            TransformType::Static => {
                let msg = {
                    let mut cache = self.static_cache.lock()?;
                    cache.insert((from.to_string(), to.to_string()), transform);
                    TFMessage {
                        transforms: cache.values().cloned().collect(),
                    }
                };
                self.static_pub.async_publish(&msg).await?;
            }
        }
        Ok(())
    }

    /// Request a transform from the server
    ///
    /// Time is in nanoseconds since Unix epoch; `0` means "latest available"
    /// (tf2 semantics).
    pub async fn request_transform(
        &self,
        from: &str,
        to: &str,
        time: i64,
    ) -> Result<schiebung::types::StampedIsometry, CommsError> {
        let request = LookupTransformRequest {
            target_frame: from.to_string(),
            source_frame: to.to_string(),
            time: ns_to_time(time),
        };

        let response = tokio::time::timeout(LOOKUP_TIMEOUT, self.lookup_client.call(&request))
            .await
            .map_err(|_| CommsError::NoResponse)??;

        if response.success {
            Ok(transform_stamped_to_stamped_isometry(&response.transform))
        } else {
            Err(CommsError::LookupFailed(response.error_message))
        }
    }

    /// Fetch the server's current buffer content (latest sample per edge).
    ///
    /// Returns one [`TransformUpdate`] per edge in the server's transform
    /// graph, with `kind` distinguishing dynamic (`/tf`-style) from static
    /// (`/tf_static`-style) edges. The order is unspecified.
    pub async fn request_all_transforms(&self) -> Result<Vec<TransformUpdate>, CommsError> {
        let response = tokio::time::timeout(
            LOOKUP_TIMEOUT,
            self.get_transforms_client
                .call(&GetTransformsRequest::default()),
        )
        .await
        .map_err(|_| CommsError::NoResponse)??;

        let mut updates = tf_message_to_updates(&response.transforms, TransformType::Dynamic);
        updates.extend(tf_message_to_updates(
            &response.static_transforms,
            TransformType::Static,
        ));
        Ok(updates)
    }
}
