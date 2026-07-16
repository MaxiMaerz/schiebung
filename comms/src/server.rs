use crate::config::{
    ZenohConfig, GET_TRANSFORMS_SERVICE, LOOKUP_TRANSFORM_SERVICE, TF_STATIC_TOPIC, TF_TOPIC,
};
use crate::conversions::{
    stamped_isometry_to_transform_stamped, tf_message_to_updates, time_to_ns,
};
use crate::error::CommsError;
use crate::msgs::ros::schiebung_msgs::srv::{GetTransforms, LookupTransform};
use crate::msgs::ros::schiebung_msgs::{
    GetTransformsRequest, GetTransformsResponse, LookupTransformRequest, LookupTransformResponse,
};
use crate::msgs::ros::tf2_msgs::TFMessage;
use crate::qos::{tf_qos, tf_static_qos};
use hiroz::msg::ZMessage;
use hiroz::node::ZNode;
use hiroz::pubsub::ZSub;
use hiroz::Builder;
use log::{debug, error, info, warn};
use schiebung::{types::TransformType, BufferTree};
use std::sync::{Arc, RwLock};
use zenoh::sample::Sample;

/// Queue-backed subscriber for `tf2_msgs/TFMessage`, used for both `/tf` and
/// `/tf_static` (they differ only in QoS).
type TfSub = ZSub<TFMessage, Sample, <TFMessage as ZMessage>::Serdes>;

/// Handle to a running server, allowing shutdown and join.
pub struct ServerHandle {
    shutdown_tx: Option<tokio::sync::oneshot::Sender<()>>,
    join_handle: Option<tokio::task::JoinHandle<Result<(), CommsError>>>,
}

impl ServerHandle {
    /// Signal the server to shut down.
    pub fn shutdown(&mut self) {
        if let Some(tx) = self.shutdown_tx.take() {
            let _ = tx.send(());
        }
    }

    /// Wait for the server task to complete.
    /// Returns the result from the server's run loop.
    pub async fn join(mut self) -> Result<(), CommsError> {
        if let Some(handle) = self.join_handle.take() {
            match handle.await {
                Ok(result) => result,
                Err(e) => Err(CommsError::Config(format!("Server task panicked: {}", e))),
            }
        } else {
            Ok(())
        }
    }

    /// Check if the server is still running.
    pub fn is_running(&self) -> bool {
        self.join_handle
            .as_ref()
            .map(|h| !h.is_finished())
            .unwrap_or(false)
    }
}

/// Server regarding Schiebung transforms
///
/// Speaks the standard ROS 2 wire format over zenoh (rmw_zenoh compatible):
/// it subscribes to `/tf` and `/tf_static` (`tf2_msgs/TFMessage`) and serves
/// transform lookups via the `schiebung_msgs/LookupTransform` service. It
/// works standalone (plain zenoh peers, no ROS required) and interoperates
/// with ROS 2 systems running `rmw_zenoh`.
#[derive(Clone)]
pub struct TransformServer {
    buffer: Arc<RwLock<BufferTree>>,
    // Keeps the node (and its liveliness tokens in the ROS graph) alive for the
    // lifetime of the server.
    node: Arc<ZNode>,
    tf_sub: Arc<TfSub>,
    static_sub: Arc<TfSub>,
}

impl TransformServer {
    /// Create a new transform server with default zenoh config (peer mode, multicast discovery).
    pub async fn new() -> Result<Self, CommsError> {
        Self::with_config(ZenohConfig::default()).await
    }

    /// Create a new transform server with an explicit zenoh config.
    ///
    /// Use this to pin the server to known endpoints (e.g. for tests or deployments
    /// where multicast discovery is unavailable).
    ///
    /// Both subscribers are created here rather than in [`Self::run`] so the
    /// transient-local `/tf_static` cache starts filling as soon as the server
    /// exists, instead of only once the run loop is polled.
    pub async fn with_config(config: ZenohConfig) -> Result<Self, CommsError> {
        let buffer = Arc::new(RwLock::new(BufferTree::new()));

        let ctx = config.to_zcontext()?;
        let node = ctx.create_node("schiebung_server").build()?;

        let tf_sub = node
            .create_sub::<TFMessage>(TF_TOPIC)
            .with_qos(tf_qos())
            .build()?;
        let static_sub = node
            .create_sub::<TFMessage>(TF_STATIC_TOPIC)
            .with_qos(tf_static_qos())
            .build()?;

        info!("hiroz context established in {} mode", config.mode);

        Ok(Self {
            buffer,
            node: Arc::new(node),
            tf_sub: Arc::new(tf_sub),
            static_sub: Arc::new(static_sub),
        })
    }

    /// Get a reference to the underlying buffer tree
    pub fn buffer(&self) -> Arc<RwLock<BufferTree>> {
        self.buffer.clone()
    }

    /// Start the transform server in a background task.
    ///
    /// Returns a [`ServerHandle`] that can be used to shut down the server
    /// and wait for it to complete. The buffer can be accessed via [`Self::buffer`]
    /// while the server is running.
    pub async fn start(&self) -> ServerHandle {
        let (shutdown_tx, shutdown_rx) = tokio::sync::oneshot::channel();

        let server = self.clone();
        let join_handle = tokio::spawn(async move {
            info!("Starting schiebung server (background)...");

            // Run the server until shutdown signal or completion
            tokio::select! {
                result = server.run() => result,
                _ = async { shutdown_rx.await.ok() } => {
                    info!("Server shutdown requested");
                    Ok(())
                }
            }
        });

        ServerHandle {
            shutdown_tx: Some(shutdown_tx),
            join_handle: Some(join_handle),
        }
    }

    /// Run the transform server
    ///
    /// The server processes incoming transforms in an unbounded loop. While this means
    /// messages could theoretically accumulate faster than they can be processed, in practice
    /// transform updates are infrequent enough that this is not a concern. If backpressure
    /// becomes necessary in the future, consider adding a bounded channel with monitoring.
    pub async fn run(&self) -> Result<(), CommsError> {
        info!("Starting schiebung server...");
        info!("Subscribed to topic: {}", TF_TOPIC);
        info!("Subscribed to topic: {}", TF_STATIC_TOPIC);

        // hiroz services are take/reply rather than callback-based, so both
        // servers are driven from the same select loop as the subscribers. They
        // must stay alive (and mutable) for the whole run.
        let mut lookup_server = self
            .node
            .create_service::<LookupTransform>(LOOKUP_TRANSFORM_SERVICE)
            .build()?;
        info!("Service registered: {}", LOOKUP_TRANSFORM_SERVICE);

        let mut get_transforms_server = self
            .node
            .create_service::<GetTransforms>(GET_TRANSFORMS_SERVICE)
            .build()?;
        info!("Service registered: {}", GET_TRANSFORMS_SERVICE);
        info!("Server is ready and processing requests");

        let shutdown = tokio::signal::ctrl_c();
        tokio::pin!(shutdown);

        loop {
            tokio::select! {
                msg = self.tf_sub.async_recv() => match msg {
                    Ok(msg) => self.apply_tf_message(&msg, TransformType::Dynamic),
                    Err(e) => {
                        error!("Error receiving {} message: {}", TF_TOPIC, e);
                        break;
                    }
                },
                msg = self.static_sub.async_recv() => match msg {
                    Ok(msg) => self.apply_tf_message(&msg, TransformType::Static),
                    Err(e) => {
                        error!("Error receiving {} message: {}", TF_STATIC_TOPIC, e);
                        break;
                    }
                },
                request = lookup_server.async_take_request() => match request {
                    Ok(request) => {
                        let (message, reply) = request.into_parts();
                        let response = handle_lookup(&self.buffer, message);
                        if let Err(e) = reply.reply(&response).await {
                            error!("Failed to reply to {}: {}", LOOKUP_TRANSFORM_SERVICE, e);
                        }
                    }
                    Err(e) => {
                        error!("Error receiving {} request: {}", LOOKUP_TRANSFORM_SERVICE, e);
                        break;
                    }
                },
                request = get_transforms_server.async_take_request() => match request {
                    Ok(request) => {
                        let (message, reply) = request.into_parts();
                        let response = handle_get_transforms(&self.buffer, message);
                        if let Err(e) = reply.reply(&response).await {
                            error!("Failed to reply to {}: {}", GET_TRANSFORMS_SERVICE, e);
                        }
                    }
                    Err(e) => {
                        error!("Error receiving {} request: {}", GET_TRANSFORMS_SERVICE, e);
                        break;
                    }
                },
                _ = &mut shutdown => {
                    info!("Shutdown signal received");
                    break;
                }
            }
        }

        Ok(())
    }

    fn apply_tf_message(&self, msg: &TFMessage, kind: TransformType) {
        let updates = tf_message_to_updates(msg, kind);
        if updates.is_empty() {
            return;
        }

        // Handle rwlock poisoning by recovering the data
        let mut buf = match self.buffer.write() {
            Ok(guard) => guard,
            Err(poisoned) => {
                warn!("Buffer rwlock was poisoned, recovering...");
                poisoned.into_inner()
            }
        };

        match buf.update(&updates) {
            Ok(()) => {
                for update in &updates {
                    debug!(
                        "Stored transform: {} -> {} ({:?})",
                        update.from, update.to, update.kind
                    );
                }
            }
            Err(e) => error!("Error storing transforms: {}", e),
        }
    }
}

/// Handle a `LookupTransform` service request against the buffer.
///
/// tf2 semantics: a zero time (`sec == 0 && nanosec == 0`) means "latest
/// available transform". Lookup failures are encoded in the response
/// (`success = false`) rather than failing the service call.
pub(crate) fn handle_lookup(
    buffer: &Arc<RwLock<BufferTree>>,
    request: LookupTransformRequest,
) -> LookupTransformResponse {
    let time_ns = time_to_ns(&request.time);
    debug!(
        "Received transform query: {} -> {} at time {}",
        request.target_frame, request.source_frame, time_ns
    );

    // Handle rwlock poisoning by recovering the data
    let buf = match buffer.read() {
        Ok(guard) => guard,
        Err(poisoned) => {
            warn!("Buffer rwlock was poisoned, recovering...");
            poisoned.into_inner()
        }
    };

    let result = if time_ns == 0 {
        buf.lookup_latest_transform(&request.target_frame, &request.source_frame)
    } else {
        buf.lookup_transform(&request.target_frame, &request.source_frame, time_ns)
    };

    match result {
        Ok(stamped_iso) => {
            debug!(
                "Found transform: {} -> {}",
                request.target_frame, request.source_frame
            );
            LookupTransformResponse {
                success: true,
                error_message: String::new(),
                transform: stamped_isometry_to_transform_stamped(
                    &request.target_frame,
                    &request.source_frame,
                    &stamped_iso,
                ),
            }
        }
        Err(e) => {
            let error_message = e.to_string();
            error!("Transform lookup error: {}", error_message);
            LookupTransformResponse {
                success: false,
                error_message,
                transform: Default::default(),
            }
        }
    }
}

/// Handle a `GetTransforms` service request against the buffer.
///
/// Returns the buffer's current content — the latest sample per edge — split
/// into dynamic (`transforms`) and static (`static_transforms`) messages,
/// mirroring the `/tf` / `/tf_static` topic split.
pub(crate) fn handle_get_transforms(
    buffer: &Arc<RwLock<BufferTree>>,
    _request: GetTransformsRequest,
) -> GetTransformsResponse {
    debug!("Received get-transforms query");

    // Handle rwlock poisoning by recovering the data
    let buf = match buffer.read() {
        Ok(guard) => guard,
        Err(poisoned) => {
            warn!("Buffer rwlock was poisoned, recovering...");
            poisoned.into_inner()
        }
    };

    let mut response = GetTransformsResponse::default();
    for update in buf.latest_transforms() {
        let transform = stamped_isometry_to_transform_stamped(
            &update.from,
            &update.to,
            &update.stamped_isometry,
        );
        match update.kind {
            TransformType::Dynamic => response.transforms.transforms.push(transform),
            TransformType::Static => response.static_transforms.transforms.push(transform),
        }
    }
    response
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::conversions::ns_to_time;
    use schiebung::types::{StampedIsometry, TransformUpdate};

    fn populated_buffer() -> Arc<RwLock<BufferTree>> {
        let mut buffer = BufferTree::new();
        buffer
            .update(&[
                TransformUpdate::new(
                    "world",
                    "base",
                    StampedIsometry::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1_000_000_000),
                    TransformType::Dynamic,
                ),
                TransformUpdate::new(
                    "world",
                    "base",
                    StampedIsometry::new([3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 3_000_000_000),
                    TransformType::Dynamic,
                ),
            ])
            .unwrap();
        Arc::new(RwLock::new(buffer))
    }

    #[test]
    fn test_handle_lookup_latest() {
        let buffer = populated_buffer();
        let response = handle_lookup(
            &buffer,
            LookupTransformRequest {
                target_frame: "world".into(),
                source_frame: "base".into(),
                time: ns_to_time(0),
            },
        );
        assert!(response.success, "{}", response.error_message);
        assert_eq!(response.transform.header.frame_id, "world");
        assert_eq!(response.transform.child_frame_id, "base");
        assert_eq!(response.transform.transform.translation.x, 3.0);
    }

    #[test]
    fn test_handle_lookup_interpolated() {
        let buffer = populated_buffer();
        let response = handle_lookup(
            &buffer,
            LookupTransformRequest {
                target_frame: "world".into(),
                source_frame: "base".into(),
                time: ns_to_time(2_000_000_000),
            },
        );
        assert!(response.success, "{}", response.error_message);
        assert!((response.transform.transform.translation.x - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_handle_get_transforms_splits_by_kind() {
        let buffer = populated_buffer();
        buffer
            .write()
            .unwrap()
            .update(&[TransformUpdate::new(
                "base",
                "sensor",
                StampedIsometry::new([0.0, 0.0, 2.0], [0.0, 0.0, 0.0, 1.0], 0),
                TransformType::Static,
            )])
            .unwrap();

        let response = handle_get_transforms(&buffer, GetTransformsRequest::default());

        // Dynamic edge: latest sample only (x=3 at t=3s, not the t=1s one).
        assert_eq!(response.transforms.transforms.len(), 1);
        let dynamic = &response.transforms.transforms[0];
        assert_eq!(dynamic.header.frame_id, "world");
        assert_eq!(dynamic.child_frame_id, "base");
        assert_eq!(dynamic.transform.translation.x, 3.0);

        assert_eq!(response.static_transforms.transforms.len(), 1);
        let stat = &response.static_transforms.transforms[0];
        assert_eq!(stat.header.frame_id, "base");
        assert_eq!(stat.child_frame_id, "sensor");
        assert_eq!(stat.transform.translation.z, 2.0);
    }

    #[test]
    fn test_handle_get_transforms_empty_buffer() {
        let buffer = Arc::new(RwLock::new(BufferTree::new()));
        let response = handle_get_transforms(&buffer, GetTransformsRequest::default());
        assert!(response.transforms.transforms.is_empty());
        assert!(response.static_transforms.transforms.is_empty());
    }

    #[test]
    fn test_handle_lookup_unknown_frame() {
        let buffer = populated_buffer();
        let response = handle_lookup(
            &buffer,
            LookupTransformRequest {
                target_frame: "world".into(),
                source_frame: "nonexistent".into(),
                time: ns_to_time(0),
            },
        );
        assert!(!response.success);
        assert!(!response.error_message.is_empty());
    }
}
