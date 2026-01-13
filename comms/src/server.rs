use crate::config::{ZenohConfig, TRANSFORM_PUB_TOPIC};
use crate::error::CommsError;
use log::{debug, error, info, warn};
use schiebung::{types::StampedIsometry, BufferTree};
use std::sync::{Arc, RwLock};

/// Server regarding Schiebung transforms
#[derive(Clone)]
pub struct TransformServer {
    buffer: Arc<RwLock<BufferTree>>,
    session: zenoh::Session,
}

impl TransformServer {
    /// Create a new transform server
    pub async fn new() -> Result<Self, CommsError> {
        // Create transform buffer
        let buffer = Arc::new(RwLock::new(BufferTree::new()));

        // Create zenoh session in peer mode (brokerless)
        let config = ZenohConfig::default();
        let zenoh_config = config.to_zenoh_config()?;

        let session = zenoh::open(zenoh_config)
            .await
            .map_err(|e| CommsError::Zenoh(format!("Failed to open zenoh session: {}", e)))?;
        info!("Zenoh session established in {} mode", config.mode);

        Ok(Self { buffer, session })
    }

    /// Get a reference to the underlying buffer tree
    pub fn buffer(&self) -> Arc<RwLock<BufferTree>> {
        self.buffer.clone()
    }

    /// Run the transform server
    ///
    /// The server processes incoming transforms in an unbounded loop. While this means
    /// messages could theoretically accumulate faster than they can be processed, in practice
    /// transform updates are infrequent enough that this is not a concern. If backpressure
    /// becomes necessary in the future, consider adding a bounded channel with monitoring.
    pub async fn run(&self) -> Result<(), CommsError> {
        info!("Starting schiebung server...");

        let subscriber = self
            .session
            .declare_subscriber(TRANSFORM_PUB_TOPIC)
            .await
            .map_err(|e| CommsError::Zenoh(format!("Failed to declare subscriber: {}", e)))?;

        info!("Subscribed to topic: {}", TRANSFORM_PUB_TOPIC);

        let queryable = self
            .session
            .declare_queryable(crate::config::TRANSFORM_QUERY_TOPIC)
            .await
            .map_err(|e| CommsError::Zenoh(format!("Failed to declare queryable: {}", e)))?;

        info!(
            "Queryable registered: {}",
            crate::config::TRANSFORM_QUERY_TOPIC
        );
        info!("Server is ready and processing requests");

        let shutdown = async {
            tokio::signal::ctrl_c()
                .await
                .expect("Failed to listen for Ctrl+C");
            info!("Shutdown signal received");
        };

        let server_sub = self.clone();
        let subscriber_task = tokio::spawn(async move {
            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        match server_sub.handle_new_transform(&sample.payload().to_bytes()) {
                            Ok(_) => debug!("Successfully processed new transform"),
                            Err(e) => error!("Error processing new transform: {}", e),
                        }
                    }
                    Err(e) => {
                        error!("Error receiving sample: {}", e);
                        break;
                    }
                }
            }
        });

        let server_query = self.clone();
        let query_future = async move {
            loop {
                match queryable.recv_async().await {
                    Ok(query) => {
                        let payload_data =
                            query.payload().map(|p| p.to_bytes()).unwrap_or_default();
                        match server_query.handle_transform_query(&payload_data) {
                            Ok(response_bytes) => {
                                if let Err(e) = query
                                    .reply(crate::config::TRANSFORM_QUERY_TOPIC, response_bytes)
                                    .await
                                {
                                    error!("Failed to send query response: {}", e);
                                }
                            }
                            Err(e) => {
                                error!("Error handling transform query: {}", e);
                                let dummy =
                                    StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
                                match crate::serializers::serialize_transform_response(
                                    &dummy,
                                    false,
                                    &e.to_string(),
                                ) {
                                    Ok(error_response) => {
                                        if let Err(e) = query
                                            .reply(
                                                crate::config::TRANSFORM_QUERY_TOPIC,
                                                error_response,
                                            )
                                            .await
                                        {
                                            error!("Failed to send error response: {}", e);
                                        }
                                    }
                                    Err(e) => {
                                        error!("Failed to serialize error response: {}", e);
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        error!("Error receiving query: {}", e);
                        break;
                    }
                }
            }
        };

        tokio::select! {
            _ = subscriber_task => {
                warn!("Subscriber task terminated");
            },
            _ = query_future => {
                warn!("Query handler terminated");
            },
            _ = shutdown => {
                info!("Shutting down gracefully...");
            },
        }

        Ok(())
    }

    fn handle_new_transform(&self, data: &[u8]) -> Result<(), CommsError> {
        let (from, to, stamped_isometry, kind) =
            crate::serializers::deserialize_new_transform(data)?;

        debug!(
            "Received new transform: {} -> {} at time {}",
            from,
            to,
            StampedIsometry::stamp(&stamped_isometry)
        );

        let transform_type = kind.into();

        // Handle rwlock poisoning by recovering the data
        let mut buf = match self.buffer.write() {
            Ok(guard) => guard,
            Err(poisoned) => {
                warn!("Buffer rwlock was poisoned, recovering...");
                poisoned.into_inner()
            }
        };

        buf.update(&from, &to, stamped_isometry, transform_type)?;
        info!(
            "Stored transform: {} -> {} ({:?})",
            from, to, transform_type
        );

        Ok(())
    }

    fn handle_transform_query(&self, data: &[u8]) -> Result<Vec<u8>, CommsError> {
        let (from, to, time) = crate::serializers::deserialize_transform_request(data)?;

        debug!(
            "Received transform query: {} -> {} at time {}",
            from, to, time
        );

        // Handle rwlock poisoning by recovering the data
        let buf = match self.buffer.read() {
            Ok(guard) => guard,
            Err(poisoned) => {
                warn!("Buffer rwlock was poisoned, recovering...");
                poisoned.into_inner()
            }
        };

        match buf.lookup_transform(&from, &to, time) {
            Ok(stamped_iso) => {
                debug!("Found transform: {} -> {}", from, to);
                crate::serializers::serialize_transform_response(&stamped_iso, true, "")
            }
            Err(e) => {
                let error_msg = e.to_string();
                error!("Transform lookup error: {}", error_msg);

                // Create a dummy StampedIsometry for error response
                let dummy = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], time);
                crate::serializers::serialize_transform_response(&dummy, false, &error_msg)
            }
        }
    }
}
