use crate::config::{ZenohConfig, TRANSFORM_PUB_TOPIC};
use crate::error::CommsError;
use schiebung::types::TransformType;

/// Client for publishing new transforms to the server
pub struct TransformClient {
    session: zenoh::Session,
}

impl TransformClient {
    /// Create a new transform publisher
    pub async fn new() -> Result<Self, CommsError> {
        let config = ZenohConfig::default();
        let zenoh_config = config.to_zenoh_config()?;

        let session = zenoh::open(zenoh_config)
            .await
            .map_err(|e| CommsError::Zenoh(format!("Failed to open zenoh session: {}", e)))?;

        Ok(TransformClient { session })
    }

    /// Send a new transform to the server
    pub async fn send_transform(
        &self,
        from: &str,
        to: &str,
        stamped_isometry: schiebung::types::StampedIsometry,
        kind: TransformType,
    ) -> Result<(), CommsError> {
        let transform_kind = kind.into();

        let payload = crate::serializers::serialize_new_transform(
            from,
            to,
            &stamped_isometry,
            transform_kind,
        )?;

        self.session
            .put(TRANSFORM_PUB_TOPIC, zenoh::bytes::ZBytes::from(payload))
            .await
            .map_err(|e| CommsError::Zenoh(e.to_string()))?;

        Ok(())
    }

    /// Request a transform from the server
    pub async fn request_transform(
        &self,
        from: &str,
        to: &str,
        time: f64,
    ) -> Result<schiebung::types::StampedIsometry, CommsError> {
        let request_data = crate::serializers::serialize_transform_request(from, to, time)?;

        let replies = self
            .session
            .get(crate::config::TRANSFORM_QUERY_TOPIC)
            .payload(zenoh::bytes::ZBytes::from(request_data))
            .await
            .map_err(|e| CommsError::Zenoh(format!("Failed to send query: {}", e)))?;

        // Wait for first reply
        while let Ok(reply) = replies.recv_async().await {
            match reply.result() {
                Ok(sample) => {
                    let response_data = sample.payload().to_bytes();
                    match crate::serializers::deserialize_transform_response(&response_data)? {
                        Ok(stamped_isometry) => return Ok(stamped_isometry),
                        Err(error_message) => {
                            return Err(CommsError::Zenoh(format!(
                                "Transform request failed: {}",
                                error_message
                            )));
                        }
                    }
                }
                Err(e) => {
                    return Err(CommsError::Zenoh(format!("Query error: {}", e)));
                }
            }
        }

        Err(CommsError::NoResponse)
    }
}
