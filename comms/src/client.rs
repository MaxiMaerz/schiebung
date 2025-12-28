use crate::config::{ZenohConfig, TRANSFORM_PUB_TOPIC};
use nalgebra::{Translation3, UnitQuaternion};
use schiebung::types::TransformType;
use std::error::Error;
use std::sync::Arc;

/// Client for publishing new transforms to the server
pub struct TransformPublisher {
    session: Arc<zenoh::Session>,
}

impl TransformPublisher {
    /// Create a new transform publisher
    pub async fn new() -> Result<Self, Box<dyn Error>> {
        let config = ZenohConfig::default();
        let zenoh_config = config.to_zenoh_config()?;

        let session = zenoh::open(zenoh_config)
            .await
            .map_err(|e| format!("Failed to open zenoh session: {}", e))?;

        Ok(TransformPublisher {
            session: Arc::new(session),
        })
    }

    /// Send a new transform to the server
    pub async fn send_transform(
        &self,
        from: &str,
        to: &str,
        translation: Translation3<f64>,
        rotation: UnitQuaternion<f64>,
        stamp: f64,
        kind: TransformType,
    ) -> Result<(), Box<dyn Error>> {
        let trans_array = [translation.x, translation.y, translation.z];
        let rot_quat = rotation.into_inner();
        let rot_array = [rot_quat.i, rot_quat.j, rot_quat.k, rot_quat.w];

        let transform_kind = kind.into();

        let payload = crate::serialize_new_transform(
            from,
            to,
            stamp,
            &trans_array,
            &rot_array,
            transform_kind,
        )?;

        self.session
            .put(TRANSFORM_PUB_TOPIC, zenoh::bytes::ZBytes::from(payload))
            .await
            .map_err(|e| e.to_string())?;

        Ok(())
    }

    pub async fn request_transform(
        &self,
        from: &str,
        to: &str,
        time: f64,
    ) -> Result<schiebung::types::StampedIsometry, Box<dyn Error>> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static REQUEST_ID: AtomicU64 = AtomicU64::new(1);

        let id = REQUEST_ID.fetch_add(1, Ordering::SeqCst);
        let request_data = crate::serialize_transform_request(id, from, to, time)?;

        let replies = self
            .session
            .get(crate::config::TRANSFORM_QUERY_TOPIC)
            .payload(zenoh::bytes::ZBytes::from(request_data))
            .await
            .map_err(|e| format!("Failed to send query: {}", e))?;

        // Wait for first reply
        while let Ok(reply) = replies.recv_async().await {
            match reply.result() {
                Ok(sample) => {
                    let response_data = sample.payload().to_bytes();
                    let (_resp_id, resp_time, translation, rotation, success, error_message) =
                        crate::deserialize_transform_response(&response_data)?;

                    if !success {
                        return Err(format!("Transform request failed: {}", error_message).into());
                    }

                    return Ok(schiebung::types::StampedIsometry::new(
                        translation,
                        rotation,
                        resp_time,
                    ));
                }
                Err(e) => {
                    return Err(format!("Query error: {}", e).into());
                }
            }
        }

        Err("No response received for transform request".into())
    }
}
