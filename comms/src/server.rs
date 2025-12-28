use crate::config::{ZenohConfig, TRANSFORM_PUB_TOPIC};
use log::{debug, error, info};
use schiebung::{types::StampedIsometry, BufferTree};
use std::sync::{Arc, Mutex};

pub async fn run_server() -> Result<(), String> {
    info!("Starting schiebung server...");

    // Create transform buffer
    let buffer = Arc::new(Mutex::new(BufferTree::new()));

    // Create zenoh session in peer mode (brokerless)
    let config = ZenohConfig::default();
    let zenoh_config = config
        .to_zenoh_config()
        .map_err(|e| format!("Config error: {}", e))?;

    let session = zenoh::open(zenoh_config)
        .await
        .map_err(|e| format!("Failed to open zenoh session: {}", e))?;
    info!("Zenoh session established in {} mode", config.mode);

    // Set up subscriber for new transforms
    let buffer_sub = Arc::clone(&buffer);
    let subscriber = session
        .declare_subscriber(TRANSFORM_PUB_TOPIC)
        .await
        .map_err(|e| format!("Failed to declare subscriber: {}", e))?;

    info!("Subscribed to topic: {}", TRANSFORM_PUB_TOPIC);

    // Set up queryable for transform requests
    let buffer_query = Arc::clone(&buffer);
    let queryable = session
        .declare_queryable(crate::config::TRANSFORM_QUERY_TOPIC)
        .await
        .map_err(|e| format!("Failed to declare queryable: {}", e))?;

    info!(
        "Queryable registered: {}",
        crate::config::TRANSFORM_QUERY_TOPIC
    );
    info!("Server is ready and processing requests");

    // Handle incoming transforms and queries concurrently
    let subscriber_task = tokio::spawn(async move {
        loop {
            match subscriber.recv_async().await {
                Ok(sample) => {
                    match handle_new_transform(&buffer_sub, &sample.payload().to_bytes()) {
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

    // Handle queries (must be on main task - queryable is !Send)
    let query_future = async move {
        loop {
            match queryable.recv_async().await {
                Ok(query) => {
                    let payload_data = query.payload().map(|p| p.to_bytes()).unwrap_or_default();
                    match handle_transform_query(&buffer_query, &payload_data) {
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
                            if let Ok(error_response) = crate::serialize_transform_response(
                                0,
                                0.0,
                                &[0.0, 0.0, 0.0],
                                &[0.0, 0.0, 0.0, 1.0],
                                false,
                                &e.to_string(),
                            ) {
                                let _ = query
                                    .reply(crate::config::TRANSFORM_QUERY_TOPIC, error_response)
                                    .await;
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

    // Wait for either task to complete (they run indefinitely)
    tokio::select! {
        _ = subscriber_task => {},
        _ = query_future => {},
    }

    Ok(())
}

fn handle_new_transform(
    buffer: &Arc<Mutex<BufferTree>>,
    data: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    let (from, to, time, translation, rotation, kind) = crate::deserialize_new_transform(data)?;

    debug!(
        "Received new transform: {} -> {} at time {}",
        from, to, time
    );

    let transform_type = kind.into();

    let mut buf = buffer
        .lock()
        .map_err(|e| format!("Buffer lock poisoned: {}", e))?;
    buf.update(
        &from,
        &to,
        StampedIsometry::new(translation, rotation, time),
        transform_type,
    )?;
    info!(
        "Stored transform: {} -> {} ({:?})",
        from, to, transform_type
    );

    Ok(())
}

fn handle_transform_query(
    buffer: &Arc<Mutex<BufferTree>>,
    data: &[u8],
) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let (id, from, to, time) = crate::deserialize_transform_request(data)?;

    debug!(
        "Received transform query: {} -> {} at time {} (id: {})",
        from, to, time, id
    );

    let buf = buffer
        .lock()
        .map_err(|e| format!("Buffer lock poisoned: {}", e))?;

    match buf.lookup_transform(&from, &to, time) {
        Ok(stamped_iso) => {
            let translation = stamped_iso.translation();
            let rotation = stamped_iso.rotation();

            debug!("Found transform: {} -> {}", from, to);

            crate::serialize_transform_response(
                id,
                stamped_iso.stamp(),
                &translation,
                &rotation,
                true,
                "",
            )
        }
        Err(e) => {
            let error_msg = e.to_string();
            error!("Transform lookup error: {}", error_msg);

            crate::serialize_transform_response(
                id,
                time,
                &[0.0, 0.0, 0.0],
                &[0.0, 0.0, 0.0, 1.0],
                false,
                &error_msg,
            )
        }
    }
}
