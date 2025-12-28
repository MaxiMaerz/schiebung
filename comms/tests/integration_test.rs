use comms::TransformClient;
use log::{debug, error, info};
use nalgebra::{Translation3, UnitQuaternion};
use schiebung::{types::TransformType, BufferTree};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Helper function to retry a query with exponential backoff
async fn retry_query<F, Fut, T>(
    mut f: F,
    max_attempts: u32,
    initial_delay_ms: u64,
) -> Result<T, String>
where
    F: FnMut() -> Fut,
    Fut: std::future::Future<Output = Result<T, comms::error::CommsError>>,
{
    let mut delay = initial_delay_ms;
    for attempt in 1..=max_attempts {
        match f().await {
            Ok(result) => return Ok(result),
            Err(e) => {
                if attempt == max_attempts {
                    return Err(format!(
                        "Query failed after {} attempts: {}",
                        max_attempts, e
                    ));
                }
                debug!(
                    "Attempt {} failed: {}, retrying in {}ms...",
                    attempt, e, delay
                );
                tokio::time::sleep(Duration::from_millis(delay)).await;
                delay = (delay * 2).min(2000); // Cap at 2 seconds
            }
        }
    }
    unreachable!()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_publish_and_query_transform() {
    env_logger::builder()
        .is_test(true)
        .filter_level(log::LevelFilter::Info)
        .try_init()
        .ok();

    info!("Starting integrated test server...");

    // Create transform buffer
    let buffer = Arc::new(Mutex::new(BufferTree::new()));

    // Create zenoh session
    let mut config = zenoh::Config::default();
    config
        .insert_json5("mode", "\"peer\"")
        .expect("Failed to configure zenoh");
    let session = zenoh::open(config)
        .await
        .expect("Failed to open zenoh session");
    info!("Zenoh session established");

    // Set up subscriber for new transforms
    let buffer_sub = Arc::clone(&buffer);
    let subscriber = session
        .declare_subscriber(comms::config::TRANSFORM_PUB_TOPIC)
        .await
        .expect("Failed to declare subscriber");

    // Set up queryable for transform requests
    let buffer_query = Arc::clone(&buffer);
    let queryable = session
        .declare_queryable(comms::config::TRANSFORM_QUERY_TOPIC)
        .await
        .expect("Failed to declare queryable");

    info!("Server components ready");

    // Spawn subscriber handler
    let subscriber_task = tokio::spawn(async move {
        while let Ok(sample) = subscriber.recv_async().await {
            let data = sample.payload().to_bytes();
            match comms::deserialize_new_transform(&data) {
                Ok((from, to, time, translation, rotation, kind)) => {
                    let transform_type: TransformType = kind.into();
                    let mut buf = buffer_sub.lock().unwrap();
                    if let Err(e) = buf.update(
                        &from,
                        &to,
                        schiebung::types::StampedIsometry::new(translation, rotation, time),
                        transform_type,
                    ) {
                        error!("Failed to update buffer: {}", e);
                    } else {
                        info!(
                            "Stored transform: {} -> {} ({:?})",
                            from, to, transform_type
                        );
                    }
                }
                Err(e) => error!("Failed to deserialize transform: {}", e),
            }
        }
    });

    // Handle queryable in main thread (it's !Send)
    let query_handler = async {
        while let Ok(query) = queryable.recv_async().await {
            let payload_data = query.payload().map(|p| p.to_bytes()).unwrap_or_default();
            match comms::deserialize_transform_request(&payload_data) {
                Ok((id, from, to, time)) => {
                    debug!("Query: {} -> {} at time {}", from, to, time);
                    let buf = buffer_query.lock().unwrap();
                    match buf.lookup_transform(&from, &to, time) {
                        Ok(stamped_iso) => {
                            let translation = stamped_iso.translation();
                            let rotation = stamped_iso.rotation();
                            if let Ok(response) = comms::serialize_transform_response(
                                id,
                                stamped_iso.stamp(),
                                &translation,
                                &rotation,
                                true,
                                "",
                            ) {
                                let _ = query
                                    .reply(comms::config::TRANSFORM_QUERY_TOPIC, response)
                                    .await;
                            }
                        }
                        Err(e) => {
                            if let Ok(error_response) = comms::serialize_transform_response(
                                id,
                                time,
                                &[0.0, 0.0, 0.0],
                                &[0.0, 0.0, 0.0, 1.0],
                                false,
                                &e.to_string(),
                            ) {
                                let _ = query
                                    .reply(comms::config::TRANSFORM_QUERY_TOPIC, error_response)
                                    .await;
                            }
                        }
                    }
                }
                Err(e) => error!("Failed to deserialize request: {}", e),
            }
        }
    };

    // Run client tests alongside query handler
    let test_future = async {
        // Small initial delay to ensure server is ready
        tokio::time::sleep(Duration::from_millis(100)).await;

        let client = TransformClient::new()
            .await
            .expect("Failed to create client");

        // Publish static transform
        client
            .send_transform(
                "world",
                "robot_base",
                Translation3::new(0.0, 0.0, 1.0),
                UnitQuaternion::identity(),
                0.0,
                TransformType::Static,
            )
            .await
            .expect("Failed to send static transform");
        println!("✓ Published static transform");

        // Query it back with retry logic
        let result = retry_query(
            || client.request_transform("world", "robot_base", 0.0),
            5,
            50,
        )
        .await
        .expect("Query failed");
        let trans = result.translation();
        println!("✓ Queried: [{}, {}, {}]", trans[0], trans[1], trans[2]);
        assert!((trans[2] - 1.0).abs() < 1e-6);

        // Publish second transform (as static to avoid interpolation issues)
        client
            .send_transform(
                "robot_base",
                "tool",
                Translation3::new(0.5, 0.0, 0.0),
                UnitQuaternion::identity(),
                0.0,
                TransformType::Static,
            )
            .await
            .expect("Failed to send static transform");
        println!("✓ Published second static transform");

        // Query composed with retry logic
        let result = retry_query(|| client.request_transform("world", "tool", 0.0), 5, 50)
            .await
            .expect("Composed query failed");
        let trans = result.translation();
        println!("✓ Composed: [{}, {}, {}]", trans[0], trans[1], trans[2]);
        assert!((trans[0] - 0.5).abs() < 1e-6);
        assert!((trans[2] - 1.0).abs() < 1e-6);

        // Test error handling - query non-existent transform
        let result = client.request_transform("world", "nonexistent", 0.0).await;
        assert!(result.is_err());
        println!("✓ Error handling works");

        println!("✓ All tests passed!");
    };

    // Run test and query handler concurrently
    tokio::select! {
        _ = query_handler => {},
        _ = test_future => {
            // Test completed, abort the background tasks
            subscriber_task.abort();
        },
    }
}
