use comms::TransformClient;
use schiebung::types::{StampedIsometry, TransformType};
use std::time::Duration;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_publish_and_query_transform() {
    env_logger::builder()
        .is_test(true)
        .filter_level(log::LevelFilter::Info)
        .try_init()
        .ok();

    // Spawn the server in the background
    let server_handle = tokio::spawn(async {
        match comms::server::TransformServer::new().await {
            Ok(server) => {
                if let Err(e) = server.run().await {
                    eprintln!("Server error: {}", e);
                }
            }
            Err(e) => eprintln!("Failed to init server: {}", e),
        }
    });

    // Give server a moment to start
    tokio::time::sleep(Duration::from_millis(500)).await;

    // Create client
    let client = TransformClient::new()
        .await
        .expect("Failed to create client");

    // Publish static transform (0 nanoseconds)
    let transform = StampedIsometry::new([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0], 0);
    client
        .send_transform("world", "robot_base", transform, TransformType::Static)
        .await
        .expect("Failed to send static transform");

    // Allow some propagation time (best effort for UDP/multicast discovery)
    tokio::time::sleep(Duration::from_millis(200)).await;

    // Query with retry to handle potential initial discovery latency
    let mut attempts = 0;
    loop {
        match client.request_transform("world", "robot_base", 0).await {
            Ok(result) => {
                let trans = result.translation();
                assert!((trans[2] - 1.0).abs() < 1e-6);
                break;
            }
            Err(e) => {
                attempts += 1;
                if attempts > 10 {
                    panic!("Query failed after {} attempts: {}", attempts, e);
                }
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
        }
    }

    // Publish second transform
    let transform = StampedIsometry::new([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
    client
        .send_transform("robot_base", "tool", transform, TransformType::Static)
        .await
        .expect("Failed to send second transform");

    tokio::time::sleep(Duration::from_millis(200)).await;

    // Query composed
    attempts = 0;
    loop {
        match client.request_transform("world", "tool", 0).await {
            Ok(result) => {
                let trans = result.translation();
                assert!((trans[0] - 0.5).abs() < 1e-6);
                assert!((trans[2] - 1.0).abs() < 1e-6);
                break;
            }
            Err(e) => {
                attempts += 1;
                if attempts > 10 {
                    panic!("Composed query failed after {} attempts: {}", attempts, e);
                }
                tokio::time::sleep(Duration::from_millis(200)).await;
            }
        }
    }

    // Test error handling
    let result = client.request_transform("world", "nonexistent", 0).await;
    assert!(result.is_err());

    // Abort server
    server_handle.abort();
}
