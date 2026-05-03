use comms::{TransformClient, ZenohConfig};
use schiebung::types::{StampedIsometry, TransformType};
use std::time::Duration;

/// Pin both ends to a localhost TCP endpoint and disable multicast scouting,
/// so the test does not depend on UDP multicast (which CI runners and many
/// corporate networks block).
const TEST_ENDPOINT: &str = "tcp/127.0.0.1:17447";

fn server_config() -> ZenohConfig {
    ZenohConfig {
        listen: vec![TEST_ENDPOINT.to_string()],
        multicast_scouting: false,
        ..ZenohConfig::default()
    }
}

fn client_config() -> ZenohConfig {
    ZenohConfig {
        connect: vec![TEST_ENDPOINT.to_string()],
        multicast_scouting: false,
        ..ZenohConfig::default()
    }
}

/// Send a transform and poll for it via query, retrying because zenoh's
/// best-effort `put` can race the publisher's view of the subscriber set.
/// If the put landed before the subscriber was matched it is silently dropped,
/// so we re-publish on each attempt until the query observes the update.
async fn publish_and_wait(
    client: &TransformClient,
    from: &str,
    to: &str,
    transform: StampedIsometry,
    expect: impl Fn(&StampedIsometry) -> bool,
) -> StampedIsometry {
    let max_attempts = 20;
    let delay = Duration::from_millis(100);
    let mut last_err: Option<String> = None;
    for attempt in 1..=max_attempts {
        if let Err(e) = client
            .send_transform(from, to, transform.clone(), TransformType::Static)
            .await
        {
            last_err = Some(format!("send failed: {}", e));
        } else {
            tokio::time::sleep(delay).await;
            match client.request_transform(from, to, 0).await {
                Ok(result) if expect(&result) => return result,
                Ok(result) => {
                    last_err = Some(format!(
                        "unexpected result on attempt {}: {}",
                        attempt, result
                    ));
                }
                Err(e) => last_err = Some(format!("query failed on attempt {}: {}", attempt, e)),
            }
        }
        tokio::time::sleep(delay).await;
    }
    panic!(
        "publish_and_wait({}, {}) failed after {} attempts: {}",
        from,
        to,
        max_attempts,
        last_err.unwrap_or_else(|| "no error captured".to_string())
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_publish_and_query_transform() {
    env_logger::builder()
        .is_test(true)
        .filter_level(log::LevelFilter::Info)
        .try_init()
        .ok();

    // Spawn the server in the background
    let server_handle = tokio::spawn(async {
        match comms::server::TransformServer::with_config(server_config()).await {
            Ok(server) => {
                if let Err(e) = server.run().await {
                    eprintln!("Server error: {}", e);
                }
            }
            Err(e) => eprintln!("Failed to init server: {}", e),
        }
    });

    // Give the server a moment to bind its listener.
    tokio::time::sleep(Duration::from_millis(200)).await;

    let client = TransformClient::with_config(client_config())
        .await
        .expect("Failed to create client");

    // First transform: world -> robot_base, z = 1.0
    let t1 = StampedIsometry::new([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0], 0);
    publish_and_wait(&client, "world", "robot_base", t1, |r| {
        (r.translation()[2] - 1.0).abs() < 1e-6
    })
    .await;

    // Second transform: robot_base -> tool, x = 0.5
    let t2 = StampedIsometry::new([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
    publish_and_wait(&client, "robot_base", "tool", t2, |r| {
        (r.translation()[0] - 0.5).abs() < 1e-6
    })
    .await;

    // Composed query world -> tool: x = 0.5, z = 1.0
    let composed = client
        .request_transform("world", "tool", 0)
        .await
        .expect("Composed query failed after both transforms were confirmed stored");
    let trans = composed.translation();
    assert!((trans[0] - 0.5).abs() < 1e-6);
    assert!((trans[2] - 1.0).abs() < 1e-6);

    // Error handling: a frame that was never published should error.
    let result = client.request_transform("world", "nonexistent", 0).await;
    assert!(result.is_err());

    server_handle.abort();
}
