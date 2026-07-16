use comms::{TransformClient, ZenohConfig};
use schiebung::types::{StampedIsometry, TransformType};
use std::time::Duration;

/// Pin both ends to a localhost TCP endpoint and disable multicast scouting,
/// so the test does not depend on UDP multicast (which CI runners and many
/// corporate networks block).
fn server_config(endpoint: &str) -> ZenohConfig {
    ZenohConfig {
        listen: vec![endpoint.to_string()],
        multicast_scouting: false,
        ..ZenohConfig::default()
    }
}

fn client_config(endpoint: &str) -> ZenohConfig {
    ZenohConfig {
        connect: vec![endpoint.to_string()],
        multicast_scouting: false,
        ..ZenohConfig::default()
    }
}

/// Send a transform and poll for it via the lookup service, retrying because
/// pub/sub and service discovery need a moment to settle after startup: a
/// publish that lands before the subscriber is matched is silently dropped,
/// so we re-publish on each attempt until the query observes the update.
async fn publish_and_wait(
    client: &TransformClient,
    from: &str,
    to: &str,
    transform: StampedIsometry,
    kind: TransformType,
    expect: impl Fn(&StampedIsometry) -> bool,
) -> StampedIsometry {
    let max_attempts = 20;
    let delay = Duration::from_millis(100);
    let mut last_err: Option<String> = None;
    for attempt in 1..=max_attempts {
        if let Err(e) = client
            .send_transform(from, to, transform.clone(), kind)
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

fn init_logging() {
    env_logger::builder()
        .is_test(true)
        .filter_level(log::LevelFilter::Info)
        .try_init()
        .ok();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_publish_and_query_transform() {
    init_logging();
    const ENDPOINT: &str = "tcp/127.0.0.1:17447";

    // Spawn the server in the background
    let server_handle = tokio::spawn(async {
        match comms::server::TransformServer::with_config(server_config(ENDPOINT)).await {
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

    let client = TransformClient::with_config(client_config(ENDPOINT))
        .await
        .expect("Failed to create client");

    // First transform (static, /tf_static path): world -> robot_base, z = 1.0
    let t1 = StampedIsometry::new([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0], 0);
    publish_and_wait(
        &client,
        "world",
        "robot_base",
        t1,
        TransformType::Static,
        |r| (r.translation()[2] - 1.0).abs() < 1e-6,
    )
    .await;

    // Second transform (static): robot_base -> tool, x = 0.5
    let t2 = StampedIsometry::new([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
    publish_and_wait(
        &client,
        "robot_base",
        "tool",
        t2,
        TransformType::Static,
        |r| (r.translation()[0] - 0.5).abs() < 1e-6,
    )
    .await;

    // Third transform (dynamic, /tf path): tool -> sensor, y = 0.25
    let t3 = StampedIsometry::new([0.0, 0.25, 0.0], [0.0, 0.0, 0.0, 1.0], 1_000_000_000);
    publish_and_wait(&client, "tool", "sensor", t3, TransformType::Dynamic, |r| {
        (r.translation()[1] - 0.25).abs() < 1e-6
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

    // Full buffer snapshot: all three confirmed edges, split by kind,
    // latest sample per edge.
    let mut all = client
        .request_all_transforms()
        .await
        .expect("get-transforms query failed after all transforms were confirmed stored");
    all.sort_by(|a, b| (&a.from, &a.to).cmp(&(&b.from, &b.to)));
    assert_eq!(all.len(), 3);
    assert_eq!(
        (all[0].from.as_str(), all[0].to.as_str(), all[0].kind),
        ("robot_base", "tool", TransformType::Static)
    );
    assert!((all[0].stamped_isometry.translation()[0] - 0.5).abs() < 1e-6);
    assert_eq!(
        (all[1].from.as_str(), all[1].to.as_str(), all[1].kind),
        ("tool", "sensor", TransformType::Dynamic)
    );
    assert!((all[1].stamped_isometry.translation()[1] - 0.25).abs() < 1e-6);
    assert_eq!(
        (all[2].from.as_str(), all[2].to.as_str(), all[2].kind),
        ("world", "robot_base", TransformType::Static)
    );
    assert!((all[2].stamped_isometry.translation()[2] - 1.0).abs() < 1e-6);

    server_handle.abort();
}

/// Static transforms use transient-local durability: a server that starts
/// *after* the client published its static transforms must still receive them
/// from the publisher's cache.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_static_transform_late_joining_server() {
    init_logging();
    const ENDPOINT: &str = "tcp/127.0.0.1:17448";

    // The client listens so the late-starting server can connect to it.
    let client = TransformClient::with_config(server_config(ENDPOINT))
        .await
        .expect("Failed to create client");

    // Publish the static transform BEFORE the server exists.
    let t = StampedIsometry::new([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
    client
        .send_transform("world", "base", t, TransformType::Static)
        .await
        .expect("Failed to publish static transform");

    // Now start the server; it must pick the transform up from the
    // transient-local cache without any republish.
    let server_handle = tokio::spawn(async {
        match comms::server::TransformServer::with_config(client_config(ENDPOINT)).await {
            Ok(server) => {
                if let Err(e) = server.run().await {
                    eprintln!("Server error: {}", e);
                }
            }
            Err(e) => eprintln!("Failed to init server: {}", e),
        }
    });

    let max_attempts = 50;
    let delay = Duration::from_millis(100);
    let mut last_err = String::new();
    for _ in 1..=max_attempts {
        tokio::time::sleep(delay).await;
        match client.request_transform("world", "base", 0).await {
            Ok(result) if (result.translation()[0] - 2.0).abs() < 1e-6 => {
                server_handle.abort();
                return;
            }
            Ok(result) => last_err = format!("unexpected result: {}", result),
            Err(e) => last_err = format!("query failed: {}", e),
        }
    }
    server_handle.abort();
    panic!(
        "late-joining server never received the static transform after {} attempts: {}",
        max_attempts, last_err
    );
}
