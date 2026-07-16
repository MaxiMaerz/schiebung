//! Host-side checks for the ROS 2 interop test (see `comms/docker/`).
//!
//! Stages (pass as first argument, default: both):
//! - `lookup`: expects a ROS `static_transform_publisher` to be broadcasting
//!   `world -> base` with `x = 1.0` on `/tf_static`. Retries the schiebung
//!   lookup service until the transform arrives (ROS -> schiebung direction).
//! - `publish`: publishes dynamic `base -> sensor` transforms on `/tf` for a
//!   while so the ROS side can observe them with `ros2 topic echo /tf`
//!   (schiebung -> ROS direction).
//!
//! Endpoints are configured via `SCHIEBUNG_CONNECT` etc., see
//! `ZenohConfig::from_env()`.

use comms::{TransformClient, ZenohConfig};
use schiebung::types::{StampedIsometry, TransformType};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

async fn check_lookup(client: &TransformClient) -> Result<(), Box<dyn std::error::Error>> {
    let max_attempts = 30;
    for attempt in 1..=max_attempts {
        match client.request_transform("world", "base", 0).await {
            Ok(t) if (t.translation()[0] - 1.0).abs() < 1e-6 => {
                println!(
                    "OK: ROS -> schiebung: received world -> base with x = {}",
                    t.translation()[0]
                );
                return Ok(());
            }
            Ok(t) => eprintln!("attempt {attempt}/{max_attempts}: unexpected transform {t}"),
            Err(e) => eprintln!("attempt {attempt}/{max_attempts}: {e}"),
        }
        tokio::time::sleep(Duration::from_secs(1)).await;
    }
    Err("ROS static transform world -> base never arrived at the schiebung server".into())
}

async fn publish_dynamics(
    client: &TransformClient,
    duration: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Publishing dynamic base -> sensor transforms on /tf for {:?}...",
        duration
    );
    let start = std::time::Instant::now();
    while start.elapsed() < duration {
        let now_ns = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as i64;
        let t = StampedIsometry::new([0.0, 0.5, 0.0], [0.0, 0.0, 0.0, 1.0], now_ns);
        client
            .send_transform("base", "sensor", t, TransformType::Dynamic)
            .await?;
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    println!("OK: done publishing dynamic transforms");
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Warn)
        .init();

    let stage = std::env::args().nth(1).unwrap_or_else(|| "all".to_string());
    let publish_secs: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(20);

    let client = TransformClient::with_config(ZenohConfig::from_env()).await?;
    // Give the session a moment to connect and match endpoints.
    tokio::time::sleep(Duration::from_millis(500)).await;

    match stage.as_str() {
        "lookup" => check_lookup(&client).await?,
        "publish" => publish_dynamics(&client, Duration::from_secs(publish_secs)).await?,
        "all" => {
            check_lookup(&client).await?;
            publish_dynamics(&client, Duration::from_secs(publish_secs)).await?;
        }
        other => return Err(format!("unknown stage: {other} (use lookup|publish|all)").into()),
    }

    Ok(())
}
