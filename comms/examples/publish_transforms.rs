use comms::TransformClient;
use schiebung::types::{StampedIsometry, TransformType};
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Info)
        .init();

    println!("Creating transform publisher...");
    let client = TransformClient::new().await?;

    println!("Publishing static transform: world -> robot_base");
    let transform = StampedIsometry::new([0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0], 0);
    client
        .send_transform("world", "robot_base", transform, TransformType::Static)
        .await?;

    println!("Publishing dynamic transform: robot_base -> tool");
    let transform = StampedIsometry::new([0.5, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0);
    client
        .send_transform("robot_base", "tool", transform, TransformType::Dynamic)
        .await?;

    // Send a few more transforms with different timestamps
    for i in 1..5 {
        let time_ns = i as i64 * 100_000_000; // 0.1s increments in nanoseconds
        let time_secs = i as f64 * 0.1;
        println!("Publishing dynamic transform at time {}s", time_secs);
        let transform = StampedIsometry::new(
            [0.5 + time_secs * 0.1, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            time_ns,
        );
        client
            .send_transform("robot_base", "tool", transform, TransformType::Dynamic)
            .await?;

        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    println!("Done publishing transforms!");

    println!("Requesting transform: robot_base -> tool");
    let response = client
        .request_transform("world", "tool", 150_000_000)
        .await?; // 0.15s
    println!("Received transform: {:?}", response);

    // Keep alive for a bit to ensure messages are sent
    tokio::time::sleep(Duration::from_secs(1)).await;

    Ok(())
}
