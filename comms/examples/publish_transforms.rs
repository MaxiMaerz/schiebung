use comms::TransformPublisher;
use nalgebra::{Translation3, UnitQuaternion};
use schiebung::types::TransformType;
use std::time::Duration;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Info)
        .init();

    println!("Creating transform publisher...");
    let publisher = TransformPublisher::new().await?;

    println!("Publishing static transform: world -> robot_base");
    publisher
        .send_transform(
            "world",
            "robot_base",
            Translation3::new(0.0, 0.0, 1.0),
            UnitQuaternion::identity(),
            0.0,
            TransformType::Static,
        )
        .await?;

    println!("Publishing dynamic transform: robot_base -> tool");
    publisher
        .send_transform(
            "robot_base",
            "tool",
            Translation3::new(0.5, 0.0, 0.0),
            UnitQuaternion::identity(),
            0.0,
            TransformType::Dynamic,
        )
        .await?;

    // Send a few more transforms with different timestamps
    for i in 1..5 {
        let time = i as f64 * 0.1;
        println!("Publishing dynamic transform at time {}", time);
        publisher
            .send_transform(
                "robot_base",
                "tool",
                Translation3::new(0.5 + time * 0.1, 0.0, 0.0),
                UnitQuaternion::identity(),
                time,
                TransformType::Dynamic,
            )
            .await?;

        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    println!("Done publishing transforms!");

    // Keep alive for a bit to ensure messages are sent
    tokio::time::sleep(Duration::from_secs(1)).await;

    Ok(())
}
