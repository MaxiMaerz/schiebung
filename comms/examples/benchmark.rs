use comms::TransformClient;
use schiebung::types::{StampedIsometry, TransformType};
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};
use tokio::time::sleep;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Info)
        .init();

    // Start the server
    println!("Starting server...");
    let mut server_process = Command::new("cargo")
        .args(["run", "--release", "--bin", "server"])
        .stdout(Stdio::null()) // clean output
        .stderr(Stdio::null())
        .spawn()?;

    // Give server some time to start
    sleep(Duration::from_secs(2)).await;

    // Run benchmark in a block to ensure we can kill server afterwards even if panic (though panic kills main thread usually)
    // Actually, simple main is fine.
    let res = run_benchmark().await;

    println!("Stopping server...");
    let _ = server_process.kill();
    let _ = server_process.wait();

    res
}

async fn run_benchmark() -> Result<(), Box<dyn std::error::Error>> {
    let client = TransformClient::new().await?;
    let iterations = 1000;

    println!("Warmup...");
    for _ in 0..100 {
        let transform = StampedIsometry::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0);
        client
            .send_transform(
                "world",
                "benchmark_frame",
                transform,
                TransformType::Dynamic,
            )
            .await?;
    }

    println!("Benchmarking send_transform ({} iterations)...", iterations);
    let mut latencies = Vec::with_capacity(iterations);

    for i in 0..iterations {
        let start = Instant::now();
        let transform = StampedIsometry::new([i as f64 * 0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0);
        client
            .send_transform(
                "world",
                "benchmark_frame",
                transform,
                TransformType::Dynamic,
            )
            .await?;
        latencies.push(start.elapsed());
    }
    print_stats("send_transform", &latencies);

    println!(
        "Benchmarking request_transform ({} iterations)...",
        iterations
    );
    latencies.clear();

    // Ensure we have something to query
    let transform = StampedIsometry::new([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 0.0);
    client
        .send_transform("world", "query_frame", transform, TransformType::Static)
        .await?;
    sleep(Duration::from_secs(1)).await;

    // Verify it exists before benchmarking
    if let Err(e) = client.request_transform("world", "query_frame", 0.0).await {
        println!("Initial request failed: {}, waiting more...", e);
        sleep(Duration::from_secs(2)).await;
        // Try one last time
        client
            .request_transform("world", "query_frame", 0.0)
            .await?;
    }

    for _ in 0..iterations {
        let start = Instant::now();
        let _ = client
            .request_transform("world", "query_frame", 0.0)
            .await?;
        latencies.push(start.elapsed());
    }
    print_stats("request_transform", &latencies);

    Ok(())
}

fn print_stats(name: &str, latencies: &[Duration]) {
    let sum: u128 = latencies.iter().map(|d| d.as_micros()).sum();
    let avg = sum as f64 / latencies.len() as f64;
    let min = latencies.iter().map(|d| d.as_micros()).min().unwrap_or(0);
    let max = latencies.iter().map(|d| d.as_micros()).max().unwrap_or(0);

    println!("{} results (microseconds):", name);
    println!("  Average: {:.2} us", avg);
    println!("  Min: {} us", min);
    println!("  Max: {} us", max);
}
