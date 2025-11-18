use std::{thread, time::Duration};

use clap::{Parser, Subcommand};
use log::{error, info};
use nalgebra::{Quaternion, Translation3, UnitQuaternion};
use schiebung::types::{StampedIsometry, StampedTransform, TransformType};
use schiebung_client::{ListenerClient, PublisherClient, VisualizerClient};

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Request a transform between two frames
    Request {
        /// Source frame
        #[arg(long)]
        from: String,
        /// Target frame
        #[arg(long)]
        to: String,
        /// Time to look up (default: 0.0)
        /// If time is 0.0, the latest transform is returned
        #[arg(long, default_value_t = 0.0)]
        time: f64,
    },
    /// Update a transform between two frames
    Update {
        /// Source frame
        #[arg(long)]
        from: String,
        /// Target frame
        #[arg(long)]
        to: String,
        /// Translation
        #[arg(long)]
        tx: f64,
        #[arg(long)]
        ty: f64,
        #[arg(long)]
        tz: f64,
        /// Rotation
        #[arg(long)]
        qw: f64,
        #[arg(long)]
        qx: f64,
        #[arg(long)]
        qy: f64,
        #[arg(long)]
        qz: f64,
    },
    /// Visualize transforms
    Visualize,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::new()
        .filter(None, log::LevelFilter::Error)
        .init();
    let cli = Cli::parse();

    match &cli.command {
        Commands::Request { from, to, time } => {
            let client = ListenerClient::new()?;
            match client.request_transform(from, to, time.clone()) {
                Ok(response) => {
                    info!("Raw response: {:?}", response);
                    let stamped_tf: StampedTransform = response.clone().into();
                    let stamped_iso: StampedIsometry = response.clone().into();
                    info!("Isometry: {:?}", stamped_iso);
                    info!("TF: {:?}", stamped_tf);
                    println!("Transform:\n{} -> {}:", from, to);
                    println!("{}", stamped_tf);
                }
                Err(e) => error!("Lookup error: {:?}", e),
            }
        }
        Commands::Update {
            from,
            to,
            tx,
            ty,
            tz,
            qx,
            qy,
            qz,
            qw,
        } => {
            let pub_client = PublisherClient::new()?;
            thread::sleep(Duration::from_secs(1));
            let translation = Translation3::new(*tx, *ty, *tz);
            let rotation = UnitQuaternion::new_normalize(Quaternion::new(*qx, *qy, *qz, *qw));
            pub_client.send_transform(from, to, translation, rotation, 1.0, TransformType::Static);
            println!(
                "Publishing transform from {} to {} with translation {:?} and rotation {:?}",
                from, to, translation, rotation
            );
            thread::sleep(Duration::from_secs(1));
        }
        Commands::Visualize => {
            info!("Starting visualization...");
            let visualizer_client = VisualizerClient::new()?;
            visualizer_client.send_visualization_request();
        }
    }

    Ok(())
}
