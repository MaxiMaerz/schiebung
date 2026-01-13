//! Standalone server binary for schiebung-server
//!
//! This binary provides a command-line interface to run the schiebung server
//! with configuration loaded from a file (TOML, YAML, or JSON).

use clap::Parser;
use schiebung_server::Server;
use std::path::PathBuf;

/// Schiebung transform server with Rerun visualization
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to configuration file (TOML, YAML, or JSON)
    #[arg(short, long, value_name = "FILE")]
    config: PathBuf,
}

/// Server configuration loaded from file
#[derive(Debug, serde::Deserialize)]
struct ServerConfig {
    /// Application ID for Rerun
    application_id: String,

    /// Recording ID for this session
    recording_id: String,

    /// Timeline name for Rerun
    timeline: String,

    /// Whether to publish static transforms to Rerun
    /// Set to false if loading URDF via Rerun's built-in loader to avoid duplicates
    #[serde(default = "default_publish_static")]
    publish_static_transforms: bool,
}

fn default_publish_static() -> bool {
    true
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logger
    env_logger::init();

    // Parse command-line arguments
    let args = Args::parse();

    // Load configuration from file
    let config_str = std::fs::read_to_string(&args.config)
        .map_err(|e| format!("Failed to read config file {:?}: {}", args.config, e))?;

    let settings = config::Config::builder()
        .add_source(config::File::from_str(
            &config_str,
            config::FileFormat::Toml,
        ))
        .build()
        .map_err(|e| format!("Failed to parse config: {}", e))?;

    let server_config: ServerConfig = settings
        .try_deserialize()
        .map_err(|e| format!("Failed to deserialize config: {}", e))?;

    log::info!("Starting server with config: {:?}", server_config);

    // Create and run the server
    let server = Server::new(
        &server_config.application_id,
        &server_config.recording_id,
        &server_config.timeline,
        server_config.publish_static_transforms,
    )
    .await?;

    log::info!("Server initialized, starting main loop...");
    server.run().await?;

    Ok(())
}
