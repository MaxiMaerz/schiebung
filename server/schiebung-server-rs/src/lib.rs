//! Centralized Transform Server with Rerun Visualization
//!
//! This library combines the functionality of `comms::TransformServer` with
//! `schiebung_rerun::RerunObserver` to provide a centralized transform server
//! that automatically logs all transforms to a Rerun recording stream.
//!
//! # Example (blocking)
//! ```no_run
//! use schiebung_server::Server;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let server = Server::new("schiebung", "session_001", "stable_time", true).await?;
//!     server.run().await?;
//!     Ok(())
//! }
//! ```
//!
//! # Example (non-blocking)
//! ```no_run
//! use schiebung_server::Server;
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let server = Server::new("schiebung", "session_001", "stable_time", true).await?;
//!     let mut handle = server.start().await;
//!
//!     // Access the buffer while server is running
//!     let buffer = server.buffer();
//!     // ... do work with buffer ...
//!
//!     // When done, signal shutdown and wait
//!     handle.shutdown();
//!     handle.join().await;
//!     Ok(())
//! }
//! ```

use comms::server::TransformServer;
use log::info;
use rerun::RecordingStreamBuilder;
use schiebung_rerun::RerunObserver;
use std::sync::{Arc, RwLock};
use tokio::sync::oneshot;

/// Handle to a running server, allowing shutdown and join.
pub struct ServerHandle {
    shutdown_tx: Option<oneshot::Sender<()>>,
    join_handle: Option<tokio::task::JoinHandle<Result<(), CommsError>>>,
}

impl ServerHandle {
    /// Signal the server to shut down.
    pub fn shutdown(&mut self) {
        if let Some(tx) = self.shutdown_tx.take() {
            let _ = tx.send(());
        }
    }

    /// Wait for the server task to complete.
    /// Returns the result from the server's run loop.
    pub async fn join(mut self) -> Result<(), CommsError> {
        if let Some(handle) = self.join_handle.take() {
            match handle.await {
                Ok(result) => result,
                Err(e) => Err(CommsError::Config(format!("Server task panicked: {}", e))),
            }
        } else {
            Ok(())
        }
    }

    /// Check if the server is still running.
    pub fn is_running(&self) -> bool {
        self.join_handle
            .as_ref()
            .map(|h| !h.is_finished())
            .unwrap_or(false)
    }
}

/// Centralized transform server with integrated Rerun visualization.
///
/// This server combines the communication capabilities of `TransformServer`
/// with automatic Rerun logging via the observer pattern. All transforms
/// received or stored are automatically logged to a Rerun recording stream.
#[derive(Clone)]
pub struct Server {
    inner: TransformServer,
}

impl Server {
    /// Create a new Server with Rerun visualization.
    ///
    /// # Arguments
    /// * `application_id` - The application ID for Rerun (e.g., "schiebung", "my_robot_app")
    /// * `recording_id` - The recording ID for this session (e.g., "session_001", "run_2024_01_13")
    /// * `timeline` - The name of the timeline for logging transforms (e.g., "stable_time")
    /// * `publish_static_transforms` - Whether to log static transforms to Rerun.
    ///   Set to `false` if loading URDF via Rerun's built-in loader to avoid duplicates.
    pub async fn new(
        application_id: &str,
        recording_id: &str,
        timeline: &str,
        publish_static_transforms: bool,
    ) -> Result<Self, CommsError> {
        // Create base server
        let inner = TransformServer::new().await?;

        let builder = RecordingStreamBuilder::new(application_id).recording_id(recording_id);
        let rec = if let Ok(addr_str) = std::env::var("RERUN_CONNECT_ADDR") {
            builder
                .connect_grpc_opts(addr_str)
                .map_err(|e| CommsError::Config(format!("Failed to connect to Rerun: {}", e)))?
        } else {
            builder
                .spawn()
                .map_err(|e| CommsError::Config(format!("Failed to create Rerun stream: {}", e)))?
        };
        let observer =
            RerunObserver::new(rec.clone(), publish_static_transforms, timeline.to_string());
        inner
            .buffer()
            .write()
            .map_err(|e| CommsError::MutexPoisoned(e.to_string()))?
            .register_observer(Box::new(observer));

        Ok(Self { inner })
    }

    /// Get a reference to the underlying buffer tree.
    ///
    /// This allows access to the transform buffer while the server is running
    /// in a background thread (via `start()`).
    pub fn buffer(&self) -> Arc<RwLock<BufferTree>> {
        self.inner.buffer()
    }

    /// Start the transform server in a background task.
    ///
    /// Returns a `ServerHandle` that can be used to shut down the server
    /// and wait for it to complete. The buffer can be accessed via `buffer()`
    /// while the server is running.
    pub async fn start(&self) -> ServerHandle {
        let (shutdown_tx, shutdown_rx) = oneshot::channel();

        let server = self.inner.clone();
        let join_handle = tokio::spawn(async move {
            info!("Starting schiebung server with Rerun visualization (background)...");

            // Run the server until shutdown signal or completion
            tokio::select! {
                result = server.run() => result,
                _ = async { shutdown_rx.await.ok() } => {
                    info!("Server shutdown requested");
                    Ok(())
                }
            }
        });

        ServerHandle {
            shutdown_tx: Some(shutdown_tx),
            join_handle: Some(join_handle),
        }
    }

    /// Run the transform server (blocking).
    ///
    /// The server processes incoming transforms in an unbounded loop.
    /// All transforms are automatically logged to Rerun via the registered observer.
    ///
    /// For non-blocking operation, use `start()` instead.
    pub async fn run(&self) -> Result<(), CommsError> {
        info!("Starting schiebung server with Rerun visualization...");
        self.inner.run().await
    }
}

// Re-export common types for convenience
pub use comms::error::CommsError;
pub use comms::TransformClient;
pub use schiebung::BufferTree;
