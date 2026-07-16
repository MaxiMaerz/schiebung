//! Python bindings for the schiebung-server library
//!
//! This module provides Python access to the Server which combines
//! the transform server with automatic Rerun visualization.
//!
//! The transport-only pieces (TransformClient, ServerHandle, BufferTreeRef,
//! ZenohConfig) are shared with the `schiebung_comms` package.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use schiebung_server::{CommsError, Server as CoreServer};
use std::sync::Arc;
use tokio::runtime::Runtime;

// Re-export Python wrapper types from the sibling binding crates to avoid duplication
pub use schiebung_comms_py::{BufferTreeRef, ServerHandle, TransformClient, ZenohConfig};
pub use schiebung_py::{StampedIsometry, TfError, TransformType};

/// Python wrapper for Server
///
/// This is a centralized transform server with integrated Rerun visualization.
/// All transforms published by clients are automatically logged to Rerun.
#[pyclass]
pub struct Server {
    inner: CoreServer,
    runtime: Arc<Runtime>,
}

#[pymethods]
impl Server {
    /// Create a new Server with Rerun visualization.
    ///
    /// Args:
    ///     application_id: The application ID for Rerun (e.g., "schiebung", "my_robot_app")
    ///     recording_id: The recording ID for this session (e.g., "session_001", "run_2024_01_13")
    ///     timeline: The name of the timeline for logging transforms (e.g., "stable_time")
    ///     publish_static_transforms: Whether to log static transforms to Rerun.
    ///                                Set to False if loading URDF via Rerun's built-in loader.
    #[new]
    pub fn new(
        application_id: String,
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
    ) -> PyResult<Self> {
        let runtime = Arc::new(Runtime::new().map_err(|e| {
            PyValueError::new_err(format!("Failed to create tokio runtime: {}", e))
        })?);

        let inner = runtime
            .block_on(async {
                CoreServer::new(
                    &application_id,
                    &recording_id,
                    &timeline,
                    publish_static_transforms,
                )
                .await
            })
            .map_err(comms_err_to_pyerr)?;

        Ok(Server { inner, runtime })
    }

    /// Get a reference to the underlying buffer tree.
    ///
    /// This allows access to the transform buffer while the server is running.
    /// Use this with `start()` for non-blocking server operation.
    ///
    /// Returns:
    ///     BufferTreeRef: A reference to the buffer that can be used to query transforms.
    #[getter]
    pub fn buffer(&self) -> BufferTreeRef {
        BufferTreeRef::from_core(self.inner.buffer())
    }

    /// Start the transform server in a background thread.
    ///
    /// This method returns immediately, allowing you to access the buffer
    /// while the server is running. Use `buffer` property to get the buffer reference.
    ///
    /// Returns:
    ///     ServerHandle: A handle to control the running server.
    ///
    /// Example:
    ///     >>> server = Server("schiebung", "session_001", "stable_time", True)
    ///     >>> handle = server.start()
    ///     >>> buffer = server.buffer
    ///     >>> # ... do work with buffer ...
    ///     >>> handle.shutdown()
    ///     >>> handle.join()
    pub fn start(&self) -> ServerHandle {
        let handle = self.runtime.block_on(self.inner.start());
        ServerHandle::from_core(handle, self.runtime.clone())
    }

    /// Run the transform server (blocking).
    ///
    /// This method blocks until the server is shut down (via Ctrl+C).
    /// All transforms received are automatically logged to Rerun.
    ///
    /// For non-blocking operation, use `start()` instead.
    pub fn run(&self) -> PyResult<()> {
        self.runtime
            .block_on(async { self.inner.run().await })
            .map_err(comms_err_to_pyerr)
    }
}

fn comms_err_to_pyerr(err: CommsError) -> PyErr {
    PyValueError::new_err(format!("CommsError: {}", err))
}

/// Python bindings for schiebung-server (transform server with Rerun visualization)
#[pymodule(name = "schiebung_server")]
fn schiebung_server_module(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<Server>()?;
    m.add_class::<ServerHandle>()?;
    m.add_class::<BufferTreeRef>()?;
    m.add_class::<TransformClient>()?;
    m.add_class::<ZenohConfig>()?;
    m.add_class::<StampedIsometry>()?;
    m.add_class::<TransformType>()?;
    m.add_class::<TfError>()?;
    Ok(())
}
