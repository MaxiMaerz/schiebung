//! Python bindings for the schiebung-server library
//!
//! This module provides Python access to the Server which combines
//! the transform server with automatic Rerun visualization.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use schiebung::BufferTree as CoreBufferTree;
use schiebung_server::{
    CommsError, Server as CoreServer, ServerHandle as CoreServerHandle,
    TransformClient as CoreTransformClient,
};
use std::sync::{Arc, Mutex, RwLock};
use tokio::runtime::Runtime;

// Re-export Python wrapper types from schiebung-py to avoid duplication
pub use schiebung_py::{StampedIsometry, TfError, TransformType};

/// Python wrapper for ServerHandle
///
/// Handle to a running server, allowing shutdown and status checking.
#[pyclass]
pub struct ServerHandle {
    inner: Mutex<Option<CoreServerHandle>>,
    runtime: Arc<Runtime>,
}

#[pymethods]
impl ServerHandle {
    /// Signal the server to shut down.
    pub fn shutdown(&self) {
        if let Ok(mut guard) = self.inner.lock() {
            if let Some(ref mut handle) = *guard {
                handle.shutdown();
            }
        }
    }

    /// Wait for the server task to complete.
    pub fn join(&self) -> PyResult<()> {
        let handle = {
            let mut guard = self
                .inner
                .lock()
                .map_err(|e| PyValueError::new_err(format!("Lock poisoned: {}", e)))?;
            guard.take()
        };

        if let Some(h) = handle {
            self.runtime.block_on(h.join()).map_err(comms_err_to_pyerr)
        } else {
            Ok(())
        }
    }

    /// Check if the server is still running.
    pub fn is_running(&self) -> bool {
        if let Ok(guard) = self.inner.lock() {
            guard.as_ref().map(|h| h.is_running()).unwrap_or(false)
        } else {
            false
        }
    }
}

/// Python wrapper for the BufferTree (read-only access while server runs)
#[pyclass]
pub struct BufferTreeRef {
    inner: Arc<RwLock<CoreBufferTree>>,
}

#[pymethods]
impl BufferTreeRef {
    /// Look up a transform between two frames at a given time.
    ///
    /// Args:
    ///     from_frame: The source frame name
    ///     to_frame: The target frame name
    ///     time: The timestamp in nanoseconds since Unix epoch
    ///
    /// Returns:
    ///     The transform at the requested time
    pub fn lookup_transform(
        &self,
        from_frame: &str,
        to_frame: &str,
        time: i64,
    ) -> PyResult<StampedIsometry> {
        let guard = self
            .inner
            .read()
            .map_err(|e| PyValueError::new_err(format!("Lock poisoned: {}", e)))?;

        let result = guard
            .lookup_transform(from_frame, to_frame, time)
            .map_err(|e| PyValueError::new_err(format!("Transform lookup error: {}", e)))?;

        Ok(StampedIsometry::from(result))
    }

    /// Look up the latest transform between two frames without time checks.
    ///
    /// This can be used for static transforms or if you don't care about timing.
    /// NOTE: This might give you outdated transforms!
    ///
    /// Args:
    ///     from_frame: The source frame name
    ///     to_frame: The target frame name
    ///
    /// Returns:
    ///     The latest transform available
    pub fn lookup_latest_transform(
        &self,
        from_frame: &str,
        to_frame: &str,
    ) -> PyResult<StampedIsometry> {
        let guard = self
            .inner
            .read()
            .map_err(|e| PyValueError::new_err(format!("Lock poisoned: {}", e)))?;

        let result = guard
            .lookup_latest_transform(from_frame, to_frame)
            .map_err(|e| PyValueError::new_err(format!("Transform lookup error: {}", e)))?;

        Ok(StampedIsometry::from(result))
    }

    /// Visualize the buffer tree as a DOT graph string.
    ///
    /// Returns:
    ///     A DOT format string representing the transform graph.
    pub fn visualize(&self) -> PyResult<String> {
        let guard = self
            .inner
            .read()
            .map_err(|e| PyValueError::new_err(format!("Lock poisoned: {}", e)))?;

        Ok(guard.visualize())
    }

    /// Save the buffer tree visualization as PDF and DOT files.
    ///
    /// Requires graphviz to be installed on the system.
    pub fn save_visualization(&self) -> PyResult<()> {
        let guard = self
            .inner
            .read()
            .map_err(|e| PyValueError::new_err(format!("Lock poisoned: {}", e)))?;

        guard
            .save_visualization()
            .map_err(|e| PyValueError::new_err(format!("Failed to save visualization: {}", e)))
    }
}

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
        BufferTreeRef {
            inner: self.inner.buffer(),
        }
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
        ServerHandle {
            inner: Mutex::new(Some(handle)),
            runtime: self.runtime.clone(),
        }
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

/// Python wrapper for TransformClient
///
/// Use this to publish transforms to the server and query transforms.
#[pyclass]
pub struct TransformClient {
    inner: CoreTransformClient,
    runtime: Runtime,
}

#[pymethods]
impl TransformClient {
    /// Create a new TransformClient.
    #[new]
    pub fn new() -> PyResult<Self> {
        let runtime = Runtime::new()
            .map_err(|e| PyValueError::new_err(format!("Failed to create tokio runtime: {}", e)))?;

        let inner = runtime
            .block_on(async { CoreTransformClient::new().await })
            .map_err(comms_err_to_pyerr)?;

        Ok(TransformClient { inner, runtime })
    }

    /// Send a transform to the server.
    ///
    /// Args:
    ///     from_frame: The source frame name
    ///     to_frame: The target frame name
    ///     stamped_isometry: The transform data
    ///     kind: The transform type (static or dynamic)
    pub fn send_transform(
        &self,
        from_frame: String,
        to_frame: String,
        stamped_isometry: StampedIsometry,
        kind: TransformType,
    ) -> PyResult<()> {
        let core_isometry = stamped_isometry.inner.clone();

        self.runtime
            .block_on(async {
                self.inner
                    .send_transform(&from_frame, &to_frame, core_isometry, kind.into())
                    .await
            })
            .map_err(comms_err_to_pyerr)
    }

    /// Request a transform from the server.
    ///
    /// Args:
    ///     from_frame: The source frame name
    ///     to_frame: The target frame name
    ///     time: The timestamp in nanoseconds since Unix epoch
    ///
    /// Returns:
    ///     The transform at the requested time
    pub fn request_transform(
        &self,
        from_frame: String,
        to_frame: String,
        time: i64,
    ) -> PyResult<StampedIsometry> {
        let result = self
            .runtime
            .block_on(async {
                self.inner
                    .request_transform(&from_frame, &to_frame, time)
                    .await
            })
            .map_err(comms_err_to_pyerr)?;

        Ok(StampedIsometry::from(result))
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
    m.add_class::<StampedIsometry>()?;
    m.add_class::<TransformType>()?;
    m.add_class::<TfError>()?;
    Ok(())
}
