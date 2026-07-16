//! Python bindings for the schiebung comms library
//!
//! This module provides Python access to the transport-only transform
//! client/server speaking the standard ROS 2 wire format over zenoh
//! (rmw_zenoh compatible). For the combined server with Rerun visualization,
//! see the `schiebung_server` package.

use comms::server::{ServerHandle as CoreServerHandle, TransformServer as CoreTransformServer};
use comms::{CommsError, TransformClient as CoreTransformClient, ZenohConfig as CoreZenohConfig};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use schiebung::types::TransformUpdate as CoreTransformUpdate;
use schiebung::BufferTree as CoreBufferTree;
use std::sync::{Arc, Mutex, RwLock};
use tokio::runtime::Runtime;

// Re-export Python wrapper types from schiebung-py to avoid duplication
pub use schiebung_py::{StampedIsometry, TfError, TransformType};

fn comms_err_to_pyerr(err: CommsError) -> PyErr {
    PyValueError::new_err(format!("CommsError: {}", err))
}

fn new_runtime() -> PyResult<Runtime> {
    Runtime::new()
        .map_err(|e| PyValueError::new_err(format!("Failed to create tokio runtime: {}", e)))
}

/// Convert core transform updates into the `(from, to, StampedIsometry,
/// TransformType)` tuples used across the Python API (matches the observer
/// batch protocol in `schiebung_py`).
fn updates_to_tuples(
    updates: Vec<CoreTransformUpdate>,
) -> Vec<(String, String, StampedIsometry, TransformType)> {
    updates
        .into_iter()
        .map(|u| {
            (
                u.from,
                u.to,
                StampedIsometry::from(u.stamped_isometry),
                TransformType::from(u.kind),
            )
        })
        .collect()
}

/// Python wrapper for ZenohConfig
///
/// Configures how a client or server joins the zenoh network. The defaults
/// (peer mode with multicast discovery) work on networks where UDP multicast
/// is available; otherwise pin explicit endpoints via `listen`/`connect`.
#[pyclass]
#[derive(Clone, Default)]
pub struct ZenohConfig {
    pub inner: CoreZenohConfig,
}

#[pymethods]
impl ZenohConfig {
    /// Create a new ZenohConfig.
    ///
    /// Args:
    ///     mode: Zenoh session mode ("peer" or "client"). Default "peer".
    ///     listen: Endpoints to listen on (e.g. ["tcp/127.0.0.1:7447"]). Empty = zenoh defaults.
    ///     connect: Endpoints to actively connect to. Empty = zenoh defaults.
    ///     multicast_scouting: Enable UDP multicast peer discovery. Default True.
    ///     domain_id: ROS domain id (must match ROS_DOMAIN_ID of any ROS 2 system to talk to). Default 0.
    #[new]
    #[pyo3(signature = (mode=None, listen=None, connect=None, multicast_scouting=None, domain_id=None))]
    pub fn new(
        mode: Option<String>,
        listen: Option<Vec<String>>,
        connect: Option<Vec<String>>,
        multicast_scouting: Option<bool>,
        domain_id: Option<usize>,
    ) -> Self {
        let mut inner = CoreZenohConfig::default();
        if let Some(mode) = mode {
            inner.mode = mode;
        }
        if let Some(listen) = listen {
            inner.listen = listen;
        }
        if let Some(connect) = connect {
            inner.connect = connect;
        }
        if let Some(multicast_scouting) = multicast_scouting {
            inner.multicast_scouting = multicast_scouting;
        }
        if let Some(domain_id) = domain_id {
            inner.domain_id = domain_id;
        }
        ZenohConfig { inner }
    }

    /// Build a config from environment variables, falling back to defaults:
    ///
    /// - SCHIEBUNG_LISTEN: comma-separated endpoints to listen on
    /// - SCHIEBUNG_CONNECT: comma-separated endpoints to connect to
    /// - SCHIEBUNG_MULTICAST: set to "false" to disable multicast scouting
    /// - ROS_DOMAIN_ID: ROS domain id (same variable ROS 2 uses)
    #[staticmethod]
    pub fn from_env() -> Self {
        ZenohConfig {
            inner: CoreZenohConfig::from_env(),
        }
    }

    #[getter]
    fn mode(&self) -> String {
        self.inner.mode.clone()
    }

    #[getter]
    fn listen(&self) -> Vec<String> {
        self.inner.listen.clone()
    }

    #[getter]
    fn connect(&self) -> Vec<String> {
        self.inner.connect.clone()
    }

    #[getter]
    fn multicast_scouting(&self) -> bool {
        self.inner.multicast_scouting
    }

    #[getter]
    fn domain_id(&self) -> usize {
        self.inner.domain_id
    }

    fn __repr__(&self) -> String {
        format!(
            "ZenohConfig(mode='{}', listen={:?}, connect={:?}, multicast_scouting={}, domain_id={})",
            self.inner.mode,
            self.inner.listen,
            self.inner.connect,
            self.inner.multicast_scouting,
            self.inner.domain_id
        )
    }
}

/// Python wrapper for ServerHandle
///
/// Handle to a running server, allowing shutdown and status checking.
#[pyclass]
pub struct ServerHandle {
    inner: Mutex<Option<CoreServerHandle>>,
    runtime: Arc<Runtime>,
}

impl ServerHandle {
    /// Build a Python handle from a core handle (for inter-crate reuse).
    pub fn from_core(handle: CoreServerHandle, runtime: Arc<Runtime>) -> Self {
        ServerHandle {
            inner: Mutex::new(Some(handle)),
            runtime,
        }
    }
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

impl BufferTreeRef {
    /// Build a Python buffer reference from a core buffer (for inter-crate reuse).
    pub fn from_core(inner: Arc<RwLock<CoreBufferTree>>) -> Self {
        BufferTreeRef { inner }
    }
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

    /// Snapshot of the buffer's current content, latest sample per edge.
    ///
    /// Returns:
    ///     A list of (from_frame, to_frame, StampedIsometry, TransformType)
    ///     tuples, one per edge in the transform graph. The order is
    ///     unspecified.
    pub fn latest_transforms(
        &self,
    ) -> PyResult<Vec<(String, String, StampedIsometry, TransformType)>> {
        let guard = self
            .inner
            .read()
            .map_err(|e| PyValueError::new_err(format!("Lock poisoned: {}", e)))?;

        Ok(updates_to_tuples(guard.latest_transforms()))
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

/// Python wrapper for the transport-only TransformServer (no Rerun)
///
/// Subscribes to the standard ROS 2 `/tf` and `/tf_static` topics and serves
/// transform lookups via the `schiebung_msgs/LookupTransform` service. Works
/// standalone (plain zenoh, no ROS required) and interoperates with ROS 2
/// systems running rmw_zenoh.
#[pyclass]
pub struct TransformServer {
    inner: CoreTransformServer,
    runtime: Arc<Runtime>,
}

#[pymethods]
impl TransformServer {
    /// Create a new TransformServer.
    ///
    /// Args:
    ///     config: Optional ZenohConfig. Defaults to peer mode with multicast discovery.
    #[new]
    #[pyo3(signature = (config=None))]
    pub fn new(config: Option<ZenohConfig>) -> PyResult<Self> {
        let runtime = Arc::new(new_runtime()?);
        let config = config.map(|c| c.inner).unwrap_or_default();

        let inner = runtime
            .block_on(async { CoreTransformServer::with_config(config).await })
            .map_err(comms_err_to_pyerr)?;

        Ok(TransformServer { inner, runtime })
    }

    /// Get a reference to the underlying buffer tree.
    ///
    /// This allows access to the transform buffer while the server is running.
    /// Use this with `start()` for non-blocking server operation.
    #[getter]
    pub fn buffer(&self) -> BufferTreeRef {
        BufferTreeRef::from_core(self.inner.buffer())
    }

    /// Start the transform server in a background thread.
    ///
    /// This method returns immediately, allowing you to access the buffer
    /// while the server is running.
    ///
    /// Returns:
    ///     ServerHandle: A handle to control the running server.
    ///
    /// Example:
    ///     >>> server = TransformServer()
    ///     >>> handle = server.start()
    ///     >>> # ... clients publish /tf, call the lookup service ...
    ///     >>> handle.shutdown()
    ///     >>> handle.join()
    pub fn start(&self) -> ServerHandle {
        let handle = self.runtime.block_on(self.inner.start());
        ServerHandle::from_core(handle, self.runtime.clone())
    }

    /// Run the transform server (blocking).
    ///
    /// This method blocks until the server is shut down (via Ctrl+C).
    /// For non-blocking operation, use `start()` instead.
    pub fn run(&self) -> PyResult<()> {
        self.runtime
            .block_on(async { self.inner.run().await })
            .map_err(comms_err_to_pyerr)
    }
}

/// Python wrapper for TransformClient
///
/// Publishes dynamic transforms on `/tf` and static transforms on
/// `/tf_static` (transient local), and requests lookups via the
/// `schiebung_msgs/LookupTransform` service.
#[pyclass]
pub struct TransformClient {
    inner: CoreTransformClient,
    runtime: Runtime,
}

#[pymethods]
impl TransformClient {
    /// Create a new TransformClient.
    ///
    /// Args:
    ///     config: Optional ZenohConfig. Defaults to peer mode with multicast discovery.
    #[new]
    #[pyo3(signature = (config=None))]
    pub fn new(config: Option<ZenohConfig>) -> PyResult<Self> {
        let runtime = new_runtime()?;
        let config = config.map(|c| c.inner).unwrap_or_default();

        let inner = runtime
            .block_on(async { CoreTransformClient::with_config(config).await })
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
    ///     time: The timestamp in nanoseconds since Unix epoch (0 = latest available)
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

    /// Fetch the server's current buffer content (latest sample per edge).
    ///
    /// Returns:
    ///     A list of (from_frame, to_frame, StampedIsometry, TransformType)
    ///     tuples, one per edge in the server's transform graph. The order
    ///     is unspecified.
    pub fn request_all_transforms(
        &self,
    ) -> PyResult<Vec<(String, String, StampedIsometry, TransformType)>> {
        let updates = self
            .runtime
            .block_on(async { self.inner.request_all_transforms().await })
            .map_err(comms_err_to_pyerr)?;

        Ok(updates_to_tuples(updates))
    }
}

/// Python bindings for schiebung comms (transform client/server over zenoh)
#[pymodule(name = "schiebung_comms")]
fn schiebung_comms_module(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<ZenohConfig>()?;
    m.add_class::<TransformServer>()?;
    m.add_class::<ServerHandle>()?;
    m.add_class::<BufferTreeRef>()?;
    m.add_class::<TransformClient>()?;
    m.add_class::<StampedIsometry>()?;
    m.add_class::<TransformType>()?;
    m.add_class::<TfError>()?;
    Ok(())
}
