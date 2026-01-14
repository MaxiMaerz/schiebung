//! Python bindings for schiebung with Rerun visualization
//!
//! This module provides a RerunBufferTree that automatically logs all
//! transforms to a Rerun recording stream.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rerun::RecordingStreamBuilder;

use ::schiebung_rerun::RerunObserver;
use schiebung::{FormatLoader as CoreFormatLoader, UrdfLoader as CoreUrdfLoader};

// Re-export Python wrapper types from schiebung-py to avoid duplication
pub use schiebung_py::{BufferTree, StampedIsometry, TfError, TransformType};

/// Python wrapper for BufferTree with integrated Rerun logging
///
/// This wraps a BufferTree from schiebung and automatically logs all
/// transforms to a Rerun recording stream via an observer.
///
/// Access the underlying buffer via the `buffer` property to interact
/// with transforms using the standard BufferTree API.
#[pyclass]
pub struct RerunBufferTree {
    buffer: Py<BufferTree>,
}

#[pymethods]
impl RerunBufferTree {
    /// Create a new RerunBufferTree with a Rerun recording stream
    ///
    /// Args:
    ///     application_id: The application ID for Rerun (e.g., "schiebung", "my_robot_app")
    ///     recording_id: The recording ID for this session (e.g., "session_001", "run_2024_01_13")
    ///     timeline: The name of the timeline for logging transforms (e.g., "stable_time")
    ///     publish_static_transforms: Whether to log static transforms to Rerun.
    ///                                Set to False if loading URDF via Rerun's built-in loader.
    ///
    /// Example:
    ///     >>> from schiebung_rerun import RerunBufferTree, StampedIsometry, TransformType
    ///     >>> tree = RerunBufferTree("schiebung", "session_001", "stable_time", True)
    ///     >>> # Access the buffer to add transforms
    ///     >>> t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
    ///     >>> tree.buffer.update("world", "robot", t, TransformType.Static)
    #[new]
    pub fn new(
        py: Python<'_>,
        application_id: String,
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
    ) -> PyResult<Self> {
        let rec = RecordingStreamBuilder::new(&*application_id)
            .recording_id(&*recording_id)
            .spawn()
            .map_err(|e| PyValueError::new_err(format!("Failed to create Rerun stream: {}", e)))?;

        // Create a BufferTree from schiebung_py
        let buffer = Py::new(py, BufferTree::new())?;

        // Register the Rerun observer
        let observer = RerunObserver::new(rec, publish_static_transforms, timeline);
        buffer
            .borrow_mut(py)
            .inner
            .register_observer(Box::new(observer));

        Ok(RerunBufferTree { buffer })
    }

    /// Get a reference to the underlying buffer tree.
    ///
    /// Use this to interact with the buffer using the standard BufferTree API
    /// from schiebung. All transform updates will be automatically logged to Rerun.
    ///
    /// Returns:
    ///     BufferTree: The underlying buffer that can be used to add and query transforms.
    ///
    /// Example:
    ///     >>> tree = RerunBufferTree("my_recording", "stable_time", True)
    ///     >>> buffer = tree.buffer
    ///     >>> buffer.update("world", "robot", transform, TransformType.Static)
    ///     >>> result = buffer.lookup_latest_transform("world", "robot")
    #[getter]
    pub fn buffer(&self, py: Python<'_>) -> Py<BufferTree> {
        self.buffer.clone_ref(py)
    }
}

/// Python wrapper for UrdfLoader
#[pyclass]
pub struct UrdfLoader {
    inner: CoreUrdfLoader,
}

#[pymethods]
impl UrdfLoader {
    #[new]
    pub fn new() -> Self {
        UrdfLoader {
            inner: CoreUrdfLoader::new(),
        }
    }

    /// Load transforms from a URDF file into the provided buffer
    ///
    /// Args:
    ///     path: Path to the URDF file
    ///     buffer: A BufferTree (from RerunBufferTree.buffer or standalone)
    pub fn load_into_buffer(&self, path: String, buffer: &mut BufferTree) -> PyResult<()> {
        self.inner
            .load_into_buffer(&path, &mut buffer.inner)
            .map_err(|e| PyValueError::new_err(format!("Failed to load URDF: {}", e)))?;
        Ok(())
    }
}

/// Python bindings for schiebung with Rerun visualization
#[pymodule(name = "schiebung_rerun")]
fn schiebung_rerun_module(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<RerunBufferTree>()?;
    m.add_class::<BufferTree>()?;
    m.add_class::<StampedIsometry>()?;
    m.add_class::<TransformType>()?;
    m.add_class::<TfError>()?;
    m.add_class::<UrdfLoader>()?;
    Ok(())
}
