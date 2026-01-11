use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rerun::RecordingStreamBuilder;

use ::schiebung_rerun::RerunObserver;
use schiebung::{
    BufferTree as CoreBufferTree, FormatLoader as CoreFormatLoader,
    StampedIsometry as CoreStampedIsometry, TfError as CoreTfError,
    TransformType as CoreTransformType, UrdfLoader as CoreUrdfLoader,
};

/// Python wrapper for TfError
#[derive(Clone, Debug, PartialEq, Eq)]
#[pyclass(eq, eq_int)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// The graph is cyclic or the target has multiple incoming edges.
    InvalidGraph,
    /// Error loading or parsing a file format (URDF, USD, etc.)
    LoaderError,
}

impl From<CoreTfError> for TfError {
    fn from(err: CoreTfError) -> Self {
        match err {
            CoreTfError::AttemptedLookupInPast(_) => TfError::AttemptedLookupInPast,
            CoreTfError::AttemptedLookUpInFuture(_) => TfError::AttemptedLookUpInFuture,
            CoreTfError::CouldNotFindTransform(_) => TfError::CouldNotFindTransform,
            CoreTfError::InvalidGraph(_) => TfError::InvalidGraph,
            CoreTfError::LoaderError(_) => TfError::LoaderError,
        }
    }
}

fn core_err_to_pyerr(err: CoreTfError) -> PyErr {
    match &err {
        CoreTfError::AttemptedLookupInPast(msg) => {
            PyValueError::new_err(format!("TfError.AttemptedLookupInPast: {}", msg))
        }
        CoreTfError::AttemptedLookUpInFuture(msg) => {
            PyValueError::new_err(format!("TfError.AttemptedLookUpInFuture: {}", msg))
        }
        CoreTfError::CouldNotFindTransform(msg) => {
            PyValueError::new_err(format!("TfError.CouldNotFindTransform: {}", msg))
        }
        CoreTfError::InvalidGraph(msg) => {
            PyValueError::new_err(format!("TfError.InvalidGraph: {}", msg))
        }
        CoreTfError::LoaderError(msg) => {
            PyValueError::new_err(format!("TfError.LoaderError: {}", msg))
        }
    }
}

impl TfError {
    fn to_string(&self) -> String {
        match self {
            TfError::AttemptedLookupInPast => "TfError.AttemptedLookupInPast".to_string(),
            TfError::AttemptedLookUpInFuture => "TfError.AttemptedLookUpInFuture".to_string(),
            TfError::CouldNotFindTransform => "TfError.CouldNotFindTransform".to_string(),
            TfError::InvalidGraph => "TfError.InvalidGraph".to_string(),
            TfError::LoaderError => "TfError.LoaderError".to_string(),
        }
    }
}

impl std::convert::From<TfError> for PyErr {
    fn from(err: TfError) -> PyErr {
        PyValueError::new_err(err.to_string())
    }
}

/// Python wrapper for TransformType
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[pyclass(eq, eq_int)]
pub enum TransformType {
    /// Changes over time
    Dynamic = 0,
    /// Does not change over time
    Static = 1,
}

impl From<CoreTransformType> for TransformType {
    fn from(transform_type: CoreTransformType) -> Self {
        match transform_type {
            CoreTransformType::Dynamic => TransformType::Dynamic,
            CoreTransformType::Static => TransformType::Static,
        }
    }
}

impl From<TransformType> for CoreTransformType {
    fn from(transform_type: TransformType) -> Self {
        match transform_type {
            TransformType::Dynamic => CoreTransformType::Dynamic,
            TransformType::Static => CoreTransformType::Static,
        }
    }
}

#[pymethods]
impl TransformType {
    /// Create a static transform type
    #[staticmethod]
    fn static_transform() -> Self {
        TransformType::Static
    }

    /// Create a dynamic transform type
    #[staticmethod]
    fn dynamic_transform() -> Self {
        TransformType::Dynamic
    }

    fn __repr__(&self) -> String {
        match self {
            TransformType::Static => "TransformType.STATIC".to_string(),
            TransformType::Dynamic => "TransformType.DYNAMIC".to_string(),
        }
    }
}

/// Python wrapper for StampedIsometry
#[derive(Clone, Debug)]
#[pyclass]
pub struct StampedIsometry {
    inner: CoreStampedIsometry,
}

impl From<CoreStampedIsometry> for StampedIsometry {
    fn from(stamped_isometry: CoreStampedIsometry) -> Self {
        StampedIsometry {
            inner: stamped_isometry,
        }
    }
}

#[pymethods]
impl StampedIsometry {
    #[new]
    fn new(translation: [f64; 3], rotation: [f64; 4], stamp: f64) -> Self {
        StampedIsometry {
            inner: CoreStampedIsometry::new(translation, rotation, stamp),
        }
    }

    /// Get the translation as [x, y, z]
    fn translation(&self) -> [f64; 3] {
        self.inner.translation()
    }

    /// Get the rotation as [x, y, z, w] quaternion
    fn rotation(&self) -> [f64; 4] {
        self.inner.rotation()
    }

    /// Get the timestamp
    fn stamp(&self) -> f64 {
        self.inner.stamp()
    }

    /// Get Euler angles (roll, pitch, yaw) in radians
    fn euler_angles(&self) -> [f64; 3] {
        self.inner.euler_angles()
    }

    fn __repr__(&self) -> String {
        format!("{}", self.inner)
    }
}

/// Python wrapper for BufferTree with integrated Rerun logging
///
/// This is the same as BufferTree from schiebung-core-py but automatically
/// logs all transforms to a Rerun recording stream.
#[pyclass]
pub struct RerunBufferTree {
    inner: CoreBufferTree,
}

#[pymethods]
impl RerunBufferTree {
    /// Create a new RerunBufferTree with a Rerun recording stream
    ///
    /// Args:
    ///     recording_id: The ID for the Rerun recording (e.g., "my_recording")
    ///     timeline: The name of the timeline for logging transforms (e.g., "stable_time")
    ///     publish_static_transforms: Whether to log static transforms to Rerun.
    ///                                Set to False if loading URDF via Rerun's built-in loader.
    #[new]
    pub fn new(
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
    ) -> PyResult<Self> {
        let rec = RecordingStreamBuilder::new(&*recording_id)
            .spawn()
            .map_err(|e| PyValueError::new_err(format!("Failed to create Rerun stream: {}", e)))?;

        let mut inner = CoreBufferTree::new();
        let observer = RerunObserver::new(rec, publish_static_transforms, timeline);
        inner.register_observer(Box::new(observer));

        Ok(RerunBufferTree { inner })
    }

    /// Either update or push a transform to the graph
    /// The transform will also be logged to Rerun.
    /// Panics if the graph becomes cyclic
    pub fn update(
        &mut self,
        from: String,
        to: String,
        stamped_isometry: StampedIsometry,
        kind: TransformType,
    ) -> PyResult<()> {
        let core_stamped_isometry = CoreStampedIsometry::new(
            stamped_isometry.translation(),
            stamped_isometry.rotation(),
            stamped_isometry.stamp(),
        );

        self.inner
            .update(&from, &to, core_stamped_isometry, kind.into())
            .map_err(core_err_to_pyerr)?;
        Ok(())
    }

    /// Lookup the latest transform without any checks
    /// This can be used for static transforms or if the user does not care if the
    /// transform is still valid.
    /// NOTE: This might give you outdated transforms!
    pub fn lookup_latest_transform(
        &mut self,
        from: String,
        to: String,
    ) -> PyResult<StampedIsometry> {
        let result = self.inner.lookup_latest_transform(&from, &to);
        match result {
            Ok(transform) => Ok(StampedIsometry::from(transform)),
            Err(e) => Err(TfError::from(e).into()),
        }
    }

    /// Lookup the transform at time
    /// This will look for a transform at the provided time and can "time travel"
    /// If any edge contains a transform older then time a AttemptedLookupInPast is raised
    /// If the time is younger then any transform AttemptedLookUpInFuture is raised
    /// If there is no perfect match the transforms around this time are interpolated
    /// The interpolation is weighted with the distance to the time stamps
    pub fn lookup_transform(
        &mut self,
        from: String,
        to: String,
        time: f64,
    ) -> PyResult<StampedIsometry> {
        let result = self.inner.lookup_transform(&from, &to, time);
        match result {
            Ok(transform) => Ok(StampedIsometry::from(transform)),
            Err(e) => Err(core_err_to_pyerr(e)),
        }
    }

    /// Visualize the buffer tree as a DOT graph
    /// Can not use internal visualizer because we Store the nodes in self.index
    pub fn visualize(&self) -> String {
        self.inner.visualize()
    }

    /// Save the buffer tree as a PDF and dot file
    /// Runs graphiz to generate the PDF, fails if graphiz is not installed
    pub fn save_visualization(&self) -> PyResult<()> {
        self.inner
            .save_visualization()
            .map_err(|e| PyValueError::new_err(format!("Failed to save visualization: {}", e)))
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
    pub fn load_into_buffer(&self, path: String, buffer: &mut RerunBufferTree) -> PyResult<()> {
        self.inner
            .load_into_buffer(&path, &mut buffer.inner)
            .map_err(core_err_to_pyerr)?;
        Ok(())
    }
}

/// Python bindings for schiebung with Rerun visualization
#[pymodule(name = "schiebung_rerun")]
fn schiebung_rerun_module(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<RerunBufferTree>()?;
    m.add_class::<StampedIsometry>()?;
    m.add_class::<TransformType>()?;
    m.add_class::<TfError>()?;
    m.add_class::<UrdfLoader>()?;
    Ok(())
}
