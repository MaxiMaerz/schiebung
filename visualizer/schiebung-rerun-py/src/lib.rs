//! Python bindings for schiebung with Rerun visualization.
//!
//! [`RerunObserver`] is a Rerun logger that plugs into the buffer's observer
//! protocol; [`RerunBufferTree`] is a convenience wrapper that creates a
//! `schiebung.BufferTree` and registers a [`RerunObserver`] on it.
//!
//! The transform types (`StampedIsometry`, `BufferTree`, `TransformType`,
//! `TfError`, `UrdfLoader`) are **re-exported from the `schiebung` package** —
//! `schiebung_rerun.StampedIsometry is schiebung.StampedIsometry`, so values
//! move freely between the two packages.
//!
//! Both constructors accept an optional `batcher_config` — a
//! `rerun.ChunkBatcherConfig` (incl. its `DEFAULT` / `LOW_LATENCY` / `ALWAYS` /
//! `NEVER` presets) — applied to the recording stream's chunk batcher.

use std::time::Duration;

use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use rerun::log::ChunkBatcherConfig;
use rerun::{RecordingStream, RecordingStreamBuilder};

use schiebung::{
    BufferObserver, StampedIsometry as CoreStampedIsometry, TransformType as CoreTransformType,
    TransformUpdate as CoreTransformUpdate,
};
use schiebung_rerun::RerunObserver as CoreRerunObserver;

/// Read a `rerun.ChunkBatcherConfig` (rerun-sdk's own pyclass, with its
/// `DEFAULT` / `LOW_LATENCY` / `ALWAYS` / `NEVER` presets) duck-typed into the
/// Rust [`ChunkBatcherConfig`] — we don't link the rerun extension module, so we
/// just call its accessors.
///
/// Starts from the Rust default and overrides the four fields rerun's Python
/// object exposes; `max_bytes_in_flight` (not exposed there) keeps its default.
fn batcher_config_from_py(obj: &Bound<'_, PyAny>) -> PyResult<ChunkBatcherConfig> {
    let mut config = ChunkBatcherConfig::default();

    // `flush_tick` is a `datetime.timedelta`; go via `total_seconds()` so we
    // don't depend on pyo3's timedelta conversion. The ALWAYS/NEVER presets set
    // `flush_tick = Duration::MAX`, which rerun's Python side can't even render
    // as a `timedelta` (it raises) — so on *any* failure here we fall back to
    // `Duration::MAX`, i.e. "effectively never tick", which preserves the intent
    // of those presets (and for ALWAYS the other thresholds force the flush
    // anyway). If the object isn't a batcher config at all, the missing
    // `flush_num_*` attrs below will surface a clear `AttributeError`.
    match obj
        .getattr("flush_tick")
        .and_then(|t| t.call_method0("total_seconds")?.extract::<f64>())
    {
        Ok(secs) => config.flush_tick = Duration::try_from_secs_f64(secs).unwrap_or(Duration::MAX),
        Err(_) => config.flush_tick = Duration::MAX,
    }
    config.flush_num_bytes = obj.getattr("flush_num_bytes")?.extract()?;
    config.flush_num_rows = obj.getattr("flush_num_rows")?.extract()?;
    config.chunk_max_rows_if_unsorted = obj.getattr("chunk_max_rows_if_unsorted")?.extract()?;

    Ok(config)
}

/// Build a Rerun [`RecordingStream`] honoring the `spawn` / `connect_addr` knobs:
///
/// * `connect_addr` set                       → connect to that gRPC endpoint
///   (overrides `spawn`).
/// * `connect_addr` `None`, `spawn` `true`    → spawn a viewer (the default).
/// * `connect_addr` `None`, `spawn` `false`   → connect to a viewer already
///   running on the default gRPC port.
///
/// `batcher_config`, if given, is applied via [`RecordingStreamBuilder::batcher_config`].
fn build_recording_stream(
    application_id: String,
    recording_id: String,
    spawn: bool,
    connect_addr: Option<String>,
    batcher_config: Option<ChunkBatcherConfig>,
) -> PyResult<RecordingStream> {
    let mut builder = RecordingStreamBuilder::new(application_id).recording_id(recording_id);
    if let Some(cfg) = batcher_config {
        builder = builder.batcher_config(cfg);
    }
    match connect_addr {
        Some(addr) => {
            let label = addr.clone();
            builder.connect_grpc_opts(addr).map_err(|e| {
                PyValueError::new_err(format!("Failed to connect to Rerun at {label}: {e}"))
            })
        }
        None if spawn => builder
            .spawn()
            .map_err(|e| PyValueError::new_err(format!("Failed to spawn Rerun viewer: {e}"))),
        None => builder
            .connect_grpc()
            .map_err(|e| PyValueError::new_err(format!("Failed to connect to Rerun: {e}"))),
    }
}

/// Pull a [`CoreStampedIsometry`] out of a Python `StampedIsometry` (from the
/// `schiebung` package) by calling its accessors. We treat it duck-typed so this
/// crate does not have to link the `schiebung` extension module.
fn core_isometry_from_py(obj: &Bound<'_, PyAny>) -> PyResult<CoreStampedIsometry> {
    let translation: [f64; 3] = obj.call_method0("translation")?.extract()?;
    let rotation: [f64; 4] = obj.call_method0("rotation")?.extract()?;
    let stamp: i64 = obj.call_method0("stamp")?.extract()?;
    Ok(CoreStampedIsometry::new(translation, rotation, stamp))
}

/// Map a Python `TransformType` (an `eq_int` enum: `Dynamic = 0`, `Static = 1`)
/// onto the core enum.
fn core_kind_from_py(obj: &Bound<'_, PyAny>) -> PyResult<CoreTransformType> {
    if obj.rich_compare(1i64, CompareOp::Eq)?.is_truthy()? {
        Ok(CoreTransformType::Static)
    } else {
        Ok(CoreTransformType::Dynamic)
    }
}

/// Rerun logger that can be registered on a `schiebung.BufferTree`.
///
/// Implements the buffer's batch-observer protocol (`on_update_batch`): every
/// insertion batch is forwarded to Rerun in a single columnar write. Static and
/// dynamic transforms live on separate entity-path namespaces.
///
/// Example:
///     >>> import schiebung, schiebung_rerun
///     >>> buf = schiebung.BufferTree()
///     >>> buf.register_observer(
///     ...     schiebung_rerun.RerunObserver("my_app", "session", "stable_time", True)
///     ... )
#[pyclass]
pub struct RerunObserver {
    inner: CoreRerunObserver,
}

#[pymethods]
impl RerunObserver {
    /// Create a Rerun logger.
    ///
    /// Args:
    ///     application_id: Rerun application id (e.g. "schiebung", "my_robot_app").
    ///     recording_id: Rerun recording id for this session.
    ///     timeline: Timeline name to attach dynamic transform timestamps to
    ///         (e.g. "stable_time").
    ///     publish_static_transforms: Whether to log static transforms. Set to
    ///         False when a URDF is loaded via Rerun's own loader (otherwise the
    ///         static frames are logged twice).
    ///     spawn: If True (the default) and no `connect_addr` is given, spawn a
    ///         Rerun viewer. If False, connect to a viewer already running on the
    ///         default gRPC port.
    ///     connect_addr: If given, connect to this Rerun gRPC endpoint
    ///         (e.g. "rerun+http://127.0.0.1:9876/proxy"); overrides `spawn`.
    ///     batcher_config: Optional `rerun.ChunkBatcherConfig` (e.g.
    ///         `rr.ChunkBatcherConfig.LOW_LATENCY()`) controlling the recording
    ///         stream's batch-flush thresholds. The `RERUN_FLUSH_TICK_SECS` /
    ///         `RERUN_FLUSH_NUM_BYTES` / `RERUN_FLUSH_NUM_ROWS` /
    ///         `RERUN_CHUNK_MAX_ROWS_IF_UNSORTED` env vars still override it.
    #[new]
    #[pyo3(signature = (application_id, recording_id, timeline, publish_static_transforms, *, spawn=true, connect_addr=None, batcher_config=None))]
    pub fn new(
        application_id: String,
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
        spawn: bool,
        connect_addr: Option<String>,
        batcher_config: Option<Bound<'_, PyAny>>,
    ) -> PyResult<Self> {
        let batcher_config = batcher_config
            .map(|c| batcher_config_from_py(&c))
            .transpose()?;
        let rec = build_recording_stream(
            application_id,
            recording_id,
            spawn,
            connect_addr,
            batcher_config,
        )?;
        Ok(RerunObserver {
            inner: CoreRerunObserver::new(rec, publish_static_transforms, timeline),
        })
    }

    /// Buffer batch-observer hook — see `BufferTree.register_observer`.
    ///
    /// `updates` is a list of `(from, to, StampedIsometry, TransformType)` tuples.
    fn on_update_batch(
        &self,
        py: Python<'_>,
        updates: Vec<(String, String, Py<PyAny>, Py<PyAny>)>,
    ) -> PyResult<()> {
        let mut core_updates: Vec<CoreTransformUpdate> = Vec::with_capacity(updates.len());
        for (from, to, iso, kind) in &updates {
            let iso = core_isometry_from_py(iso.bind(py))?;
            let kind = core_kind_from_py(kind.bind(py))?;
            core_updates.push(CoreTransformUpdate::new(
                from.clone(),
                to.clone(),
                iso,
                kind,
            ));
        }
        self.inner.on_update(&core_updates);
        Ok(())
    }
}

/// A `schiebung.BufferTree` with an attached Rerun logger.
///
/// Convenience wrapper: creates a `schiebung.BufferTree`, registers a
/// [`RerunObserver`] on it, and exposes the buffer via the `buffer` property.
/// `tree.buffer` is a genuine `schiebung.BufferTree`, so it accepts and returns
/// the `schiebung.StampedIsometry` type — no separate wrapper type.
///
/// Example:
///     >>> from schiebung_rerun import RerunBufferTree, StampedIsometry, TransformType
///     >>> tree = RerunBufferTree("schiebung", "session_001", "stable_time", True)
///     >>> t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
///     >>> tree.buffer.update("world", "robot", t, TransformType.Static)
///     >>> # ...or many at once
///     >>> tree.buffer.update_batch([("world", "robot", t, TransformType.Static)])
#[pyclass]
pub struct RerunBufferTree {
    buffer: Py<PyAny>,
}

#[pymethods]
impl RerunBufferTree {
    /// Create a `RerunBufferTree`.
    ///
    /// Takes the same arguments as [`RerunObserver`]; see there for the meaning
    /// of `spawn` / `connect_addr` / `batcher_config`.
    #[new]
    #[pyo3(signature = (application_id, recording_id, timeline, publish_static_transforms, *, spawn=true, connect_addr=None, batcher_config=None))]
    pub fn new(
        py: Python<'_>,
        application_id: String,
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
        spawn: bool,
        connect_addr: Option<String>,
        batcher_config: Option<Bound<'_, PyAny>>,
    ) -> PyResult<Self> {
        let observer = Py::new(
            py,
            RerunObserver::new(
                application_id,
                recording_id,
                timeline,
                publish_static_transforms,
                spawn,
                connect_addr,
                batcher_config,
            )?,
        )?;
        let buffer = py.import("schiebung")?.getattr("BufferTree")?.call0()?;
        buffer.call_method1("register_observer", (observer,))?;
        Ok(RerunBufferTree {
            buffer: buffer.unbind(),
        })
    }

    /// The underlying `schiebung.BufferTree`. Every update to it is logged to Rerun.
    #[getter]
    pub fn buffer(&self, py: Python<'_>) -> Py<PyAny> {
        self.buffer.clone_ref(py)
    }
}

/// Python bindings for schiebung with Rerun visualization.
#[pymodule(name = "schiebung_rerun")]
fn schiebung_rerun_module(py: Python<'_>, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<RerunBufferTree>()?;
    m.add_class::<RerunObserver>()?;

    // Re-export the core transform types from the `schiebung` package so that
    // `schiebung_rerun.X is schiebung.X` — the two packages share the exact
    // same Python type objects.
    let schiebung = py.import("schiebung")?;
    for name in [
        "BufferTree",
        "StampedIsometry",
        "TransformType",
        "TfError",
        "UrdfLoader",
    ] {
        m.add(name, schiebung.getattr(name)?)?;
    }
    Ok(())
}
