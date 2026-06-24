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
//!
//! Both constructors also accept an optional `sinks=[…]` argument that mirrors
//! rerun's `rr.set_sinks(...)` API: pass a non-empty list of [`GrpcSink`],
//! [`FileSink`], [`Stdout`], and/or [`BinaryStream`] to fan out the recording
//! to multiple destinations. When `sinks` is supplied it takes the place of
//! `spawn` / `connect_addr`; combining them raises `ValueError`.
//!
//! [`RerunObserver::from_recording`] and [`RerunBufferTree::from_recording`]
//! attach to an existing Python `rerun.RecordingStream` instead. This lets
//! regular Python Rerun logs and Schiebung transforms share one native Rerun
//! sink, including a single `rerun.FileSink` writer.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

use pyo3::basic::CompareOp;
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList};
use rerun::external::re_uri::ProxyUri;
use rerun::log::ChunkBatcherConfig;
use rerun::sink::{
    BinaryStreamSink as RrBinaryStreamSink, BinaryStreamStorage, FileSink as RrFileSink,
    GrpcSink as RrGrpcSink, LogSink,
};
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

/// gRPC log sink. Mirrors `rerun.GrpcSink(url=None)`.
///
/// The URL is validated eagerly at construction (same as rerun-sdk does in its
/// own `GrpcSink.__init__`), so a malformed endpoint raises `ValueError`
/// immediately rather than at recording-stream build time.
#[pyclass]
pub struct GrpcSink {
    uri: ProxyUri,
}

#[pymethods]
impl GrpcSink {
    #[new]
    #[pyo3(signature = (url=None))]
    fn new(url: Option<String>) -> PyResult<Self> {
        let url = url.unwrap_or_else(|| rerun::DEFAULT_CONNECT_URL.to_owned());
        let uri = url.parse::<ProxyUri>().map_err(|e| {
            PyValueError::new_err(format!("invalid Rerun gRPC endpoint {url:?}: {e}"))
        })?;
        Ok(Self { uri })
    }

    fn __repr__(&self) -> String {
        format!("GrpcSink({:?})", self.uri.to_string())
    }
}

/// File log sink. Mirrors `rerun.FileSink(path)` — writes an `.rrd` file.
#[pyclass]
pub struct FileSink {
    path: PathBuf,
}

#[pymethods]
impl FileSink {
    #[new]
    #[pyo3(signature = (path))]
    fn new(path: PathBuf) -> Self {
        Self { path }
    }

    fn __repr__(&self) -> String {
        format!("FileSink({:?})", self.path)
    }
}

/// Stdout log sink. Pipes the rrd byte stream to stdout (so
/// `python my_script.py | rerun -` works). Maps to
/// [`rerun::sink::FileSink::stdout`].
#[pyclass]
pub struct Stdout;

#[pymethods]
impl Stdout {
    #[new]
    fn new() -> Self {
        Self
    }

    fn __repr__(&self) -> String {
        "Stdout()".to_owned()
    }
}

/// In-process binary log sink. Mirrors `rerun.BinaryStream`.
///
/// `BinaryStream()` is constructed without a recording; the underlying
/// [`BinaryStreamStorage`] is created lazily the first time the sink is
/// attached to a recording (via `sinks=[BinaryStream()]`). After that, call
/// `read()` to drain the buffered rrd bytes, or `flush()` to force-flush the
/// batcher first.
///
/// Reusing the same `BinaryStream` across multiple `RerunBufferTree`s is not
/// supported — the storage binds to the first recording's flush channel.
#[pyclass]
pub struct BinaryStream {
    storage: OnceLock<BinaryStreamStorage>,
}

impl BinaryStream {
    /// Initialise the storage from a freshly built [`RecordingStream`] if not
    /// already initialised, and return a fresh [`RrBinaryStreamSink`] that
    /// writes into the shared buffer.
    fn make_sink(&self, rec: &RecordingStream) -> RrBinaryStreamSink {
        let storage = self.storage.get_or_init(|| {
            // We discard `_sink` here — we only want the storage. The actual
            // sink that will be attached to the recording is created by
            // `with_shared_storage` below, so all log messages from this
            // recording (and any siblings sharing the storage) land in the
            // same buffer.
            let (_sink, storage) = RrBinaryStreamSink::new(rec.clone());
            storage
        });
        RrBinaryStreamSink::with_shared_storage(storage)
    }
}

#[pymethods]
impl BinaryStream {
    #[new]
    fn new() -> Self {
        Self {
            storage: OnceLock::new(),
        }
    }

    /// Drain the buffered bytes as a fully encoded rrd byte string.
    ///
    /// Args:
    ///     flush: If True (default), flush the recording's batcher before
    ///         reading so all pending messages are encoded into the buffer.
    ///     flush_timeout_sec: Maximum wait when `flush=True`; defaults to a
    ///         very large value (~inf). Raises `RuntimeError` on timeout.
    ///
    /// Returns:
    ///     `bytes` — the rrd payload. Returns `b""` if no messages were
    ///     buffered (e.g. read called before any update).
    #[pyo3(signature = (*, flush=true, flush_timeout_sec=1e38_f64))]
    fn read<'py>(
        &self,
        py: Python<'py>,
        flush: bool,
        flush_timeout_sec: f64,
    ) -> PyResult<Bound<'py, PyBytes>> {
        let storage = self.storage.get().ok_or_else(|| {
            PyValueError::new_err(
                "BinaryStream has not been attached to a recording yet — pass it via `sinks=[…]` first",
            )
        })?;
        if flush {
            let timeout = Duration::try_from_secs_f64(flush_timeout_sec).unwrap_or(Duration::MAX);
            storage
                .flush(timeout)
                .map_err(|e| PyValueError::new_err(format!("Failed to flush BinaryStream: {e}")))?;
        }
        let bytes = storage.read().unwrap_or_default();
        Ok(PyBytes::new(py, &bytes))
    }

    /// Block until the underlying recording's batcher has been flushed.
    #[pyo3(signature = (timeout_sec=1e38_f64))]
    fn flush(&self, timeout_sec: f64) -> PyResult<()> {
        let storage = self.storage.get().ok_or_else(|| {
            PyValueError::new_err(
                "BinaryStream has not been attached to a recording yet — pass it via `sinks=[…]` first",
            )
        })?;
        let timeout = Duration::try_from_secs_f64(timeout_sec).unwrap_or(Duration::MAX);
        storage
            .flush(timeout)
            .map_err(|e| PyValueError::new_err(format!("Failed to flush BinaryStream: {e}")))?;
        Ok(())
    }

    fn __repr__(&self) -> String {
        let state = if self.storage.get().is_some() {
            "attached"
        } else {
            "unattached"
        };
        format!("BinaryStream({state})")
    }
}

/// Translate a Python list of our sink pyclasses into a `Vec<Box<dyn LogSink>>`
/// suitable for [`RecordingStream::set_sinks`].
///
/// `rec` is the recording stream that has just been built (in buffered state);
/// `BinaryStream` sinks bind their lazy storage to it.
fn resolve_sinks(
    sinks: Vec<Bound<'_, PyAny>>,
    rec: &RecordingStream,
) -> PyResult<Vec<Box<dyn LogSink>>> {
    if sinks.is_empty() {
        return Err(PyValueError::new_err(
            "`sinks` must contain at least one sink",
        ));
    }
    let mut out: Vec<Box<dyn LogSink>> = Vec::with_capacity(sinks.len());
    for sink in sinks {
        if let Ok(grpc) = sink.cast::<GrpcSink>() {
            let uri = grpc.borrow().uri.clone();
            out.push(Box::new(RrGrpcSink::new(uri)));
        } else if let Ok(file) = sink.cast::<FileSink>() {
            let path = file.borrow().path.clone();
            let s = RrFileSink::new(path)
                .map_err(|e| PyValueError::new_err(format!("Failed to open FileSink: {e}")))?;
            out.push(Box::new(s));
        } else if sink.cast::<Stdout>().is_ok() {
            let s = RrFileSink::stdout()
                .map_err(|e| PyValueError::new_err(format!("Failed to open Stdout sink: {e}")))?;
            out.push(Box::new(s));
        } else if let Ok(bs) = sink.cast::<BinaryStream>() {
            let sink = bs.borrow().make_sink(rec);
            out.push(Box::new(sink));
        } else {
            let type_name = sink.get_type().name()?;
            return Err(PyValueError::new_err(format!(
                "{type_name} is not a valid sink, must be one of: GrpcSink, FileSink, Stdout, BinaryStream"
            )));
        }
    }
    Ok(out)
}

/// Build a Rerun [`RecordingStream`] honoring `sinks`, `spawn`, and
/// `connect_addr`:
///
/// * `sinks` set (non-empty) → build a buffered stream, then
///   [`RecordingStream::set_sinks`] with the resolved sinks. Mutually exclusive
///   with a non-default `spawn` or `connect_addr` (raises `ValueError`).
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
    sinks: Option<Vec<Bound<'_, PyAny>>>,
) -> PyResult<RecordingStream> {
    if sinks.is_some() && (connect_addr.is_some() || !spawn) {
        return Err(PyValueError::new_err(
            "`sinks=[…]` cannot be combined with `spawn=False` or `connect_addr`; choose one routing path",
        ));
    }

    let mut builder = RecordingStreamBuilder::new(application_id).recording_id(recording_id);
    if let Some(cfg) = batcher_config {
        builder = builder.batcher_config(cfg);
    }

    if let Some(sinks) = sinks {
        let rec = builder.buffered().map_err(|e| {
            PyValueError::new_err(format!("Failed to build buffered recording stream: {e}"))
        })?;
        let resolved = resolve_sinks(sinks, &rec)?;
        rec.set_sinks(resolved);
        return Ok(rec);
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

/// One transform row prepared for Rerun's Python columnar API.
///
/// The shared-recording backend cannot move a Rust `RecordingStream` across
/// extension-module boundaries, so it retains the Python recording and sends
/// these rows through `rerun.RecordingStream.send_columns` instead.
#[derive(Clone)]
struct SharedRow {
    parent: String,
    child: String,
    translation: [f32; 3],
    quaternion: [f32; 4],
    stamp_ns: i64,
}

impl From<&CoreTransformUpdate> for SharedRow {
    fn from(update: &CoreTransformUpdate) -> Self {
        let translation = update.stamped_isometry.translation();
        let quaternion = update.stamped_isometry.rotation();
        Self {
            parent: update.from.clone(),
            child: update.to.clone(),
            translation: [
                translation[0] as f32,
                translation[1] as f32,
                translation[2] as f32,
            ],
            quaternion: [
                quaternion[0] as f32,
                quaternion[1] as f32,
                quaternion[2] as f32,
                quaternion[3] as f32,
            ],
            stamp_ns: update.stamped_isometry.stamp(),
        }
    }
}

/// Backend for a Python-owned `rerun.RecordingStream`.
///
/// Keeping this backend separate preserves the all-Rust fast path for the
/// existing constructors. It is used only by `RerunObserver.from_recording`.
struct SharedRecordingBackend {
    recording: Py<PyAny>,
    timeline: String,
    publish_static_transforms: bool,
    static_state: Mutex<HashMap<(String, String), SharedRow>>,
}

impl SharedRecordingBackend {
    fn new(recording: Py<PyAny>, timeline: String, publish_static_transforms: bool) -> Self {
        Self {
            recording,
            timeline,
            publish_static_transforms,
            static_state: Mutex::new(HashMap::new()),
        }
    }

    fn on_update(&self, py: Python<'_>, updates: &[CoreTransformUpdate]) -> PyResult<()> {
        let mut dynamic_rows = Vec::new();
        let mut static_updates = Vec::new();

        for update in updates {
            match update.kind {
                CoreTransformType::Dynamic => dynamic_rows.push(SharedRow::from(update)),
                CoreTransformType::Static if self.publish_static_transforms => {
                    static_updates.push(SharedRow::from(update));
                }
                CoreTransformType::Static => {}
            }
        }

        if !dynamic_rows.is_empty() {
            self.send_rows(py, "tf", &dynamic_rows, true)?;
        }

        if !static_updates.is_empty() {
            let snapshot: Vec<SharedRow> = {
                let mut state = self.static_state.lock().unwrap();
                for row in static_updates {
                    state.insert((row.parent.clone(), row.child.clone()), row);
                }
                state.values().cloned().collect()
            };
            self.send_rows(py, "tf_static", &snapshot, false)?;
        }

        Ok(())
    }

    /// Send a batch through Rerun's Python columnar API.
    ///
    /// Numeric Python timestamps are seconds, whereas Schiebung stamps are
    /// nanoseconds. Constructing a NumPy `datetime64[ns]` array preserves the
    /// exact values before handing them to `rerun.TimeColumn`.
    fn send_rows(
        &self,
        py: Python<'_>,
        entity_path: &str,
        rows: &[SharedRow],
        temporal: bool,
    ) -> PyResult<()> {
        let rerun = py.import("rerun")?;

        let translations: Vec<[f32; 3]> = rows.iter().map(|row| row.translation).collect();
        let quaternions: Vec<[f32; 4]> = rows.iter().map(|row| row.quaternion).collect();
        let parents: Vec<&str> = rows.iter().map(|row| row.parent.as_str()).collect();
        let children: Vec<&str> = rows.iter().map(|row| row.child.as_str()).collect();

        let transform_kwargs = PyDict::new(py);
        transform_kwargs.set_item("translation", translations)?;
        transform_kwargs.set_item("quaternion", quaternions)?;
        transform_kwargs.set_item("parent_frame", &parents)?;
        transform_kwargs.set_item("child_frame", &children)?;
        let transform_columns =
            rerun
                .getattr("Transform3D")?
                .call_method("columns", (), Some(&transform_kwargs))?;

        let frame_kwargs = PyDict::new(py);
        frame_kwargs.set_item("frame", children)?;
        let frame_columns =
            rerun
                .getattr("CoordinateFrame")?
                .call_method("columns", (), Some(&frame_kwargs))?;

        let columns = PyList::empty(py);
        for column in transform_columns.try_iter()? {
            columns.append(column?)?;
        }
        for column in frame_columns.try_iter()? {
            columns.append(column?)?;
        }

        let indexes = PyList::empty(py);
        if temporal {
            let stamps: Vec<i64> = rows.iter().map(|row| row.stamp_ns).collect();
            let array_kwargs = PyDict::new(py);
            array_kwargs.set_item("dtype", "datetime64[ns]")?;
            let timestamps =
                py.import("numpy")?
                    .call_method("asarray", (stamps,), Some(&array_kwargs))?;

            let time_kwargs = PyDict::new(py);
            time_kwargs.set_item("timestamp", timestamps)?;
            let time_column = rerun
                .getattr("TimeColumn")?
                .call((self.timeline.as_str(),), Some(&time_kwargs))?;
            indexes.append(time_column)?;
        }

        self.recording.bind(py).call_method(
            "send_columns",
            (entity_path, indexes, columns),
            None,
        )?;
        Ok(())
    }
}

enum RerunObserverBackend {
    Native(CoreRerunObserver),
    Shared(SharedRecordingBackend),
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
    backend: RerunObserverBackend,
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
    ///         default gRPC port. Ignored when `sinks` is provided.
    ///     connect_addr: If given, connect to this Rerun gRPC endpoint
    ///         (e.g. "rerun+http://127.0.0.1:9876/proxy"); overrides `spawn`.
    ///         Mutually exclusive with `sinks`.
    ///     batcher_config: Optional `rerun.ChunkBatcherConfig` (e.g.
    ///         `rr.ChunkBatcherConfig.LOW_LATENCY()`) controlling the recording
    ///         stream's batch-flush thresholds. The `RERUN_FLUSH_TICK_SECS` /
    ///         `RERUN_FLUSH_NUM_BYTES` / `RERUN_FLUSH_NUM_ROWS` /
    ///         `RERUN_CHUNK_MAX_ROWS_IF_UNSORTED` env vars still override it.
    ///     sinks: Optional non-empty list of `GrpcSink` / `FileSink` / `Stdout`
    ///         / `BinaryStream` instances. Mirrors rerun's `rr.set_sinks(...)`
    ///         and takes precedence over `spawn` / `connect_addr` (combining
    ///         them raises `ValueError`).
    ///
    /// Example:
    ///     >>> from schiebung_rerun import RerunObserver, GrpcSink, FileSink
    ///     >>> obs = RerunObserver("app", "session", "stable_time", True,
    ///     ...                     sinks=[GrpcSink(), FileSink("/tmp/session.rrd")])
    #[new]
    #[pyo3(signature = (application_id, recording_id, timeline, publish_static_transforms, *, spawn=true, connect_addr=None, batcher_config=None, sinks=None))]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        application_id: String,
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
        spawn: bool,
        connect_addr: Option<String>,
        batcher_config: Option<Bound<'_, PyAny>>,
        sinks: Option<Vec<Bound<'_, PyAny>>>,
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
            sinks,
        )?;
        Ok(RerunObserver {
            backend: RerunObserverBackend::Native(CoreRerunObserver::new(
                rec,
                publish_static_transforms,
                timeline,
            )),
        })
    }

    /// Create an observer that logs into an existing Python
    /// `rerun.RecordingStream`.
    ///
    /// The supplied recording owns all routing and lifecycle decisions, so
    /// configure its sinks before constructing the observer. This is the mode
    /// to use when regular Python Rerun logs and Schiebung transforms must land
    /// in the same file or binary stream.
    ///
    /// Args:
    ///     recording: An existing `rerun.RecordingStream`.
    ///     timeline: Timeline for dynamic transform timestamps.
    ///     publish_static_transforms: Whether static transforms are forwarded.
    ///
    /// Example:
    ///     >>> import rerun as rr
    ///     >>> import schiebung, schiebung_rerun
    ///     >>> rec = rr.RecordingStream("my_app", recording_id="session")
    ///     >>> rec.set_sinks(rr.FileSink("session.rrd"))
    ///     >>> obs = schiebung_rerun.RerunObserver.from_recording(
    ///     ...     rec, "stable_time", True
    ///     ... )
    ///     >>> buf = schiebung.BufferTree()
    ///     >>> buf.register_observer(obs)
    #[staticmethod]
    #[pyo3(signature = (recording, timeline, publish_static_transforms))]
    pub fn from_recording(
        py: Python<'_>,
        recording: Py<PyAny>,
        timeline: String,
        publish_static_transforms: bool,
    ) -> PyResult<Self> {
        let rerun = py.import("rerun")?;
        let recording_type = rerun.getattr("RecordingStream")?;
        if !recording.bind(py).is_instance(&recording_type)? {
            let type_name = recording.bind(py).get_type().name()?;
            return Err(PyTypeError::new_err(format!(
                "recording must be a rerun.RecordingStream, got {type_name}"
            )));
        }

        Ok(Self {
            backend: RerunObserverBackend::Shared(SharedRecordingBackend::new(
                recording,
                timeline,
                publish_static_transforms,
            )),
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
        match &self.backend {
            RerunObserverBackend::Native(inner) => {
                inner.on_update(&core_updates);
                Ok(())
            }
            RerunObserverBackend::Shared(inner) => inner.on_update(py, &core_updates),
        }
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
///     >>> # ...or fan out to a viewer AND an .rrd file:
///     >>> from schiebung_rerun import GrpcSink, FileSink
///     >>> tree2 = RerunBufferTree("schiebung", "session_002", "stable_time", True,
///     ...                         sinks=[GrpcSink(), FileSink("/tmp/session.rrd")])
#[pyclass]
pub struct RerunBufferTree {
    buffer: Py<PyAny>,
}

impl RerunBufferTree {
    fn with_observer(py: Python<'_>, observer: RerunObserver) -> PyResult<Self> {
        let observer = Py::new(py, observer)?;
        let buffer = py.import("schiebung")?.getattr("BufferTree")?.call0()?;
        buffer.call_method1("register_observer", (observer,))?;
        Ok(Self {
            buffer: buffer.unbind(),
        })
    }
}

#[pymethods]
impl RerunBufferTree {
    /// Create a `RerunBufferTree`.
    ///
    /// Takes the same arguments as [`RerunObserver`]; see there for the meaning
    /// of `spawn` / `connect_addr` / `batcher_config` / `sinks`.
    #[new]
    #[pyo3(signature = (application_id, recording_id, timeline, publish_static_transforms, *, spawn=true, connect_addr=None, batcher_config=None, sinks=None))]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        py: Python<'_>,
        application_id: String,
        recording_id: String,
        timeline: String,
        publish_static_transforms: bool,
        spawn: bool,
        connect_addr: Option<String>,
        batcher_config: Option<Bound<'_, PyAny>>,
        sinks: Option<Vec<Bound<'_, PyAny>>>,
    ) -> PyResult<Self> {
        Self::with_observer(
            py,
            RerunObserver::new(
                application_id,
                recording_id,
                timeline,
                publish_static_transforms,
                spawn,
                connect_addr,
                batcher_config,
                sinks,
            )?,
        )
    }

    /// Create a `RerunBufferTree` that logs into an existing Python
    /// `rerun.RecordingStream`.
    ///
    /// This is the convenience equivalent of creating a plain
    /// `schiebung.BufferTree` and registering
    /// `RerunObserver.from_recording(...)` on it.
    #[staticmethod]
    #[pyo3(signature = (recording, timeline, publish_static_transforms))]
    pub fn from_recording(
        py: Python<'_>,
        recording: Py<PyAny>,
        timeline: String,
        publish_static_transforms: bool,
    ) -> PyResult<Self> {
        Self::with_observer(
            py,
            RerunObserver::from_recording(py, recording, timeline, publish_static_transforms)?,
        )
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
    m.add_class::<GrpcSink>()?;
    m.add_class::<FileSink>()?;
    m.add_class::<Stdout>()?;
    m.add_class::<BinaryStream>()?;

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
