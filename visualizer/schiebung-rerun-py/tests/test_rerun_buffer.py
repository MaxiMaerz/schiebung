"""Tests for schiebung_rerun Python bindings."""
import pytest
import rerun as rr

import schiebung
from schiebung_rerun import (
    BinaryStream,
    FileSink,
    GrpcSink,
    RerunBufferTree,
    RerunObserver,
    StampedIsometry,
    Stdout,
    TransformType,
    TfError,
    UrdfLoader,
)

# A well-formed gRPC URL that nothing is listening on: the sink connects lazily
# and drops data, so RerunBufferTree can be exercised without a viewer.
DEAD_ADDR = "rerun+http://127.0.0.1:9999/proxy"


def test_stamped_isometry():
    """Test StampedIsometry creation and methods."""
    t = StampedIsometry([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 10)
    assert t.translation() == [1.0, 2.0, 3.0]
    assert t.rotation() == [0.0, 0.0, 0.0, 1.0]
    assert t.stamp() == 10
    # Check euler angles exist
    euler = t.euler_angles()
    assert len(euler) == 3


def test_transform_types():
    """Test TransformType enum."""
    assert TransformType.static_transform() is not None
    assert TransformType.dynamic_transform() is not None
    assert str(TransformType.Static) == "TransformType.STATIC"
    assert str(TransformType.Dynamic) == "TransformType.DYNAMIC"


def test_urdf_loader_creation():
    """Test UrdfLoader can be created."""
    loader = UrdfLoader()
    assert loader is not None


def test_rerun_buffer_tree_creation():
    """RerunBufferTree can be created without a viewer (connect_addr, no spawn)."""
    tree = RerunBufferTree("schiebung", "test_session", "stable_time", True, connect_addr=DEAD_ADDR)
    assert tree is not None
    assert tree.buffer is not None


def test_rerun_buffer_tree_update_and_lookup():
    """Test basic update and lookup with RerunBufferTree."""
    tree = RerunBufferTree("schiebung", "test_session", "stable_time", True, connect_addr=DEAD_ADDR)

    t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
    tree.buffer.update("world", "robot", t, TransformType.Static)

    result = tree.buffer.lookup_latest_transform("world", "robot")
    assert result.translation() == [1.0, 0.0, 0.0]


def test_rerun_buffer_tree_dynamic_interpolation():
    """Test dynamic transform interpolation."""
    tree = RerunBufferTree("schiebung", "test_session", "stable_time", True, connect_addr=DEAD_ADDR)

    t1 = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
    t2 = StampedIsometry([10.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10_000_000_000)

    tree.buffer.update("odom", "base_link", t1, TransformType.Dynamic)
    tree.buffer.update("odom", "base_link", t2, TransformType.Dynamic)

    # Lookup at t=5s (5_000_000_000 ns) should give [5.0, 0.0, 0.0]
    result = tree.buffer.lookup_transform("odom", "base_link", 5_000_000_000)
    assert result.translation() == [5.0, 0.0, 0.0]


def _roundtrip(tree):
    """Push one transform through `tree.buffer` and read it back."""
    tree.buffer.update(
        "world", "robot",
        StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0),
        TransformType.Static,
    )
    assert tree.buffer.lookup_latest_transform("world", "robot").translation() == [1.0, 0.0, 0.0]


@pytest.mark.parametrize(
    "cfg",
    [
        None,
        rr.ChunkBatcherConfig.LOW_LATENCY(),                       # non-trivial flush_tick (8ms)
        rr.ChunkBatcherConfig(flush_num_rows=1, flush_num_bytes=1),
    ],
)
def test_batcher_config_accepted(cfg):
    """RerunBufferTree accepts a rerun.ChunkBatcherConfig (incl. presets) and still works."""
    tree = RerunBufferTree("schiebung", "batcher", "stable_time", True,
                           connect_addr=DEAD_ADDR, batcher_config=cfg)
    _roundtrip(tree)


def test_batcher_config_on_observer():
    """RerunObserver also accepts batcher_config."""
    buf = schiebung.BufferTree()
    buf.register_observer(RerunObserver("schiebung", "batcher", "stable_time", True,
                                        connect_addr=DEAD_ADDR,
                                        batcher_config=rr.ChunkBatcherConfig.LOW_LATENCY()))
    buf.update("world", "robot",
               StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0),
               TransformType.Static)
    assert buf.lookup_latest_transform("world", "robot").translation() == [1.0, 0.0, 0.0]


def test_batcher_config_rejects_non_config():
    """Passing something that isn't a batcher config raises a clear error."""
    with pytest.raises((AttributeError, TypeError)):
        RerunBufferTree("schiebung", "batcher", "stable_time", True,
                        connect_addr=DEAD_ADDR, batcher_config=object())


# ---------------------------------------------------------------------------
# sinks=[...] — multi-sink routing
# ---------------------------------------------------------------------------
# These tests intentionally avoid GrpcSink and Stdout: GrpcSink requires a live
# viewer at the other end (the rerun client buffers and drops with no listener,
# but exercising it meaningfully in CI is flaky), and Stdout writes the rrd
# byte stream to fd 1, which clobbers the test runner's output. See
# `examples/sinks_demo.py` for manual coverage of those two paths.


def test_filesink_writes_rrd(tmp_path):
    """A FileSink-only RerunBufferTree writes a non-empty .rrd file."""
    rrd = tmp_path / "out.rrd"
    tree = RerunBufferTree("schiebung", "filesink", "stable_time", True,
                           sinks=[FileSink(str(rrd))])
    _roundtrip(tree)
    del tree  # drop the recording so any buffered data is flushed to disk
    assert rrd.exists()
    assert rrd.stat().st_size > 0


def test_binarystream_round_trip():
    """A BinaryStream sink yields rrd bytes that start with the RRD magic."""
    bs = BinaryStream()
    assert "unattached" in repr(bs)
    tree = RerunBufferTree("schiebung", "binstream", "stable_time", True, sinks=[bs])
    assert "attached" in repr(bs)
    _roundtrip(tree)
    data = bs.read()
    assert data.startswith(b"RRF2"), f"expected rrd magic, got {data[:8]!r}"
    # read() drains the buffer.
    assert bs.read() == b""


def test_multisink_filesink_and_binarystream(tmp_path):
    """Fan-out to a file AND an in-process buffer in a single recording."""
    rrd = tmp_path / "tee.rrd"
    bs = BinaryStream()
    tree = RerunBufferTree("schiebung", "tee", "stable_time", True,
                           sinks=[bs, FileSink(str(rrd))])
    _roundtrip(tree)
    data = bs.read()
    del tree
    assert data.startswith(b"RRF2")
    assert rrd.exists() and rrd.stat().st_size > 0


def test_sinks_also_on_observer(tmp_path):
    """RerunObserver accepts the same `sinks=[...]` argument."""
    rrd = tmp_path / "obs.rrd"
    buf = schiebung.BufferTree()
    buf.register_observer(RerunObserver("schiebung", "obs", "stable_time", True,
                                        sinks=[FileSink(str(rrd))]))
    buf.update("world", "robot",
               StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0),
               TransformType.Static)
    del buf
    assert rrd.exists() and rrd.stat().st_size > 0


def test_sinks_conflicts_with_connect_addr():
    """`sinks=` and `connect_addr=` together raise ValueError."""
    with pytest.raises(ValueError, match="cannot be combined"):
        RerunBufferTree("schiebung", "x", "stable_time", True,
                        connect_addr=DEAD_ADDR, sinks=[FileSink("/tmp/x.rrd")])


def test_sinks_conflicts_with_spawn_false():
    """`sinks=` and an explicit `spawn=False` together raise ValueError."""
    with pytest.raises(ValueError, match="cannot be combined"):
        RerunBufferTree("schiebung", "x", "stable_time", True,
                        spawn=False, sinks=[FileSink("/tmp/x.rrd")])


def test_sinks_empty_list_rejected():
    """An empty `sinks=[]` is not a meaningful routing and raises ValueError."""
    with pytest.raises(ValueError, match="at least one sink"):
        RerunBufferTree("schiebung", "x", "stable_time", True, sinks=[])


def test_sinks_unknown_type_rejected():
    """Passing something that isn't one of our sink classes raises ValueError."""
    with pytest.raises(ValueError, match="not a valid sink"):
        RerunBufferTree("schiebung", "x", "stable_time", True, sinks=[object()])


def test_grpcsink_invalid_url_rejected():
    """GrpcSink validates the URL at construction time."""
    with pytest.raises(ValueError, match="invalid Rerun gRPC endpoint"):
        GrpcSink("not-a-rerun-url")


def test_binarystream_read_before_attach_rejected():
    """Reading from an unattached BinaryStream raises ValueError (rather than panicking)."""
    bs = BinaryStream()
    with pytest.raises(ValueError, match="not been attached"):
        bs.read()
    with pytest.raises(ValueError, match="not been attached"):
        bs.flush()


def test_sink_constructors_smoke():
    """All four sink classes are constructable and have sensible reprs."""
    assert "rerun" in repr(GrpcSink()).lower()
    assert "/tmp/foo.rrd" in repr(FileSink("/tmp/foo.rrd"))
    assert repr(Stdout()) == "Stdout()"
    assert repr(BinaryStream()) == "BinaryStream(unattached)"
