"""Tests for schiebung_rerun Python bindings."""
import pytest
import rerun as rr

import schiebung
from schiebung_rerun import (
    RerunBufferTree,
    RerunObserver,
    StampedIsometry,
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
