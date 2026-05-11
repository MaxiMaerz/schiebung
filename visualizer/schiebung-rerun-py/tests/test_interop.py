"""schiebung <-> schiebung_rerun interoperability.

`schiebung_rerun` re-exports the transform types from `schiebung`, so they are
the *same* Python objects and values pass freely between the two packages.
"""
import pytest

import schiebung
import schiebung_rerun
from schiebung_rerun import RerunBufferTree, RerunObserver

# A well-formed gRPC URL that nothing is listening on: the sink connects lazily
# and drops data, so no viewer is needed for these tests.
DEAD_ADDR = "rerun+http://127.0.0.1:9999/proxy"


@pytest.mark.parametrize("name", ["StampedIsometry", "BufferTree", "TransformType", "TfError", "UrdfLoader"])
def test_shared_types(name):
    """schiebung_rerun.<name> is exactly schiebung.<name>."""
    assert getattr(schiebung_rerun, name) is getattr(schiebung, name)


def test_core_isometry_works_with_rerun_buffer():
    """A schiebung.StampedIsometry round-trips through a RerunBufferTree.buffer."""
    tree = RerunBufferTree("interop", "rec", "stable_time", True, connect_addr=DEAD_ADDR)

    # tree.buffer is a genuine schiebung.BufferTree.
    assert isinstance(tree.buffer, schiebung.BufferTree)

    t = schiebung.StampedIsometry([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 0)
    tree.buffer.update("world", "robot", t, schiebung.TransformType.Static)

    result = tree.buffer.lookup_latest_transform("world", "robot")
    assert isinstance(result, schiebung.StampedIsometry)
    assert result.translation() == [1.0, 2.0, 3.0]


def test_dynamic_interpolation_through_rerun_buffer():
    tree = RerunBufferTree("interop", "rec", "stable_time", True, connect_addr=DEAD_ADDR)

    t1 = schiebung.StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
    t2 = schiebung.StampedIsometry([10.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10_000_000_000)
    tree.buffer.update_batch([
        ("odom", "base_link", t1, schiebung.TransformType.Dynamic),
        ("odom", "base_link", t2, schiebung.TransformType.Dynamic),
    ])

    result = tree.buffer.lookup_transform("odom", "base_link", 5_000_000_000)
    assert result.translation() == [5.0, 0.0, 0.0]


def test_rerun_observer_on_plain_buffer():
    """RerunObserver can be registered directly on a schiebung.BufferTree."""
    buf = schiebung.BufferTree()
    buf.register_observer(RerunObserver("interop", "rec", "stable_time", True, connect_addr=DEAD_ADDR))

    t = schiebung.StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
    buf.update("world", "robot", t, schiebung.TransformType.Static)  # must not raise
    assert buf.lookup_latest_transform("world", "robot").translation() == [1.0, 0.0, 0.0]
