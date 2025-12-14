import pytest
import schiebung
from schiebung import BufferTree, StampedIsometry, TransformType, TfError

def test_buffer_creation():
    buf = BufferTree()
    assert buf is not None

def test_transform_types():
    assert TransformType.static_transform() is not None
    assert TransformType.dynamic_transform() is not None
    # Check string representation which is stable
    assert str(TransformType.Static) == "TransformType.STATIC"
    assert str(TransformType.static_transform()) == "TransformType.STATIC"

def test_stamped_isometry_creation():
    # translation: [x, y, z], rotation: [x, y, z, w], stamp: float
    t = StampedIsometry([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 10.0)
    assert t.translation() == [1.0, 2.0, 3.0]
    assert t.rotation() == [0.0, 0.0, 0.0, 1.0]
    assert t.stamp() == 10.0

def test_simple_lookup():
    buf = BufferTree()
    t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    
    # Add a static transform from map to odom
    buf.update("map", "odom", t, TransformType.Static)
    
    # Lookup latest
    res = buf.lookup_latest_transform("map", "odom")
    assert res.translation() == [1.0, 0.0, 0.0]
    
    # Lookup at specific time (should work for static)
    res2 = buf.lookup_transform("map", "odom", 100.0)
    assert res2.translation() == [1.0, 0.0, 0.0]

def test_dynamic_lookup_interpolation():
    buf = BufferTree()
    t1 = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    t2 = StampedIsometry([10.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.0)
    
    buf.update("odom", "base_link", t1, TransformType.Dynamic)
    buf.update("odom", "base_link", t2, TransformType.Dynamic)
    
    # Lookup at t=5.0, should be interpolated to [5.0, 0.0, 0.0]
    res = buf.lookup_transform("odom", "base_link", 5.0)
    assert res.translation() == [5.0, 0.0, 0.0]

def test_lookup_exceptions():
    buf = BufferTree()
    
    # Case 1: No transform exists
    with pytest.raises(ValueError, match="CouldNotFindTransform"):
        buf.lookup_latest_transform("A", "B")

    # Case 2: Graph disconnected
    t = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    buf.update("A", "B", t, TransformType.Static)
    buffer_c_d = BufferTree() # completely separate if we could... but we are using one buffer
    # Just lookup A->C
    with pytest.raises(ValueError, match="CouldNotFindTransform"):
         buf.lookup_transform("A", "C", 0.0)

def test_future_past_exceptions():
    buf = BufferTree()
    t1 = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.0)
    t2 = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 20.0)
    buf.update("A", "B", t1, TransformType.Dynamic)
    buf.update("A", "B", t2, TransformType.Dynamic)
    
    # Lookup in past (before 10.0)
    with pytest.raises(ValueError, match="AttemptedLookupInPast"):
        buf.lookup_transform("A", "B", 5.0)
        
    with pytest.raises(ValueError, match="AttemptedLookUpInFuture"):
        buf.lookup_transform("A", "B", 25.0)

def test_cycle_detection():
    buf = BufferTree()
    t = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    
    buf.update("A", "B", t, TransformType.Static)
    buf.update("B", "C", t, TransformType.Static)
    
    with pytest.raises(ValueError, match="InvalidGraph"):
         buf.update("C", "A", t, TransformType.Static)
