"""Test Python observer callback functionality."""
import pytest
from schiebung import BufferTree, StampedIsometry, TransformType


def test_python_observer_callback():
    """Test that a Python function can be registered as an observer."""
    # Track observer calls
    calls = []

    def observer(from_frame, to_frame, transform, kind):
        calls.append({
            'from': from_frame,
            'to': to_frame,
            'translation': transform.translation(),
            'kind': kind
        })

    # Create buffer and register observer
    buf = BufferTree()
    buf.register_observer(observer)

    # Add a transform - observer should be called
    t = StampedIsometry([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    buf.update("world", "robot", t, TransformType.Static)

    # Verify observer was called
    assert len(calls) == 1
    assert calls[0]['from'] == "world"
    assert calls[0]['to'] == "robot"
    assert calls[0]['translation'] == [1.0, 2.0, 3.0]
    assert calls[0]['kind'] == TransformType.Static


def test_observer_receives_existing_transforms():
    """Test that observer receives all existing transforms when registered."""
    buf = BufferTree()

    # Add some transforms before registering observer
    t1 = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    t2 = StampedIsometry([0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0], 1.0)
    buf.update("world", "link1", t1, TransformType.Static)
    buf.update("link1", "link2", t2, TransformType.Dynamic)

    # Now register observer
    calls = []

    def observer(from_frame, to_frame, transform, kind):
        calls.append({'from': from_frame, 'to': to_frame})

    buf.register_observer(observer)

    # Observer should have received callbacks for existing transforms
    assert len(calls) >= 2
    # Check that both transforms were reported (order may vary)
    from_to_pairs = {(c['from'], c['to']) for c in calls}
    assert ('world', 'link1') in from_to_pairs
    assert ('link1', 'link2') in from_to_pairs


def test_multiple_observers():
    """Test that multiple observers can be registered and all are called."""
    buf = BufferTree()

    calls1 = []
    calls2 = []

    def observer1(from_frame, to_frame, transform, kind):
        calls1.append((from_frame, to_frame))

    def observer2(from_frame, to_frame, transform, kind):
        calls2.append((from_frame, to_frame))

    # Register both observers
    buf.register_observer(observer1)
    buf.register_observer(observer2)

    # Add a transform
    t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    buf.update("a", "b", t, TransformType.Static)

    # Both observers should have been called
    assert len(calls1) == 1
    assert len(calls2) == 1
    assert calls1[0] == ("a", "b")
    assert calls2[0] == ("a", "b")


def test_observer_must_be_callable():
    """Test that non-callable objects are rejected."""
    buf = BufferTree()

    with pytest.raises(ValueError, match="Observer must be a callable"):
        buf.register_observer("not a function")

    with pytest.raises(ValueError, match="Observer must be a callable"):
        buf.register_observer(42)


def test_observer_with_class_method():
    """Test that class methods can be used as observers."""
    class MyObserver:
        def __init__(self):
            self.calls = []

        def __call__(self, from_frame, to_frame, transform, kind):
            self.calls.append((from_frame, to_frame))

    buf = BufferTree()
    obs = MyObserver()
    buf.register_observer(obs)

    t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    buf.update("x", "y", t, TransformType.Dynamic)

    assert len(obs.calls) == 1
    assert obs.calls[0] == ("x", "y")


def test_observer_exception_handling():
    """Test that exceptions in observers don't crash the program."""
    buf = BufferTree()

    def bad_observer(from_frame, to_frame, transform, kind):
        raise RuntimeError("Intentional error in observer")

    # This should not raise, errors are logged
    buf.register_observer(bad_observer)

    # Adding a transform should not crash even though observer raises
    t = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    # This should complete successfully
    buf.update("a", "b", t, TransformType.Static)
