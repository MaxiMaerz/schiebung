"""Dual-typed stamp argument on StampedIsometry: int → ns, float → s."""

import pytest

from schiebung import StampedIsometry


def test_int_stamp_is_nanoseconds():
    iso = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10_000_000_000)
    assert iso.stamp() == 10_000_000_000
    assert iso.stamp_secs() == 10.0


def test_float_stamp_is_seconds():
    iso = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 10.5)
    assert iso.stamp() == 10_500_000_000
    assert iso.stamp_secs() == 10.5


def test_int_and_float_stamps_equivalent():
    """Constructor with float seconds matches the explicit from_secs path."""
    a = StampedIsometry([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 2.5)
    b = StampedIsometry.from_secs([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 2.5)
    assert a.stamp() == b.stamp()


def test_zero_stamp_int_and_float_equivalent():
    a = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0)
    b = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 0.0)
    assert a.stamp() == b.stamp() == 0


def test_bool_stamp_treated_as_int():
    """Python booleans are an int subtype — should resolve to ns, not raise."""
    iso = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], True)
    assert iso.stamp() == 1


def test_string_stamp_rejected():
    with pytest.raises(TypeError, match="int.*float"):
        StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], "10.0")


def test_negative_float_stamp():
    """Pre-epoch timestamps round-trip too."""
    iso = StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], -1.5)
    assert iso.stamp() == -1_500_000_000
