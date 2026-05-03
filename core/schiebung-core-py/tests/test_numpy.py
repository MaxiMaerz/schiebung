"""NumPy interop for StampedIsometry: as_matrix / as_translation /
as_quaternion / __array__ / from_matrix round-trip."""

import math

import numpy as np
import pytest

from schiebung import StampedIsometry


def _identity_iso(stamp_ns: int = 0) -> StampedIsometry:
    return StampedIsometry([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], stamp_ns)


def _translation_iso(x: float, y: float, z: float) -> StampedIsometry:
    return StampedIsometry([x, y, z], [0.0, 0.0, 0.0, 1.0], 0)


def test_as_matrix_identity():
    mat = _identity_iso().as_matrix()
    assert isinstance(mat, np.ndarray)
    assert mat.shape == (4, 4)
    assert mat.dtype == np.float64
    np.testing.assert_array_equal(mat, np.eye(4))


def test_as_matrix_translation_only():
    mat = _translation_iso(1.0, 2.0, 3.0).as_matrix()
    expected = np.eye(4)
    expected[:3, 3] = [1.0, 2.0, 3.0]
    np.testing.assert_array_equal(mat, expected)


def test_as_translation_and_as_quaternion_shapes():
    iso = StampedIsometry([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 0)
    t = iso.as_translation()
    q = iso.as_quaternion()
    assert isinstance(t, np.ndarray) and t.shape == (3,) and t.dtype == np.float64
    assert isinstance(q, np.ndarray) and q.shape == (4,) and q.dtype == np.float64
    np.testing.assert_array_equal(t, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(q, [0.0, 0.0, 0.0, 1.0])


def test_array_protocol_returns_matrix():
    """np.asarray(iso) and np.array(iso) both go through __array__."""
    iso = _translation_iso(0.5, 0.0, 1.0)
    expected = iso.as_matrix()

    np.testing.assert_array_equal(np.asarray(iso), expected)
    np.testing.assert_array_equal(np.array(iso), expected)


def test_array_protocol_works_with_numpy_ops():
    """Once __array__ is exposed, numpy primitives accept iso transparently."""
    iso = _translation_iso(2.0, 0.0, 0.0)
    inv = np.linalg.inv(iso)  # numpy coerces via __array__
    expected = np.eye(4)
    expected[0, 3] = -2.0
    np.testing.assert_allclose(inv, expected)


def test_array_protocol_dtype_float64_accepted():
    iso = _identity_iso()
    np.testing.assert_array_equal(np.asarray(iso, dtype=np.float64), np.eye(4))


def test_array_protocol_other_dtype_rejected():
    iso = _identity_iso()
    with pytest.raises(ValueError, match="float64"):
        np.asarray(iso, dtype=np.float32)


def test_from_matrix_round_trip_translation_only():
    src = np.eye(4)
    src[:3, 3] = [4.0, 5.0, 6.0]
    iso = StampedIsometry.from_matrix(src, stamp=42)
    assert iso.stamp() == 42
    np.testing.assert_allclose(iso.as_matrix(), src)


def test_from_matrix_round_trip_with_rotation():
    """Rotate 90° about Z and translate, then round-trip."""
    angle = math.pi / 2
    c, s = math.cos(angle), math.sin(angle)
    src = np.array(
        [
            [c, -s, 0.0, 1.0],
            [s, c, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    iso = StampedIsometry.from_matrix(src)
    np.testing.assert_allclose(iso.as_matrix(), src, atol=1e-12)


def test_from_matrix_default_stamp_is_zero():
    iso = StampedIsometry.from_matrix(np.eye(4))
    assert iso.stamp() == 0


def test_from_matrix_rejects_wrong_shape():
    with pytest.raises(ValueError, match=r"shape \(4, 4\)"):
        StampedIsometry.from_matrix(np.eye(3))


def test_as_matrix_returns_fresh_array_each_call():
    """Mutating the returned array must not affect subsequent calls."""
    iso = _translation_iso(1.0, 2.0, 3.0)
    mat = iso.as_matrix()
    mat[0, 3] = 999.0
    again = iso.as_matrix()
    assert again[0, 3] == 1.0


def test_from_matrix_accepts_float_seconds():
    """from_matrix's stamp uses the same int/float dispatch as the ctor."""
    iso = StampedIsometry.from_matrix(np.eye(4), stamp=1.5)
    assert iso.stamp() == 1_500_000_000


def test_from_matrix_accepts_int_nanoseconds():
    iso = StampedIsometry.from_matrix(np.eye(4), stamp=42)
    assert iso.stamp() == 42
