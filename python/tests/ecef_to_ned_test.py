"""Unit test for the transform ecef_to_ned function.
"""

import numpy as np
import pytest
from kinematics_library import KinematicVector
from kinematics_library.transforms import ecef_to_ned


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            KinematicVector(-1465000.0, 2880000.0, 5483500.0),
            KinematicVector(2.1, -1.1, 1250.0),
            KinematicVector(2220.90854391709, 738.047215659648, -1205.58533167388),
            1.0e-8,
            "ecef_to_ned with KinematicVector",
        ),
    ],
)
def test_ecef_to_ned_kv(kv, ref, want, approx, desc):
    """Unit test for ecef_to_ned function with."""
    out = ecef_to_ned(kv, ref)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            KinematicVector(-1465000.0, 2880000.0, 5483500.0),
            KinematicVector(2.1, -1.1, 1250.0),
            KinematicVector(),
            KinematicVector(2220.90854391709, 738.047215659648, -1205.58533167388),
            1.0e-8,
            "ecef_to_ned with KinematicVector",
        ),
    ],
)
def test_ecef_to_ned_kv_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ecef_to_ned function with out."""
    ecef_to_ned(kv, ref, out)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            np.array((-1465000.0, 2880000.0, 5483500.0)),
            KinematicVector(2.1, -1.1, 1250.0),
            np.array((2220.90854391709, 738.047215659648, -1205.58533167388)),
            1.0e-8,
            "ecef_to_ned with 1 dimensional NumPy array",
        ),
    ],
)
def test_ecef_to_ned_numpy_1d(kv, ref, want, approx, desc):
    """Unit test for ecef_to_ned function."""
    out = ecef_to_ned(kv, ref)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            np.array((-1465000.0, 2880000.0, 5483500.0)),
            KinematicVector(2.1, -1.1, 1250.0),
            np.array((0.0, 0.0, 0.0), dtype=np.double),
            np.array((2220.90854391709, 738.047215659648, -1205.58533167388)),
            1.0e-8,
            "ecef_to_ned with 1 dimensional NumPy array",
        ),
    ],
)
def test_ecef_to_ned_numpy_1d_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ecef_to_ned function with out."""
    ecef_to_ned(kv, ref, out)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            np.array((-1465000.0, 2880000.0, 5483500.0)).reshape(1, 3),
            np.array((2.1, -1.1, 1250.0)).reshape(1, 3),
            np.array((2220.90854391709, 738.047215659648, -1205.58533167388)).reshape(1, 3),
            1.0e-8,
            "ecef_to_ned with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                )
            ),
            np.array(
                (
                    (2.1, -1.1, 1250.0),
                    (2.1, -1.1, 1250.0),
                    (2.1, -1.1, 1250.0),
                )
            ),
            np.array(
                (
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                )
            ),
            1.0e-8,
            "ecef_to_ned with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_ecef_to_ned_numpy_2d(kv, ref, want, approx, desc):
    """Unit test for ecef_to_ned function."""
    out = ecef_to_ned(kv, ref)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            np.array((-1465000.0, 2880000.0, 5483500.0)).reshape(1, 3),
            np.array((2.1, -1.1, 1250.0)),
            np.array((0.0, 0.0, 0.0), dtype=np.double).reshape(1, 3),
            np.array((2220.90854391709, 738.047215659648, -1205.58533167388)).reshape(1, 3),
            1.0e-8,
            "ecef_to_ned with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                )
            ),
            np.array(
                (
                    (2.1, -1.1, 1250.0),
                    (2.1, -1.1, 1250.0),
                    (2.1, -1.1, 1250.0),
                )
            ),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                )
            ),
            1.0e-8,
            "ecef_to_ned with 2 dimensional NumPy array multiple rows",
        ),
        (
            np.array(
                (
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                )
            ),
            np.array((2.1, -1.1, 1250.0)),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                )
            ),
            1.0e-8,
            "ecef_to_ned with 2 dimensional NumPy array multiple rows single 1d ref",
        ),
        (
            np.array(
                (
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                    (-1465000.0, 2880000.0, 5483500.0),
                )
            ),
            np.array(((2.1, -1.1, 1250.0),)),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                    (2220.90854391709, 738.047215659648, -1205.58533167388),
                )
            ),
            1.0e-8,
            "ecef_to_ned with 2 dimensional NumPy array multiple rows single 2d ref",
        ),
    ],
)
def test_ecef_to_ned_numpy_2d_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ecef_to_ned function with out."""
    ecef_to_ned(kv, ref, out)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc
