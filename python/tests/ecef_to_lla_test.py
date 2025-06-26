"""Unit test for the transform ecef_to_lla function.
"""

import numpy as np
import pytest
from kinematics_library import KinematicVector
from kinematics_library.transforms import ecef_to_lla


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            KinematicVector(4040000.0, 4813000.0, 1100000.0),
            KinematicVector(0.174440593201424, 0.872492313234789, 1888.1851099917756),
            1.0e-6,
            "ecef_to_lla with KinematicVector",
        ),
    ],
)
def test_ecef_to_lla_kv(kv, want, approx, desc):
    """Unit test for ecef_to_lla function with."""
    out = ecef_to_lla(kv)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            KinematicVector(4040000.0, 4813000.0, 1100000.0),
            KinematicVector(),
            KinematicVector(0.174440593201424, 0.872492313234789, 1888.1851099917756),
            1.0e-6,
            "ecef_to_lla with KinematicVector",
        ),
    ],
)
def test_ecef_to_lla_kv_with_out(kv, out, want, approx, desc):
    """Unit test for ecef_to_lla function with out."""
    ecef_to_lla(kv, out)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            np.array((4040000.0, 4813000.0, 1100000.0)),
            np.array((0.174440593201424, 0.872492313234789, 1888.1851099917756)),
            1.0e-6,
            "ecef_to_lla with 1 dimensional NumPy array",
        ),
    ],
)
def test_ecef_to_lla_numpy_1d(kv, want, approx, desc):
    """Unit test for ecef_to_lla function."""
    out = ecef_to_lla(kv)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            np.array((4040000.0, 4813000.0, 1100000.0)),
            np.array((0.0, 0.0, 0.0), dtype=np.double),
            np.array((0.174440593201424, 0.872492313234789, 1888.1851099917756)),
            1.0e-6,
            "ecef_to_lla with 1 dimensional NumPy array",
        ),
    ],
)
def test_ecef_to_lla_numpy_1d_with_out(kv, out, want, approx, desc):
    """Unit test for ecef_to_lla function with out."""
    ecef_to_lla(kv, out)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            np.array((4040000.0, 4813000.0, 1100000.0)).reshape(1, 3),
            np.array((0.174440593201424, 0.872492313234789, 1888.1851099917756)).reshape(1, 3),
            1.0e-6,
            "ecef_to_lla with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (4040000.0, 4813000.0, 1100000.0),
                    (4040000.0, 4813000.0, 1100000.0),
                    (4040000.0, 4813000.0, 1100000.0),
                )
            ),
            np.array(
                (
                    (0.174440593201424, 0.872492313234789, 1888.1851099917756),
                    (0.174440593201424, 0.872492313234789, 1888.1851099917756),
                    (0.174440593201424, 0.872492313234789, 1888.1851099917756),
                )
            ),
            1.0e-6,
            "ecef_to_lla with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_ecef_to_lla_numpy_2d(kv, want, approx, desc):
    """Unit test for ecef_to_lla function."""
    out = ecef_to_lla(kv)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            np.array((4040000.0, 4813000.0, 1100000.0)).reshape(1, 3),
            np.array((0.0, 0.0, 0.0), dtype=np.double).reshape(1, 3),
            np.array((0.174440593201424, 0.872492313234789, 1888.1851099917756)).reshape(1, 3),
            1.0e-6,
            "ecef_to_lla with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (4040000.0, 4813000.0, 1100000.0),
                    (4040000.0, 4813000.0, 1100000.0),
                    (4040000.0, 4813000.0, 1100000.0),
                )
            ),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (0.174440593201424, 0.872492313234789, 1888.1851099917756),
                    (0.174440593201424, 0.872492313234789, 1888.1851099917756),
                    (0.174440593201424, 0.872492313234789, 1888.1851099917756),
                )
            ),
            1.0e-6,
            "ecef_to_lla with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_ecef_to_lla_numpy_2d_with_out(kv, out, want, approx, desc):
    """Unit test for ecef_to_lla function with out."""
    ecef_to_lla(kv, out)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc
