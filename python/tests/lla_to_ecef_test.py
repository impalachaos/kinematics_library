"""Unit test for the transform lla_to_ecef function.
"""

import numpy as np
import pytest
from kinematics_library import KinematicVector
from kinematics_library.transforms import lla_to_ecef


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            KinematicVector(-0.8, 1.7, 4750.0),
            KinematicVector(-573960.23482409, 4417543.57131158, -4556021.90733023),
            1.0e-8,
            "lla_to_ecef with KinematicVector",
        ),
    ],
)
def test_lla_to_ecef_kv(kv, want, approx, desc):
    """Unit test for lla_to_ecef function with."""
    out = lla_to_ecef(kv)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            KinematicVector(-0.8, 1.7, 4750.0),
            KinematicVector(),
            KinematicVector(-573960.23482409, 4417543.57131158, -4556021.90733023),
            1.0e-8,
            "lla_to_ecef with KinematicVector",
        ),
    ],
)
def test_lla_to_ecef_kv_with_out(kv, out, want, approx, desc):
    """Unit test for lla_to_ecef function with out."""
    lla_to_ecef(kv, out)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            np.array((-0.8, 1.7, 4750.0)),
            np.array((-573960.23482409, 4417543.57131158, -4556021.90733023)),
            1.0e-8,
            "lla_to_ecef with 1 dimensional NumPy array",
        ),
    ],
)
def test_lla_to_ecef_numpy_1d(kv, want, approx, desc):
    """Unit test for lla_to_ecef function."""
    out = lla_to_ecef(kv)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            np.array((-0.8, 1.7, 4750.0)),
            np.array((0.0, 0.0, 0.0), dtype=np.double),
            np.array((-573960.23482409, 4417543.57131158, -4556021.90733023)),
            1.0e-8,
            "lla_to_ecef with 1 dimensional NumPy array",
        ),
    ],
)
def test_lla_to_ecef_numpy_1d_with_out(kv, out, want, approx, desc):
    """Unit test for lla_to_ecef function with out."""
    lla_to_ecef(kv, out)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            np.array((-0.8, 1.7, 4750.0)).reshape(1, 3),
            np.array((-573960.23482409, 4417543.57131158, -4556021.90733023)).reshape(1, 3),
            1.0e-8,
            "lla_to_ecef with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (-0.8, 1.7, 4750.0),
                    (-0.8, 1.7, 4750.0),
                    (-0.8, 1.7, 4750.0),
                )
            ),
            np.array(
                (
                    (-573960.23482409, 4417543.57131158, -4556021.90733023),
                    (-573960.23482409, 4417543.57131158, -4556021.90733023),
                    (-573960.23482409, 4417543.57131158, -4556021.90733023),
                )
            ),
            1.0e-8,
            "lla_to_ecef with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_lla_to_ecef_numpy_2d(kv, want, approx, desc):
    """Unit test for lla_to_ecef function."""
    out = lla_to_ecef(kv)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            np.array((-0.8, 1.7, 4750.0)).reshape(1, 3),
            np.array((0.0, 0.0, 0.0), dtype=np.double).reshape(1, 3),
            np.array((-573960.23482409, 4417543.57131158, -4556021.90733023)).reshape(1, 3),
            1.0e-8,
            "lla_to_ecef with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (-0.8, 1.7, 4750.0),
                    (-0.8, 1.7, 4750.0),
                    (-0.8, 1.7, 4750.0),
                )
            ),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (-573960.23482409, 4417543.57131158, -4556021.90733023),
                    (-573960.23482409, 4417543.57131158, -4556021.90733023),
                    (-573960.23482409, 4417543.57131158, -4556021.90733023),
                )
            ),
            1.0e-8,
            "lla_to_ecef with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_lla_to_ecef_numpy_2d_with_out(kv, out, want, approx, desc):
    """Unit test for lla_to_ecef function with out."""
    lla_to_ecef(kv, out)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc
