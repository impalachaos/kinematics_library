"""Unit test for the transform ned_to_lla function.
"""

import numpy as np
import pytest
from kinematics_library import KinematicVector
from kinematics_library.transforms import ned_to_lla


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            KinematicVector(12300.0, -8000.0, 6500.0),
            KinematicVector(0.1, 2.5, 9200.0),
            KinematicVector(0.10194035135339555, 2.49873974993884, 2716.9485910533103),
            1.0e-8,
            "ned_to_lla with KinematicVector",
        ),
    ],
)
def test_ned_to_lla_kv(kv, ref, want, approx, desc):
    """Unit test for ned_to_lla function with."""
    out = ned_to_lla(kv, ref)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            KinematicVector(12300.0, -8000.0, 6500.0),
            KinematicVector(0.1, 2.5, 9200.0),
            KinematicVector(),
            KinematicVector(0.10194035135339555, 2.49873974993884, 2716.9485910533103),
            1.0e-8,
            "ned_to_lla with KinematicVector",
        ),
    ],
)
def test_ned_to_lla_kv_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ned_to_lla function with out."""
    ned_to_lla(kv, ref, out)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            np.array((12300.0, -8000.0, 6500.0)),
            KinematicVector(0.1, 2.5, 9200.0),
            np.array((0.10194035135339555, 2.49873974993884, 2716.9485910533103)),
            1.0e-8,
            "ned_to_lla with 1 dimensional NumPy array",
        ),
    ],
)
def test_ned_to_lla_numpy_1d(kv, ref, want, approx, desc):
    """Unit test for ned_to_lla function."""
    out = ned_to_lla(kv, ref)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            np.array((12300.0, -8000.0, 6500.0)),
            KinematicVector(0.1, 2.5, 9200.0),
            np.array((0.0, 0.0, 0.0), dtype=np.double),
            np.array((0.10194035135339555, 2.49873974993884, 2716.9485910533103)),
            1.0e-8,
            "ned_to_lla with 1 dimensional NumPy array",
        ),
    ],
)
def test_ned_to_lla_numpy_1d_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ned_to_lla function with out."""
    ned_to_lla(kv, ref, out)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            np.array((12300.0, -8000.0, 6500.0)).reshape(1, 3),
            np.array((0.1, 2.5, 9200.0)).reshape(1, 3),
            np.array((0.10194035135339555, 2.49873974993884, 2716.9485910533103)).reshape(1, 3),
            1.0e-8,
            "ned_to_lla with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                )
            ),
            np.array(
                (
                    (0.1, 2.5, 9200.0),
                    (0.1, 2.5, 9200.0),
                    (0.1, 2.5, 9200.0),
                )
            ),
            np.array(
                (
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                )
            ),
            1.0e-8,
            "ned_to_lla with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_ned_to_lla_numpy_2d(kv, ref, want, approx, desc):
    """Unit test for ned_to_lla function."""
    out = ned_to_lla(kv, ref)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            np.array((12300.0, -8000.0, 6500.0)).reshape(1, 3),
            np.array((0.1, 2.5, 9200.0)),
            np.array((0.0, 0.0, 0.0), dtype=np.double).reshape(1, 3),
            np.array((0.10194035135339555, 2.49873974993884, 2716.9485910533103)).reshape(1, 3),
            1.0e-8,
            "ned_to_lla with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                )
            ),
            np.array(
                (
                    (0.1, 2.5, 9200.0),
                    (0.1, 2.5, 9200.0),
                    (0.1, 2.5, 9200.0),
                )
            ),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                )
            ),
            1.0e-8,
            "ned_to_lla with 2 dimensional NumPy array multiple rows",
        ),
        (
            np.array(
                (
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                )
            ),
            np.array((0.1, 2.5, 9200.0)),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                )
            ),
            1.0e-8,
            "ned_to_lla with 2 dimensional NumPy array multiple rows single 1d ref",
        ),
        (
            np.array(
                (
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                    (12300.0, -8000.0, 6500.0),
                )
            ),
            np.array(((0.1, 2.5, 9200.0),)),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                    (0.10194035135339555, 2.49873974993884, 2716.9485910533103),
                )
            ),
            1.0e-8,
            "ned_to_lla with 2 dimensional NumPy array multiple rows single 2d ref",
        ),
    ],
)
def test_ned_to_lla_numpy_2d_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ned_to_lla function with out."""
    ned_to_lla(kv, ref, out)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc
