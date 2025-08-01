"""Unit test for the transform aer_to_ned function.
"""

import numpy as np
import pytest
from kinematics_library import KinematicVector
from kinematics_library.transforms import aer_to_ned


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            KinematicVector(30 * np.pi / 180, 40 * np.pi / 180, -70),
            KinematicVector(-46.43897628, -26.81155573, 44.99513264),
            1.0e-6,
            "aer_to_ned with KinematicVector",
        ),
    ],
)
def test_aer_to_ned_kv(kv, want, approx, desc):
    """Unit test for aer_to_ned function with."""
    out = aer_to_ned(kv)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            KinematicVector(30 * np.pi / 180, 40 * np.pi / 180, -70),
            KinematicVector(),
            KinematicVector(-46.43897628, -26.81155573, 44.99513264),
            1.0e-6,
            "aer_to_ned with KinematicVector",
        ),
    ],
)
def test_aer_to_ned_kv_with_out(kv, out, want, approx, desc):
    """Unit test for aer_to_ned function with out."""
    aer_to_ned(kv, out)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            np.array((30 * np.pi / 180, 40 * np.pi / 180, -70)),
            np.array((-46.43897628, -26.81155573, 44.99513264)),
            1.0e-6,
            "aer_to_ned with 1 dimensional NumPy array",
        ),
    ],
)
def test_aer_to_ned_numpy_1d(kv, want, approx, desc):
    """Unit test for aer_to_ned function."""
    out = aer_to_ned(kv)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            np.array((30 * np.pi / 180, 40 * np.pi / 180, -70)),
            np.array((0.0, 0.0, 0.0), dtype=np.double),
            np.array((-46.43897628, -26.81155573, 44.99513264)),
            1.0e-6,
            "aer_to_ned with 1 dimensional NumPy array",
        ),
    ],
)
def test_aer_to_ned_numpy_1d_with_out(kv, out, want, approx, desc):
    """Unit test for aer_to_ned function with out."""
    aer_to_ned(kv, out)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,want,approx,desc",
    [
        (
            np.array((30 * np.pi / 180, 40 * np.pi / 180, -70)).reshape(1, 3),
            np.array((-46.43897628, -26.81155573, 44.99513264)).reshape(1, 3),
            1.0e-6,
            "aer_to_ned with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (30 * np.pi / 180, 40 * np.pi / 180, -70),
                    (30 * np.pi / 180, 40 * np.pi / 180, -70),
                    (30 * np.pi / 180, 40 * np.pi / 180, -70),
                )
            ),
            np.array(
                (
                    (-46.43897628, -26.81155573, 44.99513264),
                    (-46.43897628, -26.81155573, 44.99513264),
                    (-46.43897628, -26.81155573, 44.99513264),
                )
            ),
            1.0e-6,
            "aer_to_ned with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_aer_to_ned_numpy_2d(kv, want, approx, desc):
    """Unit test for aer_to_ned function."""
    out = aer_to_ned(kv)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,out,want,approx,desc",
    [
        (
            np.array((30 * np.pi / 180, 40 * np.pi / 180, -70)).reshape(1, 3),
            np.array((0.0, 0.0, 0.0), dtype=np.double).reshape(1, 3),
            np.array((-46.43897628, -26.81155573, 44.99513264)).reshape(1, 3),
            1.0e-6,
            "aer_to_ned with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (30 * np.pi / 180, 40 * np.pi / 180, -70),
                    (30 * np.pi / 180, 40 * np.pi / 180, -70),
                    (30 * np.pi / 180, 40 * np.pi / 180, -70),
                )
            ),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (-46.43897628, -26.81155573, 44.99513264),
                    (-46.43897628, -26.81155573, 44.99513264),
                    (-46.43897628, -26.81155573, 44.99513264),
                )
            ),
            1.0e-6,
            "aer_to_ned with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def test_aer_to_ned_numpy_2d_with_out(kv, out, want, approx, desc):
    """Unit test for aer_to_ned function with out."""
    aer_to_ned(kv, out)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc
