"""Unit test for the transform ned_to_ecef function.
"""

import numpy as np
import pytest
from kinematics_library import KinematicVector
from kinematics_library.transforms import ned_to_ecef


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            KinematicVector(-6500.0, -4800.0, 2400.0),
            KinematicVector(-2.9, -0.5, 6000.0),
            KinematicVector(-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
            1.0e-8,
            "ned_to_ecef with KinematicVector",
        ),
    ],
)
def ned_to_ecef_kv(kv, ref, want, approx, desc):
    """Unit test for ned_to_ecef function with."""
    out = ned_to_ecef(kv, ref)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            KinematicVector(-6500.0, -4800.0, 2400.0),
            KinematicVector(-2.9, -0.5, 6000.0),
            KinematicVector(),
            KinematicVector(-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
            1.0e-8,
            "ned_to_ecef with KinematicVector",
        ),
    ],
)
def ned_to_ecef_kv_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ned_to_ecef function with out."""
    ned_to_ecef(kv, ref, out)
    assert out.x == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out.y == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out.z == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            np.array((-6500.0, -4800.0, 2400.0)),
            KinematicVector(-2.9, -0.5, 6000.0),
            np.array((-5442559.838214509, 2967814.4198892456, -1510590.1708055406)),
            1.0e-8,
            "ned_to_ecef with 1 dimensional NumPy array",
        ),
    ],
)
def ned_to_ecef_numpy_1d(kv, ref, want, approx, desc):
    """Unit test for ned_to_ecef function."""
    out = ned_to_ecef(kv, ref)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            np.array((-6500.0, -4800.0, 2400.0)),
            KinematicVector(-2.9, -0.5, 6000.0),
            np.array((0.0, 0.0, 0.0), dtype=np.double),
            np.array((-5442559.838214509, 2967814.4198892456, -1510590.1708055406)),
            1.0e-8,
            "ned_to_ecef with 1 dimensional NumPy array",
        ),
    ],
)
def ned_to_ecef_numpy_1d_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ned_to_ecef function with out."""
    ned_to_ecef(kv, ref, out)
    assert out.shape == kv.shape, "validate shape"
    assert out[0] == pytest.approx(want[0], abs=approx), "x: " + desc
    assert out[1] == pytest.approx(want[1], abs=approx), "y: " + desc
    assert out[2] == pytest.approx(want[2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,want,approx,desc",
    [
        (
            np.array((-6500.0, -4800.0, 2400.0)).reshape(1, 3),
            np.array((-2.9, -0.5, 6000.0)).reshape(1, 3),
            np.array((-5442559.838214509, 2967814.4198892456, -1510590.1708055406)).reshape(1, 3),
            1.0e-8,
            "ned_to_ecef with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                )
            ),
            np.array(
                (
                    (-2.9, -0.5, 6000.0),
                    (-2.9, -0.5, 6000.0),
                    (-2.9, -0.5, 6000.0),
                )
            ),
            np.array(
                (
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                )
            ),
            1.0e-8,
            "ned_to_ecef with 2 dimensional NumPy array multiple rows",
        ),
    ],
)
def ned_to_ecef_numpy_2d(kv, ref, want, approx, desc):
    """Unit test for ned_to_ecef function."""
    out = ned_to_ecef(kv, ref)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "kv,ref,out,want,approx,desc",
    [
        (
            np.array((-6500.0, -4800.0, 2400.0)).reshape(1, 3),
            np.array((-2.9, -0.5, 6000.0)),
            np.array((0.0, 0.0, 0.0), dtype=np.double).reshape(1, 3),
            np.array((-5442559.838214509, 2967814.4198892456, -1510590.1708055406)).reshape(1, 3),
            1.0e-8,
            "ned_to_ecef with 2 dimensional NumPy array",
        ),
        (
            np.array(
                (
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                )
            ),
            np.array(
                (
                    (-2.9, -0.5, 6000.0),
                    (-2.9, -0.5, 6000.0),
                    (-2.9, -0.5, 6000.0),
                )
            ),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                )
            ),
            1.0e-8,
            "ned_to_ecef with 2 dimensional NumPy array multiple rows",
        ),
        (
            np.array(
                (
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                )
            ),
            np.array((-2.9, -0.5, 6000.0)),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                )
            ),
            1.0e-8,
            "ned_to_ecef with 2 dimensional NumPy array multiple rows single 1d ref",
        ),
        (
            np.array(
                (
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                    (-6500.0, -4800.0, 2400.0),
                )
            ),
            np.array(((-2.9, -0.5, 6000.0),)),
            np.zeros((3, 3), dtype=np.double),
            np.array(
                (
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                    (-5442559.838214509, 2967814.4198892456, -1510590.1708055406),
                )
            ),
            1.0e-8,
            "ned_to_ecef with 2 dimensional NumPy array multiple rows single 2d ref",
        ),
    ],
)
def ned_to_ecef_numpy_2d_with_out(kv, ref, out, want, approx, desc):
    """Unit test for ned_to_ecef function with out."""
    ned_to_ecef(kv, ref, out)
    assert out.shape == kv.shape, "validate shape"
    for i in range(kv.shape[0]):
        assert out[i][0] == pytest.approx(want[i][0], abs=approx), "x: " + desc
        assert out[i][1] == pytest.approx(want[i][1], abs=approx), "y: " + desc
        assert out[i][2] == pytest.approx(want[i][2], abs=approx), "z: " + desc
