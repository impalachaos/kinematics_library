# pylint:disable=invalid-name
"""Unit test for KinematicVector class.
"""
import numpy as np
import pytest
from kinematics_library import KinematicVector


@pytest.mark.parametrize(
    "args,want,desc",
    [
        ((), [0.0, 0.0, 0.0], "empty constructor"),
        ((1, 2, 3), [1.0, 2.0, 3.0], "integer values"),
        ((1.0, 2.0, 3.0), [1.0, 2.0, 3.0], "float values"),
        (([1, 2, 3]), [1.0, 2.0, 3.0], "list of integer values"),
        (([1.0, 2.0, 3.0]), [1.0, 2.0, 3.0], "list of float values"),
        (
            (np.array((1, 2, 3)),),
            [1.0, 2.0, 3.0],
            "numpy array of integer values",
        ),
        (
            (np.array((1.0, 2.0, 3.0)),),
            [1.0, 2.0, 3.0],
            "numpy array of float values",
        ),
    ],
)
def test_constructor(args, want, desc):
    """Unit test for KinematicVector constructor."""
    val = KinematicVector(*args)
    assert val.x == want[0], "x: " + desc
    assert val.y == want[1], "y: " + desc
    assert val.z == want[2], "z: " + desc


@pytest.mark.parametrize(
    "addend1,addend2,want,desc",
    [
        (
            KinematicVector(1, 2, 3),
            KinematicVector(1, 2, 3),
            KinematicVector(2, 4, 6),
            "add vector",
        ),
        (
            KinematicVector(1, 2, 3),
            2,
            KinematicVector(3, 4, 5),
            "add integer scalar",
        ),
        (
            2,
            KinematicVector(1, 2, 3),
            KinematicVector(3, 4, 5),
            "add integer scalar first addend",
        ),
        (
            KinematicVector(1, 2, 3),
            2.0,
            KinematicVector(3, 4, 5),
            "add float scalar",
        ),
        (
            2.0,
            KinematicVector(1, 2, 3),
            KinematicVector(3, 4, 5),
            "add float scalar first addend",
        ),
    ],
)
def test_add(addend1, addend2, want, desc):
    """Unit test for KinematicVector addition."""
    out = addend1 + addend2
    assert out.x == want.x, "x: " + desc
    assert out.y == want.y, "y: " + desc
    assert out.z == want.z, "z: " + desc


@pytest.mark.parametrize(
    "vector,want,approx,desc",
    [
        (
            KinematicVector(np.sqrt(3.0), 1.0, 10.0),
            np.deg2rad(30.0),
            1.0e-8,
            "Quadrant 1",
        ),
        (
            KinematicVector(-np.sqrt(3.0), 1.0, 10.0),
            np.deg2rad(150.0),
            1.0e-8,
            "Quadrant 2",
        ),
        (
            KinematicVector(-1.0, -np.sqrt(3.0), 10.0),
            np.deg2rad(-120.0),
            1.0e-8,
            "Quadrant 3",
        ),
        (
            KinematicVector(1.0, -np.sqrt(3.0), 10.0),
            np.deg2rad(-60.0),
            1.0e-8,
            "Quadrant 4",
        ),
    ],
)
def test_azimuth_angle(vector, want, approx, desc):
    """Unit test for KinematicVector azimuth_angle function."""
    out = vector.azimuth_angle()
    assert out == pytest.approx(want, abs=approx), desc


@pytest.mark.parametrize(
    "vector1,vector2,want,approx,desc",
    [
        (
            KinematicVector(2.0, -3.0, 1.0),
            KinematicVector(4.0, -1.0, 5.0),
            KinematicVector(-14.0, -6.0, 10.0),
            1.0e-7,
            "cross 1",
        ),
        (
            KinematicVector(4.0, -1.0, 5.0),
            KinematicVector(2.0, -3.0, 1.0),
            KinematicVector(14.0, 6.0, -10.0),
            0.0,
            "cross 2",
        ),
    ],
)
def test_cross(vector1, vector2, want, approx, desc):
    """Unit test for KinematicVector cross function."""
    out = vector1.cross(vector2)
    assert out.x == pytest.approx(want.x, abs=approx), "x: " + desc
    assert out.y == pytest.approx(want.y, abs=approx), "y: " + desc
    assert out.z == pytest.approx(want.z, abs=approx), "z: " + desc


@pytest.mark.parametrize(
    "input1,input2,want,desc",
    [
        (
            KinematicVector(8.0, 12.0, 16.0),
            4,
            KinematicVector(2.0, 3.0, 4.0),
            "divide integer value",
        ),
        (
            KinematicVector(8.0, 12.0, 16.0),
            4.0,
            KinematicVector(2.0, 3.0, 4.0),
            "divide float value",
        ),
        (
            12,
            KinematicVector(1.0, 2.0, 3.0),
            KinematicVector(12.0, 6.0, 4.0),
            "divide integer value",
        ),
        (
            12.0,
            KinematicVector(1.0, 2.0, 3.0),
            KinematicVector(12.0, 6.0, 4.0),
            "divide float value",
        ),
    ],
)
def test_div(input1, input2, want, desc):
    """Unit test for KinematicVector division."""
    out = input1 / input2
    assert out.x == want.x, "x: " + desc
    assert out.y == want.y, "y: " + desc
    assert out.z == want.z, "z: " + desc


@pytest.mark.parametrize(
    "vector1,vector2,want,approx,desc",
    [
        (
            KinematicVector(2.0, -7.0, -1.0),
            KinematicVector(8.0, 2.0, -4.0),
            6.0,
            0.0,
            "dot 1",
        ),
        (
            KinematicVector(8.0, 2.0, -4.0),
            KinematicVector(2.0, -7.0, -1.0),
            6.0,
            0.0,
            "dot 2",
        ),
    ],
)
def test_dot(vector1, vector2, want, approx, desc):
    """Unit test for KinematicVector dot function."""
    out = vector1.dot(vector2)
    assert out == pytest.approx(want, abs=approx), desc


@pytest.mark.parametrize(
    "vector,want,approx,desc",
    [
        (
            KinematicVector(3.0, 4.0, 5.0),
            np.deg2rad(-45.0),
            1.0e-8,
            "Above XY Plane",
        ),
        (
            KinematicVector(3.0, 4.0, 0.0),
            np.deg2rad(0.0),
            1.0e-8,
            "On XY Plane",
        ),
        (
            KinematicVector(3.0, 4.0, -5.0),
            np.deg2rad(45.0),
            1.0e-8,
            "Below XY Plane",
        ),
    ],
)
def test_elevation_angle(vector, want, approx, desc):
    """Unit test for KinematicVector elevation_angle function."""
    out = vector.elevation_angle()
    assert out == pytest.approx(want, abs=approx), desc


@pytest.mark.parametrize(
    "vector,want,approx,desc",
    [
        (
            KinematicVector(6.0, 4.0, -12.0),
            14.0,
            0.0,
            "magnitude 1",
        ),
    ],
)
def test_magnitude(vector, want, approx, desc):
    """Unit test for KinematicVector magnitude function."""
    out = vector.magnitude()
    assert out == pytest.approx(want, abs=approx), desc


@pytest.mark.parametrize(
    "input1,input2,want,desc",
    [
        (
            KinematicVector(1.0, 2.0, 3.0),
            4,
            KinematicVector(4.0, 8.0, 12.0),
            "multiply integer value",
        ),
        (
            KinematicVector(1.0, 2.0, 3.0),
            4.0,
            KinematicVector(4.0, 8.0, 12.0),
            "multiply integer value",
        ),
    ],
)
def test_mul(input1, input2, want, desc):
    """Unit test for KinematicVector multiplication."""
    out = input1 * input2
    assert out.x == want.x, "x: " + desc
    assert out.y == want.y, "y: " + desc
    assert out.z == want.z, "z: " + desc


@pytest.mark.parametrize(
    "vector,want,approx,desc",
    [
        (
            KinematicVector(0.0, 0.0, 5.0),
            np.deg2rad(0.0),
            1.0e-7,
            "on z axis",
        ),
        (
            KinematicVector(5.0, 0.0, 5.0),
            np.deg2rad(45.0),
            1.0e-8,
            "above XY plane but off z axis",
        ),
        (
            KinematicVector(5.0, 0.0, 0.0),
            np.deg2rad(90.0),
            1.0e-8,
            "in XY plane",
        ),
        (
            KinematicVector(5.0, 0.0, -5.0),
            np.deg2rad(135.0),
            1.0e-8,
            "below XY plane but off z axis",
        ),
        (
            KinematicVector(0.0, 0.0, -5.0),
            np.deg2rad(180.0),
            1.0e-7,
            "along negative z axis",
        ),
    ],
)
def test_polar_angle(vector, want, approx, desc):
    """Unit test for KinematicVector polar_angle function."""
    out = vector.polar_angle()
    assert out == pytest.approx(want, abs=approx), desc


@pytest.mark.parametrize(
    "input1,input2,want,desc",
    [
        (
            KinematicVector(1.0, 2.0, 3.0),
            KinematicVector(5.0, 6.0, 7.0),
            KinematicVector(-4.0, -4.0, -4.0),
            "sub vectors",
        ),
        (
            KinematicVector(1.0, 2.0, 3.0),
            4,
            KinematicVector(-3.0, -2.0, -1.0),
            "sub integer value",
        ),
        (
            KinematicVector(1.0, 2.0, 3.0),
            4.0,
            KinematicVector(-3.0, -2.0, -1.0),
            "sub float value",
        ),
        (
            4,
            KinematicVector(1.0, 2.0, 3.0),
            KinematicVector(3.0, 2.0, 1.0),
            "sub integer value 2",
        ),
        (
            4.0,
            KinematicVector(1.0, 2.0, 3.0),
            KinematicVector(3.0, 2.0, 1.0),
            "sub float value 2",
        ),
    ],
)
def test_sub(input1, input2, want, desc):
    """Unit test for KinematicVector subtraction."""
    out = input1 - input2
    assert out.x == want.x, "x: " + desc
    assert out.y == want.y, "y: " + desc
    assert out.z == want.z, "z: " + desc


@pytest.mark.parametrize(
    "vector,want,desc",
    [
        (
            KinematicVector(6.0, 4.0, -12.0),
            KinematicVector(0.42857142857142855, 0.2857142857142857, -0.8571428571428571),
            "unit vector",
        ),
    ],
)
def test_unit(vector, want, desc):
    """Unit test for KinematicVector unit function."""
    out = vector.unit()
    assert out.x == want.x, "x: " + desc
    assert out.y == want.y, "y: " + desc
    assert out.z == want.z, "z: " + desc


@pytest.mark.parametrize(
    "vector,want,desc",
    [
        (
            KinematicVector(6.0, 4.0, -12.0),
            KinematicVector(0, 0, 0),
            "unit vector",
        ),
    ],
)
def test_zero(vector, want, desc):
    """Unit test for KinematicVector zero function."""
    vector.zero()
    assert vector.x == want.x, "x: " + desc
    assert vector.y == want.y, "y: " + desc
    assert vector.z == want.z, "z: " + desc
