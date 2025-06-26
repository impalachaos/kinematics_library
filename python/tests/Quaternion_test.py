# pylint:disable=invalid-name
"""Unit tests for the Quaternion class.
"""

import pytest
from kinematics_library import DCM, KinematicVector, Quaternion, RotationSequence


@pytest.mark.parametrize(
    "args,want,approx,desc",
    [
        ((), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], 0.0, "empty constructor"),
        (
            (3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            [
                0.4109822630778004,
                -0.2307466279496697,
                0.8123369342780543,
                0.3434505471433091,
            ],
            1.0e-14,
            "Sequence body xyx",
        ),
        (
            (-0.45, -0.11, -2.38, RotationSequence.BODY_XYZ.value),
            [
                -0.37313015873399,
                0.03304611495798355,
                0.2267278327659468,
                0.8990418948962212,
            ],
            1.0e-14,
            "Sequence body xyz",
        ),
        (
            (0.52, -1.56, -1.32, RotationSequence.BODY_XZX.value),
            [
                0.654794729971696,
                -0.2768427714975188,
                0.5595302452540809,
                -0.426060847904116,
            ],
            1.0e-14,
            "Sequence body xzx",
        ),
        (
            (3.03, 1.45, -0.98, RotationSequence.BODY_XZY.value),
            [
                -0.2747728232875256,
                0.6768022424056369,
                -0.6038404380255478,
                -0.3190851699192839,
            ],
            1.0e-14,
            "Sequence body xzy",
        ),
        (
            (2.39, 2.00, -1.50, RotationSequence.BODY_YXY.value),
            [
                -0.4876826238147061,
                0.3075840880710135,
                -0.2325773853056927,
                0.7832403507475174,
            ],
            1.0e-14,
            "Sequence body yxy",
        ),
        (
            (-1.18, -2.13, -2.02, RotationSequence.BODY_YXZ.value),
            [
                0.198027684241322,
                0.1583369738893134,
                0.7589281071049516,
                0.5997854343177145,
            ],
            1.0e-14,
            "Sequence body yxz",
        ),
        (
            (-0.18, 1.23, 1.26, RotationSequence.BODY_YZX.value),
            [
                0.6878551218623439,
                0.4373490267388047,
                0.2792187413373984,
                0.5075608876029747,
            ],
            1.0e-14,
            "Sequence body yzx",
        ),
        (
            (-1.13, 0.19, 0.97, RotationSequence.BODY_YZY.value),
            [
                0.9923070205071465,
                -0.08228131118770329,
                -0.0795543500436339,
                0.04719818079753894,
            ],
            1.0e-14,
            "Sequence body yzy",
        ),
        (
            (2.94, 0.20, -1.10, RotationSequence.BODY_ZXY.value),
            [
                0.1372741291471433,
                0.5260005462337218,
                0.03234541911945765,
                0.8387091347366536,
            ],
            1.0e-14,
            "Sequence body zxy",
        ),
        (
            (-0.48, -2.57, -1.47, RotationSequence.BODY_ZXZ.value),
            [
                -0.1582053766458555,
                0.8442749396128262,
                0.4557631586305099,
                0.2333470127883412,
            ],
            1.0e-14,
            "Sequence body zxz",
        ),
        (
            (0.17, -0.27, 2.36, RotationSequence.BODY_ZYX.value),
            [
                0.3655312427515732,
                0.917238173920486,
                0.02669899703155687,
                0.1560391182781152,
            ],
            1.0e-14,
            "Sequence body zyx",
        ),
        (
            (2.88, -1.63, 1.11, RotationSequence.BODY_ZYZ.value),
            [
                -0.2823001156239072,
                0.5631977472744021,
                -0.460860981557468,
                0.6250776734627902,
            ],
            1.0e-14,
            "Sequence body zyz",
        ),
        (
            (DCM(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),),
            [
                0.4109822630778004,
                -0.2307466279496697,
                0.8123369342780543,
                0.3434505471433091,
            ],
            1.0e-14,
            "Sequence body xyx DCM",
        ),
    ],
)
def test_constructor(args, want, approx, desc):
    """Unit test for Quaterion constructor."""
    val = Quaternion(*args)
    assert val.a == pytest.approx(want[0], approx), "a: " + desc
    assert val.b == pytest.approx(want[1], approx), "b: " + desc
    assert val.c == pytest.approx(want[2], approx), "c: " + desc
    assert val.d == pytest.approx(want[3], approx), "d: " + desc


@pytest.mark.parametrize(
    "quat,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            2.294530123475165,
            0.0,
            "angle",
        ),
    ],
)
def test_angle(quat, want, approx, desc):
    """Unit test for Quaterion angle function."""
    out = quat.angle()
    assert out == pytest.approx(want, approx), desc


@pytest.mark.parametrize(
    "quat,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            [-0.25311063857503524, 0.8910687969752371, 0.376737844790302],
            0.0,
            "axis",
        ),
    ],
)
def test_axis(quat, want, approx, desc):
    """Unit test for Quaterion axis function."""
    out = quat.axis()
    assert out.x == pytest.approx(want[0], approx), "x: " + desc
    assert out.y == pytest.approx(want[1], approx), "y: " + desc
    assert out.z == pytest.approx(want[2], approx), "z: " + desc


@pytest.mark.parametrize(
    "quat,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            [0.4109822630778004, 0.23074662794966966, -0.8123369342780543, -0.3434505471433091],
            0.0,
            "conjugate",
        ),
    ],
)
def test_conjugate(quat, want, approx, desc):
    """Unit test for Quaterion conjugate function."""
    out = quat.conjugate()
    assert out.a == pytest.approx(want[0], approx), "a: " + desc
    assert out.b == pytest.approx(want[1], approx), "b: " + desc
    assert out.c == pytest.approx(want[2], approx), "c: " + desc
    assert out.d == pytest.approx(want[3], approx), "d: " + desc


@pytest.mark.parametrize(
    "quat,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            [0.4109822630778004, 0.23074662794966966, -0.8123369342780543, -0.3434505471433091],
            0.0,
            "inverse",
        ),
    ],
)
def test_inverse(quat, want, approx, desc):
    """Unit test for Quaterion inverse function."""
    out = quat.inverse()
    assert out.a == pytest.approx(want[0], approx), "a: " + desc
    assert out.b == pytest.approx(want[1], approx), "b: " + desc
    assert out.c == pytest.approx(want[2], approx), "c: " + desc
    assert out.d == pytest.approx(want[3], approx), "d: " + desc


@pytest.mark.parametrize(
    "quat1,quat2,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            Quaternion(-0.45, -0.11, -2.38, RotationSequence.BODY_XYZ.value),
            [-0.6386804207571435, 0.7521350314970022, 0.00887430047628518, 0.1621772528054969],
            0.0,
            "mul",
        ),
    ],
)
def test_mul(quat1, quat2, want, approx, desc):
    """Unit test for Quaterion multiplication."""
    out = quat1 * quat2
    assert out.a == pytest.approx(want[0], approx), "a: " + desc
    assert out.b == pytest.approx(want[1], approx), "b: " + desc
    assert out.c == pytest.approx(want[2], approx), "c: " + desc
    assert out.d == pytest.approx(want[3], approx), "d: " + desc


@pytest.mark.parametrize(
    "quat,kv,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            KinematicVector(1, 2, 3),
            [-0.3424474160432729, 3.465589026349593, -1.368364888357839],
            0.0,
            "rotate",
        ),
    ],
)
def test_rotate(quat, kv, want, approx, desc):
    """Unit test for Quaterion rotate function."""
    out = quat.rotate(kv)
    assert out.x == pytest.approx(want[0], approx), "x: " + desc
    assert out.y == pytest.approx(want[1], approx), "y: " + desc
    assert out.z == pytest.approx(want[2], approx), "z: " + desc


@pytest.mark.parametrize(
    "quat,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            1.0,
            0.0,
            "square",
        ),
    ],
)
def test_square(quat, want, approx, desc):
    """Unit test for Quaterion square function."""
    out = quat.square()
    assert out == pytest.approx(want, approx), desc


@pytest.mark.parametrize(
    "quat,kv,want,approx,desc",
    [
        (
            Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),
            KinematicVector(1, 2, 3),
            [-3.2195036106564716, 1.7629874376406116, 0.7257215689858816],
            0.0,
            "transform",
        ),
    ],
)
def test_transform(quat, kv, want, approx, desc):
    """Unit test for Quaterion transform function."""
    out = quat.transform(kv)
    assert out.x == pytest.approx(want[0], approx), "x: " + desc
    assert out.y == pytest.approx(want[1], approx), "y: " + desc
    assert out.z == pytest.approx(want[2], approx), "z: " + desc
