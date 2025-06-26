# pylint:disable=invalid-name
"""Unit test for DCM class.
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
                -0.5556991462506127,
                -0.09258385044673502,
                -0.8262122545041296,
                -0.6571921829277988,
                0.6575954307136367,
                0.3683295863803792,
                0.5092120320209791,
                0.7476606717896851,
                -0.4262706022048224,
            ],
            1.0e-14,
            "Sequence body xyx",
        ),
        (
            (-0.45, -0.11, -2.38, RotationSequence.BODY_XYZ.value),
            [
                -0.7193636778586622,
                -0.6559343418507525,
                0.2286176680803103,
                0.6859042379537842,
                -0.6187367489848085,
                0.3830146365515926,
                -0.1097783008371748,
                0.4323366450308489,
                0.8950048882708811,
            ],
            1.0e-14,
            "Sequence body xyz",
        ),
        (
            (0.52, -1.56, -1.32, RotationSequence.BODY_XZX.value),
            [
                0.01079611705826739,
                -0.8677686033754274,
                -0.4968511797835689,
                0.2481609880441226,
                0.4836604675055965,
                -0.8393382370565711,
                0.9686586436250022,
                -0.1142374858272359,
                0.2205679690309612,
            ],
            1.0e-14,
            "Sequence body xzx",
        ),
        (
            (3.03, 1.45, -0.98, RotationSequence.BODY_XZY.value),
            [
                0.06712275948539267,
                -0.6420092590059827,
                -0.7637530009824218,
                -0.9927129910375885,
                -0.1197532419754337,
                0.01341933163678318,
                -0.1000772330965983,
                0.7572867834492837,
                -0.6453690998403679,
            ],
            1.0e-14,
            "Sequence body xzy",
        ),
        (
            (2.39, 2.00, -1.50, RotationSequence.BODY_YXY.value),
            [
                -0.3351153743894538,
                -0.9070196245905846,
                0.2549766390385037,
                0.6208712127298156,
                -0.4161468365471424,
                -0.6643348159137937,
                0.7086724370618802,
                -0.06432115545729157,
                0.7025995772197806,
            ],
            1.0e-14,
            "Sequence body yxy",
        ),
        (
            (-1.18, -2.13, -2.02, RotationSequence.BODY_YXZ.value),
            [
                -0.8714288779471885,
                0.4778810009563105,
                -0.1106411298057783,
                0.002784518557860272,
                0.2303736709597711,
                0.9730982572098874,
                0.490513972416796,
                0.8476778401335698,
                -0.2020849381086598,
            ],
            1.0e-14,
            "Sequence body yxz",
        ),
        (
            (-0.18, 1.23, 1.26, RotationSequence.BODY_YZX.value),
            [
                0.3288376797232791,
                0.9424888019316975,
                0.05983843770991672,
                -0.4540246228471165,
                0.1022155483726017,
                0.8851065605447865,
                0.8280866031054624,
                -0.3182245117904053,
                0.4615254465931586,
            ],
            1.0e-14,
            "Sequence body yzx",
        ),
        (
            (-1.13, 0.19, 0.97, RotationSequence.BODY_YZY.value),
            [
                0.9828868742370757,
                0.1067618447856784,
                0.1501176237169703,
                -0.0805784998565755,
                0.9820042351172703,
                -0.1708062866893626,
                -0.1656517365237534,
                0.1557870043030021,
                0.9738017824367347,
            ],
            1.0e-14,
            "Sequence body yzy",
        ),
        (
            (2.94, 0.20, -1.10, RotationSequence.BODY_ZXY.value),
            [
                -0.4089584776574394,
                0.2642935484074433,
                0.8734425475223383,
                -0.1962387159074697,
                -0.96021917465776,
                0.1986693307950612,
                0.8912033044884298,
                -0.09015573686556229,
                0.4445543984476258,
            ],
            1.0e-14,
            "Sequence body zxy",
        ),
        (
            (-0.48, -2.57, -1.47, RotationSequence.BODY_ZXZ.value),
            [
                0.4756582297157966,
                0.6957453223663198,
                0.5382264346063096,
                0.8434123305557789,
                -0.5345020040709673,
                -0.05443572641739133,
                0.2498097059165311,
                0.4798396128390249,
                -0.8410404608462014,
            ],
            1.0e-14,
            "Sequence body zxz",
        ),
        (
            (0.17, -0.27, 2.36, RotationSequence.BODY_ZYX.value),
            [
                0.9498779142489947,
                0.1630530242095858,
                0.2667314366888311,
                -0.06509566707865096,
                -0.7313481482599988,
                0.6788905951361062,
                0.3057687069494934,
                -0.6622262433132495,
                -0.6840774082789577,
            ],
            1.0e-14,
            "Sequence body zyx",
        ),
        (
            (2.88, -1.63, 1.11, RotationSequence.BODY_ZYZ.value),
            [
                -0.2062298843675349,
                -0.8720307322246085,
                0.4438824583710945,
                -0.1661927342547344,
                -0.4158276007932318,
                -0.8941293986328153,
                0.9642868918919765,
                -0.2581662419340645,
                -0.05916909371414814,
            ],
            1.0e-14,
            "Sequence body zyz",
        ),
        (
            (Quaternion(3.03, -2.16, 2.23, RotationSequence.BODY_XYX.value),),
            [
                -0.5556991462506127,
                -0.09258385044673502,
                -0.8262122545041296,
                -0.6571921829277988,
                0.6575954307136367,
                0.3683295863803792,
                0.5092120320209791,
                0.7476606717896851,
                -0.4262706022048224,
            ],
            1.0e-14,
            "Sequence body xyx quaternion",
        ),
    ],
)
def test_constructor(args, want, approx, desc):
    """Unit test for DCM constructor."""
    val = DCM(*args)
    assert val.e00 == pytest.approx(want[0], approx), "e00: " + desc
    assert val.e01 == pytest.approx(want[1], approx), "e01: " + desc
    assert val.e02 == pytest.approx(want[2], approx), "e02: " + desc
    assert val.e10 == pytest.approx(want[3], approx), "e10: " + desc
    assert val.e11 == pytest.approx(want[4], approx), "e11: " + desc
    assert val.e12 == pytest.approx(want[5], approx), "e12: " + desc
    assert val.e20 == pytest.approx(want[6], approx), "e20: " + desc
    assert val.e21 == pytest.approx(want[7], approx), "e21: " + desc
    assert val.e22 == pytest.approx(want[8], approx), "e22: " + desc


@pytest.mark.parametrize(
    "dcm1,kv,want,approx,desc",
    [
        (
            DCM(1.06, 2.40, -1.46, RotationSequence.BODY_XYZ.value),
            KinematicVector(1.0, 2.0, 3.0),
            [
                0.47911151037533495,
                2.7884573495894838,
                -2.448460285597105,
            ],
            1.0e-12,
            "Rotate",
        ),
    ],
)
def test_rotate(dcm1, kv, want, approx, desc):
    """Unit test for DCM rotate function."""
    out = dcm1.rotate(kv)
    assert out.x == pytest.approx(want[0], approx), "x: " + desc
    assert out.y == pytest.approx(want[1], approx), "y: " + desc
    assert out.z == pytest.approx(want[2], approx), "z: " + desc


@pytest.mark.parametrize(
    "dcm1,kv,want,approx,desc",
    [
        (
            DCM(1.06, 2.40, -1.46, RotationSequence.BODY_XYZ.value),
            KinematicVector(1.0, 2.0, 3.0),
            [
                -3.6335322684694438,
                -0.14870390750411255,
                0.8805284787468857,
            ],
            1.0e-12,
            "Rotate",
        ),
    ],
)
def test_transform(dcm1, kv, want, approx, desc):
    """Unit test for DCM transform function."""
    out = dcm1.transform(kv)
    assert out.x == pytest.approx(want[0], approx), "x: " + desc
    assert out.y == pytest.approx(want[1], approx), "y: " + desc
    assert out.z == pytest.approx(want[2], approx), "z: " + desc


@pytest.mark.parametrize(
    "dcm1,want,approx,desc",
    [
        (
            DCM(-0.45, -0.11, -2.38, RotationSequence.BODY_XYZ.value),
            [
                -0.7193636778586622,
                0.6859042379537842,
                -0.1097783008371748,
                -0.6559343418507525,
                -0.6187367489848085,
                0.4323366450308489,
                0.2286176680803103,
                0.3830146365515926,
                0.8950048882708811,
            ],
            1.0e-12,
            "Rotate",
        ),
    ],
)
def test_transpose(dcm1, want, approx, desc):
    """Unit test for DCM transpose function."""
    out = dcm1.transpose()
    assert out.e00 == pytest.approx(want[0], approx), "e00: " + desc
    assert out.e01 == pytest.approx(want[1], approx), "e01: " + desc
    assert out.e02 == pytest.approx(want[2], approx), "e02: " + desc
    assert out.e10 == pytest.approx(want[3], approx), "e10: " + desc
    assert out.e11 == pytest.approx(want[4], approx), "e11: " + desc
    assert out.e12 == pytest.approx(want[5], approx), "e12: " + desc
    assert out.e20 == pytest.approx(want[6], approx), "e20: " + desc
    assert out.e21 == pytest.approx(want[7], approx), "e21: " + desc
    assert out.e22 == pytest.approx(want[8], approx), "e22: " + desc
