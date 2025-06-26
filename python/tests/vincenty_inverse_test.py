"""
Unit tests for `kinematics_library.vincenty_inverse`.
"""

from dataclasses import dataclass

import numpy as np
import pytest
from typing import Iterable
from kinematics_library import vincenty_inverse


@dataclass
class VincentyInverseArgs:
    lat_a_deg: float
    lon_a_deg: float
    lat_b_deg: float
    lon_b_deg: float
    abs_tol: float


@dataclass
class VincentyInverseNumpyArgs:
    lat_a_deg: np.ndarray
    lon_a_deg: np.ndarray
    lat_b_deg: np.ndarray
    lon_b_deg: np.ndarray
    abs_tol: float


@dataclass
class VincentyInverseIterableArgs:
    lat_a_deg: Iterable
    lon_a_deg: Iterable
    lat_b_deg: Iterable
    lon_b_deg: Iterable
    abs_tol: float


@dataclass
class VincentyInverseReturns:
    range_m: float
    bearing_ab_deg: float
    bearing_ba_deg: float


@dataclass
class VincentyInverseNumpyReturns:
    range_bearing_bearing: np.ndarray


@dataclass
class VincentyInverseIterableReturns:
    range_bearing_bearing: np.ndarray


@pytest.mark.parametrize(
    "args,returns,a_tol,desc",
    [
        (
            VincentyInverseArgs(45.0, 120.0, 45.0, 120.0, 1.0e-13),
            VincentyInverseReturns(0.0, 0.0, 0.0),
            1.0e-9,
            "Vincenty inverse edge case evaulation test: Same lat-lon locations.",
        ),
        (
            VincentyInverseArgs(90.0, 0.0, -90.0, 0.0, 1.0e-13),
            VincentyInverseReturns(20003931.457914233, 180.0, 0.0),
            1.0e-9,
            "Vincenty inverse edge case evaulation test: Anti-podal locations.",
        ),
        (
            VincentyInverseArgs(0.0, -10.0, 0.0, 10.0, 1.0e-13),
            VincentyInverseReturns(2226389.8158654678, 90.0, -90.0),
            1.0e-9,
            "Vincenty inverse edge case evaulation test: Both locations on equator.",
        ),
        (
            VincentyInverseArgs(-60.0, 24.0, -64.0, 26.0, 1.0e-13),
            VincentyInverseReturns(457876.09259014280, 167.65057682653136, -14.116435240448425),
            1.0e-9,
            "Vincenty inverse random evaulation test: Random test 1.",
        ),
        (
            VincentyInverseArgs(18.0, -175.0, -30.0, 150.0, 1.0e-13),
            VincentyInverseReturns(6503644.0543462737, -144.26832642124467, 39.866234335863595),
            1.0e-9,
            "Vincenty inverse random evaulation test: Random test 2.",
        ),
    ],
)
def test_vincenty_inverse_is_correct(
    args: VincentyInverseArgs, returns: VincentyInverseReturns, a_tol: float, desc: str
) -> None:
    """Unit test to validate correctness of `vincenty_inverse`."""
    out = vincenty_inverse(args.lat_a_deg, args.lon_a_deg, args.lat_b_deg, args.lon_b_deg, args.abs_tol)

    assert out[0] == pytest.approx(returns.range_m, abs=a_tol), "Incorrect range_m: " + desc
    assert out[1] == pytest.approx(returns.bearing_ab_deg, abs=a_tol), "Incorrect bearing_ab_deg: " + desc
    assert out[2] == pytest.approx(returns.bearing_ba_deg, abs=a_tol), "Incorrect bearing_ba_deg: " + desc


@pytest.mark.parametrize(
    "args,returns,a_tol,desc",
    [
        (
            VincentyInverseNumpyArgs(
                np.array((-60.0, 15.0, -8.0)),
                np.array((24.0, 175.0, -42.0)),
                np.array((-64.0, 19.0, 80.0)),
                np.array((26.0, -120.0, -15.0)),
                1.0e-13,
            ),
            VincentyInverseNumpyReturns(
                np.array(
                    [
                        (457876.0925901428, 167.6505768264915, -14.11643524048833),
                        (6898025.678811164, 76.179180677647, -82.69177816865684),
                        (9889875.799183948, 4.540670115245595, -153.25456188600506),
                    ],
                )
            ),
            1.0e-9,
            "Vincenty inverse edge case evaulation test: Same lat-lon locations.",
        ),
    ],
)
def test_vincenty_inverse_is_correct_numpy(
    args: VincentyInverseNumpyArgs, returns: VincentyInverseNumpyReturns, a_tol: float, desc: str
) -> None:
    """Unit test to validate correctness of `vincenty_inverse`."""
    out = vincenty_inverse(args.lat_a_deg, args.lon_a_deg, args.lat_b_deg, args.lon_b_deg, args.abs_tol)

    assert out.shape == returns.range_bearing_bearing.shape, "Incorrect output shape: " + desc
    assert np.allclose(out, returns.range_bearing_bearing, a_tol), "Incorrect output elements: " + desc


@pytest.mark.parametrize(
    "args,returns,a_tol,desc",
    [
        (
            VincentyInverseIterableArgs(
                [-60.0, 15.0, -8.0],
                [24.0, 175.0, -42.0],
                [-64.0, 19.0, 80.0],
                [26.0, -120.0, -15.0],
                1.0e-13,
            ),
            VincentyInverseIterableReturns(
                [
                    (457876.0925901428, 167.6505768264915, -14.11643524048833),
                    (6898025.678811164, 76.179180677647, -82.69177816865684),
                    (9889875.799183948, 4.540670115245595, -153.25456188600506),
                ]
            ),
            1.0e-9,
            "Vincenty inverse edge case evaulation test: Same lat-lon locations.",
        ),
    ],
)
def test_vincenty_inverse_is_correct_iterable(
    args: VincentyInverseIterableArgs, returns: VincentyInverseIterableReturns, a_tol: float, desc: str
) -> None:
    """Unit test to validate correctness of `vincenty_inverse`."""
    out = vincenty_inverse(args.lat_a_deg, args.lon_a_deg, args.lat_b_deg, args.lon_b_deg, args.abs_tol)

    assert len(out) == len(returns.range_bearing_bearing), "Incorrect output shape: " + desc
    assert np.allclose(out, returns.range_bearing_bearing, a_tol), "Incorrect output elements: " + desc
