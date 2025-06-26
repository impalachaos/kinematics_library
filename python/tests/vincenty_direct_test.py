"""
Unit tests for `kinematics_library.vincenty_direct`.
"""

from dataclasses import dataclass

from typing import Iterable
import numpy as np
import pytest
from kinematics_library import vincenty_direct


@dataclass
class VincentyDirectArgs:
    lat_deg: float
    lon_deg: float
    range_m: float
    bearing_deg: float
    abs_tol: float


@dataclass
class VincentyDirectNumpyArgs:
    lat_deg: np.ndarray
    lon_deg: np.ndarray
    range_m: np.ndarray
    bearing_deg: np.ndarray
    abs_tol: float


@dataclass
class VincentyDirectIterableArgs:
    lat_deg: Iterable
    lon_deg: Iterable
    range_m: Iterable
    bearing_deg: Iterable
    abs_tol: float


@dataclass
class VincentyDirectReturns:
    lat_deg: float
    lon_deg: float


@dataclass
class VincentyDirectNumpyReturns:
    lat_lon: np.ndarray


@dataclass
class VincentyDirectIterableReturns:
    lat_lon: Iterable


@pytest.mark.parametrize(
    "args,returns,a_tol,desc",
    [
        (
            VincentyDirectArgs(45.0, 120.0, 0.0, 0.0, 1.0e-13),
            VincentyDirectReturns(45.0, 120.0),
            1.0e-9,
            "Vincenty direct edge case evaulation test: Same lat-lon locations.",
        ),
        (
            VincentyDirectArgs(45.0, 120.0, 3000.0, 30.0, 1.0e-13),
            VincentyDirectReturns(45.02337670419947, 120.0190319660863),
            1.0e-9,
            "Vincenty direct random evaulation test: Random test 1.",
        ),
        (
            VincentyDirectArgs(-10.0, -80.0, 840000.0, -113.0, 1.0e-13),
            VincentyDirectReturns(-12.884359520148694, -87.12194168465778),
            1.0e-9,
            "Vincenty direct random evaulation test: Random test 2.",
        ),
        (
            VincentyDirectArgs(-60.0, 24.0, 12000000.0, -45.0, 1.0e-13),
            VincentyDirectReturns(37.34976350706342, -33.49808323545486),
            1.0e-9,
            "Vincenty direct random evaulation test: Random test 3.",
        ),
    ],
)
def test_vincenty_direct_is_correct(
    args: VincentyDirectArgs, returns: VincentyDirectReturns, a_tol: float, desc: str
) -> None:
    """Unit test to validate correctness of `vincenty_direct`."""
    out = vincenty_direct(args.lat_deg, args.lon_deg, args.range_m, args.bearing_deg, args.abs_tol)

    assert out[0] == pytest.approx(returns.lat_deg, abs=a_tol), "Incorrect lat_deg: " + desc
    assert out[1] == pytest.approx(returns.lon_deg, abs=a_tol), "Incorrect lon_deg: " + desc


@pytest.mark.parametrize(
    "args,returns,a_tol,desc",
    [
        (
            VincentyDirectNumpyArgs(
                np.array((45.0, 15.0, -8.0)),
                np.array((120.0, 175.0, -42.0)),
                np.array((50000.0, 120000.0, 10000000.0)),
                np.array((30.0, -135.0, 60.0)),
                1.0e-13,
            ),
            VincentyDirectNumpyReturns(
                np.array(
                    [
                        (45.38918149119183, 120.31923736117137),
                        (14.23176865941303, 174.21380049082546),
                        (29.780585820076507, 43.28342045071895),
                    ]
                )
            ),
            1.0e-9,
            "Vincenty direct edge case evaulation test: Same lat-lon locations.",
        ),
    ],
)
def test_vincenty_direct_is_correct_numpy(
    args: VincentyDirectNumpyArgs, returns: VincentyDirectNumpyReturns, a_tol: float, desc: str
) -> None:
    """Unit test to validate correctness of `vincenty_direct`."""
    out = vincenty_direct(args.lat_deg, args.lon_deg, args.range_m, args.bearing_deg, args.abs_tol)

    assert out.shape == returns.lat_lon.shape, "Incorrect output shape: " + desc
    assert np.allclose(out, returns.lat_lon, a_tol), "Incorrect output elements: " + desc


@pytest.mark.parametrize(
    "args,returns,a_tol,desc",
    [
        (
            VincentyDirectIterableArgs(
                [45.0, 15.0, -8.0],
                [120.0, 175.0, -42.0],
                [50000.0, 120000.0, 10000000.0],
                [30.0, -135.0, 60.0],
                1.0e-13,
            ),
            VincentyDirectIterableReturns(
                [
                    (45.38918149119183, 120.31923736117137),
                    (14.23176865941303, 174.21380049082546),
                    (29.780585820076507, 43.28342045071895),
                ]
            ),
            1.0e-9,
            "Vincenty direct edge case evaulation test: Same lat-lon locations.",
        ),
    ],
)
def test_vincenty_direct_is_correct_iterable(
    args: VincentyDirectIterableArgs, returns: VincentyDirectIterableReturns, a_tol: float, desc: str
) -> None:
    """Unit test to validate correctness of `vincenty_direct`."""
    out = vincenty_direct(args.lat_deg, args.lon_deg, args.range_m, args.bearing_deg, args.abs_tol)

    assert len(out) == len(returns.lat_lon), "Incorrect output shape: " + desc
    assert np.allclose(out, returns.lat_lon, a_tol), "Incorrect output elements: " + desc
