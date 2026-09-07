"""Tests for the stack_two_cones go_home helper."""

import math

import pytest

from marvinpro_deploy.config import JOINT_LOWER, JOINT_UPPER
from marvinpro_deploy.go_home import (
    MAX_MOVE_TIMEOUT_S,
    MODE_RAMP_S,
    STACK_TWO_CONES_HOME,
    move_timeout_s,
    trapezoid_time,
)


def test_home_pose_has_14_finite_joints():
    assert len(STACK_TWO_CONES_HOME) == 14
    assert all(math.isfinite(v) for v in STACK_TWO_CONES_HOME)


def test_home_pose_within_urdf_limits():
    for value, lower, upper in zip(STACK_TWO_CONES_HOME, JOINT_LOWER, JOINT_UPPER):
        assert lower < value < upper


def test_trapezoid_time_triangular_profile():
    # Short move: accelerate then decelerate, never reaching the velocity limit.
    distance = 0.04  # < vel^2/acc = 0.08
    assert trapezoid_time(distance) == pytest.approx(2.0 * math.sqrt(distance / 0.5))


def test_trapezoid_time_trapezoidal_profile():
    distance = 1.0
    assert trapezoid_time(distance) == pytest.approx(distance / 0.2 + 0.2 / 0.5)


def test_trapezoid_time_is_symmetric_and_rejects_bad_input():
    assert trapezoid_time(-0.5) == trapezoid_time(0.5)
    with pytest.raises(ValueError):
        trapezoid_time(float("nan"))
    with pytest.raises(ValueError):
        trapezoid_time(0.1, vel_limit=0.0)


def test_move_timeout_covers_slowest_joint_and_is_capped():
    at_home = move_timeout_s(STACK_TWO_CONES_HOME)
    assert at_home == pytest.approx(MODE_RAMP_S + 10.0)
    far = move_timeout_s(tuple(0.0 for _ in STACK_TWO_CONES_HOME))
    assert MODE_RAMP_S + 10.0 < far <= MAX_MOVE_TIMEOUT_S


def test_parser_flags():
    from marvinpro_deploy.go_home import _build_parser

    args = _build_parser().parse_args([])
    assert args.check is False and args.keep_grippers is False
    args = _build_parser().parse_args(["--check", "--keep-grippers"])
    assert args.check is True and args.keep_grippers is True
