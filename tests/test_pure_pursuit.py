"""Unit tests for the Pure Pursuit tracker."""

import math
import pytest

from src.control.pure_pursuit import PurePursuit


def test_no_path_returns_none():
    pp = PurePursuit()
    assert pp.query((0.0, 0.0), 0.0) is None


def test_reaches_goal_terminates():
    pp = PurePursuit(goal_tolerance=0.5)
    pp.set_path([(0.0, 0.0), (5.0, 0.0)])
    assert pp.query((5.0, 0.0), 0.0) is None  # at goal
    assert pp.query((4.7, 0.1), 0.0) is None  # within tolerance


def test_straight_path_steers_forward():
    pp = PurePursuit(lookahead_base=1.0, lookahead_gain=0.0)
    pp.set_path([(0.0, 0.0), (10.0, 0.0)])
    res = pp.query(pos=(0.0, 0.0), heading=0.0, speed_mps=0.0)
    assert res is not None
    assert abs(res.curvature) < 1e-3       # straight line
    assert abs(res.cross_track) < 1e-9


def test_off_path_steers_back():
    pp = PurePursuit(lookahead_base=1.0, lookahead_gain=0.0)
    pp.set_path([(0.0, 0.0), (10.0, 0.0)])
    # robot displaced +y (left of path), heading along +x
    res = pp.query(pos=(2.0, 0.5), heading=0.0, speed_mps=0.0)
    assert res is not None
    # lookahead target lies on path (+x axis, y≈0) so y_local < 0 ⇒ curvature
    # should be negative (steer right) to come back to the path
    assert res.curvature < 0.0
    # cross-track is signed: robot is left of the segment, so cross > 0
    assert res.cross_track > 0.0


def test_lookahead_scales_with_speed():
    pp = PurePursuit(lookahead_base=0.5, lookahead_gain=0.4,
                     lookahead_min=0.3, lookahead_max=3.0)
    assert pp.lookahead_distance(0.0) == pytest.approx(0.5)
    assert pp.lookahead_distance(1.0) == pytest.approx(0.9)
    assert pp.lookahead_distance(10.0) == pytest.approx(3.0)  # clipped


def test_curvature_sign_for_left_corner():
    pp = PurePursuit(lookahead_base=1.5, lookahead_gain=0.0)
    # Right-angle corner: go along +x then turn to +y
    pp.set_path([(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)])
    # Approaching the corner from the right of the path
    res = pp.query(pos=(4.0, 0.0), heading=0.0, speed_mps=0.0)
    assert res is not None
    # Lookahead should be ahead and to the left of robot heading ⇒ κ > 0
    assert res.curvature > 0.0


def test_path_progress_is_monotone():
    pp = PurePursuit(lookahead_base=0.5, lookahead_gain=0.0)
    pp.set_path([(0.0, 0.0), (3.0, 0.0), (6.0, 0.0)])
    progresses = []
    for x in (0.5, 1.5, 2.5, 3.5, 4.5, 5.0):
        r = pp.query(pos=(x, 0.0), heading=0.0, speed_mps=0.0)
        if r is not None:
            progresses.append(r.path_progress)
    assert all(progresses[i] <= progresses[i + 1] + 1e-9
               for i in range(len(progresses) - 1))


def test_steering_to_wheels_obeys_motor_limits():
    from src.control.adaptive_params import MotionParams
    pp = PurePursuit(lookahead_base=1.0, lookahead_gain=0.0)
    pp.set_path([(0.0, 0.0), (10.0, 0.0)])
    res = pp.query(pos=(0.0, 5.0), heading=0.0, speed_mps=0.0)
    assert res is not None  # large lateral offset, very tight curvature
    params = MotionParams(max_speed=12.0, turn_gain=2.5, accel_limit=4.0)
    l, r = pp.steering_to_wheels(res, params)
    assert -params.max_speed - 1e-6 <= l <= params.max_speed + 1e-6
    assert -params.max_speed - 1e-6 <= r <= params.max_speed + 1e-6
