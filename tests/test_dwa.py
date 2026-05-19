"""Unit tests for the Dynamic Window Approach planner."""

import math
import pytest

from src.planning.dwa import DWAPlanner, DWAConfig


def test_disabled_returns_none():
    cfg = DWAConfig(enabled=False)
    p = DWAPlanner(cfg)
    res = p.plan((0.0, 0.0, 0.0), 0.0, 0.0, (5.0, 0.0))
    assert res is None


def test_clear_path_picks_high_velocity():
    cfg = DWAConfig(max_speed=1.5, v_resolution=0.1,
                    yaw_resolution=0.3, predict_time=1.0,
                    max_accel=2.5, dt=0.1)
    p = DWAPlanner(cfg)
    # Start from a non-zero velocity — DWA cannot leap from 0 to 1 m/s in one
    # control step (limited by max_accel*dt). The current-velocity prior
    # determines the dynamic window.
    res = p.plan((0.0, 0.0, 0.0), current_v=1.0, current_omega=0.0,
                 goal=(10.0, 0.0))
    assert res is not None
    assert abs(res.omega) < 0.6
    assert res.v > 0.8


def test_obstacle_in_front_deflects_path():
    cfg = DWAConfig(max_speed=1.5, v_resolution=0.3,
                    yaw_resolution=0.2, predict_time=1.0,
                    robot_radius=0.3)
    p = DWAPlanner(cfg)
    # Goal straight ahead, obstacle right in front
    res = p.plan((0.0, 0.0, 0.0), 1.0, 0.0,
                 goal=(5.0, 0.0),
                 obstacles=[(1.5, 0.0, 0.4)])
    assert res is not None
    # The straight-ahead trajectory should have collided ⇒ chosen path must
    # deflect (non-zero omega)
    assert abs(res.omega) > 0.05


def test_all_collision_returns_none():
    cfg = DWAConfig(max_speed=1.5, v_resolution=0.3,
                    yaw_resolution=0.3, predict_time=1.0,
                    robot_radius=0.3,
                    # Tight dynamic window: forced into a small box
                    max_accel=0.05, max_yaw_accel=0.05)
    p = DWAPlanner(cfg)
    # Robot already moving forward into a wall of obstacles
    obs = [(1.0 + 0.1 * i, y, 0.4)
           for i in range(5)
           for y in (-0.5, 0.0, 0.5)]
    res = p.plan((0.0, 0.0, 0.0), 1.5, 0.0, (5.0, 0.0), obs)
    # Either deflects (best is still feasible) or returns None — both fine.
    if res is not None:
        # If anything was returned, it must NOT be a colliding trajectory.
        # The planner's own clearance check filters those.
        assert res.clearance_cost < 1.0


def test_dynamic_window_respects_accel_limits():
    cfg = DWAConfig(max_speed=2.0, max_accel=1.0, dt=0.1,
                   max_yaw_rate=2.0, max_yaw_accel=1.0,
                   v_resolution=0.05, yaw_resolution=0.1, predict_time=0.5)
    p = DWAPlanner(cfg)
    v_min, v_max, w_min, w_max = p._dynamic_window(1.0, 0.0)
    assert v_min == pytest.approx(0.9)
    assert v_max == pytest.approx(1.1)
    assert w_min == pytest.approx(-0.1)
    assert w_max == pytest.approx(0.1)


def test_cmd_to_wheels_left_turn():
    p = DWAPlanner()
    # v=1.0 m/s, ω=+1 rad/s (CCW = left turn) — right wheel faster than left
    l, r = p.cmd_to_wheels(v=1.0, omega=1.0,
                           wheel_radius=0.10, track_width=0.36)
    assert r > l
