"""Unit tests for src/utils/nav_common.py."""

import math
import os
import tempfile

import numpy as np
import pytest

from src.utils.nav_common import (
    get_bearing,
    compute_steering,
    lidar_to_height_grid,
    open_log_file,
    differential_wheel_speeds,
)
from src.control.adaptive_params import MotionParams


def test_get_bearing_axes():
    # compass = (0,1) -> atan2(0,1) = 0  (robot heading aligned with "north")
    assert get_bearing([0.0, 1.0, 0.0]) == pytest.approx(0.0, abs=1e-6)
    # compass = (1,0) -> atan2(1,0) = π/2
    assert get_bearing([1.0, 0.0, 0.0]) == pytest.approx(math.pi / 2, abs=1e-6)


def test_compute_steering_wraps_to_pi():
    # Robot at origin facing +X, target straight behind on -X
    beta = compute_steering([0.0, 0.0], (-1.0, 0.0), bearing=0.0)
    assert abs(beta) == pytest.approx(math.pi, abs=1e-6)


def test_compute_steering_target_to_left_is_positive():
    # Facing +X, target at +Y → must turn left → positive beta
    beta = compute_steering([0.0, 0.0], (0.0, 1.0), bearing=0.0)
    assert beta > 0


def test_lidar_to_height_grid_handles_empty_input():
    grid = lidar_to_height_grid([])
    assert grid.shape == (4, 4)
    assert np.all(grid == 0)


def test_lidar_to_height_grid_zero_mean():
    rng = np.random.default_rng(0)
    ranges = rng.uniform(1.0, 5.0, size=360).tolist()
    grid = lidar_to_height_grid(ranges)
    assert grid.shape == (4, 4)
    assert abs(grid.mean()) < 1e-9


def test_differential_wheel_speeds_forward():
    params = MotionParams(max_speed=10.0, turn_gain=3.0, accel_limit=3.0)
    left, right = differential_wheel_speeds(beta=0.0, params=params)
    assert left == pytest.approx(10.0)
    assert right == pytest.approx(10.0)


def test_differential_wheel_speeds_spin_in_place_when_target_behind():
    params = MotionParams(max_speed=10.0, turn_gain=3.0, accel_limit=3.0)
    # |beta| > π/2 should spin (opposite-sign wheels)
    left, right = differential_wheel_speeds(beta=math.pi * 0.9, params=params)
    assert left * right < 0


def test_differential_wheel_speeds_saturated():
    params = MotionParams(max_speed=10.0, turn_gain=100.0, accel_limit=3.0)
    # huge gain × tiny beta should still respect ±max_speed
    left, right = differential_wheel_speeds(beta=0.1, params=params)
    assert -10.0 <= left <= 10.0
    assert -10.0 <= right <= 10.0


def test_open_log_file_creates_directory(tmp_path):
    log_file = open_log_file(str(tmp_path), filename="t.csv")
    try:
        assert os.path.isfile(tmp_path / "data" / "logs" / "t.csv")
    finally:
        log_file.close()
