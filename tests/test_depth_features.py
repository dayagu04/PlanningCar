"""Tests for depth-based terrain feature extraction."""

import numpy as np
from src.perception.depth_features import (
    rangefinder_to_height_grid,
    compute_depth_variance,
    compute_forward_slope,
)


def test_rangefinder_to_height_grid():
    # Simulate flat ground at 2m distance
    depth = np.full((64, 64), 2.0)
    grid = rangefinder_to_height_grid(depth)
    assert grid.shape == (8, 8)
    assert np.all(grid >= 0)


def test_depth_variance_flat():
    depth = np.full((64, 64), 2.0)
    var = compute_depth_variance(depth)
    assert var < 0.01


def test_depth_variance_rough():
    depth = np.random.uniform(1.0, 3.0, (64, 64))
    var = compute_depth_variance(depth)
    assert var > 0.1


def test_forward_slope_flat():
    depth = np.full((64, 64), 2.0)
    slope = compute_forward_slope(depth)
    assert abs(slope) < 1.0


def test_forward_slope_uphill():
    depth = np.zeros((64, 64))
    depth[:32, :] = 2.0  # Top half closer (uphill ahead)
    depth[32:, :] = 3.0  # Bottom half farther
    slope = compute_forward_slope(depth)
    assert slope > 5.0
