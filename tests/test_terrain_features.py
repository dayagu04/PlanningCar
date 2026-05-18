"""tests for terrain feature extraction."""

import numpy as np
from src.perception.terrain_features import compute_slope, compute_roughness, extract_features


def test_flat_terrain_zero_slope():
    flat = np.zeros((10, 10))
    assert compute_slope(flat) == 0.0
    assert compute_roughness(flat) == 0.0


def test_slope_detection():
    grid_x, _ = np.meshgrid(np.arange(10), np.arange(10))
    sloped = grid_x * 0.1
    slope = compute_slope(sloped, cell_size=0.1)
    assert slope > 30.0


def test_extract_all_features():
    h = np.random.RandomState(0).rand(8, 8) * 0.05
    feats = extract_features(h)
    assert set(feats.keys()) == {"slope_deg", "roughness", "height_range", "mean_height"}
    assert feats["roughness"] > 0
