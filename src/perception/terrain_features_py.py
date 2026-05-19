"""terrain feature extraction (height, slope, roughness)."""

import numpy as np


def compute_slope(heights: np.ndarray, cell_size: float = 0.1) -> float:
    """estimate slope angle (degrees) from a height grid."""
    if heights.size < 2:
        return 0.0
    gy, gx = np.gradient(heights, cell_size)
    grad_mag = np.sqrt(gx**2 + gy**2)
    return float(np.degrees(np.arctan(np.mean(grad_mag))))


def compute_roughness(heights: np.ndarray) -> float:
    """std of the height field, after detrending the mean plane."""
    if heights.size < 2:
        return 0.0
    return float(np.std(heights - np.mean(heights)))


def compute_height_range(heights: np.ndarray) -> float:
    if heights.size == 0:
        return 0.0
    return float(np.max(heights) - np.min(heights))


def extract_features(heights: np.ndarray, cell_size: float = 0.1) -> dict:
    return {
        "slope_deg": compute_slope(heights, cell_size),
        "roughness": compute_roughness(heights),
        "height_range": compute_height_range(heights),
        "mean_height": float(np.mean(heights)) if heights.size else 0.0,
    }
