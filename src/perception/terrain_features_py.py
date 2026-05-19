"""terrain feature extraction (height, slope, roughness)."""

import numpy as np


def compute_slope(heights: np.ndarray, cell_size: float = 0.1) -> float:
    """estimate slope angle (degrees) from a height grid."""
    if heights.size < 2:
        return 0.0
    gy, gx = np.gradient(heights, cell_size)
    grad_mag = np.sqrt(gx**2 + gy**2)
    return float(np.degrees(np.arctan(np.mean(grad_mag))))


def compute_roughness(heights: np.ndarray, cell_size: float = 0.1) -> float:
    """std of residuals after subtracting the best-fit plane.

    Detrending the plane is what makes a uniformly sloped patch report
    roughness ≈ 0 instead of looking 'rough' just because it has a tilt.
    """
    if heights.size < 2:
        return 0.0
    rows, cols = heights.shape
    mean = float(heights.mean())
    crow = (rows - 1) * 0.5
    ccol = (cols - 1) * 0.5
    cidx = (np.arange(cols) - ccol) * cell_size
    ridx = (np.arange(rows) - crow) * cell_size
    xx, yy = np.meshgrid(cidx, ridx)
    z = heights - mean
    Sxx = float((xx * xx).sum())
    Syy = float((yy * yy).sum())
    Sxy = float((xx * yy).sum())
    Sxz = float((xx * z).sum())
    Syz = float((yy * z).sum())
    det = Sxx * Syy - Sxy * Sxy
    if abs(det) > 1e-12:
        a = (Sxz * Syy - Syz * Sxy) / det
        b = (Syz * Sxx - Sxz * Sxy) / det
    else:
        a = b = 0.0
    resid = z - (a * xx + b * yy)
    return float(np.sqrt((resid * resid).mean()))


def compute_height_range(heights: np.ndarray) -> float:
    if heights.size == 0:
        return 0.0
    return float(np.max(heights) - np.min(heights))


def extract_features(heights: np.ndarray, cell_size: float = 0.1) -> dict:
    return {
        "slope_deg": compute_slope(heights, cell_size),
        "roughness": compute_roughness(heights, cell_size),
        "height_range": compute_height_range(heights),
        "mean_height": float(np.mean(heights)) if heights.size else 0.0,
    }
