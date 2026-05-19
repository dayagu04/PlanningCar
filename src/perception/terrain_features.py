"""Terrain feature extraction — C++ backed.

Backed by `nav_core_cpp.extract_features` (pybind11 extension).
Public API identical to the original pure-Python implementation in
`terrain_features_py.py`, which is kept side-by-side for benchmarking
and as a reference implementation for the thesis.
"""

import numpy as np

from src import nav_core_cpp as _cpp


def compute_slope(heights: np.ndarray, cell_size: float = 0.1) -> float:
    if heights.size < 2:
        return 0.0
    arr = np.ascontiguousarray(heights, dtype=np.float64)
    return float(_cpp.extract_features(arr, cell_size).slope_deg)


def compute_roughness(heights: np.ndarray, cell_size: float = 0.1) -> float:
    if heights.size < 2:
        return 0.0
    arr = np.ascontiguousarray(heights, dtype=np.float64)
    return float(_cpp.extract_features(arr, cell_size).roughness)


def compute_height_range(heights: np.ndarray) -> float:
    if heights.size == 0:
        return 0.0
    arr = np.ascontiguousarray(heights, dtype=np.float64)
    return float(_cpp.extract_features(arr, 0.1).height_range)


def extract_features(heights: np.ndarray, cell_size: float = 0.1) -> dict:
    if heights.size == 0:
        return {"slope_deg": 0.0, "roughness": 0.0, "height_range": 0.0, "mean_height": 0.0}
    arr = np.ascontiguousarray(heights, dtype=np.float64)
    f = _cpp.extract_features(arr, cell_size)
    return {
        "slope_deg": float(f.slope_deg),
        "roughness": float(f.roughness),
        "height_range": float(f.height_range),
        "mean_height": float(f.mean_height),
    }
