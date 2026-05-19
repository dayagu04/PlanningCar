"""Rule-based terrain classifier — C++ backed.

Wraps `nav_core_cpp.TerrainClassifier`. Maintains the same public API as the
pure-Python reference in `rule_classifier_py.py`:

  - `TerrainType` is a Python Enum with values "flat" / "slope" / "rough" / "transition"
    (kept hashable so it can index motion-parameter tables in `adaptive_params.py`)
  - `TerrainClassifier()` accepts optional `thresholds` dict and `history_len`
  - `.classify(features: dict)` returns a `TerrainType` member
"""

from collections import deque
from enum import Enum
from typing import Optional

from src import nav_core_cpp as _cpp


class TerrainType(Enum):
    FLAT = "flat"
    SLOPE = "slope"
    ROUGH = "rough"
    TRANSITION = "transition"


_CPP_TO_PY = {
    _cpp.TerrainType.FLAT: TerrainType.FLAT,
    _cpp.TerrainType.SLOPE: TerrainType.SLOPE,
    _cpp.TerrainType.ROUGH: TerrainType.ROUGH,
    _cpp.TerrainType.TRANSITION: TerrainType.TRANSITION,
}


DEFAULT_THRESHOLDS = {
    "flat_slope_max": 5.0,
    "flat_roughness_max": 0.02,
    "flat_imu_pitch_max": 3.0,
    "slope_angle_min": 5.0,
    "slope_imu_pitch_min": 3.0,
    "slope_roughness_max": 0.05,
    "rough_roughness_min": 0.05,
    "rough_imu_roll_min": 2.0,
}


def _make_thresholds(overrides: Optional[dict]) -> "_cpp.ClassifierThresholds":
    t = _cpp.ClassifierThresholds()
    if overrides:
        for k, v in overrides.items():
            if hasattr(t, k):
                setattr(t, k, float(v))
    return t


class TerrainClassifier:
    def __init__(self, thresholds: Optional[dict] = None, history_len: int = 5,
                 vote_window: int = 5):
        """vote_window: majority-vote across the last N raw classifications,
        which damps single-frame sensor blips. Set vote_window=1 to disable."""
        self._cpp_obj = _cpp.TerrainClassifier(_make_thresholds(thresholds), history_len)
        self._vote = deque(maxlen=max(1, vote_window))

    def classify(self, features: dict) -> TerrainType:
        slope = float(features.get("slope_deg", 0.0))
        rough = float(features.get("roughness", 0.0))
        pitch = float(features.get("imu_pitch_deg", 0.0))
        roll = float(features.get("imu_roll_deg", 0.0))
        raw = _CPP_TO_PY[self._cpp_obj.classify(slope, rough, pitch, roll)]
        self._vote.append(raw)
        if len(self._vote) < self._vote.maxlen:
            return raw
        counts = {}
        for tt in self._vote:
            counts[tt] = counts.get(tt, 0) + 1
        # Tie-break: prefer the most recent raw classification
        winner = max(counts, key=lambda k: (counts[k], 0 if k != raw else 1))
        return winner

    def reset(self):
        self._cpp_obj.reset()
        self._vote.clear()
