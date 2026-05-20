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
                 vote_window: int = 9, recent_bias_window: int = 3):
        """vote_window: majority-vote across the last N raw classifications,
        which damps single-frame sensor blips. Set vote_window=1 to disable.

        recent_bias_window: when the top two terrain types are within 1 vote,
        use the majority of the last K frames as tie-breaker. This prevents
        a slow drift away from the current terrain class when noise produces
        an ambiguous mix (e.g. ROUGH state polluted by occasional FLAT frames).
        """
        self._cpp_obj = _cpp.TerrainClassifier(_make_thresholds(thresholds), history_len)
        self._vote = deque(maxlen=max(1, vote_window))
        self._recent_bias_window = max(1, min(recent_bias_window, vote_window))

    def classify(self, features: dict) -> TerrainType:
        slope = float(features.get("slope_deg", 0.0))
        rough = float(features.get("roughness", 0.0))
        pitch = float(features.get("imu_pitch_deg", 0.0))
        roll = float(features.get("imu_roll_deg", 0.0))
        raw = _CPP_TO_PY[self._cpp_obj.classify(slope, rough, pitch, roll)]
        self._vote.append(raw)
        if len(self._vote) < self._vote.maxlen:
            return raw

        # Count votes across the full window
        counts = {}
        for tt in self._vote:
            counts[tt] = counts.get(tt, 0) + 1

        # Find top candidates
        sorted_counts = sorted(counts.items(), key=lambda kv: kv[1], reverse=True)
        top_type, top_n = sorted_counts[0]

        # If top is unambiguous (margin >= 2), return it directly
        if len(sorted_counts) == 1 or top_n - sorted_counts[1][1] >= 2:
            return top_type

        # Tie or near-tie: bias toward the recent K-frame majority.
        # This stabilises the output during noisy stretches where ROUGH
        # and FLAT alternate frame-by-frame on physically-rough terrain.
        recent = list(self._vote)[-self._recent_bias_window:]
        recent_counts = {}
        for tt in recent:
            recent_counts[tt] = recent_counts.get(tt, 0) + 1
        recent_winner = max(recent_counts, key=recent_counts.get)
        # Only use recent bias if recent_winner is among the top two
        top_two = {kv[0] for kv in sorted_counts[:2]}
        if recent_winner in top_two:
            return recent_winner
        return top_type

    def reset(self):
        self._cpp_obj.reset()
        self._vote.clear()
