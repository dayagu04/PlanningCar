"""rule-based terrain classifier: flat / slope / rough / transition."""

from enum import Enum
from collections import deque


class TerrainType(Enum):
    FLAT = "flat"
    SLOPE = "slope"
    ROUGH = "rough"
    TRANSITION = "transition"


DEFAULT_THRESHOLDS = {
    "flat_slope_max": 5.0,
    "flat_roughness_max": 0.02,
    "slope_angle_min": 5.0,
    "slope_roughness_max": 0.05,
    "rough_roughness_min": 0.05,
}


class TerrainClassifier:
    def __init__(self, thresholds: dict = None, history_len: int = 5):
        self.t = thresholds or DEFAULT_THRESHOLDS
        self.history = deque(maxlen=history_len)

    def _classify_static(self, features: dict) -> TerrainType:
        slope = features["slope_deg"]
        rough = features["roughness"]
        if slope < self.t["flat_slope_max"] and rough < self.t["flat_roughness_max"]:
            return TerrainType.FLAT
        if slope >= self.t["slope_angle_min"] and rough < self.t["slope_roughness_max"]:
            return TerrainType.SLOPE
        if rough >= self.t["rough_roughness_min"]:
            return TerrainType.ROUGH
        return TerrainType.TRANSITION

    def classify(self, features: dict) -> TerrainType:
        current = self._classify_static(features)
        self.history.append(current)
        if len(set(self.history)) > 1 and len(self.history) >= 3:
            return TerrainType.TRANSITION
        return current
