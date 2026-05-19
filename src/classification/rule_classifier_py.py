"""rule-based terrain classifier: flat / slope / rough / transition."""

import math
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
    "flat_imu_pitch_max": 3.0,
    "slope_angle_min": 5.0,
    "slope_imu_pitch_min": 3.0,
    "slope_roughness_max": 0.05,
    "rough_roughness_min": 0.05,
    "rough_imu_roll_min": 2.0,
}


class TerrainClassifier:
    def __init__(self, thresholds: dict = None, history_len: int = 5):
        self.t = thresholds or DEFAULT_THRESHOLDS
        self.history = deque(maxlen=history_len)

    def _classify_static(self, features: dict) -> TerrainType:
        slope = features.get("slope_deg", 0.0)
        rough = features.get("roughness", 0.0)
        imu_pitch = abs(features.get("imu_pitch_deg", 0.0))
        imu_roll = abs(features.get("imu_roll_deg", 0.0))

        if imu_pitch >= self.t["slope_imu_pitch_min"] and rough < self.t["slope_roughness_max"]:
            return TerrainType.SLOPE

        if rough >= self.t["rough_roughness_min"] or imu_roll >= self.t["rough_imu_roll_min"]:
            return TerrainType.ROUGH

        if (slope < self.t["flat_slope_max"]
                and rough < self.t["flat_roughness_max"]
                and imu_pitch < self.t["flat_imu_pitch_max"]):
            return TerrainType.FLAT

        if slope >= self.t["slope_angle_min"]:
            return TerrainType.SLOPE

        return TerrainType.TRANSITION

    def classify(self, features: dict) -> TerrainType:
        current = self._classify_static(features)
        self.history.append(current)
        if len(self.history) >= 3 and len(set(list(self.history)[-3:])) > 1:
            recent = list(self.history)[-3:]
            if len(set(recent)) >= 3:
                return TerrainType.TRANSITION
        return current
