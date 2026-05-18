"""speed/turn parameter adaptation per terrain type."""

from dataclasses import dataclass
from src.classification.rule_classifier import TerrainType


@dataclass
class MotionParams:
    max_speed: float
    turn_gain: float
    accel_limit: float


PROFILES = {
    TerrainType.FLAT: MotionParams(max_speed=3.0, turn_gain=2.5, accel_limit=2.0),
    TerrainType.SLOPE: MotionParams(max_speed=2.0, turn_gain=2.0, accel_limit=0.5),
    TerrainType.ROUGH: MotionParams(max_speed=1.5, turn_gain=1.5, accel_limit=0.3),
    TerrainType.TRANSITION: MotionParams(max_speed=2.0, turn_gain=2.0, accel_limit=0.5),
}


def get_params(terrain: TerrainType) -> MotionParams:
    return PROFILES[terrain]
