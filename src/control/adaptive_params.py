"""speed/turn parameter adaptation per terrain type."""

from dataclasses import dataclass
from src.classification.rule_classifier import TerrainType


@dataclass
class MotionParams:
    max_speed: float
    turn_gain: float
    accel_limit: float


PROFILES = {
    TerrainType.FLAT: MotionParams(max_speed=20.0, turn_gain=4.0, accel_limit=6.0),
    TerrainType.SLOPE: MotionParams(max_speed=15.0, turn_gain=3.5, accel_limit=4.0),
    TerrainType.ROUGH: MotionParams(max_speed=12.0, turn_gain=3.0, accel_limit=3.0),
    TerrainType.TRANSITION: MotionParams(max_speed=14.0, turn_gain=3.5, accel_limit=3.5),
}


def get_params(terrain: TerrainType) -> MotionParams:
    return PROFILES[terrain]
