"""Per-terrain motion profile.

Speed is expressed in **wheel angular velocity (rad/s)**, not linear m/s,
because that is what `RotationalMotor.setVelocity(...)` consumes in Webots.

With the post-tuning robot (wheel radius 0.10 m, motor maxVelocity 20 rad/s,
maxTorque 8.0 N·m, chassis density 700), the physical speed cap is
20 × 0.10 = 2.0 m/s.

Design rationale (see docs/algorithm_design.md):
  - FLAT: full speed, aggressive turn gain — high throughput, no traction worry.
  - SLOPE: **keep the speed high**. Climbing needs power, not braking; reducing
    wheel velocity also reduces the motor's instantaneous torque output. The
    old profile (15 rad/s) under-powered hills. Turn gain is dropped to limit
    sideways tipping.
  - ROUGH: speed moderately reduced (chassis bounce dominates), turn gain low
    so heading oscillations don't multiply with terrain shocks.
  - TRANSITION: between FLAT and SLOPE — keep speed up, blunt the turning so
    the controller doesn't whiplash when crossing terrain seams.

`max_lookahead` is the Pure-Pursuit lookahead radius (m) used by the new
controller. It scales with speed; rougher terrain → shorter lookahead so
the path can hug obstacles.

`align_floor` (∈[0,1]) is the *minimum* forward-speed factor when the robot
is far off-heading. Bigger values let the robot "drive while turning"
instead of grinding to a halt mid-traverse; bumped from 0.30 → 0.55.
"""

from dataclasses import dataclass
from src.classification.rule_classifier import TerrainType


@dataclass
class MotionParams:
    max_speed: float        # wheel rad/s
    turn_gain: float        # P-gain on heading error
    accel_limit: float      # rad/s² — slew-rate cap on wheel speed
    max_lookahead: float = 1.5    # m, Pure-Pursuit lookahead at peak speed
    align_floor: float = 0.55     # min forward fraction when heading off-axis


PROFILES = {
    TerrainType.FLAT:
        MotionParams(max_speed=18.0, turn_gain=3.5, accel_limit=8.0,
                     max_lookahead=1.8, align_floor=0.65),
    TerrainType.SLOPE:
        MotionParams(max_speed=18.0, turn_gain=2.5, accel_limit=5.0,
                     max_lookahead=2.0, align_floor=0.70),
    TerrainType.ROUGH:
        MotionParams(max_speed=14.0, turn_gain=2.2, accel_limit=4.0,
                     max_lookahead=1.2, align_floor=0.65),
    TerrainType.TRANSITION:
        MotionParams(max_speed=15.0, turn_gain=2.8, accel_limit=5.0,
                     max_lookahead=1.5, align_floor=0.65),
}


def get_params(terrain: TerrainType) -> MotionParams:
    return PROFILES[terrain]
