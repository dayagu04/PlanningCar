"""Dynamic Window Approach (DWA) local planner.

DWA is a reactive, sample-based local planner for differential-drive robots.
At every control step it:

  1. Samples the *dynamic window* — the (v, ω) commands the robot can
     actually achieve in the next step, given current speed and accel limits.
  2. Forward-simulates each candidate for a short horizon (``predict_time``).
  3. Scores trajectories on three criteria:
       - **heading**: alignment with the goal direction at the end of the rollout
       - **clearance**: distance to the nearest obstacle along the rollout
       - **velocity**: prefer higher forward speed (reward going fast)
     and picks the highest-scoring (v, ω).

This is the recommended supplement to Pure Pursuit on rough terrain: PP
gives a smooth nominal track, DWA arbitrates last-second avoidance of
obstacles that A*/Theta* may not have seen (e.g. terrain too rough to drive
straight over, even if grid-passable).

Algorithm reference:
  Fox D., Burgard W., Thrun S., "The dynamic window approach to collision
  avoidance," IEEE Robot. Autom. Mag., 1997.
"""

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

Point = Tuple[float, float]


@dataclass
class DWAConfig:
    # Kinematic limits — chosen for the post-tuning robot
    # (wheel r=0.10 m, max wheel ω=20 rad/s → v_max ≈ 2.0 m/s)
    max_speed: float = 2.0          # m/s
    min_speed: float = 0.0          # m/s (no reverse)
    max_yaw_rate: float = 2.4       # rad/s
    max_accel: float = 2.5          # m/s²
    max_yaw_accel: float = 3.5      # rad/s²

    # Discretisation
    v_resolution: float = 0.10      # m/s
    yaw_resolution: float = 0.15    # rad/s
    dt: float = 0.10                # forward-sim step
    predict_time: float = 1.2       # forward-sim horizon (s)

    # Cost weights
    heading_weight: float = 0.7
    clearance_weight: float = 0.25
    velocity_weight: float = 0.15

    # Geometry
    robot_radius: float = 0.30      # m, used for clearance check

    # Stalls when set to 0 — used by the controller to soft-disable DWA
    # if the path is clear ahead.
    enabled: bool = True


@dataclass
class Trajectory:
    poses: List[Tuple[float, float, float]] = field(default_factory=list)  # (x, y, yaw)
    v: float = 0.0
    omega: float = 0.0
    heading_cost: float = 0.0
    clearance_cost: float = 0.0
    velocity_cost: float = 0.0
    total_cost: float = 0.0


class DWAPlanner:
    def __init__(self, config: DWAConfig = None):
        self.cfg = config or DWAConfig()

    # ------------------------------------------------------------------
    # Dynamic window
    # ------------------------------------------------------------------
    def _dynamic_window(self, v: float, omega: float) -> Tuple[float, float, float, float]:
        c = self.cfg
        v_min = max(c.min_speed, v - c.max_accel * c.dt)
        v_max = min(c.max_speed, v + c.max_accel * c.dt)
        w_min = max(-c.max_yaw_rate, omega - c.max_yaw_accel * c.dt)
        w_max = min(c.max_yaw_rate, omega + c.max_yaw_accel * c.dt)
        return v_min, v_max, w_min, w_max

    # ------------------------------------------------------------------
    # Rollout
    # ------------------------------------------------------------------
    def _rollout(self, x: float, y: float, yaw: float,
                 v: float, omega: float) -> List[Tuple[float, float, float]]:
        c = self.cfg
        poses: List[Tuple[float, float, float]] = []
        t = 0.0
        while t < c.predict_time:
            x += v * math.cos(yaw) * c.dt
            y += v * math.sin(yaw) * c.dt
            yaw += omega * c.dt
            poses.append((x, y, yaw))
            t += c.dt
        return poses

    # ------------------------------------------------------------------
    # Cost terms
    # ------------------------------------------------------------------
    def _heading_cost(self, end_pose: Tuple[float, float, float],
                      goal: Point) -> float:
        ex, ey, eyaw = end_pose
        ang_to_goal = math.atan2(goal[1] - ey, goal[0] - ex)
        diff = ang_to_goal - eyaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        # 0 when perfectly aligned, ~1 when 180° off
        return abs(diff) / math.pi

    def _clearance_cost(self, poses: List[Tuple[float, float, float]],
                        obstacles: Iterable[Tuple[float, float, float]]) -> Tuple[float, bool]:
        """Return (cost ∈[0,1], collision_flag).

        cost = 1 / (1 + min_clearance); collision_flag True when any pose
        is inside an obstacle (after inflation by robot_radius).
        """
        min_dist = float("inf")
        for x, y, _ in poses:
            for ox, oy, oradius in obstacles:
                d = math.hypot(x - ox, y - oy) - (oradius + self.cfg.robot_radius)
                if d < min_dist:
                    min_dist = d
        if min_dist <= 0.0:
            return 1.0, True
        if not math.isfinite(min_dist):
            return 0.0, False
        # smaller clearance ⇒ larger cost
        return 1.0 / (1.0 + min_dist), False

    def _velocity_cost(self, v: float) -> float:
        # 0 when v=max_speed, 1 when v=0
        return 1.0 - (v / self.cfg.max_speed if self.cfg.max_speed > 1e-6 else 0.0)

    # ------------------------------------------------------------------
    # Main
    # ------------------------------------------------------------------
    def plan(self, pose: Tuple[float, float, float],
             current_v: float, current_omega: float,
             goal: Point,
             obstacles: Iterable[Tuple[float, float, float]] = ()) -> Optional[Trajectory]:
        """Return the best (v, ω) and its rollout, or None if all collide."""
        if not self.cfg.enabled:
            return None

        v_min, v_max, w_min, w_max = self._dynamic_window(current_v, current_omega)
        obs_list = list(obstacles)

        best: Optional[Trajectory] = None
        # iterate v then omega
        v = v_min
        while v <= v_max + 1e-9:
            w = w_min
            while w <= w_max + 1e-9:
                poses = self._rollout(pose[0], pose[1], pose[2], v, w)
                clearance, collided = self._clearance_cost(poses, obs_list)
                if collided:
                    w += self.cfg.yaw_resolution
                    continue
                heading = self._heading_cost(poses[-1], goal)
                velocity = self._velocity_cost(v)
                c = self.cfg
                total = (c.heading_weight * heading
                         + c.clearance_weight * clearance
                         + c.velocity_weight * velocity)
                if best is None or total < best.total_cost:
                    best = Trajectory(
                        poses=poses, v=v, omega=w,
                        heading_cost=heading, clearance_cost=clearance,
                        velocity_cost=velocity, total_cost=total,
                    )
                w += self.cfg.yaw_resolution
            v += self.cfg.v_resolution

        return best

    # ------------------------------------------------------------------
    # Convenience: command-to-wheels for differential drive
    # ------------------------------------------------------------------
    def cmd_to_wheels(self, v: float, omega: float,
                      wheel_radius: float = 0.10,
                      track_width: float = 0.36,
                      max_wheel_omega: float = 20.0) -> Tuple[float, float]:
        """Convert (v_linear, omega) into (left_rad/s, right_rad/s)."""
        v_l = (v - omega * track_width * 0.5) / wheel_radius
        v_r = (v + omega * track_width * 0.5) / wheel_radius
        m = max_wheel_omega
        return max(-m, min(m, v_l)), max(-m, min(m, v_r))
