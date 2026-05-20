"""Shared navigation helpers used by both Webots controllers."""

import math
import os
from typing import Iterable, Tuple

import numpy as np


def get_bearing(compass_values: Iterable[float]) -> float:
    """Robot forward direction in world frame.

    Empirically calibrated for Webots compass: +X → 0, +Y → π/2.
    """
    return math.atan2(compass_values[0], compass_values[1])


def compute_steering(current_pos: Iterable[float],
                     target: Tuple[float, float],
                     bearing: float) -> float:
    """Bearing error in [-π, π]. Positive value means target lies to the left."""
    dx = target[0] - current_pos[0]
    dy = target[1] - current_pos[1]
    target_angle = math.atan2(dy, dx)
    beta = target_angle - bearing
    while beta > math.pi:
        beta -= 2 * math.pi
    while beta < -math.pi:
        beta += 2 * math.pi
    return beta


def lidar_to_height_grid(lidar_ranges: Iterable[float]) -> np.ndarray:
    """Reduce a 2D lidar scan to a 4x4 zero-mean height grid for feature extraction.

    A 2D lidar does not directly observe terrain height, so we use per-sector
    minimum range as a proxy and zero-mean it; the resulting grid is then fed
    to extract_features for roughness / slope-of-grid statistics.
    """
    ranges = np.array(list(lidar_ranges), dtype=np.float64)
    ranges = ranges[np.isfinite(ranges)]
    if len(ranges) < 10:
        return np.zeros((4, 4))

    sector_size = len(ranges) // 16
    if sector_size == 0:
        return np.zeros((4, 4))

    heights = [
        float(np.min(ranges[i * sector_size:(i + 1) * sector_size]))
        for i in range(16)
    ]
    grid = np.array(heights).reshape(4, 4)
    return grid - np.mean(grid)


def open_log_file(project_root: str, filename: str = "navigation.csv"):
    """Create the navigation log file and write the standard header.

    Parallel-experiment override: env vars `KIROZ_LOG_DIR` and `KIROZ_LOG_NAME`
    redirect the file to a worker-private path so multiple Webots instances
    do not stomp on each other's logs. Default behaviour (data/logs/navigation.csv)
    is preserved when the env vars are unset.
    """
    log_dir = os.environ.get("KIROZ_LOG_DIR") or os.path.join(project_root, "data", "logs")
    log_name = os.environ.get("KIROZ_LOG_NAME") or filename
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, log_name)
    log_file = open(log_path, "w", encoding="utf-8")
    log_file.write(
        "step,time_s,x,y,z,roll,pitch,yaw,terrain,speed,target_idx,dist_to_target\n"
    )
    return log_file


def differential_wheel_speeds(beta: float, params,
                              prev_beta: float = 0.0,
                              dt: float = 0.032,
                              forward_target: float = None) -> Tuple[float, float]:
    """Map heading error and motion params to (left, right) wheel speeds.

    PD controller on heading + speed shaping:
    - drives forward at all times unless ``|beta| > 5π/6`` (target almost
      directly behind); below that threshold we drive *and* turn, instead of
      grinding to a halt every time the heading is off-axis
    - forward speed scales smoothly with alignment but never drops below
      ``params.align_floor`` × max_speed (so the robot keeps moving in turns)
    - PD derivative damps oscillation at high speed
    - ``forward_target`` (optional) overrides the alignment-shaped speed,
      e.g. used by Pure Pursuit and DWA so the planner — not the heading-PD —
      decides cruising speed
    """
    floor = getattr(params, "align_floor", 0.3)

    # Reverse-spin region: only when target is almost directly behind.
    # The old threshold (π/2) made the robot stop and spin on every turn;
    # 5π/6 ≈ 150° keeps forward motion through any normal navigation turn.
    if abs(beta) > 5 * math.pi / 6:
        spin_speed = params.max_speed * 0.7
        return (-spin_speed, spin_speed) if beta > 0 else (spin_speed, -spin_speed)

    # PD on bearing error
    kp = params.turn_gain
    kd = 0.6 * kp
    d_beta = (beta - prev_beta) / max(dt, 1e-3)
    turn = kp * beta + kd * d_beta

    # Forward speed shaping with raised floor (drive while turning).
    # cos(beta) ∈ [cos(5π/6), 1] = [-0.866, 1] in this branch; clamp ≥0.
    align = max(0.0, math.cos(beta))
    if forward_target is not None:
        forward = max(0.0, min(forward_target, params.max_speed))
    else:
        forward = params.max_speed * (floor + (1.0 - floor) * align)

    # Saturate turn so neither wheel exceeds max_speed
    max_turn = params.max_speed * 0.8
    turn = max(-max_turn, min(max_turn, turn))

    left = forward - turn
    right = forward + turn
    left = max(-params.max_speed, min(params.max_speed, left))
    right = max(-params.max_speed, min(params.max_speed, right))
    return left, right
