"""Webots adaptive navigation controller — *optimised* algorithm stack.

Pipeline per step:
  1. read sensors (lidar, GPS, compass, IMU)
  2. classify terrain  (TerrainClassifier)
  3. (re)plan if needed:
        - Theta* over the cost-map → coarse waypoints
        - LOS-aware simplification (done inside C++ planner)
        - Catmull-Rom smoothing  → dense reference path
  4. Pure Pursuit tracks the smoothed path
  5. (optional) DWA arbitrates last-second avoidance on top of PP output
  6. apply wheel commands, scaled by adaptive params

The baseline algorithm — direct point-to-point with a PD heading controller —
is preserved in ``adaptive_navigator_baseline.py`` for thesis comparisons.

Runtime config (``data/runtime_config.json``) keys consumed:
  - waypoints           : list of (x, y) goals to visit (set by launch_demo)
  - controller_mode     : "optimised" | "baseline"   default "optimised"
  - dwa_enabled         : bool, default True
"""

import sys
import os
import math
import json
import time

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from controller import Robot, Supervisor  # type: ignore
import numpy as np
from src.perception.terrain_features import extract_features
from src.classification.rule_classifier import TerrainClassifier, TerrainType
from src.control.adaptive_params import get_params
from src.control.pure_pursuit import PurePursuit
from src.planning.astar import AStarPlanner
from src.planning.path_smoother import smooth_path
from src.planning.dwa import DWAPlanner, DWAConfig
from src.utils.terrain_sampling import sample_terrain_height
from src.utils.nav_common import (
    get_bearing,
    compute_steering,
    lidar_to_height_grid,
    open_log_file,
    differential_wheel_speeds,
)


DEFAULT_TARGETS = [
    (8.0, 0.0),
    (8.0, 8.0),
    (-8.0, 8.0),
    (-8.0, -8.0),
    (8.0, -8.0),
    (0.0, 0.0),
]
GOAL_TOLERANCE = 1.0  # m — goal-reach radius (wider than legacy 0.8 to handle
                      # GPS jitter on rough terrain + obstacle deflection)


def load_runtime_config():
    """Read waypoints + controller flags from runtime_config.json.

    Parallel-experiment override: env var `KIROZ_RUNTIME_CONFIG` points at a
    worker-private config file so multiple Webots instances can run in parallel
    without sharing the same data/runtime_config.json.
    """
    cfg_path = os.environ.get("KIROZ_RUNTIME_CONFIG") or os.path.join(
        PROJECT_ROOT, "data", "runtime_config.json"
    )
    cfg = {}
    if os.path.exists(cfg_path):
        try:
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
        except Exception as e:
            print(f"[adaptive_navigator] failed to read runtime config: {e}")
    return cfg


def parse_obstacles(cfg):
    obs = cfg.get("obstacles", []) or []
    out = []
    for o in obs:
        if isinstance(o, (list, tuple)) and len(o) >= 3:
            out.append((float(o[0]), float(o[1]), float(o[2])))
        elif isinstance(o, dict):
            out.append((float(o["x"]), float(o["y"]), float(o.get("r", 0.5))))
    return out


def main():
    cfg = load_runtime_config()
    targets = [tuple(wp) for wp in (cfg.get("waypoints") or DEFAULT_TARGETS)]
    obstacles = parse_obstacles(cfg)
    mode = cfg.get("controller_mode", "optimised")
    dwa_enabled = bool(cfg.get("dwa_enabled", True))
    max_sim_time = float(cfg.get("max_sim_time", 0.0)) or None  # 0 / missing → run forever
    world_file = cfg.get("world_file", "")
    worlds_dir = os.path.join(PROJECT_ROOT, "worlds")

    def sample_local_heights(x: float, y: float, yaw: float,
                             rows: int = 5, cols: int = 5,
                             cell_size: float = 0.4) -> np.ndarray:
        """Sample real terrain height in a forward-looking grid centred at (x, y).

        2D lidar can't observe ground elevation directly; we tap the terrain
        heightmap (when available) so the classifier sees a meaningful patch
        instead of horizontal lidar distance noise. The grid is rotated so it
        always points in the robot's heading direction — that's what 'forward
        roughness' should mean for an adaptive controller.
        """
        if not world_file:
            return np.zeros((rows, cols), dtype=np.float64)
        cos_h = math.cos(yaw)
        sin_h = math.sin(yaw)
        # Centre the grid 0.6 m ahead of the robot
        grid = np.empty((rows, cols), dtype=np.float64)
        for r in range(rows):
            for c in range(cols):
                # robot frame: x forward, y left
                fx = (r - rows // 2) * cell_size + 0.6
                fy = (c - cols // 2) * cell_size
                wx = x + fx * cos_h - fy * sin_h
                wy = y + fx * sin_h + fy * cos_h
                grid[r, c] = sample_terrain_height(world_file, worlds_dir, wx, wy)
        return grid

    # Robot is declared `supervisor TRUE` in the world files so we can call
    # simulationQuit() when the controller is done. Falls back to plain
    # Robot() when the world isn't a supervisor (e.g. legacy .wbt).
    try:
        robot = Supervisor()
        is_supervisor = True
    except Exception:
        robot = Robot()
        is_supervisor = False
    timestep = int(robot.getBasicTimeStep())

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)

    motor_names = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
    motors = []
    for name in motor_names:
        m = robot.getDevice(name)
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        motors.append(m)

    classifier = TerrainClassifier()
    planner = AStarPlanner(grid_size=0.5, world_size=24.0,
                           heuristic_weight=1.2, use_any_angle=True)
    # Inflate obstacles by half-chassis-width + safety margin so paths
    # don't graze tree trunks.
    if obstacles:
        inflation = 0.30  # universal: robot half-width 0.20 + 0.10 safety
        inflated = [(x, y, r + inflation) for x, y, r in obstacles]
        planner.add_obstacles(inflated)

    pp = PurePursuit(lookahead_base=0.6, lookahead_gain=0.4,
                     lookahead_min=0.4, lookahead_max=2.5,
                     goal_tolerance=GOAL_TOLERANCE)
    dwa = DWAPlanner(DWAConfig(enabled=dwa_enabled,
                               max_speed=2.0,
                               robot_radius=0.35))

    def tune_dwa_for_terrain(t: TerrainType):
        """Re-balance DWA cost weights to suit each terrain class.

        Rough terrain favours clearance (avoid obstacles) over speed; slope
        favours heading (commit to direction) since changing course mid-climb
        can cause traction loss. Flat / transition use the default balance.
        """
        if t == TerrainType.ROUGH:
            dwa.cfg.heading_weight = 0.45
            dwa.cfg.clearance_weight = 0.35
            dwa.cfg.velocity_weight = 0.20
        elif t == TerrainType.SLOPE:
            dwa.cfg.heading_weight = 0.60
            dwa.cfg.clearance_weight = 0.20
            dwa.cfg.velocity_weight = 0.20
        else:  # FLAT / TRANSITION
            dwa.cfg.heading_weight = 0.65
            dwa.cfg.clearance_weight = 0.20
            dwa.cfg.velocity_weight = 0.15

    # Soft world-edge boundary — keeps the robot from sailing off the 20×20
    # terrain plane when the planner's path overshoots. Reactive PD-style
    # nudge applied on top of any wheel command.
    WORLD_EDGE = 9.0   # half-extent — terrain is 20×20 m centred at origin
    EDGE_BUFFER = 1.5  # start steering inward from this distance to the wall

    target_idx = 0
    last_plan_step = -10_000
    prev_pos = None
    prev_beta = 0.0
    current_v = 0.0       # estimated linear speed (m/s) for PP/DWA
    current_omega = 0.0
    last_terrain = TerrainType.FLAT
    dt = timestep / 1000.0
    step_count = 0
    t_start = time.time()

    # Wheel speed rate limiter — prevents sudden command jumps that cause
    # wheel slip on rough terrain. 20 rad/s² lets the robot accelerate
    # 0 → 18 rad/s in 0.9 s while smoothing out single-step PP/DWA spikes.
    max_wheel_accel_per_step = 20.0 * dt
    prev_left_speed = 0.0
    prev_right_speed = 0.0

    def rate_limit(target, prev):
        delta = target - prev
        if delta > max_wheel_accel_per_step:
            return prev + max_wheel_accel_per_step
        elif delta < -max_wheel_accel_per_step:
            return prev - max_wheel_accel_per_step
        return target

    # Stuck detection: track displacement over 2-second windows. If the robot
    # barely moved while still > 1 m from the goal, inject a virtual obstacle
    # ahead and replan. After 3 such recoveries on the same waypoint we
    # treat it as unreachable and skip to the next one (otherwise dense
    # forests can trap the robot indefinitely).
    stuck_check_pos = None
    stuck_check_step = 0
    last_unstuck_step = -999
    stuck_count_for_target = 0
    last_target_idx_for_stuck = -1
    last_unstuck_step = -999
    stuck_count_for_target = 0
    last_target_idx_for_stuck = -1

    log_file = open_log_file(PROJECT_ROOT)

    print(f"[adaptive_navigator] mode={mode}  DWA={'on' if dwa_enabled else 'off'}  "
          f"waypoints={len(targets)}  obstacles={len(obstacles)}")

    def replan_to(goal):
        path = planner.plan((pos[0], pos[1]), goal)
        if not path or len(path) < 2:
            print(f"[plan] failed for goal={goal}, falling back to direct line")
            pp.set_path([(pos[0], pos[1]), goal])
            return
        # smooth path (skip if only 2 points)
        smoothed = smooth_path(path, samples_per_segment=6) if len(path) >= 3 else path
        pp.set_path(smoothed)
        print(f"[plan] goal={goal}: raw={len(path)} pts, smoothed={len(smoothed)} pts")

    # initial position before first step
    if robot.step(timestep) == -1:
        return
    step_count += 1
    pos = gps.getValues()
    prev_pos = (pos[0], pos[1])
    replan_to(targets[target_idx])
    last_plan_step = step_count

    while robot.step(timestep) != -1:
        step_count += 1
        sim_time = step_count * timestep / 1000.0

        if max_sim_time is not None and sim_time >= max_sim_time:
            print(f"[adaptive_navigator] reached max_sim_time={max_sim_time}s — exiting.")
            break

        pos = gps.getValues()
        rpy = imu.getRollPitchYaw()
        compass_vals = compass.getValues()
        lidar_ranges = lidar.getRangeImage()

        # Estimate body-frame velocity from GPS delta
        slip_brake = 1.0
        if prev_pos is not None:
            dx_pos = pos[0] - prev_pos[0]
            dy_pos = pos[1] - prev_pos[1]
            current_v = math.hypot(dx_pos, dy_pos) / max(dt, 1e-3)
            # Slip detection: heading mismatch between GPS-derived velocity
            # vector and the compass-reported yaw indicates traction loss.
            # When slip > 20°, scale wheel commands down (gentle: ≥75%).
            if current_v > 0.3:
                gps_heading = math.atan2(dy_pos, dx_pos)
                yaw_compass = math.atan2(compass_vals[0], compass_vals[1])
                slip_angle = gps_heading - yaw_compass
                while slip_angle > math.pi:
                    slip_angle -= 2 * math.pi
                while slip_angle < -math.pi:
                    slip_angle += 2 * math.pi
                if abs(slip_angle) > 0.35:
                    slip_brake = max(0.75, 1.0 - abs(slip_angle) / 3.0)
        prev_pos = (pos[0], pos[1])

        # Terrain classification — prefer ground-truth heightmap sampling
        # over 2D-lidar inference, which can't observe ground elevation.
        # Falls back to lidar-derived heights when the world has no heightmap
        # (e.g. flat_terrain) so the legacy code path still works.
        yaw_now = math.atan2(compass_vals[0], compass_vals[1])
        if world_file and world_file != "flat_terrain.wbt":
            height_grid = sample_local_heights(pos[0], pos[1], yaw_now,
                                               rows=5, cols=5, cell_size=0.4)
            features = extract_features(height_grid, cell_size=0.4)
        else:
            height_grid = lidar_to_height_grid(lidar_ranges)
            features = extract_features(height_grid, cell_size=0.5)
        features["imu_pitch_deg"] = math.degrees(rpy[1])
        features["imu_roll_deg"] = math.degrees(rpy[0])
        terrain = classifier.classify(features)
        params = get_params(terrain)
        if terrain != last_terrain:
            tune_dwa_for_terrain(terrain)

        # Waypoint cycling
        target = targets[target_idx]
        dist_to_goal = math.hypot(target[0] - pos[0], target[1] - pos[1])
        if dist_to_goal < GOAL_TOLERANCE:
            target_idx = (target_idx + 1) % len(targets)
            target = targets[target_idx]
            replan_to(target)
            last_plan_step = step_count
            print(f"[step {step_count}] reached waypoint, next={target}")

        # Stuck detection: 2-second window, < 0.4 m moved + >1 m to goal,
        # OR robot perched on top of an obstacle (z significantly above
        # expected terrain height). After 3 stuck recoveries on the same
        # waypoint, skip it (treat as unreachable).
        if stuck_check_pos is None:
            stuck_check_pos = (pos[0], pos[1])
            stuck_check_step = step_count
        elif step_count - stuck_check_step > 60:  # ~2 s
            moved = math.hypot(pos[0] - stuck_check_pos[0],
                               pos[1] - stuck_check_pos[1])
            if world_file:
                gnd_z = sample_terrain_height(world_file, worlds_dir, pos[0], pos[1])
                on_obstacle = (pos[2] - gnd_z) > 0.55
            else:
                on_obstacle = pos[2] > 0.6
            no_progress = (moved < 0.4 and dist_to_goal > 1.0
                           and step_count - last_unstuck_step > 100)
            perched = on_obstacle and step_count - last_unstuck_step > 80
            if no_progress or perched:
                if target_idx != last_target_idx_for_stuck:
                    stuck_count_for_target = 0
                    last_target_idx_for_stuck = target_idx
                stuck_count_for_target += 1
                if stuck_count_for_target >= 3:
                    print(f"[stuck-skip] giving up on wp[{target_idx}] after "
                          f"{stuck_count_for_target} recoveries")
                    target_idx = (target_idx + 1) % len(targets)
                    target = targets[target_idx]
                    stuck_count_for_target = 0
                    planner.clear_obstacles()
                    if obstacles:
                        planner.add_obstacles([(x, y, r + 0.30) for x, y, r in obstacles])
                    replan_to(target)
                else:
                    yaw_now2 = math.atan2(compass_vals[0], compass_vals[1])
                    vox = pos[0] + 0.7 * math.cos(yaw_now2)
                    voy = pos[1] + 0.7 * math.sin(yaw_now2)
                    planner.add_obstacles([(vox, voy, 0.6)])
                    replan_to(target)
                    reason = "perched" if perched else "no-progress"
                    print(f"[stuck-{reason}] step={step_count} z={pos[2]:.2f} "
                          f"moved={moved:.2f}m count={stuck_count_for_target}")
                last_unstuck_step = step_count
                last_plan_step = step_count
            stuck_check_pos = (pos[0], pos[1])
            stuck_check_step = step_count

        # Periodic replan + reactive replan on large cross-track error
        # (PP exposes its last lateral offset for this purpose)
        cross_track = pp.last_cross_track if (mode != "baseline" and pp.has_path()) else 0.0
        needs_replan = (step_count - last_plan_step > 400
                        or last_terrain != terrain
                        or cross_track > 2.0)
        if needs_replan:
            if world_file:
                planner.update_cost_map(height_grid, terrain.value, None)
            replan_to(target)
            last_plan_step = step_count
            last_terrain = terrain

        if mode == "baseline":
            # Direct point-to-point control (legacy behaviour, for comparison)
            bearing = get_bearing(compass_vals)
            beta = compute_steering(pos, target, bearing)
            left_speed, right_speed = differential_wheel_speeds(
                beta, params, prev_beta=prev_beta, dt=dt)
            prev_beta = beta
        else:
            # Pure Pursuit on smoothed path
            yaw = math.atan2(compass_vals[0], compass_vals[1])
            res = pp.query(pos=(pos[0], pos[1]), heading=yaw,
                           speed_mps=current_v,
                           max_lookahead=params.max_lookahead)
            if res is None:
                # path exhausted but we're not yet at the final goal — fallback
                bearing = get_bearing(compass_vals)
                beta = compute_steering(pos, target, bearing)
                left_speed, right_speed = differential_wheel_speeds(
                    beta, params, prev_beta=prev_beta, dt=dt)
                prev_beta = beta
            else:
                # Compute the bearing from the robot to the lookahead point so
                # PP can detect "target behind" and spin in place rather than
                # trying to follow a curvature that points off-map.
                t_dx = res.target[0] - pos[0]
                t_dy = res.target[1] - pos[1]
                target_bearing = math.atan2(t_dy, t_dx)
                heading_to_target = target_bearing - yaw
                while heading_to_target > math.pi:
                    heading_to_target -= 2 * math.pi
                while heading_to_target < -math.pi:
                    heading_to_target += 2 * math.pi
                d_to_goal = math.hypot(target[0] - pos[0], target[1] - pos[1])
                left_speed, right_speed = pp.steering_to_wheels(
                    res, params, dist_to_goal=d_to_goal,
                    heading_to_target=heading_to_target)

                # DWA layer — only when obstacles are registered
                if dwa_enabled and obstacles:
                    dwa_res = dwa.plan(
                        pose=(pos[0], pos[1], yaw),
                        current_v=current_v,
                        current_omega=current_omega,
                        goal=res.target,
                        obstacles=obstacles,
                    )
                    if dwa_res is not None:
                        wl, wr = dwa.cmd_to_wheels(
                            dwa_res.v, dwa_res.omega,
                            wheel_radius=0.10, track_width=0.36,
                            max_wheel_omega=params.max_speed,
                        )
                        # Soft blend with PP (heavier PP if path is clear)
                        blend = 0.4 if dwa_res.clearance_cost > 0.5 else 0.0
                        left_speed = (1 - blend) * left_speed + blend * wl
                        right_speed = (1 - blend) * right_speed + blend * wr
                        current_omega = dwa_res.omega

        # Soft edge fence: when the robot is close to the world boundary AND
        # heading further out, override the wheel command with an inward turn.
        # Cheap insurance against PP/DWA producing a path that overshoots.
        edge_x = abs(pos[0]) - (WORLD_EDGE - EDGE_BUFFER)
        edge_y = abs(pos[1]) - (WORLD_EDGE - EDGE_BUFFER)
        if edge_x > 0 or edge_y > 0:
            yaw_now = math.atan2(compass_vals[0], compass_vals[1])
            inward = math.atan2(-pos[1], -pos[0])  # heading back toward origin
            err = inward - yaw_now
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi
            spin_strength = max(edge_x, edge_y) / EDGE_BUFFER  # 0..1+
            spin_strength = min(1.5, spin_strength)
            kappa = 4.0 * err
            v = params.max_speed * (1.0 - 0.6 * spin_strength)
            v_diff = kappa * 0.18 * spin_strength
            left_speed = max(-params.max_speed,
                             min(params.max_speed, v - v_diff))
            right_speed = max(-params.max_speed,
                              min(params.max_speed, v + v_diff))

        # Apply slip-brake + rate-limit before sending to motors
        left_speed *= slip_brake
        right_speed *= slip_brake
        left_speed = rate_limit(left_speed, prev_left_speed)
        right_speed = rate_limit(right_speed, prev_right_speed)
        prev_left_speed = left_speed
        prev_right_speed = right_speed

        # Apply commands (front/rear wheels share each side)
        motors[0].setVelocity(left_speed)
        motors[2].setVelocity(left_speed)
        motors[1].setVelocity(right_speed)
        motors[3].setVelocity(right_speed)

        if step_count % 10 == 0:
            log_file.write(
                f"{step_count},{sim_time:.2f},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                f"{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f},"
                f"{terrain.value},{params.max_speed:.1f},{target_idx},{dist_to_goal:.3f}\n")
            log_file.flush()

        if step_count % 100 == 1:
            wall = time.time() - t_start
            print(f"[step {step_count}|t={sim_time:.1f}s|wall={wall:.1f}s] "
                  f"pos=({pos[0]:.2f},{pos[1]:.2f}) v={current_v:.2f}m/s "
                  f"terrain={terrain.value} target[{target_idx}]={target} "
                  f"dist={dist_to_goal:.2f}")

    log_file.close()
    print("[adaptive_navigator] simulation ended.")
    if is_supervisor:
        try:
            robot.simulationQuit(0)
        except Exception as e:
            print(f"[adaptive_navigator] simulationQuit failed: {e}")


if __name__ == "__main__":
    main()
