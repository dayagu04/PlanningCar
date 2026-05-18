"""Webots A* navigation controller — uses global path planning with terrain cost."""

import sys
import os
import math
import json

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from controller import Robot  # type: ignore
from src.perception.terrain_features import extract_features
from src.classification.rule_classifier import TerrainClassifier
from src.control.adaptive_params import get_params
from src.planning.astar import AStarPlanner
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


def load_targets():
    config_path = os.path.join(PROJECT_ROOT, "data", "runtime_config.json")
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            wps = cfg.get("waypoints")
            if wps and len(wps) > 0:
                return [tuple(wp) for wp in wps]
        except Exception as e:
            print(f"[astar_navigator] Failed to load runtime config: {e}")
    return DEFAULT_TARGETS


TARGETS = load_targets()
DISTANCE_TOLERANCE = 0.8
REPLAN_INTERVAL = 200


def main():
    robot = Robot()
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
    planner = AStarPlanner(grid_size=0.5, world_size=20.0)

    target_idx = 0
    step_count = 0
    path = None
    path_idx = 0

    log_file = open_log_file(PROJECT_ROOT)

    print("[astar_navigator] Started. Planning paths with A*...")

    while robot.step(timestep) != -1:
        step_count += 1
        sim_time = step_count * timestep / 1000.0

        pos = gps.getValues()
        rpy = imu.getRollPitchYaw()
        compass_vals = compass.getValues()
        lidar_ranges = lidar.getRangeImage()

        height_grid = lidar_to_height_grid(lidar_ranges)
        features = extract_features(height_grid, cell_size=0.5)
        features["imu_pitch_deg"] = math.degrees(rpy[1])
        features["imu_roll_deg"] = math.degrees(rpy[0])
        terrain = classifier.classify(features)
        params = get_params(terrain)

        final_target = TARGETS[target_idx]
        dist_to_final = math.sqrt((final_target[0] - pos[0]) ** 2 + (final_target[1] - pos[1]) ** 2)

        if dist_to_final < DISTANCE_TOLERANCE:
            target_idx = (target_idx + 1) % len(TARGETS)
            final_target = TARGETS[target_idx]
            path = None
            print(f"[step {step_count}] Reached waypoint! Next: {final_target}")

        if path is None or step_count % REPLAN_INTERVAL == 0:
            planner.update_cost_map(height_grid, terrain.value)
            new_path = planner.plan((pos[0], pos[1]), final_target)
            if new_path and len(new_path) >= 2:
                path = new_path
                path_idx = 1
            else:
                path = [final_target]
                path_idx = 0

        if path and path_idx < len(path):
            current_waypoint = path[path_idx]
            dist_to_wp = math.sqrt((current_waypoint[0] - pos[0]) ** 2 +
                                   (current_waypoint[1] - pos[1]) ** 2)
            if dist_to_wp < DISTANCE_TOLERANCE * 2 and path_idx < len(path) - 1:
                path_idx += 1
                current_waypoint = path[path_idx]
        else:
            current_waypoint = final_target

        bearing = get_bearing(compass_vals)
        beta = compute_steering(pos, current_waypoint, bearing)

        left_speed, right_speed = differential_wheel_speeds(beta, params)

        motors[0].setVelocity(left_speed)
        motors[2].setVelocity(left_speed)
        motors[1].setVelocity(right_speed)
        motors[3].setVelocity(right_speed)

        if step_count % 10 == 0:
            log_file.write(f"{step_count},{sim_time:.2f},{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
                           f"{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f},"
                           f"{terrain.value},{params.max_speed:.1f},{target_idx},{dist_to_final:.3f}\n")
            log_file.flush()

        if step_count % 100 == 1:
            print(f"[step {step_count}] pos=({pos[0]:.2f},{pos[1]:.2f}) "
                  f"terrain={terrain.value} speed={params.max_speed:.1f} "
                  f"path_len={len(path) if path else 0} target[{target_idx}] dist={dist_to_final:.2f}")

    log_file.close()
    print("[astar_navigator] Simulation ended.")


if __name__ == "__main__":
    main()
