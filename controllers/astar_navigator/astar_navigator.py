"""Webots A* navigation controller — uses global path planning with terrain cost."""

import sys
import os
import math

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from controller import Robot  # type: ignore
from src.perception.terrain_features import extract_features
from src.classification.rule_classifier import TerrainClassifier, TerrainType
from src.control.adaptive_params import get_params
from src.planning.astar import AStarPlanner

import numpy as np

TARGETS = [
    (8.0, 0.0),
    (8.0, 8.0),
    (-8.0, 8.0),
    (-8.0, -8.0),
    (8.0, -8.0),
    (0.0, 0.0),
]
DISTANCE_TOLERANCE = 0.8
REPLAN_INTERVAL = 200


def get_bearing(compass_values):
    return math.atan2(compass_values[0], compass_values[1])


def compute_steering(current_pos, target, bearing):
    dx = target[0] - current_pos[0]
    dy = target[1] - current_pos[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.01:
        return 0.0
    dx /= dist
    dy /= dist
    target_angle = math.atan2(dy, dx)
    beta = (target_angle - bearing) % (2 * math.pi) - math.pi
    if beta > 0:
        beta = math.pi - beta
    else:
        beta = -beta - math.pi
    return beta


def lidar_to_height_grid(lidar_ranges):
    ranges = np.array(lidar_ranges, dtype=np.float64)
    ranges = ranges[np.isfinite(ranges)]
    if len(ranges) < 10:
        return np.zeros((4, 4))
    sector_size = len(ranges) // 16
    if sector_size == 0:
        return np.zeros((4, 4))
    heights = []
    for i in range(16):
        sector = ranges[i * sector_size:(i + 1) * sector_size]
        if len(sector) > 0:
            heights.append(np.min(sector))
        else:
            heights.append(0.0)
    for i in range(16):
        sector = ranges[i * sector_size:(i + 1) * sector_size]
        heights.append(np.min(sector))
    grid = np.array(heights).reshape(4, 4)
    return grid - np.mean(grid)


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

    log_path = os.path.join(PROJECT_ROOT, "data", "logs", "navigation.csv")
    log_file = open(log_path, "w", encoding="utf-8")
    log_file.write("step,time_s,x,y,z,roll,pitch,yaw,terrain,speed,target_idx,dist_to_target\n")

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

        base_speed = params.max_speed - math.pi
        left_speed = base_speed - params.turn_gain * beta
        right_speed = base_speed + params.turn_gain * beta
        left_speed = max(-params.max_speed, min(params.max_speed, left_speed))
        right_speed = max(-params.max_speed, min(params.max_speed, right_speed))

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
