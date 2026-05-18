"""Webots adaptive navigation controller.

Reads sensors, classifies terrain, adjusts speed accordingly.
Runs continuously until simulation is stopped.
"""

import sys
import os
import math
import time as pytime

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from controller import Robot  # type: ignore
from src.perception.terrain_features import extract_features
from src.classification.rule_classifier import TerrainClassifier, TerrainType
from src.control.adaptive_params import get_params

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


def get_bearing(compass_values):
    """Get robot heading angle in world frame (radians).
    Compass returns north vector in robot frame.
    Webots convention: north is +Y axis."""
    # north vector x and y components in robot frame
    return math.atan2(compass_values[0], compass_values[1])


def compute_steering(current_pos, target, bearing):
    """Compute angle error using Webots official autopilot algorithm.
    Returns beta: positive = need to turn left, negative = need to turn right."""
    dx = target[0] - current_pos[0]
    dy = target[1] - current_pos[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < 0.01:
        return 0.0

    # Normalize direction
    dx /= dist
    dy /= dist

    # Webots official: robot_angle = atan2(north[0], north[1])
    #                  target_angle = atan2(direction.v, direction.u)
    # where direction.u = dx, direction.v = dy
    target_angle = math.atan2(dy, dx)

    # beta = mod(target - robot, 2*pi) - pi
    beta = (target_angle - bearing) % (2 * math.pi) - math.pi

    # Move singularity (from Webots example)
    if beta > 0:
        beta = math.pi - beta
    else:
        beta = -beta - math.pi

    return beta


def lidar_to_height_grid(lidar_ranges, num_layers=1):
    """Convert lidar range data to a simulated height grid for feature extraction.
    For single-layer 2D lidar, we estimate terrain roughness from range variance."""
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
            min_range = np.min(sector)
            heights.append(min_range)
        else:
            heights.append(0.0)

    grid = np.array(heights).reshape(4, 4)
    grid = grid - np.mean(grid)
    return grid


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
    target_idx = 0
    step_count = 0

    log_path = os.path.join(PROJECT_ROOT, "data", "logs", "navigation.csv")
    log_file = open(log_path, "w", encoding="utf-8")
    log_file.write("step,time_s,x,y,z,roll,pitch,yaw,terrain,speed,target_idx,dist_to_target\n")

    print("[adaptive_navigator] Started. Navigating through waypoints...")

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

        target = TARGETS[target_idx]
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < DISTANCE_TOLERANCE:
            target_idx = (target_idx + 1) % len(TARGETS)
            target = TARGETS[target_idx]
            print(f"[step {step_count}] Reached waypoint! Next target: {target}")
            dx = target[0] - pos[0]
            dy = target[1] - pos[1]
            dist = math.sqrt(dx * dx + dy * dy)

        bearing = get_bearing(compass_vals)
        beta = compute_steering(pos, target, bearing)

        # Webots official autopilot formula
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
                           f"{terrain.value},{params.max_speed:.1f},{target_idx},{dist:.3f}\n")
            log_file.flush()

        if step_count % 100 == 1:
            print(f"[step {step_count}] pos=({pos[0]:.2f},{pos[1]:.2f}) "
                  f"terrain={terrain.value} speed={params.max_speed:.1f} "
                  f"target[{target_idx}]={target} dist={dist:.2f}")

    log_file.close()
    print("[adaptive_navigator] Simulation ended.")


if __name__ == "__main__":
    main()
