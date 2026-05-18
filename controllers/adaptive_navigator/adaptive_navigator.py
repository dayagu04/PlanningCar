"""Webots adaptive navigation controller.

Reads sensors, classifies terrain, adjusts speed accordingly.
Runs continuously until simulation is stopped.
"""

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
DISTANCE_TOLERANCE = 0.8


def load_targets():
    """Load waypoints from runtime config if available, else use defaults."""
    config_path = os.path.join(PROJECT_ROOT, "data", "runtime_config.json")
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            wps = cfg.get("waypoints")
            if wps and len(wps) > 0:
                return [tuple(wp) for wp in wps]
        except Exception as e:
            print(f"[adaptive_navigator] Failed to load runtime config: {e}")
    return DEFAULT_TARGETS


TARGETS = load_targets()


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

    log_file = open_log_file(PROJECT_ROOT)

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

        left_speed, right_speed = differential_wheel_speeds(beta, params)

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
                  f"target[{target_idx}]={target} dist={dist:.2f} "
                  f"compass=({compass_vals[0]:.3f},{compass_vals[1]:.3f}) "
                  f"bearing={math.degrees(bearing):.1f} beta={math.degrees(beta):.1f}")

    log_file.close()
    print("[adaptive_navigator] Simulation ended.")


if __name__ == "__main__":
    main()
