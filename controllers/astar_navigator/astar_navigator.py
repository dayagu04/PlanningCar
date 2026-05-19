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
from src.utils.terrain_sampling import sample_terrain_height

DEFAULT_TARGETS = [
    (8.0, 0.0),
    (8.0, 8.0),
    (-8.0, 8.0),
    (-8.0, -8.0),
    (8.0, -8.0),
    (0.0, 0.0),
]


def load_runtime_config():
    config_path = os.path.join(PROJECT_ROOT, "data", "runtime_config.json")
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"[astar_navigator] Failed to load runtime config: {e}")
    return {}


def load_targets():
    cfg = load_runtime_config()
    wps = cfg.get("waypoints")
    if wps and len(wps) > 0:
        return [tuple(wp) for wp in wps]
    return DEFAULT_TARGETS


def load_obstacles():
    """Return [(x, y, radius), ...] from runtime config."""
    cfg = load_runtime_config()
    obs = cfg.get("obstacles", [])
    return [(o["x"], o["y"], o["r"]) for o in obs]


def detect_world_file() -> str:
    """Find the .wbt currently being run by inspecting the worlds dir for *_temp.wbt."""
    cfg = load_runtime_config()
    if cfg.get("world_file"):
        return cfg["world_file"]
    worlds_dir = os.path.join(PROJECT_ROOT, "worlds")
    try:
        for f in os.listdir(worlds_dir):
            if f.endswith("_temp.wbt"):
                return f
    except FileNotFoundError:
        pass
    return ""


TARGETS = load_targets()
OBSTACLES = load_obstacles()
WORLD_FILE = detect_world_file()
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
DISTANCE_TOLERANCE = 0.8
REPLAN_INTERVAL = 200
LOOKAHEAD = 2.0  # meters along the path to pick a pure-pursuit point
ROBOT_CLEARANCE = 0.4  # half-diagonal of robot footprint + safety margin


def _pick_lookahead(path, pos, current_idx):
    """Advance current_idx until accumulated path distance from pos > LOOKAHEAD."""
    if not path:
        return current_idx, None
    target_idx = current_idx
    acc = 0.0
    px, py = pos[0], pos[1]
    for k in range(current_idx, len(path)):
        wx, wy = path[k]
        d = math.sqrt((wx - px) ** 2 + (wy - py) ** 2)
        if d >= LOOKAHEAD or k == len(path) - 1:
            return k, (wx, wy)
        # advance — also handle the case where we're already past this waypoint
        if d < DISTANCE_TOLERANCE * 1.5 and k > target_idx:
            target_idx = k
        px, py = wx, wy
        acc += d
        if acc >= LOOKAHEAD:
            return k, (wx, wy)
    return len(path) - 1, path[-1]


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0

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
    planner = AStarPlanner(grid_size=0.5, world_size=30.0, heuristic_weight=1.4)

    # Register static obstacles (trees + rocks) so A* routes around them
    if OBSTACLES:
        planner.add_obstacles([(x, y, r + ROBOT_CLEARANCE) for x, y, r in OBSTACLES])
        print(f"[astar_navigator] Registered {len(OBSTACLES)} obstacles")

    elevation_sampler = None
    if WORLD_FILE:
        def elevation_sampler(x, y):
            return sample_terrain_height(WORLD_FILE, WORLDS_DIR, x, y)

    target_idx = 0
    step_count = 0
    path = None
    path_idx = 0
    prev_beta = 0.0
    # Stuck detection: ring buffer of recent positions
    pos_history = []  # (step, x, y)
    last_unstuck_step = -999

    log_file = open_log_file(PROJECT_ROOT)

    print(f"[astar_navigator] Started. world={WORLD_FILE or 'unknown'}")

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

        # Stuck detection — if we've barely moved in the last ~30 steps while
        # there's still significant distance to go, treat whatever's in front
        # as a new obstacle and force a replan that routes around it.
        pos_history.append((step_count, pos[0], pos[1]))
        if len(pos_history) > 30:
            pos_history.pop(0)
        if (len(pos_history) >= 30
                and dist_to_final > 1.5
                and step_count - last_unstuck_step > 50):
            x0, y0 = pos_history[0][1], pos_history[0][2]
            moved = math.hypot(pos[0] - x0, pos[1] - y0)
            if moved < 0.25:
                # Inject a virtual obstacle just ahead in robot heading direction
                bearing_now = get_bearing(compass_vals)
                ox = pos[0] + 0.7 * math.cos(bearing_now)
                oy = pos[1] + 0.7 * math.sin(bearing_now)
                planner.add_obstacles([(ox, oy, 0.8)])
                last_unstuck_step = step_count
                path = None
                print(f"[step {step_count}] STUCK detected — adding virtual obstacle at "
                      f"({ox:.2f},{oy:.2f}), replanning")

        if path is None or step_count % REPLAN_INTERVAL == 0:
            planner.update_cost_map(height_grid, terrain.value, elevation_sampler)
            new_path = planner.plan((pos[0], pos[1]), final_target)
            if new_path and len(new_path) >= 2:
                path = new_path
                path_idx = 1
            else:
                path = [final_target]
                path_idx = 0

        # Pure-pursuit lookahead
        path_idx, current_waypoint = _pick_lookahead(path, pos, path_idx)
        if current_waypoint is None:
            current_waypoint = final_target

        bearing = get_bearing(compass_vals)
        beta = compute_steering(pos, current_waypoint, bearing)

        left_speed, right_speed = differential_wheel_speeds(beta, params,
                                                            prev_beta=prev_beta, dt=dt)
        prev_beta = beta

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
