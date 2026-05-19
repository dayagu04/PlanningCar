"""Headless Webots smoke-test runner.

Bypasses the interactive prompts in launch_demo.py — accepts world/controller
on the command line, writes a runtime_config.json, patches the .wbt for a
fixed start + waypoints, and launches Webots in batch + fast mode for a
fixed wall-clock budget. The controller honours `max_sim_time` in the config
and exits cleanly when it elapses, which terminates Webots.

Usage:

    python scripts/tools/headless_run.py --world rough_terrain.wbt \
        --controller adaptive_navigator --seed 42 --sim-seconds 25 \
        --log-tag rough_run1

Output: data/logs/headless_<tag>.log + data/logs/navigation.csv
"""

import argparse
import json
import os
import random
import re
import shutil
import subprocess
import sys
import time
from typing import List, Tuple

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

from src.planning.waypoints import generate_random_waypoints, generate_random_robot_start
from src.planning.tsp_solver import optimize_waypoint_order
from src.utils.terrain_sampling import sample_terrain_height

WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
LOGS_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
RUNTIME_CONFIG = os.path.join(PROJECT_ROOT, "data", "runtime_config.json")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"


def patch_world(world_file: str, controller: str,
                robot_start: Tuple[float, float],
                waypoints: List[Tuple[float, float]]) -> str:
    src = os.path.join(WORLDS_DIR, world_file)
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()

    content = re.sub(r'controller ".*?"', f'controller "{controller}"', content)

    z = sample_terrain_height(world_file, WORLDS_DIR, robot_start[0], robot_start[1]) + 0.25
    content = re.sub(
        r'(Robot \{[\s\n]*translation )[\-\d\.]+\s+[\-\d\.]+\s+[\-\d\.]+',
        f'\\g<1>{robot_start[0]:.3f} {robot_start[1]:.3f} {z:.3f}',
        content, count=1,
    )
    # Strip prior target markers
    content = re.sub(
        r'DEF TARGET_MARKER_\d+ Solid \{.*?name "target_\d+"\s*\}\s*',
        '', content, flags=re.DOTALL,
    )

    # Re-add waypoint markers (flag-pole with coloured sphere)
    colors = [
        ("1 0 0", "0.8 0 0"), ("1 0.55 0", "0.8 0.35 0"),
        ("1 1 0", "0.8 0.8 0"), ("0.1 1 0.1", "0 0.7 0"),
        ("0 0.7 1", "0 0.5 0.8"), ("0.7 0.2 1", "0.4 0 0.7"),
    ]
    pole_height = 1.5
    markers_str = ""
    for i, (wx, wy) in enumerate(waypoints):
        base_color, emis_color = colors[i % len(colors)]
        tz = sample_terrain_height(world_file, WORLDS_DIR, wx, wy)
        pole_z = tz + pole_height / 2.0
        sphere_z = tz + pole_height + 0.12
        markers_str += (
            f'DEF TARGET_MARKER_{i+1} Solid {{\n'
            f'  translation {wx:.3f} {wy:.3f} 0\n'
            f'  children [\n'
            f'    Pose {{\n'
            f'      translation 0 0 {pole_z:.3f}\n'
            f'      children [\n'
            f'        Shape {{\n'
            f'          appearance PBRAppearance {{ baseColor 0.95 0.95 0.95 roughness 0.4 metalness 0.1 }}\n'
            f'          geometry Cylinder {{ height {pole_height:.3f} radius 0.04 }}\n'
            f'        }}\n'
            f'      ]\n'
            f'    }}\n'
            f'    Pose {{\n'
            f'      translation 0 0 {sphere_z:.3f}\n'
            f'      children [\n'
            f'        Shape {{\n'
            f'          appearance PBRAppearance {{ baseColor {base_color} emissiveColor {emis_color} roughness 0.25 metalness 0 }}\n'
            f'          geometry Sphere {{ radius 0.25 subdivision 2 }}\n'
            f'        }}\n'
            f'      ]\n'
            f'    }}\n'
            f'  ]\n'
            f'  name "target_{i+1}"\n'
            f'}}\n'
        )

    # Insert markers before the Robot block
    robot_idx = content.find("Robot {")
    if robot_idx != -1:
        content = content[:robot_idx] + markers_str + content[robot_idx:]

    out = src.replace(".wbt", "_temp.wbt")
    with open(out, "w", encoding="utf-8") as f:
        f.write(content)
    return out


def write_runtime_cfg(world_file, robot_start, waypoints, obstacles, sim_seconds,
                      controller_mode):
    cfg = {
        "robot_start": list(robot_start),
        "waypoints": [list(wp) for wp in waypoints],
        "obstacles": obstacles,
        "world_file": world_file,
        "max_sim_time": float(sim_seconds),
        "controller_mode": controller_mode,
        "dwa_enabled": True,
    }
    os.makedirs(os.path.dirname(RUNTIME_CONFIG), exist_ok=True)
    with open(RUNTIME_CONFIG, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--world", default="rough_terrain.wbt")
    p.add_argument("--controller", default="adaptive_navigator",
                   choices=["adaptive_navigator", "astar_navigator"])
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--sim-seconds", type=float, default=25.0,
                   help="simulated wall-clock to run before the controller exits")
    p.add_argument("--wall-timeout", type=float, default=120.0,
                   help="hard wall-clock cap on the Webots subprocess")
    p.add_argument("--log-tag", default="run")
    p.add_argument("--mode", default="optimised", choices=["optimised", "baseline"])
    p.add_argument("--visual", action="store_true",
                   help="Launch with full 3D rendering (for observation)")
    args = p.parse_args()

    if not os.path.exists(WEBOTS_EXE):
        print(f"FATAL: Webots not found at {WEBOTS_EXE}")
        sys.exit(2)

    obs_path = os.path.join(WORLDS_DIR, args.world.replace(".wbt", "_obstacles.json"))
    obstacles = json.load(open(obs_path, "r", encoding="utf-8")) if os.path.exists(obs_path) else []

    robot_start = generate_random_robot_start(max_radius=3.0, seed=args.seed)
    raw_wps = generate_random_waypoints(num_points=4, min_radius=5.0,
                                        max_radius=10.0, min_separation=4.0,
                                        seed=args.seed)
    waypoints, tsp_info = optimize_waypoint_order(robot_start, raw_wps)

    write_runtime_cfg(args.world, robot_start, waypoints, obstacles,
                      args.sim_seconds, args.mode)
    temp_world = patch_world(args.world, args.controller, robot_start, waypoints)

    os.makedirs(LOGS_DIR, exist_ok=True)
    log_path = os.path.join(LOGS_DIR, f"headless_{args.log_tag}.log")

    env = os.environ.copy()
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"
    if args.visual:
        # Visual mode: full rendering, real-time speed, no batch exit
        cmd = [WEBOTS_EXE, "--mode=realtime", "--stdout", "--stderr", temp_world]
    else:
        # Headless mode: no rendering, fast sim, batch auto-exit
        cmd = [WEBOTS_EXE, "--batch", "--mode=fast", "--no-rendering",
               "--stdout", "--stderr", temp_world]

    print(f"[headless] world={args.world} ctrl={args.controller} "
          f"seed={args.seed} sim_s={args.sim_seconds} "
          f"wall_timeout={args.wall_timeout}s")
    print(f"[headless] start={robot_start}, {len(waypoints)} waypoints")
    print(f"[headless] log -> {log_path}")
    t0 = time.time()
    try:
        with open(log_path, "w", encoding="utf-8") as logf:
            logf.write(f"# headless run: {vars(args)}\n")
            logf.write(f"# start={robot_start}\n# waypoints={waypoints}\n")
            logf.write(f"# tsp={tsp_info}\n\n")
            logf.flush()
            proc = subprocess.Popen(cmd, env=env, stdout=logf,
                                    stderr=subprocess.STDOUT, text=True)
            try:
                proc.wait(timeout=args.wall_timeout)
                rc = proc.returncode
                print(f"[headless] webots exited cleanly rc={rc} after "
                      f"{time.time()-t0:.1f}s")
            except subprocess.TimeoutExpired:
                print(f"[headless] wall timeout — killing Webots")
                proc.kill()
                proc.wait(timeout=5)
                rc = -1
    finally:
        if os.path.exists(temp_world):
            try:
                os.remove(temp_world)
            except OSError:
                pass

    print(f"[headless] log content tail:")
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    for line in lines[-40:]:
        print(f"    {line.rstrip()}")
    sys.exit(0 if rc == 0 else 1)


if __name__ == "__main__":
    main()
