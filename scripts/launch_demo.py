"""Interactive Webots launcher with terrain selection."""

import os
import sys
import subprocess
import json
import random
import math
import re

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
RUNTIME_CONFIG = os.path.join(PROJECT_ROOT, "data", "runtime_config.json")

from src.planning.waypoints import generate_random_waypoints, generate_random_robot_start
from src.planning.tsp_solver import optimize_waypoint_order
from src.utils.terrain_sampling import sample_terrain_height

WORLDS = {
    "1": ("flat_terrain.wbt", "平坦地形 (Flat Terrain)"),
    "2": ("slope_terrain.wbt", "斜坡地形 (Slope Terrain - 5°)"),
    "3": ("rough_terrain.wbt", "凹凸地形 (Rough Terrain)"),
    "4": ("transition_terrain.wbt", "过渡区地形 (Transition Terrain)"),
}

CONTROLLERS = {
    "1": "adaptive_navigator",
    "2": "astar_navigator",
}


def print_banner():
    print("=" * 60)
    print("  不规则地面机器人自适应导航系统 - Webots 启动器")
    print("  Adaptive Navigation System - Webots Launcher")
    print("=" * 60)
    print()


def select_world():
    print("请选择地形世界 (Select Terrain World):")
    print()
    for key, (filename, desc) in sorted(WORLDS.items()):
        print(f"  [{key}] {desc}")
    print()

    while True:
        choice = input("输入选项 (1-4): ").strip()
        if choice in WORLDS:
            return WORLDS[choice][0]
        print("无效选项，请重新输入！")


def select_controller():
    print()
    print("请选择导航算法 (Select Navigation Algorithm):")
    print()
    print("  [1] 自适应导航 (Adaptive Navigator)")
    print("  [2] A* 路径规划 (A* Path Planning)")
    print()

    while True:
        choice = input("输入选项 (1-2, 默认1): ").strip() or "1"
        if choice in CONTROLLERS:
            return CONTROLLERS[choice]
        print("无效选项，请重新输入！")


def patch_world_controller(world_file: str, controller: str,
                           robot_start: tuple, waypoints: list) -> str:
    """Create a temp world file with random robot start and visual waypoint markers."""
    world_path = os.path.join(WORLDS_DIR, world_file)
    with open(world_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Replace controller name
    content = re.sub(
        r'controller ".*?"',
        f'controller "{controller}"',
        content
    )

    # Anchor robot start z to terrain height + clearance so it doesn't spawn buried/floating
    robot_terrain_z = sample_terrain_height(world_file, WORLDS_DIR, robot_start[0], robot_start[1])
    z_height = robot_terrain_z + 0.25
    pattern = r'(Robot \{[\s\n]*translation )[\-\d\.]+\s+[\-\d\.]+\s+[\-\d\.]+'
    replacement = f'\\g<1>{robot_start[0]:.3f} {robot_start[1]:.3f} {z_height:.3f}'
    content = re.sub(pattern, replacement, content, count=1)

    # Remove all existing TARGET_MARKER blocks
    content = re.sub(
        r'DEF TARGET_MARKER_\d+ Solid \{.*?name "target_\d+"\s*\}\s*',
        '',
        content,
        flags=re.DOTALL
    )

    # Build flag-pole markers anchored at the local terrain height — visible above any slope
    # Each marker: a slim pole (1.5 m) with a glowing sphere head colored per waypoint index.
    colors = [
        ("1 0 0", "0.8 0 0"),
        ("1 0.55 0", "0.8 0.35 0"),
        ("1 1 0", "0.8 0.8 0"),
        ("0.1 1 0.1", "0 0.7 0"),
        ("0 0.7 1", "0 0.5 0.8"),
        ("0.7 0.2 1", "0.4 0 0.7"),
        ("1 0.2 0.6", "0.7 0 0.3"),
        ("0.2 1 0.7", "0 0.7 0.4"),
    ]
    pole_height = 1.5
    pole_radius = 0.04
    sphere_radius = 0.25
    markers_str = ""
    for i, (wx, wy) in enumerate(waypoints):
        base_color, emis_color = colors[i % len(colors)]
        terrain_z = sample_terrain_height(world_file, WORLDS_DIR, wx, wy)
        pole_z = terrain_z + pole_height / 2.0
        sphere_z = terrain_z + pole_height + sphere_radius * 0.5
        markers_str += f'''DEF TARGET_MARKER_{i+1} Solid {{
  translation {wx:.3f} {wy:.3f} 0
  children [
    Pose {{
      translation 0 0 {pole_z:.3f}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 0.95 0.95 0.95
            roughness 0.4
            metalness 0.1
          }}
          geometry Cylinder {{
            height {pole_height:.3f}
            radius {pole_radius:.3f}
          }}
        }}
      ]
    }}
    Pose {{
      translation 0 0 {sphere_z:.3f}
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {base_color}
            emissiveColor {emis_color}
            roughness 0.25
            metalness 0
          }}
          geometry Sphere {{
            radius {sphere_radius:.3f}
            subdivision 2
          }}
        }}
      ]
    }}
  ]
  name "target_{i+1}"
}}
'''

    # Insert markers before the Robot block
    robot_block_idx = content.find("Robot {")
    if robot_block_idx != -1:
        content = content[:robot_block_idx] + markers_str + content[robot_block_idx:]

    temp_path = world_path.replace(".wbt", "_temp.wbt")
    with open(temp_path, "w", encoding="utf-8") as f:
        f.write(content)

    return temp_path


def launch_webots(world_path: str, mode: str = "normal"):
    """Launch Webots with the selected world."""
    if not os.path.exists(WEBOTS_EXE):
        print(f"错误：找不到 Webots 可执行文件: {WEBOTS_EXE}")
        print("请检查 Webots 安装路径。")
        return False

    env = os.environ.copy()
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

    args = [WEBOTS_EXE]
    if mode == "batch":
        args.extend(["--batch", "--mode=fast"])

    args.append(world_path)

    print()
    print("=" * 60)
    print(f"启动 Webots: {os.path.basename(world_path)}")
    print("=" * 60)
    print()
    print("提示：")
    print("  - 点击播放按钮 ▶️ 开始仿真")
    print("  - 按 Ctrl+Shift+R 重置仿真")
    print("  - 关闭 Webots 窗口以退出")
    print()

    try:
        subprocess.run(args, env=env, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Webots 启动失败: {e}")
        return False
    except KeyboardInterrupt:
        print("\n用户中断。")
        return False


def cleanup_temp_files():
    """Remove temporary world files."""
    import glob
    temp_files = glob.glob(os.path.join(WORLDS_DIR, "*_temp.wbt"))
    for f in temp_files:
        try:
            os.remove(f)
        except:
            pass


def main():
    print_banner()

    world_file = select_world()
    controller = select_controller()

    print()
    print(f"已选择: {world_file} + {controller}")

    # Load obstacle list for this world (trees + rocks) so we can avoid them
    obs_path = os.path.join(WORLDS_DIR, world_file.replace(".wbt", "_obstacles.json"))
    obstacles = []
    if os.path.exists(obs_path):
        with open(obs_path, "r", encoding="utf-8") as f:
            obstacles = json.load(f)

    # Generate random robot start and waypoints
    seed = random.randint(0, 10000)
    print(f"\n随机种子: {seed} (用于复现实验)")

    robot_start = generate_random_robot_start(max_radius=3.0, seed=seed)
    random_waypoints = generate_random_waypoints(
        num_points=5,
        min_radius=6.0,
        max_radius=12.0,
        min_separation=5.0,
        seed=seed,
    )

    # Reject waypoints that land inside an obstacle (tree/rock + clearance)
    def too_close_to_obstacle(x, y, clearance=1.0):
        for obs in obstacles:
            if math.hypot(x - obs["x"], y - obs["y"]) < obs["r"] + clearance:
                return True
        return False

    cleaned_waypoints = []
    rng = random.Random(seed + 1)
    for wx, wy in random_waypoints:
        if not too_close_to_obstacle(wx, wy):
            cleaned_waypoints.append((wx, wy))
            continue
        # Try to nudge the waypoint outward until it clears
        for _ in range(30):
            nx = wx + rng.uniform(-2.0, 2.0)
            ny = wy + rng.uniform(-2.0, 2.0)
            if 6.0 <= math.hypot(nx, ny) <= 13.0 and not too_close_to_obstacle(nx, ny):
                cleaned_waypoints.append((nx, ny))
                break
        else:
            cleaned_waypoints.append((wx, wy))  # give up; keep original
    random_waypoints = cleaned_waypoints

    print(f"\n机器人起始位置: ({robot_start[0]:.2f}, {robot_start[1]:.2f})")
    print(f"随机生成 {len(random_waypoints)} 个目标点:")
    for i, (wx, wy) in enumerate(random_waypoints):
        print(f"  随机目标 {i+1}: ({wx:.2f}, {wy:.2f})")

    # Optimize visit order using TSP solver (nearest-neighbor + 2-opt)
    print("\n执行 TSP 路径优化...")
    waypoints, tsp_info = optimize_waypoint_order(robot_start, random_waypoints)
    print(f"  原始顺序总路径长度: {tsp_info['original_length']:.2f} m")
    print(f"  贪心最近邻优化后:   {tsp_info['greedy_length']:.2f} m")
    print(f"  2-opt 优化后:       {tsp_info['optimized_length']:.2f} m")
    print(f"  路径缩短:           {tsp_info['improvement_pct']:.1f}%")
    print(f"\n优化后访问顺序:")
    for i, (wx, wy) in enumerate(waypoints):
        print(f"  第 {i+1} 个: ({wx:.2f}, {wy:.2f})")

    # Save runtime config for controller to read
    os.makedirs(os.path.dirname(RUNTIME_CONFIG), exist_ok=True)
    with open(RUNTIME_CONFIG, "w", encoding="utf-8") as f:
        json.dump({
            "robot_start": list(robot_start),
            "waypoints": [list(wp) for wp in waypoints],
            "obstacles": obstacles,
            "world_file": world_file,
            "seed": seed,
            "tsp_info": tsp_info,
        }, f, indent=2)
    print(f"\n运行时配置已保存: {RUNTIME_CONFIG}")

    # Patch world file
    temp_world = patch_world_controller(world_file, controller, robot_start, waypoints)

    try:
        launch_webots(temp_world)
    finally:
        if os.path.exists(temp_world):
            os.remove(temp_world)
        cleanup_temp_files()

    print()
    print("演示结束。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断。")
        cleanup_temp_files()
        sys.exit(0)
