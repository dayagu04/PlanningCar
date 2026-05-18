"""Interactive Webots launcher with terrain selection."""

import os
import sys
import subprocess

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

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


def patch_world_controller(world_file: str, controller: str) -> str:
    """Create a temporary world file with the selected controller."""
    import re

    world_path = os.path.join(WORLDS_DIR, world_file)
    with open(world_path, "r", encoding="utf-8") as f:
        content = f.read()

    content = re.sub(
        r'controller ".*?"',
        f'controller "{controller}"',
        content
    )

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
    print()

    # Patch world file with selected controller
    temp_world = patch_world_controller(world_file, controller)

    try:
        launch_webots(temp_world)
    finally:
        # Cleanup
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
