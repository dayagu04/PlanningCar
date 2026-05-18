"""Run comparison experiments: adaptive_navigator vs astar_navigator on all terrains."""

import os
import sys
import subprocess
import time
import shutil
import re

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
LOGS_DIR = os.path.join(PROJECT_ROOT, "data", "experiments")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
NAV_LOG = os.path.join(PROJECT_ROOT, "data", "logs", "navigation.csv")

CONTROLLERS = ["adaptive_navigator", "astar_navigator"]
TERRAINS = ["flat_terrain", "slope_terrain", "rough_terrain", "transition_terrain"]
DURATION_S = 45


def patch_world_controller(world_path: str, controller_name: str) -> str:
    """Create a temp copy of the world file with the specified controller."""
    with open(world_path, "r", encoding="utf-8") as f:
        content = f.read()

    content = re.sub(
        r'controller ".*?"',
        f'controller "{controller_name}"',
        content
    )

    temp_path = world_path.replace(".wbt", f"_{controller_name}_tmp.wbt")
    with open(temp_path, "w", encoding="utf-8") as f:
        f.write(content)
    return temp_path


def run_single(controller: str, terrain: str) -> bool:
    world_path = os.path.join(WORLDS_DIR, f"{terrain}.wbt")
    if not os.path.exists(world_path):
        print(f"    [SKIP] {world_path} not found")
        return False

    if os.path.exists(NAV_LOG):
        os.remove(NAV_LOG)

    temp_world = patch_world_controller(world_path, controller)

    env = os.environ.copy()
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

    proc = subprocess.Popen(
        [WEBOTS_EXE, "--stdout", "--stderr", "--batch", "--mode=fast", temp_world],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )

    time.sleep(DURATION_S + 8)
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()

    os.remove(temp_world)

    if os.path.exists(NAV_LOG):
        dest = os.path.join(LOGS_DIR, f"{terrain}_{controller}.csv")
        shutil.copy2(NAV_LOG, dest)
        print(f"    [OK] -> {dest}")
        return True
    else:
        print(f"    [FAIL] No log generated")
        return False


def main():
    os.makedirs(LOGS_DIR, exist_ok=True)

    total = len(CONTROLLERS) * len(TERRAINS)
    print("=" * 60)
    print("Comparison Experiment Runner")
    print(f"Controllers: {CONTROLLERS}")
    print(f"Terrains: {TERRAINS}")
    print(f"Total runs: {total}")
    print(f"Duration per run: {DURATION_S}s")
    print(f"Estimated total time: {total * (DURATION_S + 10) // 60} min")
    print("=" * 60)

    results = {}
    for ctrl in CONTROLLERS:
        for terrain in TERRAINS:
            key = f"{terrain}/{ctrl}"
            print(f"\n[{key}] Running...")
            success = run_single(ctrl, terrain)
            results[key] = success

    print("\n" + "=" * 60)
    print("Results:")
    for key, ok in results.items():
        print(f"  {key}: {'OK' if ok else 'FAILED'}")

    success_count = sum(results.values())
    print(f"\n{success_count}/{total} experiments completed.")
    if success_count > 0:
        print(f"\nData in: {LOGS_DIR}")
        print("Next: python scripts/compare_algorithms.py")


if __name__ == "__main__":
    main()
