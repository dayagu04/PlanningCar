"""Run navigation experiments across all terrain worlds and collect data."""

import os
import sys
import subprocess
import time
import shutil

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
LOGS_DIR = os.path.join(PROJECT_ROOT, "data", "experiments")
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
NAV_LOG = os.path.join(PROJECT_ROOT, "data", "logs", "navigation.csv")

EXPERIMENTS = [
    {"name": "flat", "world": "flat_terrain.wbt", "duration_s": 60},
    {"name": "slope", "world": "slope_terrain.wbt", "duration_s": 60},
    {"name": "rough", "world": "rough_terrain.wbt", "duration_s": 60},
    {"name": "transition", "world": "transition_terrain.wbt", "duration_s": 60},
]


def run_experiment(name: str, world_file: str, duration_s: int):
    world_path = os.path.join(WORLDS_DIR, world_file)
    if not os.path.exists(world_path):
        print(f"  [SKIP] World file not found: {world_path}")
        return False

    if os.path.exists(NAV_LOG):
        os.remove(NAV_LOG)

    env = os.environ.copy()
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

    print(f"  Launching Webots ({world_file})...")
    proc = subprocess.Popen(
        [WEBOTS_EXE, "--stdout", "--stderr", "--batch", "--mode=fast", world_path],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    time.sleep(duration_s + 10)

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()

    if os.path.exists(NAV_LOG):
        dest = os.path.join(LOGS_DIR, f"{name}_navigation.csv")
        shutil.copy2(NAV_LOG, dest)
        print(f"  [OK] Data saved: {dest}")
        return True
    else:
        print(f"  [FAIL] No log generated for {name}")
        return False


def main():
    os.makedirs(LOGS_DIR, exist_ok=True)

    print("=" * 60)
    print("Batch Experiment Runner")
    print(f"Running {len(EXPERIMENTS)} experiments")
    print("=" * 60)

    results = {}
    for exp in EXPERIMENTS:
        print(f"\n[{exp['name'].upper()}] Running experiment...")
        success = run_experiment(exp["name"], exp["world"], exp["duration_s"])
        results[exp["name"]] = success

    print("\n" + "=" * 60)
    print("Results Summary:")
    for name, success in results.items():
        status = "OK" if success else "FAILED"
        print(f"  {name}: {status}")
    print("=" * 60)

    success_count = sum(results.values())
    if success_count == len(EXPERIMENTS):
        print(f"\nAll {success_count} experiments completed successfully!")
        print(f"Data saved to: {LOGS_DIR}")
        print(f"Run 'python scripts/visualize_results.py' to generate figures.")
    else:
        print(f"\n{success_count}/{len(EXPERIMENTS)} experiments succeeded.")


if __name__ == "__main__":
    main()
