"""Run baseline experiments: all terrains × 3 controllers × 5 seeds.

Generates complete comparison data for iter 00 baseline establishment.
Output: data/experiments/iter00/<terrain>_<controller>_seed<N>.csv
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

WORLDS_DIR = PROJECT_ROOT / "worlds"
LOGS_DIR = PROJECT_ROOT / "data" / "logs"
EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"
RUNTIME_CONFIG = PROJECT_ROOT / "data" / "runtime_config.json"
NAV_LOG = LOGS_DIR / "navigation.csv"
WEBOTS_EXE = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

TERRAINS = ["flat", "slope", "rough", "transition"]
CONTROLLERS = ["adaptive_navigator", "adaptive_navigator_baseline", "astar_navigator"]
SEEDS = [42, 43, 44, 45, 46]
SIM_SECONDS = 90


def run_single_experiment(terrain: str, controller: str, seed: int, iter_tag: str) -> bool:
    """Run one experiment configuration and save results."""
    world_file = WORLDS_DIR / f"{terrain}_terrain.wbt"
    if not world_file.exists():
        print(f"    [SKIP] World not found: {world_file}")
        return False

    # Prepare runtime config
    config = {
        "world": f"{terrain}_terrain.wbt",
        "controller": controller,
        "random_seed": seed,
        "max_sim_time": SIM_SECONDS,
        "start_position": [0.0, 0.5, 0.0],
        "goal_position": [18.0, 0.5, 18.0],
        "waypoints": []  # Direct path for baseline
    }

    with open(RUNTIME_CONFIG, "w") as f:
        json.dump(config, f, indent=2)

    # Launch Webots headless
    env = os.environ.copy()
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

    print(f"    Launching Webots (timeout: {SIM_SECONDS + 15}s)...")
    try:
        proc = subprocess.Popen(
            [WEBOTS_EXE, "--stdout", "--stderr", "--batch", "--mode=fast", str(world_file)],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for simulation to complete
        time.sleep(SIM_SECONDS + 15)

        # Terminate gracefully
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

        # Give file system time to release locks
        time.sleep(2)

    except Exception as e:
        print(f"    [ERROR] Webots launch failed: {e}")
        return False

    # Save results - copy immediately while file is fresh
    if NAV_LOG.exists():
        output_dir = EXPERIMENTS_DIR / iter_tag
        output_dir.mkdir(parents=True, exist_ok=True)
        dest = output_dir / f"{terrain}_{controller}_seed{seed}.csv"

        # Retry copy if file is temporarily locked
        for attempt in range(5):
            try:
                shutil.copy2(NAV_LOG, dest)
                print(f"    [OK] Saved: {dest.name}")
                return True
            except PermissionError:
                if attempt < 4:
                    time.sleep(1)
                else:
                    print(f"    [FAIL] Could not copy log (file locked)")
                    return False
    else:
        print(f"    [FAIL] No navigation log generated")
        return False


def main():
    parser = argparse.ArgumentParser(description="Run baseline experiments")
    parser.add_argument("--iter-tag", default="iter00", help="Iteration tag for output directory")
    parser.add_argument("--quick", action="store_true", help="Quick test: 1 terrain, 1 controller, 1 seed")
    args = parser.parse_args()

    if args.quick:
        terrains = ["flat"]
        controllers = ["adaptive_navigator"]
        seeds = [42]
        print("\n[QUICK MODE] Running 1 experiment for smoke test\n")
    else:
        terrains = TERRAINS
        controllers = CONTROLLERS
        seeds = SEEDS

    total = len(terrains) * len(controllers) * len(seeds)
    print("=" * 70)
    print(f"Baseline Experiment Runner - {args.iter_tag}")
    print(f"Total experiments: {total} ({len(terrains)} terrains × {len(controllers)} controllers × {len(seeds)} seeds)")
    print(f"Simulation time per run: {SIM_SECONDS}s")
    print(f"Estimated total time: {total * (SIM_SECONDS + 20) / 60:.1f} minutes")
    print("=" * 70)

    results = {}
    completed = 0

    for terrain in terrains:
        for controller in controllers:
            for seed in seeds:
                exp_key = f"{terrain}_{controller}_s{seed}"
                print(f"\n[{completed + 1}/{total}] {exp_key}")

                success = run_single_experiment(terrain, controller, seed, args.iter_tag)
                results[exp_key] = success

                if success:
                    completed += 1

                # Brief pause between runs
                time.sleep(2)

    # Summary
    print("\n" + "=" * 70)
    print("Experiment Summary:")
    print(f"  Completed: {completed}/{total}")
    print(f"  Failed: {total - completed}/{total}")
    print(f"  Output directory: {EXPERIMENTS_DIR / args.iter_tag}")
    print("=" * 70)

    if completed == total:
        print("\n[SUCCESS] All experiments completed successfully!")
        return 0
    else:
        print(f"\n[FAILED] {total - completed} experiments failed. Check logs above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
