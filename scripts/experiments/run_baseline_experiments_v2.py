"""Run baseline experiments using headless_run.py wrapper.

Simpler approach: call the existing headless_run.py script for each experiment.
"""

import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
HEADLESS_SCRIPT = PROJECT_ROOT / "scripts" / "tools" / "headless_run.py"
EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"
LOGS_DIR = PROJECT_ROOT / "data" / "logs"
NAV_LOG = LOGS_DIR / "navigation.csv"

TERRAINS = ["flat", "slope", "rough", "transition"]
CONTROLLERS = ["adaptive_navigator", "adaptive_navigator_baseline", "astar_navigator"]
SEEDS = [42, 43, 44, 45, 46]
SIM_SECONDS = 90


def run_single_experiment(terrain: str, controller: str, seed: int, iter_tag: str) -> bool:
    """Run one experiment using headless_run.py."""
    world_file = f"{terrain}_terrain.wbt"
    log_tag = f"{iter_tag}_{terrain}_{controller}_s{seed}"

    # Call headless_run.py with explicit wall-timeout
    cmd = [
        sys.executable,
        str(HEADLESS_SCRIPT),
        "--world", world_file,
        "--controller", controller,
        "--seed", str(seed),
        "--sim-seconds", str(SIM_SECONDS),
        "--wall-timeout", str(SIM_SECONDS + 60),  # 2.5x safety margin
        "--log-tag", log_tag,
        "--mode", "optimised"
    ]

    print(f"    Running: {' '.join(cmd[-10:])}")

    try:
        result = subprocess.run(
            cmd,
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=SIM_SECONDS + 90  # outer timeout
        )

        # Don't fail on non-zero exit code — wall timeout still produces valid log
        if result.returncode != 0:
            print(f"    [WARN] Exit code {result.returncode} (likely wall timeout)")

    except subprocess.TimeoutExpired:
        print(f"    [WARN] Outer timeout, but log may still be valid")
    except Exception as e:
        print(f"    [ERROR] {e}")
        return False

    # Copy navigation.csv to experiment directory — success is judged by log presence
    if NAV_LOG.exists() and NAV_LOG.stat().st_size > 100:
        output_dir = EXPERIMENTS_DIR / iter_tag
        output_dir.mkdir(parents=True, exist_ok=True)
        dest = output_dir / f"{terrain}_{controller}_seed{seed}.csv"

        import shutil
        for attempt in range(5):
            try:
                shutil.copy2(NAV_LOG, dest)
                print(f"    [OK] Saved: {dest.name} ({dest.stat().st_size} bytes)")
                return True
            except PermissionError:
                if attempt < 4:
                    time.sleep(1)
                else:
                    print(f"    [FAIL] Could not copy log (locked)")
                    return False
        return False
    else:
        print(f"    [FAIL] No navigation.csv or empty log")
        return False


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--iter-tag", default="iter00")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip experiments that already have output CSV")
    args = parser.parse_args()

    if args.quick:
        terrains = ["flat"]
        controllers = ["adaptive_navigator"]
        seeds = [42]
        print("\n[QUICK MODE] Running 1 experiment\n")
    else:
        terrains = TERRAINS
        controllers = CONTROLLERS
        seeds = SEEDS

    total = len(terrains) * len(controllers) * len(seeds)
    output_dir = EXPERIMENTS_DIR / args.iter_tag

    # Count existing experiments
    existing_count = 0
    if args.skip_existing and output_dir.exists():
        for terrain in terrains:
            for controller in controllers:
                for seed in seeds:
                    expected = output_dir / f"{terrain}_{controller}_seed{seed}.csv"
                    if expected.exists() and expected.stat().st_size > 100:
                        existing_count += 1

    remaining = total - existing_count

    print("=" * 70)
    print(f"Baseline Experiment Runner - {args.iter_tag}")
    print(f"Total: {total} experiments")
    if args.skip_existing:
        print(f"Existing: {existing_count}, Remaining: {remaining}")
    print(f"Estimated time: {remaining * (SIM_SECONDS + 30) / 60:.1f} minutes")
    print("=" * 70)

    results = {}
    completed = existing_count
    attempted = 0

    for terrain in terrains:
        for controller in controllers:
            for seed in seeds:
                exp_key = f"{terrain}_{controller}_s{seed}"
                expected_file = output_dir / f"{terrain}_{controller}_seed{seed}.csv"

                if args.skip_existing and expected_file.exists() and expected_file.stat().st_size > 100:
                    print(f"\n[SKIP] {exp_key} - already exists")
                    results[exp_key] = True
                    continue

                attempted += 1
                print(f"\n[{completed + 1}/{total}] (attempt {attempted}/{remaining}) {exp_key}")

                success = run_single_experiment(terrain, controller, seed, args.iter_tag)
                results[exp_key] = success

                if success:
                    completed += 1

                time.sleep(2)

    print("\n" + "=" * 70)
    print(f"Completed: {completed}/{total}")
    print(f"Output: {output_dir}")
    print("=" * 70)

    return 0 if completed == total else 1


if __name__ == "__main__":
    sys.exit(main())
