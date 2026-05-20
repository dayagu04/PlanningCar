"""Run experiments with optional parallel execution.

Supports --workers N to run multiple Webots instances concurrently.
Each worker uses an isolated workdir (via headless_run.py --workdir) so
runtime_config.json, navigation.csv, and temp world files don't collide.
"""

import concurrent.futures
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
HEADLESS_SCRIPT = PROJECT_ROOT / "scripts" / "tools" / "headless_run.py"
EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"
LOGS_DIR = PROJECT_ROOT / "data" / "logs"
WORKDIRS_ROOT = PROJECT_ROOT / "data" / "_workdirs"

TERRAINS = ["flat", "slope", "rough", "transition"]
CONTROLLERS = ["adaptive_navigator", "adaptive_navigator_baseline", "astar_navigator"]
SEEDS = [42, 43, 44, 45, 46]
SIM_SECONDS = 90


def run_single_experiment(task: dict) -> dict:
    """Run one experiment with full path isolation. Designed for ProcessPool."""
    terrain = task["terrain"]
    controller = task["controller"]
    seed = task["seed"]
    iter_tag = task["iter_tag"]
    worker_id = task.get("worker_id", 0)

    world_file = f"{terrain}_terrain.wbt"
    log_tag = f"{iter_tag}_{terrain}_{controller}_s{seed}"
    exp_key = f"{terrain}_{controller}_s{seed}"

    # Create isolated workdir for this run
    workdir = WORKDIRS_ROOT / f"w{worker_id}_{terrain}_{seed}"
    workdir.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(HEADLESS_SCRIPT),
        "--world", world_file,
        "--controller", controller,
        "--seed", str(seed),
        "--sim-seconds", str(SIM_SECONDS),
        "--wall-timeout", str(SIM_SECONDS + 60),
        "--log-tag", log_tag,
        "--mode", "optimised",
        "--workdir", str(workdir),
        "--port", str(1234 + worker_id),
    ]


    try:
        result = subprocess.run(
            cmd,
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=SIM_SECONDS + 90
        )
        rc = result.returncode
    except subprocess.TimeoutExpired:
        rc = -1
    except Exception as e:
        return {"key": exp_key, "success": False, "error": str(e)}

    # Collect navigation.csv from the isolated workdir
    nav_csv = workdir / "navigation.csv"
    output_dir = EXPERIMENTS_DIR / iter_tag
    output_dir.mkdir(parents=True, exist_ok=True)
    dest = output_dir / f"{terrain}_{controller}_seed{seed}.csv"

    if nav_csv.exists() and nav_csv.stat().st_size > 100:
        for attempt in range(5):
            try:
                shutil.copy2(nav_csv, dest)
                break
            except PermissionError:
                if attempt < 4:
                    time.sleep(1)
                else:
                    return {"key": exp_key, "success": False, "error": "log locked"}
        # Cleanup workdir
        shutil.rmtree(workdir, ignore_errors=True)
        return {"key": exp_key, "success": True, "size": dest.stat().st_size}
    else:
        shutil.rmtree(workdir, ignore_errors=True)
        return {"key": exp_key, "success": False, "error": "no nav log"}


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Run experiments with optional parallel execution")
    parser.add_argument("--iter-tag", default="iter00")
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--skip-existing", action="store_true",
                        help="Skip experiments that already have output CSV")
    parser.add_argument("--workers", type=int, default=1,
                        help="Number of parallel Webots workers (default: 1 = serial)")
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

    # Collect tasks, skipping existing
    tasks = []
    skipped = 0
    for terrain in terrains:
        for controller in controllers:
            for seed in seeds:
                expected = output_dir / f"{terrain}_{controller}_seed{seed}.csv"
                if args.skip_existing and expected.exists() and expected.stat().st_size > 100:
                    skipped += 1
                    continue
                tasks.append({
                    "terrain": terrain,
                    "controller": controller,
                    "seed": seed,
                    "iter_tag": args.iter_tag,
                })

    # Assign worker_ids for workdir naming
    for i, t in enumerate(tasks):
        t["worker_id"] = i

    remaining = len(tasks)
    mode_str = f"{args.workers} workers (parallel)" if args.workers > 1 else "serial"

    print("=" * 70)
    print(f"Experiment Runner - {args.iter_tag} [{mode_str}]")
    print(f"Total: {total} | Skipped: {skipped} | Remaining: {remaining}")
    est_minutes = remaining * (SIM_SECONDS + 30) / 60 / max(args.workers, 1)
    print(f"Estimated time: {est_minutes:.1f} minutes")
    print("=" * 70)

    # Ensure workdirs root exists
    WORKDIRS_ROOT.mkdir(parents=True, exist_ok=True)

    # Execute
    t0 = time.time()
    completed = skipped
    failed_keys = []

    if args.workers > 1:
        with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers) as pool:
            future_map = {pool.submit(run_single_experiment, t): t for t in tasks}
            for future in concurrent.futures.as_completed(future_map):
                r = future.result()
                if r["success"]:
                    completed += 1
                    print(f"  [OK {completed}/{total}] {r['key']}")
                else:
                    failed_keys.append(r["key"])
                    print(f"  [FAIL] {r['key']}: {r.get('error', '?')}")
    else:
        for task in tasks:
            r = run_single_experiment(task)
            if r["success"]:
                completed += 1
                print(f"  [OK {completed}/{total}] {r['key']}")
            else:
                failed_keys.append(r["key"])
                print(f"  [FAIL] {r['key']}: {r.get('error', '?')}")
            time.sleep(2)

    elapsed = time.time() - t0
    print("\n" + "=" * 70)
    print(f"Done in {elapsed / 60:.1f} min | Success: {completed}/{total}")
    if failed_keys:
        print(f"Failed ({len(failed_keys)}): {', '.join(failed_keys[:10])}")
    print(f"Output: {output_dir}")
    print("=" * 70)

    # Cleanup workdirs root
    if WORKDIRS_ROOT.exists():
        shutil.rmtree(WORKDIRS_ROOT, ignore_errors=True)

    return 0 if completed == total else 1


if __name__ == "__main__":
    sys.exit(main())
