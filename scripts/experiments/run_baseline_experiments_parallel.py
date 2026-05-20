"""Parallel baseline experiment runner (added in `infra/parallel-experiments`).

Drop-in replacement for `run_baseline_experiments_v2.py` that runs the
`adaptive_navigator` controller with a configurable level of concurrency
(several Webots instances side-by-side), while `adaptive_navigator_baseline`
and `astar_navigator` — both on the do-not-modify list and therefore unable
to honour the per-worker `KIROZ_RUNTIME_CONFIG` env-var override — fall back
to a serial channel.

Per-worker isolation:
  - workdir   = data/parallel/<iter_tag>/<worker_id>/
  - in there: runtime_config.json + navigation.csv + <world>_temp_<id>.wbt
  - env vars `KIROZ_RUNTIME_CONFIG`, `KIROZ_LOG_DIR`, `KIROZ_LOG_NAME` thread
    the workdir into the controller process Webots launches.

The output CSV layout under `data/experiments/<iter_tag>/` is unchanged, so
downstream analysis scripts keep working without any modification.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
HEADLESS_SCRIPT = PROJECT_ROOT / "scripts" / "tools" / "headless_run.py"
EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"
WORKERS_ROOT = PROJECT_ROOT / "data" / "parallel"
LEGACY_NAV_LOG = PROJECT_ROOT / "data" / "logs" / "navigation.csv"

TERRAINS = ["flat", "slope", "rough", "transition"]
PARALLEL_OK = {"adaptive_navigator"}                     # honours KIROZ_* env vars
SERIAL_ONLY = {"adaptive_navigator_baseline", "astar_navigator"}  # do-not-modify
ALL_CONTROLLERS = list(PARALLEL_OK) + list(SERIAL_ONLY)
SEEDS = [42, 43, 44, 45, 46]
SIM_SECONDS = 90


def _expected_csv(iter_tag: str, terrain: str, controller: str, seed: int) -> Path:
    return EXPERIMENTS_DIR / iter_tag / f"{terrain}_{controller}_seed{seed}.csv"


def run_single(task: dict) -> dict:
    """Run one experiment. Designed for ProcessPoolExecutor.map / submit."""
    terrain = task["terrain"]
    controller = task["controller"]
    seed = task["seed"]
    iter_tag = task["iter_tag"]
    parallel = task["parallel"]
    worker_id = task["worker_id"]

    world_file = f"{terrain}_terrain.wbt"
    log_tag = f"{iter_tag}_{terrain}_{controller}_s{seed}_w{worker_id}"
    dest_csv = _expected_csv(iter_tag, terrain, controller, seed)
    dest_csv.parent.mkdir(parents=True, exist_ok=True)

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
    ]

    if parallel:
        workdir = WORKERS_ROOT / iter_tag / f"w{worker_id}"
        workdir.mkdir(parents=True, exist_ok=True)
        cmd.extend(["--workdir", str(workdir)])
        nav_log = workdir / "navigation.csv"
    else:
        nav_log = LEGACY_NAV_LOG

    t0 = time.time()
    rc = None
    try:
        result = subprocess.run(
            cmd,
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=SIM_SECONDS + 120,
        )
        rc = result.returncode
    except subprocess.TimeoutExpired:
        rc = -2
    except Exception as e:
        return {"key": f"{terrain}/{controller}/s{seed}", "ok": False,
                "elapsed": time.time() - t0, "error": str(e)}

    if not nav_log.exists() or nav_log.stat().st_size <= 100:
        return {"key": f"{terrain}/{controller}/s{seed}", "ok": False,
                "elapsed": time.time() - t0, "error": f"nav log missing (rc={rc})"}

    for attempt in range(5):
        try:
            shutil.copy2(nav_log, dest_csv)
            break
        except PermissionError:
            if attempt < 4:
                time.sleep(1)
            else:
                return {"key": f"{terrain}/{controller}/s{seed}", "ok": False,
                        "elapsed": time.time() - t0, "error": "log file locked"}

    return {"key": f"{terrain}/{controller}/s{seed}", "ok": True,
            "elapsed": time.time() - t0, "rc": rc, "size": dest_csv.stat().st_size}


def cleanup_workdirs(iter_tag: str):
    workdir_root = WORKERS_ROOT / iter_tag
    if workdir_root.exists():
        shutil.rmtree(workdir_root, ignore_errors=True)
    # Stray *_temp_*.wbt from previous runs
    worlds_dir = PROJECT_ROOT / "worlds"
    for stray in worlds_dir.glob("*_temp_*.wbt"):
        try:
            stray.unlink()
        except OSError:
            pass


def build_task_lists(terrains, controllers, seeds, iter_tag, skip_existing):
    parallel_tasks = []
    serial_tasks = []
    skipped = 0
    for terrain in terrains:
        for controller in controllers:
            for seed in seeds:
                expected = _expected_csv(iter_tag, terrain, controller, seed)
                if skip_existing and expected.exists() and expected.stat().st_size > 100:
                    skipped += 1
                    continue
                task = {"terrain": terrain, "controller": controller,
                        "seed": seed, "iter_tag": iter_tag,
                        "parallel": controller in PARALLEL_OK,
                        "worker_id": 0}
                if task["parallel"]:
                    parallel_tasks.append(task)
                else:
                    serial_tasks.append(task)
    return parallel_tasks, serial_tasks, skipped


def assign_worker_ids(tasks, num_workers):
    """Round-robin worker ids so the temp world filenames stay deterministic."""
    for idx, t in enumerate(tasks):
        t["worker_id"] = idx % max(1, num_workers)
    return tasks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iter-tag", required=True)
    parser.add_argument("--workers", type=int, default=4,
                        help="adaptive_navigator concurrency level")
    parser.add_argument("--quick", action="store_true",
                        help="single experiment for smoke test")
    parser.add_argument("--skip-existing", action="store_true")
    parser.add_argument("--terrains", nargs="*", default=None,
                        help="subset of terrains")
    parser.add_argument("--controllers", nargs="*", default=None,
                        help="subset of controllers")
    parser.add_argument("--seeds", nargs="*", type=int, default=None)
    parser.add_argument("--cleanup", action="store_true",
                        help="remove the per-iter parallel workdir tree on exit")
    args = parser.parse_args()

    if args.quick:
        terrains = ["flat"]
        controllers = ["adaptive_navigator"]
        seeds = [42]
    else:
        terrains = args.terrains or TERRAINS
        controllers = args.controllers or ALL_CONTROLLERS
        seeds = args.seeds or SEEDS

    workers = max(1, args.workers)
    parallel_tasks, serial_tasks, skipped = build_task_lists(
        terrains, controllers, seeds, args.iter_tag, args.skip_existing
    )
    assign_worker_ids(parallel_tasks, workers)

    total = len(parallel_tasks) + len(serial_tasks) + skipped
    print("=" * 70)
    print(f"Parallel experiment runner — iter_tag={args.iter_tag}")
    print(f"Total: {total}  Skipped(existing)={skipped}  "
          f"Parallel={len(parallel_tasks)}  Serial={len(serial_tasks)}")
    print(f"Workers (adaptive_navigator): {workers}")
    print("=" * 70)

    t_start = time.time()
    ok = skipped
    failed = []

    if parallel_tasks:
        print(f"\n--- Parallel phase: {len(parallel_tasks)} tasks × {workers} workers ---")
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {pool.submit(run_single, t): t for t in parallel_tasks}
            for i, fut in enumerate(as_completed(futures), 1):
                res = fut.result()
                tag = "OK" if res["ok"] else "FAIL"
                print(f"  [{i}/{len(parallel_tasks)}] {tag} {res['key']} "
                      f"({res['elapsed']:.1f}s)")
                if res["ok"]:
                    ok += 1
                else:
                    failed.append((res["key"], res.get("error", "?")))

    if serial_tasks:
        print(f"\n--- Serial phase: {len(serial_tasks)} tasks (do-not-modify controllers) ---")
        for i, task in enumerate(serial_tasks, 1):
            res = run_single(task)
            tag = "OK" if res["ok"] else "FAIL"
            print(f"  [{i}/{len(serial_tasks)}] {tag} {res['key']} "
                  f"({res['elapsed']:.1f}s)")
            if res["ok"]:
                ok += 1
            else:
                failed.append((res["key"], res.get("error", "?")))

    if args.cleanup:
        cleanup_workdirs(args.iter_tag)

    elapsed = time.time() - t_start
    print("\n" + "=" * 70)
    print(f"Done: {ok}/{total} ok in {elapsed:.1f}s")
    if failed:
        print(f"Failed ({len(failed)}):")
        for k, e in failed:
            print(f"  - {k}: {e}")
    print("=" * 70)
    return 0 if ok == total else 1


if __name__ == "__main__":
    sys.exit(main())
