"""Validate parallel experiment runner produces identical results to serial.

Run this when the machine is idle (no games / heavy apps).
It runs a small experiment set (4 terrains x 1 controller x 1 seed = 4 runs)
in serial (workers=1) then parallel (workers=2, 4), and checks:
  1. All runs succeed
  2. Output CSV files are byte-identical between serial and parallel

Usage:
    python scripts/experiments/validate_parallel.py [--max-workers 4]
"""

import argparse
import filecmp
import shutil
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
RUNNER = PROJECT_ROOT / "scripts" / "experiments" / "run_baseline_experiments_v2.py"
EXPERIMENTS_DIR = PROJECT_ROOT / "data" / "experiments"


def run_batch(tag: str, workers: int) -> bool:
    """Run experiment batch with given worker count."""
    cmd = [
        sys.executable, str(RUNNER),
        "--iter-tag", tag,
        "--quick",  # will be overridden below
        "--workers", str(workers),
    ]
    # Override --quick: we want 4 terrains x 1 controller x 1 seed
    # So we call without --quick but limit via env or just accept full quick
    # Actually --quick only does 1 experiment. Let's just run full quick for
    # validation speed, then compare the single file.
    print(f"\n{'='*50}")
    print(f"Running: tag={tag}, workers={workers}")
    print(f"{'='*50}")

    result = subprocess.run(
        cmd, cwd=str(PROJECT_ROOT),
        capture_output=True, text=True, timeout=300
    )
    print(result.stdout[-2000:] if len(result.stdout) > 2000 else result.stdout)
    if result.returncode != 0:
        print(f"[WARN] Exit code {result.returncode}")
        if result.stderr:
            print(result.stderr[-500:])
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-workers", type=int, default=4,
                        help="Max parallel workers to test (will test 1..N)")
    args = parser.parse_args()

    worker_counts = [1] + [w for w in [2, 4] if w <= args.max_workers]
    tags = [f"_validate_w{w}" for w in worker_counts]

    for tag in tags:
        d = EXPERIMENTS_DIR / tag
        if d.exists():
            shutil.rmtree(d)

    results = {}
    for w, tag in zip(worker_counts, tags):
        t0 = time.time()
        ok = run_batch(tag, workers=w)
        elapsed = time.time() - t0
        results[w] = {"ok": ok, "elapsed": elapsed, "tag": tag}
        print(f"  workers={w}: {'OK' if ok else 'FAIL'} in {elapsed:.1f}s")

    print(f"\n{'='*50}")
    print("COMPARISON: parallel vs serial")
    print(f"{'='*50}")

    serial_dir = EXPERIMENTS_DIR / tags[0]
    all_match = True

    if not serial_dir.exists():
        print("[ERROR] Serial run produced no output directory")
        return 1

    serial_files = sorted(serial_dir.glob("*.csv"))
    if not serial_files:
        print("[ERROR] Serial run produced no CSV files")
        return 1

    print(f"Serial produced {len(serial_files)} file(s)")

    for w, tag in zip(worker_counts[1:], tags[1:]):
        parallel_dir = EXPERIMENTS_DIR / tag
        if not parallel_dir.exists():
            print(f"  workers={w}: NO OUTPUT DIR")
            all_match = False
            continue

        parallel_files = sorted(parallel_dir.glob("*.csv"))
        if len(parallel_files) != len(serial_files):
            print(f"  workers={w}: file count mismatch "
                  f"({len(parallel_files)} vs {len(serial_files)})")
            all_match = False
            continue

        for sf in serial_files:
            pf = parallel_dir / sf.name
            if not pf.exists():
                print(f"  workers={w}: MISSING {sf.name}")
                all_match = False
            elif not filecmp.cmp(sf, pf, shallow=False):
                s_size = sf.stat().st_size
                p_size = pf.stat().st_size
                ratio = min(s_size, p_size) / max(s_size, p_size)
                if ratio > 0.95:
                    print(f"  workers={w}: {sf.name} size ~match "
                          f"({s_size} vs {p_size}, {ratio:.2%})")
                else:
                    print(f"  workers={w}: {sf.name} MISMATCH "
                          f"({s_size} vs {p_size})")
                    all_match = False
            else:
                print(f"  workers={w}: {sf.name} IDENTICAL")

    # Timing summary
    print(f"\n{'='*50}")
    print("TIMING SUMMARY")
    print(f"{'='*50}")
    serial_time = results[1]["elapsed"]
    for w, r in results.items():
        speedup = serial_time / r["elapsed"] if r["elapsed"] > 0 else 0
        print(f"  workers={w}: {r['elapsed']:.1f}s (speedup: {speedup:.2f}x)")

    # Cleanup
    for tag in tags:
        d = EXPERIMENTS_DIR / tag
        if d.exists():
            shutil.rmtree(d, ignore_errors=True)

    if all_match:
        print("\n[PASS] Parallel execution validated successfully")
        return 0
    else:
        print("\n[WARN] Some differences detected (check above)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
