"""Stress test to find safe parallel Webots worker count.

Launches N concurrent headless Webots instances, monitors peak memory/CPU.
"""

import concurrent.futures
import psutil
import shutil
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
HEADLESS_SCRIPT = PROJECT_ROOT / "scripts" / "tools" / "headless_run.py"


def run_single_test(worker_id: int) -> dict:
    """Run one short Webots experiment with workdir isolation."""
    workdir = PROJECT_ROOT / "data" / "_workdirs" / f"stress_w{worker_id}"
    workdir.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(HEADLESS_SCRIPT),
        "--world", "rough_terrain.wbt",
        "--controller", "adaptive_navigator",
        "--seed", str(42 + worker_id),
        "--sim-seconds", "90",
        "--wall-timeout", "120",
        "--log-tag", f"stress_w{worker_id}",
        "--workdir", str(workdir),
        "--port", str(1234 + worker_id),
    ]

    t0 = time.time()
    try:
        result = subprocess.run(
            cmd,
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=150
        )
        elapsed = time.time() - t0
        # Judge success by whether navigation.csv was produced
        nav_csv = workdir / "navigation.csv"
        success = nav_csv.exists() and nav_csv.stat().st_size > 100
        return {
            "worker_id": worker_id,
            "success": success,
            "elapsed": elapsed,
            "rc": result.returncode,
        }
    except subprocess.TimeoutExpired:
        return {
            "worker_id": worker_id,
            "success": False,
            "elapsed": time.time() - t0,
            "error": "timeout",
        }
    except Exception as e:
        return {
            "worker_id": worker_id,
            "success": False,
            "elapsed": time.time() - t0,
            "error": str(e),
        }


def monitor_resources(duration_sec: float) -> dict:
    """Monitor system resources for duration_sec."""
    peak_mem_mb = 0
    peak_cpu_pct = 0
    samples = 0

    t_end = time.time() + duration_sec
    while time.time() < t_end:
        mem = psutil.virtual_memory()
        cpu = psutil.cpu_percent(interval=0.5)

        peak_mem_mb = max(peak_mem_mb, mem.used / 1024 / 1024)
        peak_cpu_pct = max(peak_cpu_pct, cpu)
        samples += 1

    return {
        "peak_mem_mb": peak_mem_mb,
        "peak_cpu_pct": peak_cpu_pct,
        "samples": samples,
    }


def main():
    print("=" * 70)
    print("Webots Parallel Stress Test")
    print(f"CPU cores: {psutil.cpu_count(logical=True)}")
    print(f"Total RAM: {psutil.virtual_memory().total / 1024 / 1024 / 1024:.1f} GB")
    print("=" * 70)

    worker_counts = [1, 2, 3, 4, 5, 6, 7, 8, 10, 12]
    results = {}

    for n_workers in worker_counts:
        print(f"\n[TEST] {n_workers} workers")
        print(f"  Starting {n_workers} concurrent Webots instances...")

        # Inline resource monitoring during execution
        import threading
        monitor_stop = threading.Event()
        monitor_result = {"peak_mem_mb": 0, "peak_cpu_pct": 0}

        def monitor_thread():
            while not monitor_stop.is_set():
                mem = psutil.virtual_memory()
                cpu = psutil.cpu_percent(interval=0.5)
                monitor_result["peak_mem_mb"] = max(
                    monitor_result["peak_mem_mb"], mem.used / 1024 / 1024)
                monitor_result["peak_cpu_pct"] = max(
                    monitor_result["peak_cpu_pct"], cpu)

        mon = threading.Thread(target=monitor_thread, daemon=True)
        mon.start()

        # Launch workers
        t0 = time.time()
        with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers) as executor:
            futures = [executor.submit(run_single_test, i) for i in range(n_workers)]
            worker_results = [f.result() for f in concurrent.futures.as_completed(futures)]

        elapsed_total = time.time() - t0
        monitor_stop.set()
        mon.join(timeout=2)

        # Aggregate
        success_count = sum(1 for r in worker_results if r.get("success"))
        avg_time = sum(r["elapsed"] for r in worker_results) / len(worker_results)

        results[n_workers] = {
            "success_count": success_count,
            "total_workers": n_workers,
            "avg_time": avg_time,
            "total_time": elapsed_total,
            "peak_mem_mb": monitor_result.get("peak_mem_mb", 0),
            "peak_cpu_pct": monitor_result.get("peak_cpu_pct", 0),
        }

        print(f"  Success: {success_count}/{n_workers}")
        print(f"  Avg worker time: {avg_time:.1f}s")
        print(f"  Total wall time: {elapsed_total:.1f}s")
        print(f"  Peak memory: {monitor_result.get('peak_mem_mb', 0):.0f} MB")
        print(f"  Peak CPU: {monitor_result.get('peak_cpu_pct', 0):.1f}%")

        # Safety check: if memory > 15.2 GB (97% of 15.6 GB) or all workers failed, stop
        if monitor_result.get("peak_mem_mb", 0) > 15500:
            print(f"  [STOP] Memory exceeded 15.5 GB, machine at limit")
            break
        if success_count == 0:
            print(f"  [STOP] All workers failed, no point going higher")
            break

        time.sleep(3)  # cooldown

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    for n, r in results.items():
        print(f"{n} workers: {r['success_count']}/{r['total_workers']} success, "
              f"{r['peak_mem_mb']:.0f} MB peak, {r['peak_cpu_pct']:.1f}% CPU, "
              f"{r['total_time']:.1f}s wall")

    # Recommendation
    safe_counts = [n for n, r in results.items()
                   if r["success_count"] == r["total_workers"]
                   and r["peak_mem_mb"] < 15000]
    partial_counts = [n for n, r in results.items()
                      if r["success_count"] > 0
                      and r["peak_mem_mb"] < 15000]
    if safe_counts:
        recommended = max(safe_counts)
        print(f"\n[RECOMMEND] Max safe worker count (100% success): {recommended}")
    elif partial_counts:
        recommended = max(partial_counts)
        print(f"\n[RECOMMEND] Max partial worker count (some success): {recommended}")
    else:
        print(f"\n[RECOMMEND] Safe worker count: 1 (serial only)")

    # Cleanup workdirs
    wd_root = PROJECT_ROOT / "data" / "_workdirs"
    if wd_root.exists():
        shutil.rmtree(wd_root, ignore_errors=True)


if __name__ == "__main__":
    main()
