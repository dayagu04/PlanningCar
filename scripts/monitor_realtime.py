"""Real-time performance monitor for navigation experiments.

Reads the navigation.csv log file and displays live metrics in terminal.
"""

import os
import sys
import time
import pandas as pd
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
LOG_FILE = os.path.join(PROJECT_ROOT, "data", "logs", "navigation.csv")


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def format_metric(name: str, value: float, unit: str, width: int = 25) -> str:
    return f"{name:<{width}}: {value:>8.3f} {unit}"


def monitor_loop(refresh_interval: float = 1.0):
    print("Real-time Navigation Monitor")
    print("Press Ctrl+C to stop\n")

    last_mtime = 0
    last_row_count = 0

    while True:
        try:
            if not os.path.exists(LOG_FILE):
                print(f"Waiting for log file: {LOG_FILE}")
                time.sleep(refresh_interval)
                continue

            mtime = os.path.getmtime(LOG_FILE)
            if mtime == last_mtime:
                time.sleep(refresh_interval)
                continue

            last_mtime = mtime
            df = pd.read_csv(LOG_FILE)

            if len(df) == 0:
                print("Log file is empty, waiting...")
                time.sleep(refresh_interval)
                continue

            if len(df) == last_row_count:
                time.sleep(refresh_interval)
                continue

            last_row_count = len(df)
            clear_screen()

            latest = df.iloc[-1]
            recent = df.tail(50)

            dt = recent['time_s'].diff().fillna(0.032)
            speed_actual = np.sqrt(recent['x'].diff()**2 + recent['y'].diff()**2) / dt
            speed_actual = speed_actual.clip(0, 15)

            print("=" * 60)
            print("  REAL-TIME NAVIGATION MONITOR")
            print("=" * 60)
            print()
            print("[CURRENT STATE]")
            print(format_metric("Time", latest['time_s'], "s"))
            print(format_metric("Position X", latest['x'], "m"))
            print(format_metric("Position Y", latest['y'], "m"))
            print(format_metric("Position Z", latest['z'], "m"))
            print(format_metric("Terrain Type", 0, latest['terrain']))
            print(format_metric("Target Speed", latest['speed'], "rad/s"))
            print(format_metric("Actual Speed", speed_actual.iloc[-1] if len(speed_actual) > 0 else 0, "m/s"))
            print(format_metric("Distance to Target", latest['dist_to_target'], "m"))
            print(format_metric("Waypoint Index", latest['target_idx'], ""))
            print()
            print("[ATTITUDE]")
            print(format_metric("Roll", np.degrees(latest['roll']), "deg"))
            print(format_metric("Pitch", np.degrees(latest['pitch']), "deg"))
            print(format_metric("Yaw", np.degrees(latest['yaw']), "deg"))
            print()
            print("[STATISTICS (last 50 samples)]")
            print(format_metric("Avg Speed", speed_actual.mean(), "m/s"))
            print(format_metric("Attitude Stability", np.sqrt(np.degrees(recent['roll']).std()**2 + np.degrees(recent['pitch']).std()**2), "deg"))
            print(format_metric("Avg Tracking Error", recent['dist_to_target'].mean(), "m"))
            print()
            terrain_counts = df['terrain'].value_counts().to_dict()
            print("[TERRAIN DISTRIBUTION]")
            for t, count in sorted(terrain_counts.items()):
                pct = 100 * count / len(df)
                print(f"  {t:<12}: {count:>5} ({pct:>5.1f}%)")
            print()
            print("=" * 60)
            print(f"Total samples: {len(df)}  |  Refresh: {refresh_interval}s")
            print("Press Ctrl+C to stop")

            time.sleep(refresh_interval)

        except KeyboardInterrupt:
            print("\n\nMonitor stopped.")
            break
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(refresh_interval)


if __name__ == "__main__":
    interval = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    monitor_loop(interval)
