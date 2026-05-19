"""Initialize progression.csv for tracking iteration metrics over time.

This CSV is the primary data source for plotting iteration convergence curves
in the thesis. Each row represents one complete iteration.
"""

import csv
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
THESIS_ARCHIVE = PROJECT_ROOT / "results" / "thesis_archive"


def initialize_progression_csv():
    """Create progression.csv with header."""
    THESIS_ARCHIVE.mkdir(parents=True, exist_ok=True)

    csv_path = THESIS_ARCHIVE / "progression.csv"

    # Header
    fieldnames = [
        'iter',
        'date',
        'topic',
        'accept',
        'success_overall',
        'path_eff_overall',
        'classification_acc_overall',
        'energy_overall',
        'time_to_goal_overall',
        'cross_track_mean_overall',
        'replan_count_overall'
    ]

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

    print(f"Initialized: {csv_path}")
    return csv_path


def append_iteration(iter_num: int, date: str, topic: str, accept: bool, metrics: dict):
    """Append one iteration row to progression.csv."""
    csv_path = THESIS_ARCHIVE / "progression.csv"

    if not csv_path.exists():
        initialize_progression_csv()

    row = {
        'iter': iter_num,
        'date': date,
        'topic': topic,
        'accept': 'Y' if accept else 'N',
        'success_overall': metrics.get('success_overall', ''),
        'path_eff_overall': metrics.get('path_eff_overall', ''),
        'classification_acc_overall': metrics.get('classification_acc_overall', ''),
        'energy_overall': metrics.get('energy_overall', ''),
        'time_to_goal_overall': metrics.get('time_to_goal_overall', ''),
        'cross_track_mean_overall': metrics.get('cross_track_mean_overall', ''),
        'replan_count_overall': metrics.get('replan_count_overall', '')
    }

    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=row.keys())
        writer.writerow(row)

    print(f"Appended iter {iter_num} to progression.csv")


if __name__ == '__main__':
    initialize_progression_csv()
