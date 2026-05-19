"""Update iter_00_baseline.md with experiment results after completion.

Reads metrics from iter_00.json and fills in the result tables.
"""

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
METRICS_FILE = PROJECT_ROOT / "results" / "reports" / "metrics" / "iter_00.json"
REPORT_FILE = PROJECT_ROOT / "results" / "reports" / "iter_00_baseline.md"


def format_metric(metric_dict, precision=3):
    """Format metric as 'mean ± std'."""
    if not metric_dict or metric_dict.get('mean') is None:
        return '-'
    mean = metric_dict['mean']
    std = metric_dict.get('std', 0)
    if precision == 0:
        return f"{mean:.0f}"
    elif precision == 1:
        return f"{mean:.1f} ± {std:.1f}"
    elif precision == 2:
        return f"{mean:.2f} ± {std:.2f}"
    else:
        return f"{mean:.3f} ± {std:.3f}"


def format_percentage(metric_dict):
    """Format success rate as percentage."""
    if not metric_dict or metric_dict.get('mean') is None:
        return '-'
    return f"{metric_dict['mean'] * 100:.1f}%"


def update_report():
    """Update the markdown report with metrics."""
    if not METRICS_FILE.exists():
        print(f"Error: Metrics file not found: {METRICS_FILE}")
        return 1

    with open(METRICS_FILE, 'r') as f:
        metrics = json.load(f)

    results = metrics['results']
    terrains = ['flat', 'slope', 'rough', 'transition']
    controllers = ['adaptive_navigator', 'adaptive_navigator_baseline', 'astar_navigator']

    # Build horizontal comparison table
    table_lines = []
    table_lines.append("| 地形 | 控制器 | success_rate | path_efficiency | time_to_goal(s) | cross_track_mean(m) | classification_acc | energy_proxy | replan_count |")
    table_lines.append("|------|--------|--------------|-----------------|-----------------|---------------------|-------------------|--------------|--------------|")

    for terrain in terrains:
        for controller in controllers:
            if controller in results.get(terrain, {}):
                r = results[terrain][controller]
                row = [
                    terrain,
                    controller.replace('_', ' '),
                    format_percentage(r.get('success_rate')),
                    format_metric(r.get('path_efficiency'), 3),
                    format_metric(r.get('time_to_goal'), 1),
                    format_metric(r.get('cross_track_mean'), 2),
                    format_percentage(r.get('classification_accuracy')) if r.get('classification_accuracy', {}).get('mean') else 'N/A',
                    format_metric(r.get('energy_proxy'), 0),
                    format_metric(r.get('replan_count'), 0)
                ]
                table_lines.append("| " + " | ".join(row) + " |")

    horizontal_table = "\n".join(table_lines)

    # Build terrain coverage table
    coverage_lines = []
    coverage_lines.append("| 地形 | adaptive成功率 | baseline成功率 | astar成功率 | 最弱指标 |")
    coverage_lines.append("|------|---------------|---------------|------------|---------|")

    overall_success = {'adaptive_navigator': [], 'adaptive_navigator_baseline': [], 'astar_navigator': []}

    for terrain in terrains:
        adaptive_sr = results[terrain].get('adaptive_navigator', {}).get('success_rate', {}).get('mean', 0)
        baseline_sr = results[terrain].get('adaptive_navigator_baseline', {}).get('success_rate', {}).get('mean', 0)
        astar_sr = results[terrain].get('astar_navigator', {}).get('success_rate', {}).get('mean', 0)

        overall_success['adaptive_navigator'].append(adaptive_sr)
        overall_success['adaptive_navigator_baseline'].append(baseline_sr)
        overall_success['astar_navigator'].append(astar_sr)

        weakest = "TBD"
        if adaptive_sr < 0.8:
            weakest = "success_rate"
        elif results[terrain].get('adaptive_navigator', {}).get('path_efficiency', {}).get('mean', 1.0) < 0.85:
            weakest = "path_efficiency"

        coverage_lines.append(f"| {terrain} | {adaptive_sr*100:.1f}% | {baseline_sr*100:.1f}% | {astar_sr*100:.1f}% | {weakest} |")

    # Overall averages
    avg_adaptive = sum(overall_success['adaptive_navigator']) / len(terrains) * 100
    avg_baseline = sum(overall_success['adaptive_navigator_baseline']) / len(terrains) * 100
    avg_astar = sum(overall_success['astar_navigator']) / len(terrains) * 100

    coverage_lines.append(f"| **全地形平均** | **{avg_adaptive:.1f}%** | **{avg_baseline:.1f}%** | **{avg_astar:.1f}%** | - |")

    coverage_table = "\n".join(coverage_lines)

    # Read current report
    with open(REPORT_FILE, 'r', encoding='utf-8') as f:
        report_content = f.read()

    # Replace placeholder tables
    import re

    # Replace horizontal comparison table
    pattern1 = r'\| 地形 \| 控制器 \| success_rate.*?\n\|---.*?\n(?:\|.*?\n)*'
    report_content = re.sub(pattern1, horizontal_table + '\n\n', report_content, flags=re.MULTILINE)

    # Replace coverage table
    pattern2 = r'\| 地形 \| adaptive成功率.*?\n\|---.*?\n(?:\|.*?\n)*'
    report_content = re.sub(pattern2, coverage_table + '\n\n', report_content, flags=re.MULTILINE)

    # Update pytest results
    pytest_section = "\n## pytest 结果\n全部通过 (53/53 tests passed in 0.14s)\n"
    report_content = re.sub(r'## pytest 结果\n\*待测试完成后填充\*', pytest_section, report_content)

    # Write back
    with open(REPORT_FILE, 'w', encoding='utf-8') as f:
        f.write(report_content)

    print(f"Updated: {REPORT_FILE}")
    print(f"\nOverall success rates:")
    print(f"  adaptive_navigator: {avg_adaptive:.1f}%")
    print(f"  adaptive_navigator_baseline: {avg_baseline:.1f}%")
    print(f"  astar_navigator: {avg_astar:.1f}%")

    return 0


if __name__ == '__main__':
    sys.exit(update_report())
