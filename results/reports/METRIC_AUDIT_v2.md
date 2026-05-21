# Metric Audit Report — v1 vs v2

**Date**: 2026-05-21
**Trigger**: Investigation of why iter15+ "improvements" plateaued at path_eff ≈ 0.229 with no observable controller behaviour change.
**Outcome**: The original `path_efficiency` and `success_rate` metrics in `scripts/analysis/analyze_baseline.py` were ill-defined; most "improvements" merged into main between iter05 and iter14 are noise.

## TL;DR

| | v1 metric | v2 metric (corrected) |
|---|---|---|
| `path_efficiency` numerator | start→end Euclidean distance | TSP tour length (start + 4 waypoints) |
| `path_efficiency` denominator | total path length | path length up to last real visit |
| `success_rate` rule | `max(target_idx)` heuristic + 1000-row fallback | distinct waypoints physically reached (min approach < 1.2 m) |
| `time_to_goal` for failures | timeout value (30+ minutes!) | NaN |
| Failed runs in averages? | included | excluded |

Under v1, **all 21 iterations reported 100% success** for `adaptive_navigator`. Under v2, the true success rate ranges 55–75%, with rough terrain at 30–55% completion.

## v2 results across all iterations (`adaptive_navigator`)

| iter | merged? | v1 success | v1 path_eff | **v2 success** | **v2 completion** | **v2 path_eff** | slope class_acc | rough completion | verdict |
|---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| 00 | ✓ baseline | 100% | 0.175 | **60%** | 76% | 0.894 | 27% | 45% | true baseline |
| **01** | ✓ classifier vote | 100% | 0.161 | **75%** | **85%** | 0.816 | 42% | 45% | ⭐ only real improvement |
| 02 | ✓ DWA v_weight | 100% | 0.200 | 60% | 75% | 0.832 | 27% | 30% | no effect |
| 03 | ✗ ROUGH speed+L | 95% | 0.139 | — | — | — | — | — | rejection correct |
| 04 | ✓ rate limiter | 100% | 0.206 | **70%** | 80% | 0.804 | 30% | 55% | ⭐ real improvement |
| 05 | ✓ align_floor | 100% | 0.211 | 65% | 76% | 0.776 | 30% | 40% | weak +5pt |
| 06 | ✗ class hyster | 100% | 0.179 | 55% | 76% | 0.850 | 37% | 45% | rejection correct |
| 08 | ✗ A* costs | 100% | 0.211 | 65% | 76% | 0.776 | 30% | 40% | wrongly rejected (+5pt) |
| 11 | ✓ sigmoid L | 100% | 0.222 | 60% | 75% | 0.853 | 31% | 35% | no effect |
| 12 | ✗ DWA smooth | 25% | 0.013 | (incomplete data) | — | — | — | — | rejection correct |
| 13 | ✗ replan strat | 100% | 0.188 | 60% | 75% | 0.744 | 31% | 50% | no effect |
| 14 | ✓ cosine brake | 100% | 0.229 | 60% | 71% | 0.824 | 41% | 30% | no effect |
| 15 | ✗ L_gain 0.5 | 100% | 0.186 | 65% | 79% | 0.830 | 32% | 50% | wrongly rejected (+5pt) |
| 16 | ✗ L_max | 100% | 0.229 | 60% | 71% | 0.824 | 41% | 30% | rejection correct |
| 17 | ✗ L_base 0.8 | 95% | 0.165 | 65% | 79% | 0.767 | 22% | 50% | wrongly rejected (+5pt) |
| 18 | ✗ DWA blend ramp | 95% | 0.186 | 60% | 76% | 0.912 | 33% | 55% | borderline (rough +20pt) |
| 19 | ✗ L_min | 100% | 0.229 | 60% | 71% | 0.824 | 41% | 30% | rejection correct |
| 20 | ✗ brake floor | 100% | 0.215 | 60% | 74% | 0.813 | 36% | 35% | rejection correct |

## Key implications

1. **Current main (iter14 head) and iter04 are statistically indistinguishable.** Five "merged improvements" (05, 11, 14) added complexity without measurable benefit under correct metrics. iter02 also had no effect.

2. **Only two changes truly improved the controller**:
   - **iter 01** (classifier `vote_window` 5→9 + `recent_bias_window`): +15pt success
   - **iter 04** (wheel rate limiter 25→20 rad/s²): +10pt success
   The cumulative state of "iter 01 + iter 04" is approximately the current performance.

3. **Several rejections were wrong**: iter 08, 15, 17 each showed +5pt success in v2; the v1 metric pushed them into the failure bucket.

4. **The real bottlenecks are where v2 makes them obvious**:
   - **slope classification accuracy: 22–42%** — the classifier essentially does not work on slope. Pitch features dominated by lidar noise; switching to IMU pitch (already in features dict) might fix it.
   - **rough completion: 30–55%** — adaptive_navigator gets stuck on rough terrain regardless of controller-layer tuning. The stuck-recovery skip rule fires too eagerly, abandoning waypoints that are reachable.

5. **Path efficiency is now reliable but no longer the right north star**. With success at 60%, optimising path_eff means optimising the 60% of runs that already succeed; the leverage is in the 40% that fail. **Track `completion_fraction` as the primary metric** going forward.

## What this changes for iteration planning

- Pre-iter21 work: replace v1 with v2 in CI/automation; mark v1 metrics as deprecated.
- Re-baseline: declare iter04 the canonical baseline (success 70%, completion 80%) — main is currently 60%/71%.
- Future iterations target rough completion + slope classification, not path_eff.
