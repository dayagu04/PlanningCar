# Iteration Reports Index

This index tracks all iterations in chronological order. Each row represents one complete iteration cycle.

| Iter | Branch | Topic | Accept | Success Δ | Path_Eff Δ | New_Terrain | Tag | Report |
|------|--------|-------|--------|-----------|------------|-------------|-----|--------|
| 00 | iter/00-baseline | Baseline establishment | ✓ | - (100% adaptive) | - (0.175 adaptive) | - | iter-00-baseline | [iter_00_baseline.md](iter_00_baseline.md) |
| 01 | iter/01-classifier-rough-imu-features | Classifier vote_window 5→9 + recent_bias | ✓ | 0% (100%→100%) | -7.6% (CV>15%) | - | iter-01-merged | [iter_01.md](iter_01.md) |
| 02 | iter/02-dwa-rough-velocity-weight | DWA velocity_weight tuning all terrains | ✓ | 0% (100%→100%) | +23.8% | - | iter-02-merged | [iter_02.md](iter_02.md) |
| 03 | iter/03-rough-speed-lookahead | ROUGH max_speed 14→15.5 + lookahead 1.2→1.5 | ✗ | -5% (100%→95%) | -30.4% | - | - | [REJECTED] slope success 80% |
| 04 | iter/04-wheel-rate-limiter-smoothing | Wheel rate limiter 25→20 rad/s² | ✓ | 0% (100%→100%) | +3.0% | - | iter-04-merged | time -8.4%, replan -16.1% |
| 05 | iter/05-align-floor-boost | align_floor 0.55→0.65 (FLAT/ROUGH/TRANS) | ✓ | 0% (100%→100%) | +2.4% | - | iter-05-merged | marginal; path_eff +2.4% |
| 06 | failed/iter-06-classifier-hysteresis | Classifier median+hysteresis (confirm_frames=2) | ✗ | 0% (100%→100%) | -15.0% | - | - | [REJECTED] class_acc +2.9% but replan_count +58% fragmented paths |
| 07 | (no run — reused as planning slot) | n/a | - | - | - | - | - | - |
| 08 | failed/iter-08-terrain-cost-weights | A* cost_multipliers ROUGH 3.0→4.5, SLOPE 2.0→2.5 | ✗ | 0% (100%→100%) | +0.0% | - | - | [REJECTED] no effect — update_cost_map only takes single terrain class |
| 09 | (no run) | n/a | - | - | - | - | - | - |
| 10 | (no run) | n/a | - | - | - | - | - | - |
| 11 | iter/11-pp-lookahead-adaptive | PP lookahead sigmoid curvature taper (0.3→1.5 rad) | ✓ | 0% (100%→100%) | +5.4% | - | iter-11-merged | smoother lookahead transition than linear |
| 12 | failed/iter-12-dwa-smoothness-too-strong | DWA yaw_resolution 0.15→0.10 + smoothness penalty 0.10·\|w\|/w_max | ✗ | -25% (15/60 fail slope+rough) | n/a | - | - | [REJECTED] penalty too strong, robot can't turn on rough |
| 13 | failed/iter-13-replan-strategy | Remove `last_terrain != terrain` from replan trigger | ✗ | 0% (100%→100%) | -15.2% | - | - | [REJECTED] replan_count +33%, cross-track grew |
| 14 | iter/14-endgame-brake-curve | PP endgame brake linear→cosine (0.5+0.25d → 0.4+0.6cos) | ✓ | 0% (100%→100%) | +3.1% | - | iter-14-merged | time -1.1%, smoother decel at goal |
| 15 | failed/iter-15-lookahead-gain-too-high | PP lookahead_gain 0.4→0.5 | ✗ | 0% (100%→100%) | -18.9% | - | - | [REJECTED] corner-cutting, cross-track up |
| 16 | failed/iter-16-no-effect | FLAT max_lookahead 1.8→2.2, TRANSITION 1.5→1.8 | ✗ | 0% (100%→100%) | +0.0% | - | - | [REJECTED] no effect — `base+gain·v` is the bottleneck, not max_lookahead |
| 17 | failed/iter-17-lookahead-base-too-high | PP lookahead_base 0.6→0.8 | ✗ | -5% (100%→95%) | -27.9% | - | - | [REJECTED] broke baseline success rate |
| 18 | failed/iter-18-dwa-graduated-blend | DWA blend binary (0.4@cc>0.5) → ramp (0–0.5) | ✗ | -5% (100%→95%) | -18.8% | - | - | [REJECTED] slope success 80%, smoother blend invited DWA into traction zones |
| 19 | failed/iter-19-no-effect | PP lookahead_min 0.4→0.3 | ✗ | 0% (100%→100%) | +0.0% | - | - | [REJECTED] no effect — sigmoid taper floor 0.60·L still > 0.4 in practice |
| 20 | failed/iter-20-curvature-brake-floor | PP curvature brake floor 0.65→0.72 | ✗ | 0% (100%→100%) | -6.1% | - | - | [REJECTED] time +28%, looser brake on hairpins kept ω rate-limit pegged |

---

## Legend
- **Accept**: ✓ = merged to main, ✗ = rejected
- **Success Δ**: Change in overall success rate vs previous iteration
- **Path_Eff Δ**: Change in path efficiency vs previous iteration
- **New_Terrain**: New terrain files added in this iteration

## Quick Stats (as of iter 20)
- Total iterations attempted: 20 (iter 07/09/10 reserved as planning slots, no runs)
- Accepted (merged to main): **8** — 00, 01, 02, 04, 05, 11, 14
- Rejected (kept as `failed/iter-NN-*` branches): **12** — 03, 06, 08, 12, 13, 15, 16, 17, 18, 19, 20
- **Current best success rate**: 100% (adaptive_navigator, all terrains)
- **Current best path efficiency**: 0.229 (adaptive_navigator, averaged across terrains)
  - cumulative gain over iter 00 baseline (0.175): **+30.9%**
  - vs. iter 05 (last full TSP-tour baseline 0.211): **+8.5%**
- **time_to_goal**: 27.8 s (-3.1% vs iter 05)
- **replan_count**: 6.05 (stable)

## Key Lessons (iter 06–20)
1. **Classifier-replan coupling** — improving classification (iter 06) increases terrain transitions, which fragments paths via the `last_terrain != terrain` replan trigger. Classifier work needs synchronized replan-strategy adjustment.
2. **A\* cost-map architecture limit** — `update_cost_map(grid, terrain.value, None)` only accepts a single global terrain label, so per-cell terrain weighting (iter 08) has no effect. Real per-terrain cost shaping requires API-level changes (out of scope for parameter sweeps).
3. **Lookahead is highly sensitive** — `lookahead_base` and `lookahead_gain` are exquisitely tuned; +33% on either parameter (iter 15, 17) breaks the path-tracker. `max_lookahead` and `lookahead_min` had no observable effect (iter 16, 19), confirming `base+gain·v` is the binding term.
4. **DWA blend is fragile** — both stricter blending (iter 12 smoothness penalty) and looser blending (iter 18 graduated ramp) regress or break success on slope/rough. The current binary `0.4 if cc>0.5 else 0` switch is at a stable local optimum.
5. **Brake floor is co-tuned with iter 11 sigmoid** — relaxing the curvature brake floor (iter 20: 0.65→0.72) without retuning lookahead causes the wheel rate-limiter to dominate, blowing out time-to-goal.

## Controller Performance Snapshot (iter 00 baseline)

| Controller | Avg Success | Avg Path Eff | Notes |
|-----------|-------------|--------------|-------|
| adaptive_navigator | **100.0%** | 0.175 | Theta* + DWA + classifier |
| adaptive_navigator_baseline | 0.0% | 0.039 | PD-only, fails on all terrains |
| astar_navigator | 10.0% | 0.120 | A* only, no local avoidance |
