# Iteration Reports Index

This index tracks all iterations in chronological order. Each row represents one complete iteration cycle.

| Iter | Branch | Topic | Accept | Success Δ | Path_Eff Δ | New_Terrain | Tag | Report |
|------|--------|-------|--------|-----------|------------|-------------|-----|--------|
| 00 | iter/00-baseline | Baseline establishment | ✓ | - (100% adaptive) | - (0.175 adaptive) | - | iter-00-baseline | [iter_00_baseline.md](iter_00_baseline.md) |
| 01 | iter/01-classifier-rough-imu-features | Classifier vote_window 5→9 + recent_bias | ✓ | 0% (100%→100%) | -7.6% (CV>15%) | - | iter-01-merged | [iter_01.md](iter_01.md) |
| 02 | iter/02-dwa-rough-velocity-weight | DWA velocity_weight tuning all terrains | ✓ | 0% (100%→100%) | +23.8% | - | iter-02-merged | [iter_02.md](iter_02.md) |
| 03 | iter/03-rough-speed-lookahead | ROUGH max_speed 14→15.5 + lookahead 1.2→1.5 | ✗ | -5% (100%→95%) | -30.4% | - | - | [REJECTED] slope success 80% |

---

## Legend
- **Accept**: ✓ = merged to main, ✗ = rejected
- **Success Δ**: Change in overall success rate vs previous iteration
- **Path_Eff Δ**: Change in path efficiency vs previous iteration
- **New_Terrain**: New terrain files added in this iteration

## Quick Stats (as of iter 00)
- Total iterations: 1
- Accepted: 1
- Rejected: 0
- **Current best success rate**: 100% (adaptive_navigator, all terrains)
- **Current best path efficiency**: 0.175 (adaptive_navigator, averaged across terrains; using start→end definition, TSP-tour adjusted def planned for iter 01)
- **Current classification accuracy**: 89.0% averaged (lowest: rough 78.5%)

## Controller Performance Snapshot (iter 00 baseline)

| Controller | Avg Success | Avg Path Eff | Notes |
|-----------|-------------|--------------|-------|
| adaptive_navigator | **100.0%** | 0.175 | Theta* + DWA + classifier |
| adaptive_navigator_baseline | 0.0% | 0.039 | PD-only, fails on all terrains |
| astar_navigator | 10.0% | 0.120 | A* only, no local avoidance |
