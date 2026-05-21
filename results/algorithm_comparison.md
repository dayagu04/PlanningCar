# Baseline vs Optimised — kinematic-simulator benchmark

Headless unicycle simulation. baseline = old P2P+PD controller; optimised = Theta* + Catmull-Rom + Pure Pursuit.

| Scene | Controller | Reached | Sim time (s) | Path (m) | Avg v (m/s) | Yaw σ (rad) | Collisions |
|-------|------------|--------:|-------------:|---------:|------------:|------------:|-----------:|
| flat_clear | baseline | ✓ | 33.25 | 58.37 | 1.76 | 0.01899 | 0 |
| flat_clear | optimised | ✓ | 36.22 | 58.61 | 1.62 | 0.03517 | 0 |
| flat_obstacles | baseline | ✓ | 36.29 | 63.92 | 1.76 | 0.01294 | 0 |
| flat_obstacles | optimised | ✓ | 39.01 | 63.99 | 1.64 | 0.03315 | 0 |
| cluttered_field | baseline | 0/3 | 160.00 | 7.79 | 0.05 | 0.00179 | 4854 |
| cluttered_field | optimised | ✓ | 57.38 | 94.00 | 1.64 | 0.03428 | 0 |
| slope_climb | baseline | 0/2 | 160.00 | 10.14 | 0.06 | 0.00158 | 4812 |
| slope_climb | optimised | ✓ | 25.50 | 37.99 | 1.49 | 0.06201 | 0 |
| rough_zigzag | baseline | 3/4 | 159.90 | 43.54 | 0.27 | 0.00847 | 3980 |
| rough_zigzag | optimised | ✓ | 49.44 | 62.34 | 1.26 | 0.01841 | 0 |
| transition_corridor | baseline | ✓ | 47.74 | 70.27 | 1.47 | 0.01036 | 0 |
| transition_corridor | optimised | ✓ | 52.29 | 71.04 | 1.36 | 0.03625 | 0 |

## Speed-up vs baseline (when both completed)
| Scene | Sim time speed-up | Path length ratio | Avg v ratio |
|-------|------------------:|------------------:|------------:|
| flat_clear | 0.92× | 1.00 | 0.92× |
| flat_obstacles | 0.93× | 1.00 | 0.93× |
| cluttered_field | — | — | — |
| slope_climb | — | — | — |
| rough_zigzag | — | — | — |
| transition_corridor | 0.91× | 1.01 | 0.92× |
