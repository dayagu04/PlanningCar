# Baseline vs Optimised — kinematic-simulator benchmark

Headless unicycle simulation. baseline = old P2P+PD controller; optimised = Theta* + Catmull-Rom + Pure Pursuit.

| Scene | Controller | Reached | Sim time (s) | Path (m) | Avg v (m/s) | Yaw σ (rad) | Collisions |
|-------|------------|--------:|-------------:|---------:|------------:|------------:|-----------:|
| flat_clear | baseline | ✓ | 33.25 | 58.37 | 1.76 | 0.01899 | 0 |
| flat_clear | optimised | ✓ | 35.55 | 58.76 | 1.65 | 0.03492 | 0 |
| flat_obstacles | baseline | ✓ | 36.29 | 63.92 | 1.76 | 0.01294 | 0 |
| flat_obstacles | optimised | ✓ | 38.40 | 64.15 | 1.67 | 0.03287 | 0 |
| cluttered_field | baseline | 0/3 | 160.00 | 7.79 | 0.05 | 0.00179 | 4854 |
| cluttered_field | optimised | ✓ | 105.82 | 169.21 | 1.60 | 0.03271 | 103 |
| slope_climb | baseline | 0/2 | 160.00 | 10.14 | 0.06 | 0.00158 | 4812 |
| slope_climb | optimised | ✓ | 23.71 | 36.88 | 1.55 | 0.06260 | 0 |
| rough_zigzag | baseline | 3/4 | 159.90 | 43.54 | 0.27 | 0.00847 | 3980 |
| rough_zigzag | optimised | ✓ | 48.42 | 62.26 | 1.29 | 0.01909 | 0 |
| transition_corridor | baseline | ✓ | 47.74 | 70.27 | 1.47 | 0.01036 | 0 |
| transition_corridor | optimised | ✓ | 51.68 | 71.24 | 1.38 | 0.03555 | 0 |

## Speed-up vs baseline (when both completed)
| Scene | Sim time speed-up | Path length ratio | Avg v ratio |
|-------|------------------:|------------------:|------------:|
| flat_clear | 0.94× | 1.01 | 0.94× |
| flat_obstacles | 0.95× | 1.00 | 0.95× |
| cluttered_field | — | — | — |
| slope_climb | — | — | — |
| rough_zigzag | — | — | — |
| transition_corridor | 0.92× | 1.01 | 0.94× |
