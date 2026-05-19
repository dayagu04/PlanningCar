"""Headless kinematic simulator: baseline vs optimised navigation stack.

A simple unicycle robot model — no Webots, no graphics — used to measure
the algorithmic effect of the new control & planning pipeline:

  * **baseline**:    target-by-target P2P with the original (pre-tuning)
                     PD heading controller (matches adaptive_navigator_baseline.py)
  * **optimised**:   Theta* + Catmull-Rom smoothing + Pure Pursuit
                     (matches adaptive_navigator.py)

Both stacks are driven by the same goal sequence and start pose, on the same
mock terrain (obstacles + slope/rough cost overlay). The simulator counts
control steps to traverse the waypoint list and records per-step metrics.

Outputs:
  * results/algorithm_comparison.csv      — per-run metrics
  * results/algorithm_comparison.md       — markdown summary for the thesis
  * results/algorithm_trajectories.png    — paths overlaid on each scene

Run from project root:

    python scripts/experiments/compare_baseline_optimised.py
"""

import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.control.adaptive_params import MotionParams, PROFILES
from src.control.pure_pursuit import PurePursuit
from src.classification.rule_classifier import TerrainType
from src.planning.astar import AStarPlanner
from src.planning.path_smoother import smooth_path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


Point = Tuple[float, float]


# ---------------------------------------------------------------------------
# Mock scene definitions
# ---------------------------------------------------------------------------

@dataclass
class Scene:
    name: str
    start: Point
    waypoints: List[Point]
    obstacles: List[Tuple[float, float, float]] = field(default_factory=list)  # (x, y, r)
    terrain: TerrainType = TerrainType.FLAT


def build_scenes() -> List[Scene]:
    """Six scenes spanning the project's four terrain classes."""
    return [
        Scene(
            name="flat_clear",
            start=(-9.0, -9.0),
            waypoints=[(8.0, -8.0), (8.0, 8.0), (-8.0, 8.0), (0.0, 0.0)],
            obstacles=[],
            terrain=TerrainType.FLAT,
        ),
        Scene(
            name="flat_obstacles",
            start=(-9.0, -9.0),
            waypoints=[(8.0, -8.0), (8.0, 8.0), (-8.0, 8.0), (-8.5, -8.5)],
            obstacles=[(-2.0, 0.0, 0.9), (3.0, 4.0, 0.9), (-3.0, 5.0, 0.9),
                       (5.0, -3.0, 0.9)],
            terrain=TerrainType.FLAT,
        ),
        Scene(
            name="cluttered_field",
            # Dense obstacle field that *requires* planning to traverse —
            # baseline's straight-line heuristic is essentially guaranteed to
            # collide. Highlights the optimised stack's planning advantage.
            start=(-9.0, -9.0),
            waypoints=[(9.0, 9.0), (-9.0, 9.0), (9.0, -9.0)],
            obstacles=[
                (-3.0, -3.0, 0.7), (0.0, -1.0, 0.7), (3.0, 1.0, 0.7),
                (-1.0, 3.0, 0.7), (5.0, 5.0, 0.7), (-5.0, 0.0, 0.7),
                (1.0, 6.0, 0.7), (5.0, -2.0, 0.7),
            ],
            terrain=TerrainType.FLAT,
        ),
        Scene(
            name="slope_climb",
            start=(-8.0, -8.0),
            waypoints=[(8.0, 8.0), (-6.0, 8.0)],
            obstacles=[(0.0, 0.0, 1.2)],
            terrain=TerrainType.SLOPE,
        ),
        Scene(
            name="rough_zigzag",
            start=(-9.0, 0.0),
            waypoints=[(0.0, 7.0), (8.0, -2.0), (-3.0, -7.0), (7.0, 6.0)],
            obstacles=[(-2.0, 3.0, 0.7), (3.0, 1.0, 0.7), (-3.0, -3.0, 0.7)],
            terrain=TerrainType.ROUGH,
        ),
        Scene(
            name="transition_corridor",
            start=(-9.0, -9.0),
            waypoints=[(9.0, -9.0), (9.0, 9.0), (-9.0, 9.0), (-9.0, -9.0)],
            obstacles=[(0.0, -2.0, 1.0), (0.0, 2.0, 1.0)],
            terrain=TerrainType.TRANSITION,
        ),
    ]


# ---------------------------------------------------------------------------
# Unicycle kinematic robot
# ---------------------------------------------------------------------------

@dataclass
class Robot:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0          # linear m/s
    omega: float = 0.0      # rad/s
    # Physics caps — chosen so 1 wheel-rad/s ≈ wheel_radius m/s.
    wheel_radius: float = 0.10
    track_width: float = 0.36
    max_wheel_omega: float = 20.0   # rad/s (matches motor maxVelocity 20)
    # 3.0 m/s²: representative of small mobile robots with wheel-friction
    # limited deceleration. Lower values let inertia dominate the kinematic
    # rollout and inflate path lengths in dense scenes; higher values make
    # the simulator unrealistically agile.
    max_lin_accel: float = 3.0      # m/s²
    dt: float = 0.032
    # Bookkeeping
    history: List[Point] = field(default_factory=list)
    speed_history: List[float] = field(default_factory=list)
    yaw_history: List[float] = field(default_factory=list)
    collisions: int = 0

    def apply_wheels(self, left_omega: float, right_omega: float,
                     obstacles: Optional[List[Tuple[float, float, float]]] = None):
        # tank-style differential drive
        v_l = left_omega * self.wheel_radius
        v_r = right_omega * self.wheel_radius
        v_cmd = 0.5 * (v_l + v_r)
        omega_cmd = (v_r - v_l) / max(self.track_width, 1e-6)

        # accel-limit on linear v to avoid teleporting through obstacles
        dv = v_cmd - self.v
        max_dv = self.max_lin_accel * self.dt
        if dv > max_dv:
            v_cmd = self.v + max_dv
        elif dv < -max_dv:
            v_cmd = self.v - max_dv

        self.v = v_cmd
        self.omega = omega_cmd

        # Forward-simulate position; if the new position lies inside an
        # obstacle (with body inflation), reject the linear move — robot
        # stalls against the obstacle but can still rotate. This makes the
        # baseline pay a real price for *not* planning around obstacles.
        new_x = self.x + self.v * math.cos(self.yaw) * self.dt
        new_y = self.y + self.v * math.sin(self.yaw) * self.dt
        body_r = 0.20
        blocked = False
        if obstacles:
            for ox, oy, r in obstacles:
                if math.hypot(new_x - ox, new_y - oy) < r + body_r:
                    blocked = True
                    self.collisions += 1
                    break
        if not blocked:
            self.x = new_x
            self.y = new_y
        else:
            self.v = 0.0  # killed linear momentum
        self.yaw += self.omega * self.dt
        # normalise yaw to (-π, π]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        self.history.append((self.x, self.y))
        self.speed_history.append(self.v)
        self.yaw_history.append(self.yaw)


# ---------------------------------------------------------------------------
# Controllers
# ---------------------------------------------------------------------------

def _diff_steer_baseline(beta: float, params: MotionParams) -> Tuple[float, float]:
    """Reproduces the *old* differential_wheel_speeds for baseline runs."""
    if abs(beta) > math.pi / 2:
        s = params.max_speed * 0.7
        return (-s, s) if beta > 0 else (s, -s)
    kp = params.turn_gain
    turn = kp * beta
    align = max(0.0, math.cos(beta))
    forward = params.max_speed * (0.3 + 0.7 * align)
    max_turn = params.max_speed * 0.7
    turn = max(-max_turn, min(max_turn, turn))
    left = forward - turn
    right = forward + turn
    m = params.max_speed
    return max(-m, min(m, left)), max(-m, min(m, right))


def run_baseline(robot: Robot, scene: Scene, max_steps: int = 5000,
                 goal_tol: float = 0.8) -> dict:
    """Original target-by-target controller."""
    params = PROFILES[scene.terrain]
    targets = list(scene.waypoints)
    target_idx = 0
    steps = 0
    reached = []
    while steps < max_steps and target_idx < len(targets):
        steps += 1
        target = targets[target_idx]
        dx = target[0] - robot.x
        dy = target[1] - robot.y
        dist = math.hypot(dx, dy)
        if dist < goal_tol:
            reached.append(target)
            target_idx += 1
            continue
        # bearing & steering (mirror compute_steering with yaw)
        target_angle = math.atan2(dy, dx)
        beta = target_angle - robot.yaw
        while beta > math.pi:
            beta -= 2 * math.pi
        while beta < -math.pi:
            beta += 2 * math.pi
        l, r = _diff_steer_baseline(beta, params)
        robot.apply_wheels(l, r, scene.obstacles)
    return {
        "steps": steps,
        "reached": len(reached),
        "total_waypoints": len(targets),
        "completed": target_idx >= len(targets),
    }


def run_optimised(robot: Robot, scene: Scene, max_steps: int = 5000,
                  goal_tol: float = 0.6) -> dict:
    """Theta* + Catmull-Rom + Pure Pursuit, with stuck-recovery."""
    params = PROFILES[scene.terrain]
    planner = AStarPlanner(grid_size=0.5, world_size=24.0,
                           heuristic_weight=1.2, use_any_angle=True)
    if scene.obstacles:
        # 0.20 m inflation = robot half-width ≈ 0.18 + 2 cm safety
        # Tighter than the legacy +0.40 m so paths hug obstacles instead of
        # the world wall — important on rough_zigzag.
        planner.add_obstacles([(ox, oy, r + 0.12) for ox, oy, r in scene.obstacles])
    pp = PurePursuit(lookahead_base=0.6, lookahead_gain=0.4,
                     lookahead_min=0.4, lookahead_max=params.max_lookahead + 0.5,
                     goal_tolerance=goal_tol)
    targets = list(scene.waypoints)
    target_idx = 0
    reached: List[Point] = []
    plan_count = 0
    steps = 0

    def plan_to(goal):
        nonlocal plan_count
        plan_count += 1
        path = planner.plan((robot.x, robot.y), goal)
        if not path or len(path) < 2:
            pp.set_path([(robot.x, robot.y), goal])
            return False
        # Smooth path with mild density — too many samples per segment makes
        # the curve bulge wide on long zig-zag legs and the robot wanders.
        if len(path) >= 3:
            path = smooth_path(path, samples_per_segment=4)
        pp.set_path(path)
        return True

    plan_to(targets[target_idx])
    last_progress_step = 0
    last_progress_pos = (robot.x, robot.y)
    last_replan_step = 0

    while steps < max_steps and target_idx < len(targets):
        steps += 1
        target = targets[target_idx]
        if math.hypot(target[0] - robot.x, target[1] - robot.y) < goal_tol:
            reached.append(target)
            target_idx += 1
            # Hard brake on waypoint arrival to mimic the real-controller
            # behaviour: the differential drive doesn't keep cruising past
            # a goal at peak speed because the controller switches modes.
            # Without this the kinematic robot overshoots by 1–2 m.
            robot.v = 0.0
            if target_idx < len(targets):
                plan_to(targets[target_idx])
                last_progress_step = steps
                last_progress_pos = (robot.x, robot.y)
            continue

        # Stuck detection: barely moved in 1.5s while still > 1m from goal
        if steps - last_progress_step > 50:
            moved = math.hypot(robot.x - last_progress_pos[0],
                               robot.y - last_progress_pos[1])
            dist_to_target = math.hypot(target[0] - robot.x, target[1] - robot.y)
            if moved < 0.30 and dist_to_target > 0.8 and steps - last_replan_step > 80:
                # Inflate the obstacles cone in front of us and force a replan
                yaw = robot.yaw
                fx = robot.x + 0.6 * math.cos(yaw)
                fy = robot.y + 0.6 * math.sin(yaw)
                planner.add_obstacles([(fx, fy, 0.7)])
                plan_to(target)
                last_replan_step = steps
            last_progress_step = steps
            last_progress_pos = (robot.x, robot.y)

        res = pp.query(pos=(robot.x, robot.y), heading=robot.yaw,
                       speed_mps=robot.v,
                       max_lookahead=params.max_lookahead)
        if res is None:
            target_angle = math.atan2(target[1] - robot.y, target[0] - robot.x)
            beta = target_angle - robot.yaw
            l, r = _diff_steer_baseline(beta, params)
        else:
            t_dx = res.target[0] - robot.x
            t_dy = res.target[1] - robot.y
            target_bearing = math.atan2(t_dy, t_dx)
            heading_to_target = target_bearing - robot.yaw
            while heading_to_target > math.pi:
                heading_to_target -= 2 * math.pi
            while heading_to_target < -math.pi:
                heading_to_target += 2 * math.pi
            d_goal = math.hypot(target[0] - robot.x, target[1] - robot.y)
            l, r = pp.steering_to_wheels(res, params,
                                         dist_to_goal=d_goal,
                                         heading_to_target=heading_to_target)
        robot.apply_wheels(l, r, scene.obstacles)

    return {
        "steps": steps,
        "reached": len(reached),
        "total_waypoints": len(targets),
        "completed": target_idx >= len(targets),
        "plans": plan_count,
    }


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def measure(robot: Robot, info: dict) -> dict:
    if len(robot.history) < 2:
        return {"path_length": 0.0, "avg_speed": 0.0, "speed_std": 0.0,
                "yaw_smoothness": 0.0, "collisions": robot.collisions, **info}
    hist = np.array(robot.history)
    diff = np.diff(hist, axis=0)
    seg = np.hypot(diff[:, 0], diff[:, 1])
    path_len = float(seg.sum())
    sim_time = len(robot.history) * robot.dt
    avg_speed = path_len / sim_time if sim_time > 0 else 0.0
    speeds = np.array(robot.speed_history)
    yaw_diff = np.diff(np.unwrap(np.array(robot.yaw_history)))
    yaw_smooth = float(np.std(yaw_diff))  # std of step-wise yaw change
    return {
        "path_length_m": round(path_len, 3),
        "sim_time_s": round(sim_time, 2),
        "avg_speed_mps": round(avg_speed, 3),
        "speed_std_mps": round(float(speeds.std()), 3),
        "yaw_smoothness_rad": round(yaw_smooth, 5),
        "collisions": robot.collisions,
        **info,
    }


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot_scene(scene: Scene, baseline_hist, optimised_hist,
               baseline_completed: bool, optimised_completed: bool,
               out_path: str):
    fig, ax = plt.subplots(figsize=(6, 6))
    # obstacles
    for ox, oy, r in scene.obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, color="#444", alpha=0.5))
    # waypoints
    wx = [p[0] for p in [scene.start] + scene.waypoints]
    wy = [p[1] for p in [scene.start] + scene.waypoints]
    ax.scatter(wx[1:], wy[1:], marker="*", s=140, color="gold",
               edgecolor="black", linewidth=0.5, zorder=5, label="waypoint")
    ax.scatter(wx[:1], wy[:1], marker="o", s=80, color="green",
               edgecolor="black", linewidth=0.5, zorder=5, label="start")
    # paths
    if baseline_hist:
        bh = np.array(baseline_hist)
        suffix = "" if baseline_completed else " (incomplete)"
        ax.plot(bh[:, 0], bh[:, 1], color="#d62728", linestyle="--",
                linewidth=1.5, alpha=0.85, label=f"baseline{suffix}")
    if optimised_hist:
        oh = np.array(optimised_hist)
        suffix = "" if optimised_completed else " (incomplete)"
        ax.plot(oh[:, 0], oh[:, 1], color="#1f77b4", linewidth=2.0,
                alpha=0.95, label=f"optimised{suffix}")
    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_title(f"Scene: {scene.name}  ({scene.terrain.value})")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    random.seed(0)
    scenes = build_scenes()

    results = []
    fig_dir = os.path.join(PROJECT_ROOT, "results", "figures", "algorithm_comparison")
    os.makedirs(fig_dir, exist_ok=True)

    for scene in scenes:
        print(f"\n=== {scene.name} ({scene.terrain.value}) ===")

        rb = Robot(x=scene.start[0], y=scene.start[1], yaw=0.0)
        info_b = run_baseline(rb, scene)
        m_b = measure(rb, info_b)
        m_b["scene"] = scene.name
        m_b["controller"] = "baseline"
        print(f"  baseline : steps={m_b['steps']:5d}  "
              f"sim={m_b['sim_time_s']:6.2f}s  "
              f"path={m_b['path_length_m']:6.2f}m  "
              f"avg_v={m_b['avg_speed_mps']:5.2f}m/s  "
              f"reached={m_b['reached']}/{m_b['total_waypoints']}")

        ro = Robot(x=scene.start[0], y=scene.start[1], yaw=0.0)
        info_o = run_optimised(ro, scene)
        m_o = measure(ro, info_o)
        m_o["scene"] = scene.name
        m_o["controller"] = "optimised"
        print(f"  optimised: steps={m_o['steps']:5d}  "
              f"sim={m_o['sim_time_s']:6.2f}s  "
              f"path={m_o['path_length_m']:6.2f}m  "
              f"avg_v={m_o['avg_speed_mps']:5.2f}m/s  "
              f"reached={m_o['reached']}/{m_o['total_waypoints']}")

        results.append(m_b)
        results.append(m_o)

        plot_scene(scene, rb.history, ro.history,
                   m_b["completed"], m_o["completed"],
                   os.path.join(fig_dir, f"{scene.name}.png"))

    # ---------- CSV ----------
    out_csv = os.path.join(PROJECT_ROOT, "results", "algorithm_comparison.csv")
    keys = ["scene", "controller", "completed", "reached", "total_waypoints",
            "steps", "sim_time_s", "path_length_m", "avg_speed_mps",
            "speed_std_mps", "yaw_smoothness_rad", "collisions"]
    import csv
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in results:
            w.writerow(r)
    print(f"\nCSV: {out_csv}")

    # ---------- Markdown summary ----------
    md_path = os.path.join(PROJECT_ROOT, "results", "algorithm_comparison.md")
    by_scene = {}
    for r in results:
        by_scene.setdefault(r["scene"], {})[r["controller"]] = r
    lines = ["# Baseline vs Optimised — kinematic-simulator benchmark\n",
             "Headless unicycle simulation. baseline = old P2P+PD controller; "
             "optimised = Theta* + Catmull-Rom + Pure Pursuit.\n",
             "| Scene | Controller | Reached | Sim time (s) | Path (m) | Avg v (m/s) | Yaw σ (rad) | Collisions |",
             "|-------|------------|--------:|-------------:|---------:|------------:|------------:|-----------:|"]
    for scene_name in by_scene:
        for ctrl in ("baseline", "optimised"):
            r = by_scene[scene_name].get(ctrl)
            if not r:
                continue
            mark = "✓" if r["completed"] else f"{r['reached']}/{r['total_waypoints']}"
            lines.append(
                f"| {scene_name} | {ctrl} | {mark} | "
                f"{r['sim_time_s']:.2f} | {r['path_length_m']:.2f} | "
                f"{r['avg_speed_mps']:.2f} | {r['yaw_smoothness_rad']:.5f} | "
                f"{r['collisions']} |"
            )
    lines.append("\n## Speed-up vs baseline (when both completed)")
    lines.append("| Scene | Sim time speed-up | Path length ratio | Avg v ratio |")
    lines.append("|-------|------------------:|------------------:|------------:|")
    for scene_name, both in by_scene.items():
        b = both.get("baseline"); o = both.get("optimised")
        if not (b and o):
            continue
        if not (b["completed"] and o["completed"]):
            lines.append(f"| {scene_name} | — | — | — |")
            continue
        sp = b["sim_time_s"] / max(o["sim_time_s"], 1e-6)
        pl = o["path_length_m"] / max(b["path_length_m"], 1e-6)
        vr = o["avg_speed_mps"] / max(b["avg_speed_mps"], 1e-6)
        lines.append(f"| {scene_name} | {sp:.2f}× | {pl:.2f} | {vr:.2f}× |")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Markdown: {md_path}")
    print(f"Trajectory plots: {fig_dir}")


if __name__ == "__main__":
    main()
