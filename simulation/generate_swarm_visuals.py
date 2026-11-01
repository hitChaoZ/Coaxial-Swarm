"""Render deterministic visual artifacts for the nine-UAV swarm example.

The MATLAB files remain the reference implementation.  This renderer mirrors
the documented world, RRT path construction, APF forces, dynamic obstacles,
and double-integrator update so that PNG/GIF artifacts can be produced on a
machine without a working MATLAB graphics session.
"""

from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon
from PIL import Image


BOUNDS = np.array([[0.0, 150.0], [0.0, 90.0]])
DT = 0.10
TMAX = 70.0
VMAX = 9.0
AMAX = 7.0
DSAFE = 6.0
MPC_N = 10

NOFLY = np.array([[52.0, 72.0, 12.0], [118.0, 6.0, 9.0], [28.0, 6.0, 8.0]])
STATIC_OBS = np.array([[78.0, 72.0, 7.0], [120.0, 72.0, 6.0], [48.0, 10.0, 6.0], [88.0, 30.0, 6.0]])

START = np.array([[18.0 + ix * 14.0, 20.0 + iy * 14.0] for iy in range(3) for ix in range(3)])
GOAL = START + np.array([100.0, 8.0])

UAV_COLORS = np.array(
    [
        [0.00, 0.45, 0.74],
        [0.85, 0.33, 0.10],
        [0.47, 0.67, 0.19],
        [0.49, 0.01, 0.66],
        [0.90, 0.70, 0.00],
        [0.00, 0.60, 0.65],
        [0.78, 0.00, 0.16],
        [0.35, 0.35, 0.35],
        [0.00, 0.30, 0.55],
    ]
)


def point_segment_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    t = float(np.dot(point - a, ab) / max(denom, 1e-9))
    t = min(1.0, max(0.0, t))
    return float(np.linalg.norm(point - (a + t * ab)))


def segment_collides(a: np.ndarray, b: np.ndarray, margin: float) -> bool:
    for cx, cy, radius in np.vstack((NOFLY, STATIC_OBS)):
        if point_segment_distance(np.array([cx, cy]), a, b) < radius + margin:
            return True
    return False


def reconstruct(nodes: list[np.ndarray], parents: list[int], index: int) -> np.ndarray:
    out = []
    while index >= 0:
        out.append(nodes[index])
        index = parents[index]
    return np.asarray(out[::-1])


def smooth_path(path: np.ndarray, margin: float) -> np.ndarray:
    if len(path) < 3:
        return path
    path = path.copy()
    for _ in range(5):
        changed = False
        i = 0
        while i < len(path) - 1:
            advanced = False
            for j in range(len(path) - 1, i + 1, -1):
                if not segment_collides(path[i], path[j], margin):
                    path = np.vstack((path[: i + 1], path[j:]))
                    changed = True
                    advanced = True
                    break
            if not advanced:
                i += 1
        if not changed:
            break
    return path


def rrt_path(rng: np.random.Generator, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
    nodes = [start.copy()]
    parents = [-1]
    max_iter, step, goal_bias, goal_radius, margin = 4000, 6.0, 0.10, 7.0, 2.5
    for _ in range(max_iter):
        sample = goal.copy() if rng.random() < goal_bias else np.array(
            [rng.uniform(*BOUNDS[0]), rng.uniform(*BOUNDS[1])]
        )
        distances = np.sum((np.asarray(nodes) - sample) ** 2, axis=1)
        nearest = int(np.argmin(distances))
        direction = sample - nodes[nearest]
        distance = float(np.linalg.norm(direction))
        if distance < 1e-8:
            continue
        direction = direction / distance * min(distance, step)
        new_point = nodes[nearest] + direction
        if segment_collides(nodes[nearest], new_point, margin):
            continue
        nodes.append(new_point)
        parents.append(nearest)
        if np.linalg.norm(new_point - goal) < goal_radius and not segment_collides(new_point, goal, margin):
            return smooth_path(np.vstack((reconstruct(nodes, parents, len(nodes) - 1), goal)), margin)
    return smooth_path(np.vstack((start, goal)), margin)


def resample_path(path: np.ndarray, spacing: float = 1.0) -> np.ndarray:
    if len(path) < 2:
        return path
    segment = np.diff(path, axis=0)
    lengths = np.linalg.norm(segment, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    samples = np.arange(0.0, cumulative[-1], spacing)
    samples = np.r_[samples, cumulative[-1]]
    return np.column_stack([np.interp(samples, cumulative, path[:, dim]) for dim in range(2)])


def path_point(path: np.ndarray, arc_length: float) -> np.ndarray:
    if len(path) < 2:
        return path[-1].copy()
    segment = np.diff(path, axis=0)
    lengths = np.linalg.norm(segment, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    s = min(cumulative[-1], max(0.0, arc_length))
    return np.array([np.interp(s, cumulative, path[:, dim]) for dim in range(2)])


def nearest_arc_length(path: np.ndarray, position: np.ndarray) -> float:
    segment = np.diff(path, axis=0)
    lengths = np.linalg.norm(segment, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    best_distance, best_s = float("inf"), 0.0
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        ab = b - a
        t = float(np.dot(position - a, ab) / max(np.dot(ab, ab), 1e-9))
        t = min(1.0, max(0.0, t))
        projected = a + t * ab
        distance = float(np.linalg.norm(position - projected))
        if distance < best_distance:
            best_distance = distance
            best_s = cumulative[i] + t * lengths[i]
    return best_s


def apf_force(position: np.ndarray, waypoint: np.ndarray, neighbors: np.ndarray, dynamic: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    force = np.zeros(2)
    repulsive = np.zeros(2)
    delta = waypoint - position
    distance = float(np.linalg.norm(delta))
    if distance > 1e-3:
        force += 1.3 * delta / distance * min(distance, 25.0)

    def add_repulsive(center: np.ndarray, influence: float, gain: float) -> None:
        nonlocal force, repulsive
        delta_local = position - center
        distance_local = float(np.linalg.norm(delta_local))
        if 1e-3 < distance_local < influence:
            contribution = gain * (1.0 / distance_local - 1.0 / influence) / (distance_local**2) * delta_local
            repulsive += contribution
            force += contribution

    for cx, cy, _ in STATIC_OBS:
        add_repulsive(np.array([cx, cy]), 11.0, 35.0)
    for cx, cy, _ in NOFLY:
        add_repulsive(np.array([cx, cy]), 20.0, 70.0)
    for obstacle in dynamic:
        add_repulsive(obstacle["pos"], 11.0 + obstacle["r"], 35.0)
    for neighbor in neighbors:
        add_repulsive(neighbor, DSAFE + 5.0, 40.0)
    return force, repulsive


def clip_norm(vector: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit or norm < 1e-12 else vector / norm * limit


def simulate(seed: int, max_time: float) -> dict:
    rng = np.random.default_rng(seed)
    center_start = START.mean(axis=0)
    center_goal = GOAL.mean(axis=0)
    center_path = resample_path(rrt_path(rng, center_start, center_goal))
    form_offsets = START - center_start
    paths = [center_path + offset for offset in form_offsets]
    dynamic = [
        {"pos": np.array([70.0, 40.0]), "vel": np.array([0.0, 7.0]), "r": 5.0},
        {"pos": np.array([105.0, 40.0]), "vel": np.array([0.0, -6.0]), "r": 5.0},
    ]
    state = np.zeros((9, 4))
    state[:, :2] = START
    arrived = np.zeros(9, dtype=bool)
    trails = [[p.copy()] for p in START]
    states = [state.copy()]
    dynamic_history = [[{key: value.copy() if isinstance(value, np.ndarray) else value for key, value in item.items()} for item in dynamic]]
    times = [0.0]
    min_separation = float("inf")
    all_arrived_step = None
    max_steps = int(math.ceil(max_time / DT))

    for step in range(1, max_steps + 1):
        for obstacle in dynamic:
            obstacle["pos"] = obstacle["pos"] + obstacle["vel"] * DT
            for dim in range(2):
                lo, hi = BOUNDS[dim, 0] + obstacle["r"], BOUNDS[dim, 1] - obstacle["r"]
                if obstacle["pos"][dim] < lo or obstacle["pos"][dim] > hi:
                    obstacle["pos"][dim] = np.clip(obstacle["pos"][dim], lo, hi)
                    obstacle["vel"][dim] *= -1.0

        for i in range(9):
            position, velocity = state[i, :2].copy(), state[i, 2:].copy()
            if np.linalg.norm(position - GOAL[i]) < 3.0:
                arrived[i] = True
            neighbors = np.delete(state[:, :2], i, axis=0)
            if arrived[i]:
                desired_velocity = np.zeros(2)
                target = GOAL[i]
            else:
                s0 = nearest_arc_length(paths[i], position)
                waypoint = path_point(paths[i], s0 + 10.0)
                force, repulsive = apf_force(position, waypoint, neighbors, dynamic)
                speed = float(np.linalg.norm(force))
                desired_velocity = clip_norm(force, VMAX) if speed > 1e-6 else np.zeros(2)
                bend = clip_norm(repulsive, VMAX) * min(1.5, 1.0)
                target = path_point(paths[i], s0 + VMAX * DT) + clip_norm(bend, 5.0) * 0.22

            # A damped acceleration layer reproduces the visible DMPC envelope:
            # bounded acceleration, bounded velocity, and neighbor/obstacle APF guidance.
            acceleration = 1.2 * (target - position) + 1.8 * (desired_velocity - velocity)
            acceleration = clip_norm(acceleration, AMAX)
            next_position = position + velocity * DT + 0.5 * acceleration * DT**2
            next_velocity = clip_norm(velocity + acceleration * DT, VMAX)
            next_position[0] = np.clip(next_position[0], BOUNDS[0, 0] + 1.0, BOUNDS[0, 1] - 1.0)
            next_position[1] = np.clip(next_position[1], BOUNDS[1, 0] + 1.0, BOUNDS[1, 1] - 1.0)
            state[i] = np.r_[next_position, next_velocity]
            trails[i].append(next_position.copy())

        for i in range(8):
            for j in range(i + 1, 9):
                min_separation = min(min_separation, float(np.linalg.norm(state[i, :2] - state[j, :2])))
        current_time = step * DT
        times.append(current_time)
        states.append(state.copy())
        dynamic_history.append([{key: value.copy() if isinstance(value, np.ndarray) else value for key, value in item.items()} for item in dynamic])
        if np.all(arrived) and all_arrived_step is None:
            all_arrived_step = step
        # Keep the full horizon for the visual artifact.  The MATLAB reference
        # stops after a settling window, while a fixed 70 s timeline makes
        # late snapshots useful for checking the moving obstacles and the
        # settled formation.

    return {
        "times": np.asarray(times),
        "states": np.asarray(states),
        "dynamic": dynamic_history,
        "paths": paths,
        "trails": [np.asarray(trail) for trail in trails],
        "arrived": arrived,
        "min_separation": min_separation,
        "arrive_time": (all_arrived_step * DT if all_arrived_step is not None else None),
    }


def draw_frame(result: dict, index: int, output: io.BytesIO | Path | None = None, dpi: int = 110) -> None:
    time = float(result["times"][index])
    state = result["states"][index]
    dynamic = result["dynamic"][index]
    fig, ax = plt.subplots(figsize=(12.0, 7.0), facecolor="#f4f6f8")
    ax.set_facecolor("#fbfcfd")
    ax.set_xlim(BOUNDS[0])
    ax.set_ylim(BOUNDS[1])
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#dfe5ea", linewidth=0.7, alpha=0.8)
    ax.set_xlabel("x position (m)", color="#26323a")
    ax.set_ylabel("y position (m)", color="#26323a")
    ax.set_title(f"9-UAV coordinated obstacle avoidance   |   t = {time:05.1f} s", loc="left", fontsize=15, fontweight="bold", color="#1e2931", pad=12)

    ax.add_patch(plt.Rectangle((0, 0), 150, 90, fill=False, edgecolor="#67727b", linewidth=1.5))
    for cx, cy, radius in NOFLY:
        ax.add_patch(Circle((cx, cy), radius, facecolor="#df5b5b", edgecolor="#b52d36", alpha=0.18, linewidth=1.6))
    for cx, cy, radius in STATIC_OBS:
        ax.add_patch(Circle((cx, cy), radius, facecolor="#65717b", edgecolor="#38434b", alpha=0.52, linewidth=1.5))
    for obstacle in dynamic:
        cx, cy = obstacle["pos"]
        ax.add_patch(Circle((cx, cy), obstacle["r"], facecolor="#f59e0b", edgecolor="#c56b00", alpha=0.55, linewidth=1.7))
        ax.add_patch(Circle((cx, cy), obstacle["r"] + 2.5, fill=False, edgecolor="#f59e0b", alpha=0.42, linestyle=":", linewidth=1.0))

    for path in result["paths"]:
        ax.plot(path[:, 0], path[:, 1], linestyle="--", color="#8eb8dd", linewidth=1.0, alpha=0.75, zorder=2)
    ax.scatter(START[:, 0], START[:, 1], marker="s", s=26, c="#1f2933", label="start", zorder=4)
    ax.scatter(GOAL[:, 0], GOAL[:, 1], marker="*", s=82, c="#2e9b57", edgecolors="#1f6d3b", linewidths=0.5, label="goal", zorder=4)

    for i, (position, velocity) in enumerate(zip(state[:, :2], state[:, 2:])):
        color = UAV_COLORS[i]
        trail = result["trails"][i][: index + 1]
        if len(trail) > 1:
            ax.plot(trail[:, 0], trail[:, 1], color=color, linewidth=1.6, alpha=0.62, zorder=3)
        ax.add_patch(Circle(tuple(position), DSAFE, fill=False, edgecolor=color, linestyle=":", linewidth=0.8, alpha=0.44, zorder=4))
        angle = math.atan2(velocity[1], velocity[0]) if np.linalg.norm(velocity) > 0.15 else 0.0
        base = np.array([[2.5, 0.0], [-1.7, 1.1], [-1.7, -1.1]])
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        vertices = base @ rotation.T + position
        ax.add_patch(Polygon(vertices, closed=True, facecolor=color, edgecolor="#15202a", linewidth=0.6, zorder=6))
        ax.text(position[0], position[1] + 3.1, f"U{i + 1}", color=color, fontsize=8, ha="center", va="bottom", zorder=7)
        ax.arrow(position[0], position[1], velocity[0] * 0.55, velocity[1] * 0.55, color=color, width=0.07, head_width=0.9, head_length=1.2, length_includes_head=True, alpha=0.88, zorder=5)

    arrived_count = int(np.sum(np.linalg.norm(state[:, :2] - GOAL, axis=1) < 3.0))
    ax.text(0.015, 0.98, f"arrived  {arrived_count}/9\nmin separation  {result['min_separation']:.2f} m\nstatic / dynamic obstacles  {len(STATIC_OBS)} / {len(dynamic)}", transform=ax.transAxes, va="top", ha="left", fontsize=9.5, color="#33404a", bbox={"boxstyle": "round,pad=0.45", "facecolor": "#ffffff", "edgecolor": "#d5dce2", "alpha": 0.92})
    handles = [
        Line2D([0], [0], color="#8eb8dd", linestyle="--", label="RRT reference path"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#df5b5b", markeredgecolor="#b52d36", label="no-fly zone", markersize=8),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#65717b", markeredgecolor="#38434b", label="static obstacle", markersize=8),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#f59e0b", markeredgecolor="#c56b00", label="dynamic obstacle", markersize=8),
        Line2D([0], [0], marker="^", color="#1f2933", markerfacecolor="#1f2933", label="UAV + safety circle", markersize=7),
    ]
    ax.legend(handles=handles, loc="lower left", ncol=3, fontsize=8.5, frameon=True, facecolor="#ffffff", edgecolor="#d5dce2", framealpha=0.94)
    fig.tight_layout(pad=1.2)
    if output is None:
        return
    if isinstance(output, io.BytesIO):
        fig.savefig(output, format="png", dpi=dpi)
    else:
        fig.savefig(output, dpi=dpi)
    plt.close(fig)


def write_metrics_plot(result: dict, output: Path) -> None:
    times = result["times"]
    states = result["states"]
    distance_to_goal = np.linalg.norm(states[:, :, :2] - GOAL[None, :, :], axis=2)
    separation = np.full(len(times), np.nan)
    for k, state in enumerate(states):
        pairwise = [np.linalg.norm(state[i, :2] - state[j, :2]) for i in range(9) for j in range(i + 1, 9)]
        separation[k] = min(pairwise)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, facecolor="#f4f6f8")
    for ax in axes:
        ax.set_facecolor("#fbfcfd")
        ax.grid(True, color="#dfe5ea", linewidth=0.7)
    axes[0].plot(times, distance_to_goal, color="#4f83b7", alpha=0.28, linewidth=0.8)
    axes[0].plot(times, np.mean(distance_to_goal, axis=1), color="#1f5d8c", linewidth=2.2, label="mean distance")
    axes[0].axhline(3.0, color="#2e9b57", linestyle="--", linewidth=1.0, label="arrival threshold")
    axes[0].set_ylabel("distance to goal (m)")
    axes[0].legend(loc="upper right", frameon=False)
    axes[1].plot(times, separation, color="#d96b3d", linewidth=2.0)
    axes[1].axhline(DSAFE, color="#b52d36", linestyle="--", linewidth=1.0, label="configured safety distance")
    axes[1].set_ylabel("minimum pair separation (m)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="upper right", frameon=False)
    fig.suptitle("Swarm-level convergence and safety diagnostics", x=0.08, ha="left", fontsize=14, fontweight="bold", color="#1e2931")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(output, dpi=125)
    plt.close(fig)


def nearest_index(times: np.ndarray, target: float) -> int:
    return int(np.argmin(np.abs(times - target)))


def render(result: dict, output_dir: Path, frame_step: int, fps: int, snapshot_times: Iterable[float]) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    for stale_snapshot in figure_dir.glob("swarm_t*.png"):
        stale_snapshot.unlink()
    frame_indices = list(range(0, len(result["times"]), max(1, frame_step)))
    if frame_indices[-1] != len(result["times"]) - 1:
        frame_indices.append(len(result["times"]) - 1)
    frames: list[Image.Image] = []
    for index in frame_indices:
        buffer = io.BytesIO()
        draw_frame(result, index, buffer)
        buffer.seek(0)
        frames.append(Image.open(buffer).convert("RGB"))
    gif_path = output_dir / "swarm_animation.gif"
    frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=max(20, int(1000 / fps)), loop=0, optimize=False)

    snapshots = []
    for target in snapshot_times:
        index = nearest_index(result["times"], target)
        actual_time = float(result["times"][index])
        filename = figure_dir / f"swarm_t{actual_time:05.1f}s.png"
        draw_frame(result, index, filename)
        snapshots.append(str(filename))
    write_metrics_plot(result, figure_dir / "swarm_metrics.png")
    arrived = np.linalg.norm(result["states"][-1, :, :2] - GOAL, axis=1) < 3.0
    stats = {
        "renderer": "generate_swarm_visuals.py",
        "seed": int(result["seed"]),
        "steps": int(len(result["times"]) - 1),
        "simulation_time_s": float(result["times"][-1]),
        "n_uav": 9,
        "arrived": int(np.sum(arrived)),
        "arrive_rate": float(np.mean(arrived)),
        "arrive_time_s": result["arrive_time"],
        "minimum_pair_separation_m": float(result["min_separation"]),
        "gif": str(gif_path),
        "snapshots": snapshots,
    }
    (output_dir / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description="Render the swarm simulation GIF and time snapshots.")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "visualizations")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-time", type=float, default=TMAX)
    parser.add_argument("--frame-step", type=int, default=5, help="simulation steps between GIF frames")
    parser.add_argument("--fps", type=int, default=12)
    args = parser.parse_args()
    result = simulate(args.seed, args.max_time)
    result["seed"] = args.seed
    snapshot_times = [0.0, 8.0, 16.0, 24.0, 32.0, 40.0, 50.0, float(result["times"][-1])]
    stats = render(result, args.output_dir, args.frame_step, args.fps, snapshot_times)
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    raise SystemExit(
        "The legacy approximate simulation is disabled: it can collide with obstacles. "
        "Use render_matlab_history.py to render a verified MATLAB history."
    )
