"""Render a recorded MATLAB swarm run as an animated GIF and snapshots."""

from __future__ import annotations

import argparse
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.io import loadmat

from generate_swarm_visuals import draw_frame, write_metrics_plot


def read_history(path: Path) -> tuple[dict, dict]:
    data = loadmat(path, simplify_cells=True)
    history, stats = data["history"], data["stats"]
    states = np.asarray(history["state"]).transpose(2, 0, 1)
    dynamic = [
        [{"pos": np.asarray(obs.pos), "vel": np.asarray(obs.vel), "r": float(obs.r)}
         for obs in np.atleast_1d(frame)]
        for frame in history["dynObs"]
    ]
    result = {
        "times": np.asarray(history["time"]).reshape(-1),
        "states": states,
        "dynamic": dynamic,
        "paths": list(history["paths"]),
        "trails": [states[:, i, :2] for i in range(states.shape[1])],
        "min_separation": float(stats["minSeparation"]),
    }
    return result, stats


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    base = Path(__file__).resolve().parent
    parser.add_argument("mat_file", nargs="?", type=Path, default=base / "visualizations" / "matlab_simulation.mat")
    parser.add_argument("--output-dir", type=Path, default=base / "visualizations")
    parser.add_argument("--frame-step", type=int, default=3)
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args()
    if args.frame_step < 1 or args.fps < 1:
        parser.error("--frame-step and --fps must be positive")
    result, stats = read_history(args.mat_file)
    output = args.output_dir
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    indices = list(range(0, len(result["times"]), args.frame_step))
    if indices[-1] != len(result["times"]) - 1:
        indices.append(len(result["times"]) - 1)
    frames = []
    for index in indices:
        buffer = io.BytesIO()
        draw_frame(result, index, buffer)
        buffer.seek(0)
        with Image.open(buffer) as image:
            frames.append(image.convert("RGB"))
    frames[0].save(output / "swarm_animation.gif", save_all=True, append_images=frames[1:],
                   duration=max(20, round(1000 / args.fps)), loop=0, optimize=False)

    for old in figures.glob("swarm_t*.png"):
        old.unlink()
    snapshot_times = np.linspace(0.0, float(result["times"][-1]), 7)
    snapshots = []
    for time in snapshot_times:
        index = int(np.argmin(np.abs(result["times"] - time)))
        actual = float(result["times"][index])
        path = figures / f"swarm_t{actual:05.1f}s.png"
        draw_frame(result, index, path)
        snapshots.append(path.name)
    write_metrics_plot(result, figures / "swarm_metrics.png")
    summary = {
        "source": args.mat_file.name,
        "controller": "MATLAB swarm_mpc.m",
        "simulation_time_s": float(result["times"][-1]),
        "arrived_latched": int(stats["arrived"]),
        "arrive_time_s": float(stats["arriveTime"]),
        "sampled_minimum_separation_m": float(stats["minSeparation"]),
        "sampled_minimum_static_clearance_m": float(stats["minStaticClearance"]),
        "sampled_minimum_nofly_clearance_m": float(stats["minNoflyClearance"]),
        "sampled_minimum_dynamic_clearance_m": float(stats["minDynamicClearance"]),
        "sampled_static_collisions": int(stats["staticCollisionSamples"]),
        "sampled_nofly_collisions": int(stats["noflyCollisionSamples"]),
        "sampled_dynamic_collisions": int(stats["dynamicCollisionSamples"]),
        "sampled_pair_violations": int(stats["separationViolationSamples"]),
        "snapshots": snapshots,
    }
    (output / "stats.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
