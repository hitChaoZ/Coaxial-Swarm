"""Check inter-step clearances in a saved MATLAB swarm history.

UAV motion is reconstructed as constant acceleration over each integration
step. Dynamic obstacle centers are linearly interpolated between recorded
positions. The minimum of each squared relative distance is found from its
stationary points, not from a time grid.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat


def min_distance(relative_coefficients: np.ndarray) -> tuple[float, float]:
    """Minimum norm of q(t)=c0+c1*t+c2*t^2 on the unit interval."""
    squared = np.zeros(5)
    for axis in range(2):
        product = np.polynomial.polynomial.polymul(
            relative_coefficients[:, axis], relative_coefficients[:, axis]
        )
        squared[:len(product)] += product
    derivative = np.polynomial.polynomial.polyder(squared)
    roots = np.polynomial.polynomial.polyroots(derivative)
    candidates = [0.0, 1.0]
    candidates.extend(float(root.real) for root in roots if abs(root.imag) < 1e-8 and 0 < root.real < 1)
    values = [max(0.0, float(np.polynomial.polynomial.polyval(t, squared))) for t in candidates]
    index = int(np.argmin(values))
    return float(np.sqrt(values[index])), candidates[index]


def obstacle_positions(items: object) -> np.ndarray:
    return np.asarray([np.asarray(item.pos, dtype=float) for item in np.atleast_1d(items)])


def audit(path: Path) -> dict:
    data = loadmat(path, simplify_cells=True)
    world, history = data["W"], data["history"]
    times = np.asarray(history["time"], dtype=float).reshape(-1)
    states = np.asarray(history["state"], dtype=float).transpose(2, 0, 1)
    dynamic = history["dynObs"]
    static = np.atleast_2d(world["static_obs"])
    nofly = np.atleast_2d(world["nofly"])
    dynamic_radii = np.asarray([item.r for item in np.atleast_1d(dynamic[0])], dtype=float)
    best = {name: {"value_m": float("inf")} for name in ("static", "nofly", "dynamic", "pair")}

    def record(name: str, value: float, step: int, fraction: float, uav: int, other: int) -> None:
        if value < best[name]["value_m"]:
            best[name] = {
                "value_m": float(value),
                "time_s": float(times[step] + fraction * (times[step + 1] - times[step])),
                "uav": uav + 1,
                "other_index": other + 1,
            }

    for step in range(len(times) - 1):
        dt = times[step + 1] - times[step]
        p0, p1 = states[step, :, :2], states[step + 1, :, :2]
        # Normalized time tau in [0,1]: p = p0 + v0*dt*tau + a*dt^2*tau^2/2.
        linear = states[step, :, 2:] * dt
        quadratic = p1 - p0 - linear
        coeff = np.stack((p0, linear, quadratic), axis=1)
        dyn0 = obstacle_positions(dynamic[step])
        dyn1 = obstacle_positions(dynamic[step + 1])
        for i in range(len(p0)):
            for name, obstacles in (("static", static), ("nofly", nofly)):
                for j, obstacle in enumerate(obstacles):
                    rel = coeff[i].copy()
                    rel[0] -= obstacle[:2]
                    distance, tau = min_distance(rel)
                    record(name, distance - obstacle[2], step, tau, i, j)
            for j, (start, end) in enumerate(zip(dyn0, dyn1)):
                rel = coeff[i].copy()
                rel[0] -= start
                rel[1] -= end - start
                distance, tau = min_distance(rel)
                record("dynamic", distance - dynamic_radii[j], step, tau, i, j)
            for j in range(i + 1, len(p0)):
                distance, tau = min_distance(coeff[i] - coeff[j])
                record("pair", distance, step, tau, i, j)

    return {
        "source": path.name,
        "model": "constant UAV acceleration and linear dynamic-obstacle motion per recorded step",
        "duration_s": float(times[-1]),
        "samples": int(len(times)),
        "minimum_clearance": best,
        "pair_threshold_m": float(world["dsafe"]),
        "passed": bool(
            all(best[name]["value_m"] >= 0 for name in ("static", "nofly", "dynamic"))
            and best["pair"]["value_m"] >= world["dsafe"]
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mat_file", type=Path, nargs="?", default=Path(__file__).resolve().parent / "visualizations" / "matlab_simulation.mat")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.mat_file)
    output = json.dumps(result, indent=2)
    print(output)
    if args.output:
        args.output.write_text(output + "\n", encoding="utf-8")
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
