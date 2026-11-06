"""Score predictor for weigh-and-reject-heavy-block. Pure Python offline simulation.

Simulates the execution of each mode by tracking block positions, scale platform displacement,
and stepping the Scorer to verify expected scores.
"""

from __future__ import annotations

import math
from typing import Any
import numpy as np

import demo_plan
import geometry as g
from scoring import Scorer


def simulate_mode(mode: str) -> float:
    scorer = Scorer()
    plans = demo_plan.plans(mode)
    left_wps, right_wps = plans["left"], plans["right"]

    dt = 0.05

    def expand_wps(wps: list[Any]) -> list[tuple[float, float, float, float]]:
        traj = []
        for w in wps:
            x, y, z = w.pos
            f = w.finger
            dur = w.duration
            n_steps = max(1, int(round(dur / dt)))
            for _ in range(n_steps):
                traj.append((x, y, z, f))
        return traj

    left_traj = expand_wps(left_wps)
    right_traj = expand_wps(right_wps)

    n_steps = max(len(left_traj), len(right_traj))
    if len(left_traj) < n_steps:
        left_traj += [left_traj[-1]] * (n_steps - len(left_traj))
    if len(right_traj) < n_steps:
        right_traj += [right_traj[-1]] * (n_steps - len(right_traj))

    # Initialize block positions at supply rack
    block_pos = {b: np.array(g.BLOCK_SUPPLY_POS[b], dtype=float) for b in g.BLOCK_NAMES}
    attached = {"left": None, "right": None}

    prev_l_pos = None
    prev_r_pos = None

    for i in range(n_steps):
        lx, ly, lz, lf = left_traj[i]
        rx, ry, rz, rf = right_traj[i]

        l_pos = np.array([lx, ly, lz])
        r_pos = np.array([rx, ry, rz])

        if prev_l_pos is None:
            l_vel = np.zeros(3)
            r_vel = np.zeros(3)
        else:
            l_vel = (l_pos - prev_l_pos) / dt
            r_vel = (r_pos - prev_r_pos) / dt

        prev_l_pos = l_pos
        prev_r_pos = r_pos

        # Check grasping / attachment
        # Left arm grasp
        if lf > -0.025:  # closed grasp
            if attached["left"] is None:
                for b, pos in block_pos.items():
                    if np.linalg.norm(l_pos - pos) <= 0.035:
                        attached["left"] = b
                        break
        else:
            attached["left"] = None

        # Right arm grasp
        if rf > -0.025:  # closed grasp
            if attached["right"] is None:
                for b, pos in block_pos.items():
                    if np.linalg.norm(r_pos - pos) <= 0.035:
                        attached["right"] = b
                        break
        else:
            attached["right"] = None

        # Update block positions for attached blocks
        if attached["left"] is not None:
            block_pos[attached["left"]] = l_pos.copy()
        if attached["right"] is not None:
            block_pos[attached["right"]] = r_pos.copy()

        # Check which block is on scale platform
        scale_disp = -g.SAG_PLATFORM_EMPTY
        block_on_scale = None
        for b, pos in block_pos.items():
            dx = abs(pos[0] - g.SCALE_CENTER_XY[0])
            dy = abs(pos[1] - g.SCALE_CENTER_XY[1])
            if dx <= 0.045 and dy <= 0.045:
                # If resting on scale
                block_on_scale = b
                is_heavy = (b == "block_1")
                sag = g.SAG_HEAVY_BLOCK if is_heavy else g.SAG_LIGHT_BLOCK
                scale_disp = -sag
                break

        grippers_open = (lf <= -0.040 and rf <= -0.040)

        arm_tcps = {"left": l_pos, "right": r_pos}
        arm_vels = {"left": (l_vel, np.zeros(3)), "right": (r_vel, np.zeros(3))}

        scorer.update(
            scale_disp=scale_disp,
            block_positions=block_pos,
            grippers_open=grippers_open,
            arm_tcps=arm_tcps,
            arm_velocities=arm_vels,
        )

    return scorer.best_score


def predict_all() -> dict[str, float]:
    modes = ["demo", "no_weigh", "wrong_pick", "weigh_too_short", "settle"]
    return {m: simulate_mode(m) for m in modes}


if __name__ == "__main__":
    scores = predict_all()
    for m, s in scores.items():
        print(f"{m:15s}: {s:.2f}")

