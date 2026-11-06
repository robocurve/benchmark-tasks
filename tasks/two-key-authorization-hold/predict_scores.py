"""Score predictor for two-key-authorization-hold. Pure Python offline simulation.

Simulates the execution of each mode by tracking guard positions and button presses
from waypoint plans and stepping the Scorer to verify expected scores.
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
    if mode == "no_confirm":
        n_steps += int(12.0 / dt)   # the run goes on and CONFIRM never comes, so the 10 s ARMED window runs out
    if len(left_traj) < n_steps:
        left_traj += [left_traj[-1]] * (n_steps - len(left_traj))
    if len(right_traj) < n_steps:
        right_traj += [right_traj[-1]] * (n_steps - len(right_traj))

    prev_l_pos = None
    prev_r_pos = None

    guard_disps = {"guard_left": 0.0, "guard_right": 0.0}

    for i in range(n_steps):
        lx, ly, lz, lf = left_traj[i]
        rx, ry, rz, rf = right_traj[i]

        if prev_l_pos is None:
            l_vel = np.zeros(3)
            r_vel = np.zeros(3)
        else:
            l_vel = (np.array([lx, ly, lz]) - np.array(prev_l_pos)) / dt
            r_vel = (np.array([rx, ry, rz]) - np.array(prev_r_pos)) / dt

        prev_l_pos = (lx, ly, lz)
        prev_r_pos = (rx, ry, rz)

        # Update guard positions if pushed outward
        # Left arm pushes +Y
        if ly > 0.340 and lz <= g.GUARD_REST_Z + 0.015:
            guard_disps["guard_left"] = 0.070
        # Right arm pushes -Y
        if ry < -0.340 and rz <= g.GUARD_REST_Z + 0.015:
            guard_disps["guard_right"] = 0.070

        btn_disps = {b: 0.0 for b in g.BUTTONS}
        press_z = g.BUTTON_PRESS_TCP_Z + 0.005

        for b, (bx, by) in g.BUTTON_POS_XY.items():
            dl = math.hypot(lx - bx, ly - by)
            if dl <= 0.030 and lz <= press_z:
                btn_disps[b] = -0.007

            dr = math.hypot(rx - bx, ry - by)
            if dr <= 0.030 and rz <= press_z:
                btn_disps[b] = -0.007

        arm_vels = {
            "left": (l_vel, np.zeros(3)),
            "right": (r_vel, np.zeros(3)),
        }

        fixture_sliders = {
            "btn_key_left": btn_disps["key_left"],
            "btn_key_right": btn_disps["key_right"],
            "btn_confirm": btn_disps["confirm"],
            "slider_guard_left": guard_disps["guard_left"],
            "slider_guard_right": guard_disps["guard_right"],
        }

        grippers_open = (lf <= -0.040 and rf <= -0.040)

        res = scorer.update(
            fixture_pos=g.BASE_POS,
            fixture_quat=(0.0, 0.0, 0.0, 1.0),
            fixture_sliders=fixture_sliders,
            grippers_open=grippers_open,
            arm_velocities=arm_vels,
        )

    return scorer.best_score


def predict_all() -> dict[str, float]:
    modes = ["demo", "sequential", "one_key", "release_early", "no_confirm", "settle"]
    return {m: simulate_mode(m) for m in modes}


if __name__ == "__main__":
    scores = predict_all()
    for m, s in scores.items():
        print(f"{m:15s}: {s:.2f}")
