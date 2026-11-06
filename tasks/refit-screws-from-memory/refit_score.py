"""This task's measurements and score, from plain positions, so they can be tested without the simulator: see test_refit_score.py."""

from __future__ import annotations

import math
import pathlib
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import refit_usd as u  # noqa: E402

TOP = TABLE_TOP["station"]
STAND_XY = (0.21, 0.0)
RACK_XY = (0.32, 0.0)
WORN_MAT_XY = (0.22, 0.27)
NEW_MAT_XY = (0.22, -0.27)
STAND_TOP = TOP + u.STAND[2]
PULLEY_TOP = STAND_TOP + u.PULLEY_THICK  # with a pulley on the stand
RACK_TOP = TOP + u.RACK_TOP_H
MAT_TOP = TOP + u.MAT_H

SCREW_NAME = {"far_left": "screw_a", "near_left": "screw_b", "far_right": "screw_c", "near_right": "screw_d"}  # each screw's starting hole
SCREW_LENGTH = {SCREW_NAME[h]: (u.LONG if h in u.DEEP_HOLES else u.SHORT) for h in u.SCREW_HOLES}

SEATED_AXIS, SEATED_HEIGHT, SEATED_TILT = 0.005, 0.003, 5.0  # the new pulley on the stand
ON_MAT, ON_MAT_HEIGHT = 0.05, 0.01  # the worn pulley's axis from the mat's centre, and its underside above the mat's top
SCREW_AXIS, SCREW_HEIGHT, SCREW_TILT = 0.004, 0.003, 10.0  # a screw in a hole
IN_SLOT = 0.006  # a screw's shank from a rack slot's axis
HANDS_OFF = 0.03
CLEAR = 0.05
STILL = 0.01
HOLD_S = 1.0
BOTTOM_MARGIN = 0.002  # a long screw's tip this close to a shallow hole's floor has bottomed out


def hole_world(name: str) -> np.ndarray:
    x, y = u.hole_xy(name)
    return np.array([STAND_XY[0] + x, STAND_XY[1] + y])


def hole_floor(name: str) -> float:
    return STAND_TOP - (u.DEEP if name in u.DEEP_HOLES else u.SHALLOW)


def slot_world(i: int) -> np.ndarray:
    return np.array([RACK_XY[0], RACK_XY[1] + u.SLOTS_Y[i]])


@dataclass
class Body:
    """A pulley or a screw: its origin, its axis (unit, pointing up the part), and its speed."""

    pos: np.ndarray
    up: np.ndarray
    speed: float = 0.0


def task_score(worn_on_mat: bool, new_seated: bool, n_right: int, full_held: bool) -> float:
    """0.1 for the worn pulley on its mat, 0.1 for the new pulley seated, 0.2 for each right hole. A total of 1.0 needs the 1 s hold, and scores 0.9 without it."""
    if n_right == 4 and worn_on_mat and new_seated and not full_held:
        return 0.9
    return round(0.1 * worn_on_mat + 0.1 * new_seated + 0.2 * n_right, 3)


def _axis_at(b: Body, height: float) -> np.ndarray:
    """Where a part's axis crosses a height."""
    return b.pos + b.up * ((height - b.pos[2]) / b.up[2])


class Scorer:
    """Keeps what stays true for the rest of a trial, and scores each control step."""

    def __init__(self, step_dt: float):
        self.step_dt = step_dt
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.full_steps = 0
        self.lost: dict[str, float] = {}  # shallow holes where a long screw has bottomed out, and when
        self.hung: dict[str, float] = {}  # screws that have hung in the rack, and when

    def update(self, worn: Body, new: Body, screws: dict[str, Body], tips: np.ndarray) -> dict:
        """One control step. `tips` holds both grippers' fingertips, one point a row."""
        self.steps += 1
        t = round(self.steps * self.step_dt, 3)
        stand = np.array(STAND_XY)
        new_seated = bool(np.linalg.norm(new.pos[:2] - stand) <= SEATED_AXIS and abs(new.pos[2] - STAND_TOP) <= SEATED_HEIGHT and new.up[2] >= math.cos(math.radians(SEATED_TILT)))
        worn_on_mat = bool(np.linalg.norm(worn.pos[:2] - np.array(WORN_MAT_XY)) <= ON_MAT and abs(worn.pos[2] - MAT_TOP) <= ON_MAT_HEIGHT and worn.speed < STILL)

        def to_screw(name: str) -> float:
            b = screws[name]
            a, c = b.pos + b.up * u.HEAD_H, b.pos - b.up * SCREW_LENGTH[name]
            d = c - a
            s = np.clip((tips - a) @ d / (d @ d), 0.0, 1.0)
            return float(np.linalg.norm(tips - (a + s[:, None] * d), axis=1).min())

        def to_pulley(p: np.ndarray) -> float:
            radial = np.maximum(np.hypot(tips[:, 0] - p[0], tips[:, 1] - p[1]) - u.PULLEY_R, 0.0)
            height = np.maximum(np.maximum(p[2] - tips[:, 2], tips[:, 2] - (p[2] + u.PULLEY_THICK)), 0.0)
            return float(np.hypot(radial, height).min())

        gaps = {n: to_screw(n) for n in screws}
        upright = math.cos(math.radians(SCREW_TILT))
        in_holes: dict[str, str | None] = {h: None for h in u.SCREW_HOLES}
        for name, b in screws.items():
            length = SCREW_LENGTH[name]
            tip = b.pos - b.up * length
            for hole in u.SCREW_HOLES:
                c = hole_world(hole)
                if length == u.LONG and hole not in u.DEEP_HOLES and hole not in self.lost and np.linalg.norm(tip[:2] - c) <= SCREW_AXIS and tip[2] <= hole_floor(hole) + BOTTOM_MARGIN:
                    self.lost[hole] = t
                if b.up[2] >= upright and abs(b.pos[2] - PULLEY_TOP) <= SCREW_HEIGHT and np.linalg.norm(_axis_at(b, PULLEY_TOP - u.PULLEY_THICK / 2)[:2] - c) <= SCREW_AXIS:
                    in_holes[hole] = name
            if name not in self.hung and b.up[2] >= upright and abs(b.pos[2] - RACK_TOP) <= SCREW_HEIGHT and gaps[name] >= HANDS_OFF:
                at = _axis_at(b, RACK_TOP - u.RACK_PLATE / 2)[:2]
                if any(np.linalg.norm(at - slot_world(i)) <= IN_SLOT for i in range(len(u.SLOTS_Y))):
                    self.hung[name] = t
        right = {}
        for hole, name in in_holes.items():
            fits = name is not None and (SCREW_LENGTH[name] == u.LONG) == (hole in u.DEEP_HOLES)
            right[hole] = bool(new_seated and fits and name in self.hung and hole not in self.lost and gaps[name] >= HANDS_OFF)
        n_right = sum(right.values())
        clear = min(*gaps.values(), to_pulley(new.pos), to_pulley(worn.pos)) >= CLEAR
        still = all(b.speed < STILL for b in [*screws.values(), worn, new])
        full = worn_on_mat and new_seated and n_right == 4 and clear and still
        self.full_steps = self.full_steps + 1 if full else 0
        held = self.full_steps * self.step_dt >= HOLD_S - 1e-6
        return {"step": self.steps, "t": t, "score": task_score(worn_on_mat, new_seated, n_right, held), "success_now": full, "full_for_s": round(self.full_steps * self.step_dt, 3),
                "worn_on_mat": worn_on_mat, "new_seated": new_seated, "in_holes": in_holes, "right": right, "n_right": n_right, "lost": dict(self.lost), "hung": dict(self.hung), "clear": clear, "still": still,
                "fingertip_gap": {n: round(v, 4) for n, v in gaps.items()}}
