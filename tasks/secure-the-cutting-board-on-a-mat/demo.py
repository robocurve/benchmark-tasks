"""Scripted demo for secure-the-cutting-board-on-a-mat, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the board's state every 10 steps

full        Both arms pinch the board's end handles, lift it 5 cm, carry it toward the robot over the non-slip mat,
            set it down on the mat and let go.
Failure and cheating variants:
drag        the right arm alone slides the board by its handle across the counter onto the mat (expected: the mat is
            pushed away)
one_hand    the right arm alone lifts the board by its handle and carries it to the mat (recorded: carried 8 deg
            tilted and set down short, 40% over the mat, 0.5; a one-arm carry is possible)
half_on     both arms set the board down 7 cm short, partly over the mat's edge (0.5)
drop        both arms let go of the board 5 cm above the mat (0)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "drag", "one_hand", "half_on", "drop")
_CACHE: dict = {}
LIFT = 0.050


def _closing(axis: int):
    return [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[axis, 0]) >= 0.95]


def handle_offset(t, side: str) -> np.ndarray:
    sy = 1.0 if side == "left" else -1.0
    return np.array([0.0, sy * (t.EAR_Y + 0.004), t.EAR_Z])


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    b0 = np.array(t.BOARD_POS)
    b1 = np.array([*t.MAT_XY, t.TOP + t.MAT[2] + 0.0005]) + ([0.07, 0, 0] if variant == "half_on" else [0, 0, 0])
    if "grips" not in _CACHE:
        _CACHE["grips"] = {}
        for side in ("left", "right"):
            g = handle_offset(t, side)
            pts = [b0 + g, b0 + g + [0, 0, LIFT], b1 + g + [0, 0, LIFT], b1 + g, b1 + g + [0.12, 0, 0],
                   np.array([*t.MAT_XY, t.TOP]) + g]
            _CACHE["grips"][side], e = dc.best_rotation(side, pts, _closing(0))
            print(f"{side} handle grip chosen: worst IK error {1000 * e:.1f} (mm + 0.05 rad)", flush=True)
    G = _CACHE["grips"]
    users = ("right",) if variant in ("drag", "one_hand") else ("left", "right")
    for side in users:
        A, g, R = arms[side], handle_offset(t, side), G[side]
        A.add(2.5, b0 + g + [0, 0, 0.06], R, 1.0, "board: above the handle")
        A.add(1.5, b0 + g, R, 1.0, "board: around the handle")
        A.add(1.0, b0 + g, R, 0.0, "board: grip")
        if variant == "drag":
            A.add(4.0, np.array([*t.MAT_XY, b0[2]]) + g, R, 0.0, "board: slide")
            last = np.array([*t.MAT_XY, b0[2]]) + g
        else:
            A.add(1.5, b0 + g + [0, 0, LIFT], R, 0.0, "board: lift")
            A.add(3.0, b1 + g + [0, 0, LIFT], R, 0.0, "board: over the mat")
            if variant == "drop":
                A.add(1.0, b1 + g + [0, 0, 0.05], R, 0.0, "board: hold")
                last = b1 + g + [0, 0, 0.05]
            else:
                A.add(1.8, b1 + g + [0, 0, 0.002], R, 0.0, "board: down")
                A.add(1.2, b1 + g - [0, 0, 0.008], R, 0.0, "board: onto the mat")
                A.add(0.5, b1 + g - [0, 0, 0.008], R, 0.0, "board: settle onto the mat")
                last = b1 + g - [0, 0, 0.001]
        A.add(0.8, grip=1.0, label="board: let go")
        A.add(1.5, last + [0, 0, 0.05], R, 1.0, "board: up")
        A.add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the mat", "slide"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: share {r.get('mat_share')} tilt {r.get('tilt')} h {r.get('height_over_mat')} max "
          f"{r.get('max_lift_tilt')} mat_moved {r.get('mat_moved')} rel {r.get('released')} drop {r.get('drop_flag')} "
          f"| {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
