"""Scripted demo for hot-glass-dish-on-a-dry-rack, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the dish's state every 10 steps

full        Both arms pinch the dish's lip handles, lift it 4 cm off the oven rack, carry it level to the dry cooling
            rack on the left, set it down and let go.
Failure and cheating variants:
wet_towel   both arms set the dish down on the wet towel on the right (0)
one_handle  the right arm alone lifts the dish by its right handle (expected: it tips or slips)
drop        both arms let go of the dish 5 cm above the cooling rack (0)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "wet_towel", "one_handle", "drop")
_CACHE: dict = {}
LIFT = 0.040  # m: the dish's underside above the oven rack while carried


def _closing(axis: int):
    return [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[axis, 0]) >= 0.95]


def handle_offset(t, side: str) -> np.ndarray:
    sy = 1.0 if side == "left" else -1.0
    return np.array([0.0, sy * (t.EAR_Y + 0.003), t.EAR_Z])


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    d0 = np.array(t.DISH_POS)
    carry_z = d0[2] + LIFT
    goals = {"rack": np.array([*t.RACK_XY, t.TOP + t.RACK_H + 0.0005]),
             "towel": np.array([*t.TOWEL_XY, t.TOP + t.TOWEL_H + 0.0005])}
    d1 = goals["towel" if variant == "wet_towel" else "rack"]
    key = "towel" if variant == "wet_towel" else "rack"
    if key not in _CACHE:
        _CACHE[key] = {}
        for side in ("left", "right"):
            g = handle_offset(t, side)
            pts = [d0 + g, d0 + g + [0, 0, LIFT], np.array([*d1[:2], carry_z]) + g, d1 + g, d1 + g + [0, 0, 0.05]]
            _CACHE[key][side], e = dc.best_rotation(side, pts, _closing(0))
            print(f"{side} handle grip ({key}) chosen: worst IK error {1000 * e:.1f} (mm + 0.05 rad)", flush=True)
    G = _CACHE[key]
    users = ("right",) if variant == "one_handle" else ("left", "right")
    for side in users:
        A, g, R = arms[side], handle_offset(t, side), G[side]
        A.add(2.5, d0 + g + [0, 0, 0.06], R, 1.0, "dish: above the handle")
        A.add(1.5, d0 + g, R, 1.0, "dish: around the handle")
        A.add(1.0, d0 + g, R, 0.0, "dish: grip")
        A.add(1.5, d0 + g + [0, 0, LIFT], R, 0.0, "dish: lift")
        A.add(3.5, np.array([*d1[:2], carry_z]) + g, R, 0.0, "dish: over the pad")
        if variant == "drop":
            A.add(1.5, d1 + g + [0, 0, 0.05], R, 0.0, "dish: hold")
            last = d1 + g + [0, 0, 0.05]
        else:
            A.add(2.0, d1 + g + [0, 0, 0.002], R, 0.0, "dish: down")
            A.add(1.2, d1 + g - [0, 0, 0.008], R, 0.0, "dish: onto the pad")
            A.add(0.5, d1 + g - [0, 0, 0.008], R, 0.0, "dish: settle onto the pad")
            last = d1 + g - [0, 0, 0.001]
        A.add(0.8, grip=1.0, label="dish: let go")
        A.add(1.5, last + [0, 0, 0.05], R, 1.0, "dish: up")
        A.add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the pad"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: share {r.get('rack_share')} resting {r.get('resting_on_rack')} wet {r.get('wet')} counter "
          f"{r.get('counter')} rise {r.get('rise')} tilt {r.get('tilt')} max {r.get('max_carry_tilt')} rel "
          f"{r.get('released')} drop {r.get('drop_flag')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
