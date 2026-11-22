"""Scripted demo for lift-two-cloches-together, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the cloches' state every 10 steps

full        Each arm pinches the knob of the cloche in front of it; both lift together, straight up 6 cm, carry the
            cloches back over the service tray and set them down on it, then let go.
Failure and cheating variants:
staggered   the left arm starts 0.6 s after the right (0.5)
sequential  the left arm waits until the right arm has put its cloche on the tray (0)
drag        both lift only 1.5 cm and slide the cloches sideways off the plates before rising (0.25)
on_table    both lift together but set the cloches down on the table beside the plates (0)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "staggered", "sequential", "drag", "on_table")
_CACHE: dict = {}
LIFT = 0.060  # m: rim above the plate while carried
ASIDE = {"left": (0.28, 0.38), "right": (0.28, -0.38)}  # on_table: on the counter beside the plates


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    knob = np.array([0.0, 0.0, t.KNOB_Z])
    tray_z = t.TOP + t.TRAY_H
    dest = {s: np.array([*(ASIDE[s] if variant == "on_table" else t.TRAY_SPOTS[s]),
                         t.TOP if variant == "on_table" else tray_z]) for s in ("left", "right")}
    if "grips" not in _CACHE:
        cands = [dc.R_approach(tl, az, roll) for tl in (0, 5, 10) for az in (0, 90, 180, 270) for roll in (0, 90)]
        _CACHE["grips"] = {}
        for s in ("left", "right"):
            c0 = np.array(t.CL_POS[s])
            pts = [c0 + knob, c0 + knob + [0, 0, LIFT], dest[s] + knob + [0, 0, LIFT], dest[s] + knob]
            for v in ASIDE, t.TRAY_SPOTS:
                pts.append(np.array([*v[s], t.TOP + t.TRAY_H]) + knob)
            _CACHE["grips"][s], e = dc.best_rotation(s, pts, cands)
            print(f"{s} knob grip chosen: worst IK error {1000 * e:.1f} (mm + 0.05 rad)", flush=True)
    G = _CACHE["grips"]

    def one(side: str) -> None:
        A, R, c0 = arms[side], G[side], np.array(t.CL_POS[side])
        A.add(2.5, c0 + knob + [0, 0, 0.06], R, 1.0, "cloche: above the knob")
        A.add(1.5, c0 + knob, R, 1.0, "cloche: around the knob")
        A.add(1.0, c0 + knob, R, 0.0, "cloche: grip")
        if variant == "drag":
            A.add(0.8, c0 + knob + [0, 0, 0.015], R, 0.0, "cloche: lift a little")
            mid = c0 + knob + [-0.12, 0.0, 0.015]
            A.add(2.0, mid, R, 0.0, "cloche: slide off the plate")
            A.add(1.2, mid + [0, 0, LIFT], R, 0.0, "cloche: rise")
        else:
            A.add(1.5, c0 + knob + [0, 0, LIFT], R, 0.0, "cloche: lift straight up")
        A.add(3.0, dest[side] + knob + [0, 0, LIFT], R, 0.0, "cloche: over the tray")
        A.add(1.8, dest[side] + knob + [0, 0, 0.003], R, 0.0, "cloche: down")
        A.add(0.6, dest[side] + knob - [0, 0, 0.002], R, 0.0, "cloche: onto the tray")
        A.add(0.8, grip=1.0, label="cloche: let go")
        A.add(1.5, dest[side] + knob + [0, 0, 0.07], R, 1.0, "cloche: up")
        A.add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")

    one("right")
    if variant == "staggered":
        arms["left"].add(0.6, label="left: wait")
    elif variant == "sequential":
        arms["left"].add(arms["right"].duration, label="left: wait")
    one("left")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "onto the tray", "let go"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: lift {r.get('lift_steps')} gap {r.get('lift_gap_steps')} dragged {r.get('dragged')} "
          f"L {r.get('cloche_left')} R {r.get('cloche_right')} tray {r.get('both_on_tray')} | {labL} | {labR}",
          flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
