"""Scripted demo for serve-two-plates-together, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the plates' state every 10 steps

full        Each arm pinches the outer rim lip of the plate on its side, lifts it off the pass, carries it out to its
            place setting while turning it a quarter turn so the steak faces the guest, and both set their plates
            down together, then let go.
Failure and cheating variants:
sequential  the right arm serves its plate first; the left arm starts when the right has finished (0)
late        the left arm starts 1.5 s after the right (0.5)
no_turn     both plates set down together without turning: the steaks point sideways (0.25)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "sequential", "late", "no_turn")
_CACHE: dict = {}
LIP_OPEN = 0.30  # gripper opening while approaching a plate's rim lip (about 2.8 cm)
RELEASE_OPEN = 0.18  # opening to let go of a lip (about 1.7 cm): at LIP_OPEN the inner finger came within 8 mm of the food
LIFT = 0.040  # m: the plate above the pass while carried
TURN = {"left": 90.0, "right": -90.0}  # deg of yaw for each plate (steak toward +x)


def rz(deg):
    return dc.rot([0.0, 0.0, 1.0], math.radians(deg))


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    lip_z = t.PLATE_T + t.LIP_H - 0.006
    lip = {"left": np.array([0.0, (t.PLATE - t.LIP_W) / 2 + 0.0, lip_z]),  # world offset of the outer lip (+y / -y)
           "right": np.array([0.0, -(t.PLATE - t.LIP_W) / 2, lip_z])}
    turn = {s: 0.0 if variant == "no_turn" else TURN[s] for s in TURN}
    key = ("grips", variant == "no_turn")
    if key not in _CACHE:
        cands = [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90, 180)
                 for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.95]  # closing across the lip (y)
        _CACHE[key] = {}
        for s in ("left", "right"):
            p0 = np.array(t.PLATE_POS[s])
            c1 = np.array([*t.COVER_XY[s], t.TOP + 0.0005])
            g0, g1 = lip[s], rz(turn[s]) @ lip[s]
            poses = [(p0 + g0, 0.0), (p0 + g0 + [0, 0, LIFT], 0.0), (c1 + g1 + [0, 0, LIFT + 0.05], turn[s]),
                     (c1 + g1, turn[s]), (c1 + g1 + [0, 0, 0.05], turn[s])]
            u_out = np.array([0.0, float(np.sign(lip[s][1])), 0.0])
            best = (None, np.inf)
            for R0 in [R for R in cands if float(R[:, 2] @ u_out) <= 0.02]:  # vertical, or the hand leaning outward
                worst = 0.0
                for pos, a in poses:
                    q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rz(a) @ R0), np.array(READY, float),
                                         base_pos=bases[s], iters=200)
                    e = pe + 0.05 * re
                    if e > 0.001:
                        for sd in dc.SEEDS[:5]:
                            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rz(a) @ R0), sd, base_pos=bases[s], iters=200)
                            e = min(e, pe + 0.05 * re)
                    worst = max(worst, e)
                    if worst >= best[1]:
                        break
                if worst < best[1]:
                    best = (R0, worst)
            _CACHE[key][s] = best[0]
            print(f"{s} lip grip chosen: worst IK error {1000 * best[1]:.1f} (mm + 0.05 rad)", flush=True)
    G = _CACHE[key]

    def one(side: str) -> None:
        A, R0 = arms[side], G[side]
        p0 = np.array(t.PLATE_POS[side])
        c1 = np.array([*t.COVER_XY[side], t.TOP + 0.0005])
        g0, g1, R1 = lip[side], rz(turn[side]) @ lip[side], rz(turn[side]) @ R0
        A.add(2.5, p0 + g0 + [0, 0, 0.05], R0, LIP_OPEN, "plate: above the rim")
        A.add(1.5, p0 + g0, R0, LIP_OPEN, "plate: around the rim")
        A.add(1.0, p0 + g0, R0, 0.0, "plate: grip")
        A.add(1.5, p0 + g0 + [0, 0, LIFT], R0, 0.0, "plate: lift")
        A.add(3.5, c1 + g1 + [0, 0, LIFT + 0.05], R1, 0.0, "plate: out to the guest")
        A.add(2.0, c1 + g1 + [0, 0, 0.004], R1, 0.0, "plate: down")
        A.add(0.6, c1 + g1 - [0, 0, 0.003], R1, 0.0, "plate: onto the table")
        A.add(0.8, grip=RELEASE_OPEN, label="plate: let go")
        A.add(1.5, c1 + g1 + [0, 0, 0.05], R1, RELEASE_OPEN, "plate: up")
        A.add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")

    one("right")
    if variant == "late":
        arms["left"].add(1.5, label="left: wait")
    elif variant == "sequential":
        arms["left"].add(arms["right"].duration, label="left: wait")
    one("left")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the table"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: down {r.get('down_steps')} gap {r.get('gap_steps')} placed {r.get('placed')} facing "
          f"{r.get('facing')} turned {r.get('turned_on_table')} tilt {r.get('max_carry_tilt')} food "
          f"{r.get('food_touched')} L {r.get('plate_left')} R {r.get('plate_right')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
