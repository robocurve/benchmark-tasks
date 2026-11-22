"""Scripted demo for clear-from-the-right-serve-from-the-left, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the plates' state every 10 steps

full        The left arm pinches the starter plate's left rim, lifts it and takes it off the cover toward the guest's
            right (the robot's left), then sets it on the bus tray. Meanwhile the right arm lifts the main plate off
            the pass by its right rim, brings it round to the guest's left (the robot's right) and, once the starter
            is gone, slides it in and sets it on the cover with the steak toward the guest.
Failure and cheating variants:
clear_front the starter is taken off straight toward the robot (0.5)
serve_front the main is brought straight in from the robot's side (0.5)
stack       the main is set down on top of the starter before it is cleared (0)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "clear_front", "serve_front", "stack")
_CACHE: dict = {}
LIP_OPEN = 0.30  # gripper opening while approaching a plate's rim lip (about 2.8 cm)
RELEASE_OPEN = 0.18  # opening to let go of a lip (about 1.7 cm): at LIP_OPEN the inner finger came within 8 mm of the food
LIFT = 0.050


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    lz = t.PLATE_T + t.LIP_H - 0.006
    gl = np.array([0.0, (t.PLATE - t.LIP_W) / 2, lz])  # the left (+y) rim lip
    gr = np.array([0.0, -(t.PLATE - t.LIP_W) / 2, lz])  # the right (-y) rim lip
    s0, m0 = np.array(t.STARTER_POS), np.array(t.MAIN_POS)
    cover = np.array([*t.COVER_XY, t.TOP + 0.0005])
    tray = np.array([*t.TUB_XY, t.TOP + 0.006 + 0.0005])
    up = np.array([0, 0, LIFT])
    clear_front = variant == "clear_front"
    serve_front = variant == "serve_front"
    # sideways 22 cm off / onto the cover (at 16 cm the arm's lag cut the corner and the plate crossed only about 4 cm
    # to the side, under the 5 cm needed); the front variants go 16 cm toward the robot (further would cross the pass
    # shelf)
    s_way = s0 + ([-0.16, 0.0, 0.0] if clear_front else [0.0, 0.22, 0.0]) + up
    m_way = cover + ([-0.16, 0.0, 0.0] if serve_front else [0.0, -0.22, 0.0]) + up
    if "grips" not in _CACHE:
        cands = [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90, 180)
                 for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.95]
        lp = [s0 + gl, s0 + gl + up, s0 + gl + up + [-0.16, 0, 0], s0 + gl + up + [0, 0.22, 0], tray + gl, tray + gl + up]
        out_l, out_r = np.array([0.0, float(np.sign(gl[1])), 0.0]), np.array([0.0, float(np.sign(gr[1])), 0.0])
        RL, e1 = dc.best_rotation("left", lp, [R for R in cands if float(R[:, 2] @ out_l) <= 0.02])  # hand not leaning in
        rp = [m0 + gr, m0 + gr + up, cover + gr + up + [0, -0.22, 0], cover + gr + up + [-0.16, 0, 0], cover + gr,
              s0 + gr + [0, 0, 0.03]]
        RR, e2 = dc.best_rotation("right", rp, [R for R in cands if float(R[:, 2] @ out_r) <= 0.02])
        _CACHE["grips"] = (RL, RR)
        print(f"starter grip {1000 * e1:.1f}, main grip {1000 * e2:.1f} (mm + 0.05 rad)", flush=True)
    RL, RR = _CACHE["grips"]

    if variant != "stack":
        L.add(2.5, s0 + gl + [0, 0, 0.05], RL, LIP_OPEN, "starter: above the rim")
        L.add(1.5, s0 + gl, RL, LIP_OPEN, "starter: around the rim")
        L.add(1.0, s0 + gl, RL, 0.0, "starter: grip")
        L.add(1.2, s0 + gl + up, RL, 0.0, "starter: lift")
        L.add(2.0, s_way + gl, RL, 0.0, "starter: off the cover")
        L.add(2.5, tray + gl + up, RL, 0.0, "starter: over the tray")
        L.add(1.5, tray + gl + [0, 0, 0.003], RL, 0.0, "starter: down")
        L.add(0.5, tray + gl - [0, 0, 0.002], RL, 0.0, "starter: onto the tray")
        L.add(0.8, grip=RELEASE_OPEN, label="starter: let go")
        L.add(1.5, tray + gl + [0, 0, 0.06], RL, RELEASE_OPEN, "starter: up")
        L.add(2.0, starts["left"][0], starts["left"][1], 1.0, "left: home")
    Rt.add(2.5, m0 + gr + [0, 0, 0.05], RR, LIP_OPEN, "main: above the rim")
    Rt.add(1.5, m0 + gr, RR, LIP_OPEN, "main: around the rim")
    Rt.add(1.0, m0 + gr, RR, 0.0, "main: grip")
    Rt.add(1.2, m0 + gr + up, RR, 0.0, "main: lift")
    Rt.add(2.5, m_way + gr, RR, 0.0, "main: beside the cover")
    if variant == "stack":
        dest = s0 + [0, 0, t.PLATE_T + 0.004]
        Rt.add(2.0, dest + gr + up, RR, 0.0, "main: over the starter")
        Rt.add(1.5, dest + gr + [0, 0, 0.003], RR, 0.0, "main: down")
        last = dest + gr
    else:
        Rt.add(max(0.5, 9.5 - Rt.duration), m_way + gr, RR, 0.0, "main: wait for the clear")
        Rt.add(2.0, cover + gr + up, RR, 0.0, "main: onto the cover")
        Rt.add(1.5, cover + gr + [0, 0, 0.003], RR, 0.0, "main: down")
        Rt.add(0.5, cover + gr - [0, 0, 0.002], RR, 0.0, "main: onto the table")
        last = cover + gr
    Rt.add(0.8, grip=RELEASE_OPEN, label="main: let go")
    Rt.add(1.5, last + [0, 0, 0.06], RR, RELEASE_OPEN, "main: up")
    Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
    return L, Rt, {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the tray", "onto the table"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: exit {r.get('exit_side_y')} entry {r.get('entry_side_y')} tray {r.get('in_tub')} served "
          f"{r.get('served')} facing {r.get('facing_deg')} early {r.get('set_before_cleared')} lap {r.get('over_lap')} "
          f"| {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
