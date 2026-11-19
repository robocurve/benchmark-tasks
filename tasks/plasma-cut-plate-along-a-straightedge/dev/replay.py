"""Dev only, no simulator: play the demo's TCP plan with ideal tracking through note_step and score. The tools are rigid in
the hands while gripped; the straightedge stops at the plate and (when on it) stops the shield; the arm's give that the plan
aims against is taken out. The offcut drops when every segment is cut. python dev/replay.py   (PLASMA_DEMO=... for variants)"""
import math
import os
import sys
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import task as t  # noqa: E402
from plasma_cut import mech as M  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402


def st(p, yaw=0.0):
    p = np.asarray(p, float)
    return NS(pos=p, quat_xyzw=np.array((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)


def main():
    wps = t.demo()
    plans = {a: ArmPlan(wps[a], BASES[a]) for a in wps}
    T = max((p.duration for p in plans.values()), default=0.0) + 1.5
    t.reset_latch()
    torch = np.array((M.TORCH_REST["x"], M.TORCH_REST["y"], t.TOP + M.TORCH_REST_Z))
    guide = np.array((M.GUIDE_REST["x"], M.GUIDE_REST["y"], t.TOP + M.GUIDE_REST_Z))
    off = np.array((M.OFFCUT_C[0], M.OFFCUT_C[1], t.TOP + M.OFFCUT_C[2]))
    held = {"left": False, "right": False}
    park = {"left": np.array((0.32, 0.305, 1.0)), "right": np.array((0.32, -0.305, 1.0))}
    give = {"left": np.array((t.GUIDE_DX, t.GUIDE_DY, 0.0)), "right": np.array((-t.LAG_T, -t.SIDE_T, t.SAG_T))}
    for k in range(int(round(T / t.DT)) + 1):
        tt = k * t.DT
        objs = {}
        for a in ("left", "right"):
            if a in plans:
                pos, _, finger, _ = plans[a].target(tt)
            else:
                pos, finger = park[a], t.FINGER_OPEN
            tool, opened = (guide, t.GUIDE_OPEN) if a == "left" else (torch, t.TORCH_OPEN)
            closed = finger > opened / 2
            if closed and np.linalg.norm(pos - (tool + (0, 0, t.GRIP_DZ))) < 0.012:
                held[a] = True
            if not closed:
                held[a] = False
            if held[a]:
                p = pos - (0, 0, t.GRIP_DZ)
                near_rest = np.linalg.norm(p[:2] - ((M.GUIDE_REST["x"], M.GUIDE_REST["y"]) if a == "left" else (M.TORCH_REST["x"], M.TORCH_REST["y"]))) < 0.03
                if not near_rest:
                    p = p - give[a]
                if a == "left":
                    over = M.X0 - 0.05 < p[0] < M.X1 + 0.05 and p[1] < M.KEPT_Y1
                    p[2] = max(p[2], (t.PLATE_Z if over else t.TOP + M.GUIDE_PAD_T) - M.GUIDE_Z0)
                    guide = p
                else:
                    rim = p + t._rz(M.TORCH_REST["heading"]) @ M.RIM
                    on_plate = M.X0 <= rim[0] <= M.X1 and abs(rim[1] - M.LINE_Y) < 0.03
                    if on_plate:
                        rim[2] = max(rim[2], t.PLATE_Z)
                    if abs(guide[2] + M.GUIDE_Z0 - t.PLATE_Z) < 0.001 and rim[2] < t.PLATE_Z + M.BAR["t"] and abs(rim[0] - guide[0]) < M.BAR["len"] / 2:
                        rim[1] = min(rim[1], guide[1] + M.EDGE_Y - M.SHIELD_R)  # the straightedge stops the shield
                    p = rim - t._rz(M.TORCH_REST["heading"]) @ M.RIM
                    torch = p
                pos = p + (0, 0, t.GRIP_DZ)
            objs[f"{a}_tcp"] = st(pos)
        if t.cut_through().all():
            off = np.array((off[0], off[1], t.TOP + M.TRAY["base"] + M.PLATE_T / 2))
        objs["torch"], objs["guide"], objs["offcut"] = st(torch, M.TORCH_REST["heading"]), st(guide), st(off)
        t.note_step(objs, t.DT)
        if k % 20 == 0:
            print(t.debug_line(objs, tt)[:150], "...", t.debug_line(objs, tt)[-95:])
    print("faults:", t.faults())
    print("dwell:", np.round(t._trial["dwell"], 2).tolist())
    print("score", t.score(objs))


main()
