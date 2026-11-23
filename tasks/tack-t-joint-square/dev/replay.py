"""Dev only, no simulator: play the demo's plan with perfect tracking through note_step and score. A part is rigid in the
hand that grips it; the coupon stays where it is once it is tacked or let go. The offsets against the arms' give are zeroed. python dev/replay.py"""
import math
import os
import sys
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import task as t  # noqa: E402
from tack_t_joint import mech as M  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402

DT = 0.05


def st(p, R):
    p = np.asarray(p, float)
    return NS(pos=p, quat_xyzw=ik.matrix_to_quat_xyzw(R), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)


def main():
    t.HOLD_CAL.update(x=0.0, y=0.0, z=0.0, tilt=0.0, yaw=0.0)  # perfect tracking: no give to aim against
    t.TORCH_CAL.update({i: (0.0, 0.0, 0.0) for i in range(3)})
    wps = t.demo()
    plans = {a: ArmPlan(wps[a], BASES[a]) for a in wps}
    T = max(p.duration for p in plans.values())
    t.reset_latch()
    pose = {"coupon": (np.array((M.COUPON_START[0], M.COUPON_START[1], t.TOP + M.COUPON_START[2])), np.eye(3)),
            "torch": (np.array((M.TORCH_REST["x"], M.TORCH_REST["y"], t.TOP + M.TORCH_REST_Z)), t._rz(M.TORCH_REST["lean"]))}
    hand = {"coupon": "right", "torch": "left"}
    closed_at = {"coupon": t._fingers(0.074), "torch": t.TORCH_OPEN / 2}
    grasp = {}
    worst = 0.0
    q_prev = {}
    for k in range(int(round(T / DT)) + 1):
        tt = k * DT
        objs = {}
        tgt = {}
        for a, pl in plans.items():
            pos, R, finger, label = pl.target(tt)
            q, label = pl.action7(tt)
            if a in q_prev:
                worst = max(worst, float(np.abs(q[:6] - q_prev[a]).max()))
            q_prev[a] = q[:6]
            tgt[a] = (pos, R, finger)
            objs[f"{a}_tcp"] = st(pos, np.eye(3))
        for part, a in hand.items():
            pos, R, finger = tgt[a]
            p0, R0 = pose[part]
            if finger > closed_at[part] and part not in grasp and np.linalg.norm(pos - p0) < 0.012:
                grasp[part] = (R.T @ (p0 - pos), R.T @ R0)
            if finger <= closed_at[part] or (part == "coupon" and finger < t._fingers(0.0765)):
                grasp.pop(part, None)
            tacked = part == "coupon" and any(tk["intact"] for tk in t._trial["tacks"])
            if part in grasp and not tacked:
                pose[part] = (pos + R @ grasp[part][0], R @ grasp[part][1])
            objs[part] = st(*pose[part])
        for a in plans:  # a gripped coupon is pressed by both of that hand's fingers
            f = 10.0 if hand["coupon"] == a and "coupon" in grasp else 0.0
            objs[f"{a}_grip"] = st((f, f, 0.0), np.eye(3))
        t.note_step(objs, DT)
        for cmd in t.take_commands():  # a tack set on an unheld coupon pulls it to where it set
            if cmd["do"] == "lock":
                pose["coupon"] = (np.asarray(cmd["pos"], float), t._rot(cmd["quat"]))
        if k % 20 == 0 or t._trial["arc"] and k % 5 == 0:
            print(f"t={tt:5.1f} " + t.debug_line(objs)[:330])
    print("final:", t.debug_line(objs))
    print("largest joint step per control step (deg):", round(math.degrees(worst), 1), " IK worst (mm, deg):",
          {a: (round(p.worst_ik_mm, 2), round(p.worst_ik_deg, 2)) for a, p in plans.items()})
    print("score", t.score(objs))


main()
