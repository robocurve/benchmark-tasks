"""Scripted full-score demo for serve-a-bowl-of-soup-with-a-ladle, played in the yam_arena embodiment.

    python demo.py task.py out                   # full score
    python demo.py task.py out --variant rush    # carries each ladleful across quickly, no pause for drips
    python demo.py task.py out --variant one     # serves one ladleful
    python demo.py task.py out --variant idle    # does nothing

The right gripper takes the ladle's soft handle across its flat faces, 4 cm from its end, pointing down the handle,
and the arm tracks the ladle's measured pose (the soft arm sags under a full ladle). It lifts the ladle out of its
lean, levels it over the pot, dips it under the soup, lifts it out level, tips off the brim over the pot and holds it
there for the drips, carries it over the gap between the pot and the bowl and down, and pours it by tipping it about
an axis between its handle and level, turning about the lip so the lip stays low over the middle of the bowl. Twice;
then it lowers the ladle back into the pot and lets go. Levels are read from the task state (the top camera sees the
soup in the pot, in the ladle and in the bowl).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from demokit import main, wait  # noqa: E402

GRIP_HANDLE = 0.0            # closes on the 16 mm handle (about 40 N a finger)
DIP = 0.025                 # m: the rim under the soup's surface (the ladle's bowl kept 8 mm off the floor)
CLEAR = 0.080               # m: the ladle's rim over the pot's rim while lifting out and waiting for drips
OVER_BOWL = 0.060           # m: the lip over the bowl's rim while pouring (the ladle's bowl clear of it)
POUR_ROLL = 100.0           # deg
SHED_ROLL = 6.0             # deg: tipping off the brim over the pot


def _frame(a, c):
    from yam_arena.demo import grip_rotation

    R0 = grip_rotation(0.0)
    ci, ai = int(np.argmax(np.abs(R0[1, :]))), int(np.argmax(np.abs(R0[2, :])))
    cs, as_ = np.sign(R0[1, ci]), np.sign(-R0[2, ai])
    a = a / np.linalg.norm(a)
    c = c - np.dot(c, a) * a
    c /= np.linalg.norm(c)
    R = np.zeros((3, 3))
    R[:, ai], R[:, ci] = as_ * a, cs * c
    k = 3 - ai - ci
    R[:, k] = np.cross(R[:, (k + 1) % 3], R[:, (k + 2) % 3])
    return R


def rot_axis(axis, t):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


def script(ctx, variant):
    import task
    import make_assets as A
    import soup

    S = task._STATE
    yield from wait(0.5)
    if variant == "idle":
        yield from wait(15.0)
        return
    R = ctx.right
    h, fn = A.handle_frame()

    def ladle_now():
        return np.asarray(S["ladle_pos"], float), task.quat_matrix(S["ladle_quat"])

    lp, Rl = ladle_now()
    grip = lp + Rl @ A.handle_point(A.GRIP_AT)
    a = -(Rl @ h)
    rot = _frame(a, Rl @ fn)
    yield from R.move(grip - 0.08 * a + [0, 0, 0.03], rot, 1.8, grip=1.0)
    yield from R.move(grip - 0.04 * a, rot, 0.6)
    yield from R.move(grip, rot, 0.8)
    yield from R.set_grip(GRIP_HANDLE, 0.5)
    yield from wait(0.3)

    # track the ladle's goal pose through the grip (measured as it closed), correcting for the arm's sag
    lp, Rl = ladle_now()
    tcp_in = Rl.T @ (R.pos - lp)
    rot_in = Rl.T @ R.rot
    corr_p, corr_r = np.zeros(3), np.zeros(3)

    def rotvec(Rm):
        from yam_arena.ik import _rot_error
        return _rot_error(Rm, np.eye(3))

    def track(p_goal, R_goal):
        nonlocal corr_p, corr_r
        p, Rn = ladle_now()
        cr = np.clip(corr_r + 0.12 * rotvec(R_goal @ Rn.T), -0.6, 0.6)
        cp = np.clip(corr_p + 0.12 * (p_goal - p), -0.08, 0.08)
        # the corrected command, shrunk towards the plain goal until the arm can reach it
        from yam_arena import ik
        best = None
        for shrink in (1.0, 0.7, 0.4, 0.0):
            c_r, c_p = cr * shrink, cp * shrink
            Rc = rot_axis(c_r, np.linalg.norm(c_r)) @ R_goal if np.linalg.norm(c_r) > 1e-9 else R_goal
            pos, rot_ = p_goal + c_p + Rc @ tcp_in, Rc @ rot_in
            q, err, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot_), q_init=R.q, base_pos=R.base, iters=80)
            if best is None or err < best[0]:
                best = (err, q, pos, rot_, c_r, c_p)
            if err < 0.008:
                break
        err, q, pos, rot_, corr_r, corr_p = best
        if err < 0.030:                      # the nearest the arm can get (never a jump to another posture)
            R.q, R.pos, R.rot = q, np.asarray(pos, float), np.asarray(rot_, float)
        R.worst_mm = max(R.worst_mm, err * 1000)

    goal = {"p": lp.copy(), "R": Rl.copy()}

    def slerp(R0, R1, s):
        from yam_arena import ik
        return ik.slerp_matrix(R0, R1, s)

    def carry(p_end, R_end, seconds):
        from yam_arena import ik
        p0, R0 = goal["p"].copy(), goal["R"].copy()
        n = max(1, round(seconds * 20))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            goal["p"] = p0 + s * (np.asarray(p_end, float) - p0)
            goal["R"] = slerp(R0, R_end, s)
            yield
            track(goal["p"], goal["R"])

    def hold(seconds):
        for _ in range(max(1, round(seconds * 20))):
            yield
            track(goal["p"], goal["R"])

    yaw = task.LADLE_YAW
    R_level = task.rot_z(yaw)
    u = np.array([math.cos(yaw), math.sin(yaw), 0.0])           # towards the handle
    pot = np.asarray(S["pot_pos"], float)
    bowl = np.asarray(S["bowl_pos"], float)
    rim_z = pot[2] + soup.H_POT

    # the lip that pours: the side of the ladle's rim (in its frame), and the roll about the handle that lowers it
    lip_loc = None
    lip_loc = np.array([0.0, -soup.A_RIM, 0.0])          # the rim's side towards the robot's right
    tip_axis = 0.5 * np.array([1.0, 0.0, 0.0]) + math.sin(math.radians(60)) * h   # between the handle and level
    tip_axis /= np.linalg.norm(tip_axis)
    roll_sgn = next(s_ for s_ in (1.0, -1.0)
                    if (rot_axis(R_level @ tip_axis, s_ * math.radians(30)) @ R_level @ lip_loc)[2] < -0.005)

    def rolled(t_deg):
        return rot_axis(R_level @ tip_axis, roll_sgn * math.radians(t_deg)) @ R_level

    up_pot = pot + [0, 0, soup.H_POT + CLEAR] + 0.02 * u
    n_ladles = 1 if variant == "one" else 2
    # straight up out of its lean first, then level over the middle of the pot
    yield from carry(goal["p"] + [0, 0, 0.05], goal["R"], 1.2)
    for k in range(n_ladles):
        if k == 0:
            yield from carry(up_pot, R_level, 1.6)
        else:
            yield from carry(mid, R_level, 1.4)
            yield from carry(up_pot, R_level, 1.6)
        surf = pot[2] + S["pot_V"] / (math.pi * soup.R_POT ** 2)
        dip = np.array([pot[0], pot[1], max(surf - DIP, pot[2] + soup.H_CUP + 0.008)]) + 0.02 * u
        yield from carry(dip, R_level, 1.6)
        yield from hold(1.0)
        yield from carry(up_pot, R_level, 1.8)
        # tip off the brim back into the pot, level again, and let it drip over the pot
        yield from carry(up_pot, rolled(SHED_ROLL), 0.8)
        yield from hold(0.6)
        yield from carry(up_pot, R_level, 0.6)
        yield from hold(0.2 if variant == "rush" else 4.0)
        # over the bowl: the lip above its middle
        lip_w = np.array([bowl[0], bowl[1], bowl[2] + soup.BOWL_H + OVER_BOWL])
        p_over = lip_w - R_level @ lip_loc
        # across over the two rims (the gap between pot and bowl), then down over the bowl
        mid = np.array([(up_pot[0] + p_over[0]) / 2, pot[1] + soup.R_POT + 0.025, up_pot[2] - 0.005])
        yield from carry(mid, R_level, 0.5 if variant == "rush" else 1.6)
        yield from carry(p_over + [0, 0, 0.01], R_level, 0.4 if variant == "rush" else 1.2)
        yield from carry(p_over, R_level, 0.5)
        # pour: roll about the handle, turning about the lip so it stays over the middle
        n = 70
        for i in range(1, n + 1):
            Rr = rolled(POUR_ROLL * i / n)
            goal["p"], goal["R"] = lip_w - Rr @ lip_loc, Rr
            yield
            track(goal["p"], goal["R"])
            if S["L"] < 1.0e-6 and i > n // 2:
                break
        yield from hold(1.2)
        yield from carry(lip_w - R_level @ lip_loc + [0, 0, 0.01], R_level, 1.0)
        yield from hold(0.3 if variant == "rush" else 1.5)
    # back in the pot: up, across, down to its floor, level, and let go
    yield from carry(mid, R_level, 1.4)
    yield from carry(up_pot, R_level, 1.6)
    down = np.array([pot[0], pot[1], pot[2] + soup.H_CUP + 0.004]) + 0.02 * u
    yield from carry(down, R_level, 2.0)
    yield from hold(0.4)
    yield from R.set_grip(1.0, 0.5)
    yield from R.move(R.pos + [0, 0, 0.06], R.rot, 0.8)
    yield from R.go_home(1.8)
    yield from wait(3.0)


def report(task):
    S = task._STATE
    return {"violation": S["violation"] and [float(S["violation"][0]), S["violation"][1]], "bowl_mL": round(S["W"] * 1e6, 1), "spilt_mL": round(S["spilt"] * 1e6, 2),
            "rim_mL": round(S["rim"] * 1e6, 2), "pours": S["pours"], "pot_mL": round(S["pot_V"] * 1e6, 1),
            "ladle_mL": round(S["L"] * 1e6, 1), "ladle_home": bool(S["ladle_home"]), "max_tilt": round(S["max_tilt"], 1),
            "spill_by": {k: round(v * 1e6, 2) for k, v in S.get("cause", {}).items()},
            "spill_log": [[k[0], k[1], k[2], round(v * 1e6, 3)] for k, v in sorted(S["soup"].log.items(), key=lambda kv: kv[0][2]) if v > 2e-8]}


if __name__ == "__main__":
    main(script, variants=("", "rush", "one", "idle"), report=report)
