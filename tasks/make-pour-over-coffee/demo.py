"""Scripted full-score demo for make-pour-over-coffee, played in the yam_arena embodiment.

    python demo.py task.py out                   # full score
    python demo.py task.py out --variant full    # pours at the kettle's full flow without stopping
    python demo.py task.py out --variant once    # fills the cone once to near its rim, then stops
    python demo.py task.py out --variant idle    # does nothing

The right gripper takes the kettle by its loop handle (pitched 12 deg down, fingers either side of the handle's bar),
lifts it and brings the spout tip over the cone, just short of its middle. It tilts the kettle until the water runs at a
brisk flow, and watches the coffee rise in the cone: when it is about two-thirds full it tilts the kettle back and
waits for the cone to drain, then pours again. It stops pouring a little before the mug is full, since what is still
in the cone drips through, and puts the kettle back where it was. The levels are read from the task state (a camera
sees the same coffee in the cone from above and in the clear mug from the side). The kettle's pose in the gripper is
measured once it is lifted, and the spout tip is steered onto its mark from the kettle's measured pose.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from demokit import main, wait, wait_until  # noqa: E402

GRASP_PITCH = 12.0          # deg the approach points down at the handle
WATCH_FROM = (-0.15, 0.23, 0.05)   # m from the mug to the left wrist camera's tool point
GRIP_UP = 0.020             # m above the middle of the handle bar
GRIP_HANDLE = 0.06          # closes on the 16 mm handle bar
LIFT = 0.07                 # m the kettle is lifted before it travels
TIP_ABOVE_RIM = 0.060       # m: the spout tip over the cone's rim
HEAD = 0.028                # m of water over the spout tip while pouring (about 10 mL/s)
TILT_DPS = 15.0
FULL_HI, FULL_LO = 105e-6, 60e-6      # m3 standing in the cone: tilt back / pour again
TARGET = 220e-6             # m3 brewed (mug + cone), counting what is still in the stream
MAX_TILT = 49.0


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


def kettle_rot(yaw, phi):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(phi), math.sin(phi)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    return Rz @ Ry


def tilt_for_head(brew, V, yaw, h):
    lo, hi = 0.0, math.radians(MAX_TILT)
    for _ in range(30):
        m = (lo + hi) / 2
        if brew.head(V, kettle_rot(yaw, m), (0, 0, 0)) < h:
            lo = m
        else:
            hi = m
    return hi


def script(ctx, variant):
    import task

    S = task._STATE
    brew = task.brew
    yield from wait(0.5)
    if variant == "idle":
        yield from wait(15.0)
        return
    R = ctx.right
    L = ctx.left
    # the left arm stands off to the side, its wrist camera on the glass mug (to see the coffee rise in it)
    look_at = np.array([*task.MUG_XY, task.TOP + 0.10])
    eye = look_at + np.array(WATCH_FROM)
    la = (look_at - eye) / np.linalg.norm(look_at - eye)
    lc = -np.cross(la, [0.0, 0.0, 1.0])
    watch = (eye, _frame(la, lc))
    yaw = task.KETTLE_YAW
    u = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    c = np.array([-u[1], u[0], 0.0])
    z = np.array([0.0, 0.0, 1.0])
    b = math.radians(GRASP_PITCH)
    a = math.cos(b) * u - math.sin(b) * z
    rot = _frame(a, c)
    kp0 = np.asarray(S["kettle_pos"] if S["kettle_pos"] is not None else task.KETTLE.pos, float)
    bar = task.handle_point(kp0, kettle_rot(yaw, 0.0)) + [0, 0, GRIP_UP]

    # take the handle (and send the left arm to watch)
    from demokit import together
    yield from together(R.move(bar - 0.09 * a + [0, 0, 0.03], rot, 1.8, grip=1.0), L.move(*watch, 2.0, grip=1.0))
    yield from R.move(bar - 0.09 * a, rot, 0.6)
    yield from R.move(bar, rot, 1.0)
    yield from R.set_grip(GRIP_HANDLE, 0.5)
    yield from wait(0.3)
    # the kettle's goal pose is tracked from its measured pose: the tool command is the goal pose carried through the
    # grip (measured as the gripper closed), corrected for the arm's sag in position and in tilt
    def kettle_now():
        return np.asarray(S["kettle_pos"], float), task.quat_matrix(S["kettle_quat"])

    kp, Rk = kettle_now()
    tcp_in_k = Rk.T @ (R.pos - kp)
    rot_in_k = Rk.T @ R.rot
    corr_p, corr_r = np.zeros(3), np.zeros(3)

    def rotvec(Rm):
        from yam_arena.ik import _rot_error
        return _rot_error(Rm, np.eye(3))

    def expm(v):
        th = float(np.linalg.norm(v))
        if th < 1e-9:
            return np.eye(3)
        k = v / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K

    def track(kp_goal, Rk_goal):
        nonlocal corr_p, corr_r
        kp, Rk = kettle_now()
        corr_r = np.clip(corr_r + 0.08 * rotvec(Rk_goal @ Rk.T), -0.7, 0.7)
        corr_p = np.clip(corr_p + 0.08 * (kp_goal - kp), -0.06, 0.06)
        Rc = expm(corr_r) @ Rk_goal
        R.aim(kp_goal + corr_p + Rc @ tcp_in_k, Rc @ rot_in_k)

    goal = {"kp": kp.copy(), "phi": 0.0}

    def carry(kp_end, phi_end, seconds):
        from yam_arena import ik
        k0, f0 = goal["kp"].copy(), goal["phi"]
        n = max(1, round(seconds * 20))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            goal["kp"] = k0 + s * (np.asarray(kp_end, float) - k0)
            goal["phi"] = f0 + s * (phi_end - f0)
            yield
            track(goal["kp"], kettle_rot(yaw, goal["phi"]))

    def hold(seconds):
        for _ in range(max(1, round(seconds * 20))):
            yield
            track(goal["kp"], kettle_rot(yaw, goal["phi"]))

    yield from carry(kp + [0, 0, LIFT], 0.0, 1.4)
    yield from hold(0.6)

    cone = np.asarray(S["dripper_pos"], float) + [0, 0, task.A.CONE_Z0 + brew.CONE_D]
    tip_goal = cone + [0, 0, TIP_ABOVE_RIM] - 0.030 * u          # short of the middle: the body clears the cone

    def kp_for(phi, tip):
        return tip - kettle_rot(yaw, phi) @ np.array(brew.TIP)

    yield from carry(kp_for(0.0, tip_goal + [0, 0, 0.03]), 0.0, 2.2)
    yield from carry(kp_for(0.0, tip_goal), 0.0, 0.8)
    yield from hold(0.5)

    phi = 0.0
    mode = "pour"
    poured_once = False
    tip_corr = np.zeros(3)

    def steer(phi_target):
        nonlocal phi, tip_corr
        step = math.radians(TILT_DPS) / 20.0
        phi += max(-step, min(step, phi_target - phi))
        kp, Rk = kettle_now()
        tip = kp + Rk @ np.array(brew.TIP)
        goal["kp"], goal["phi"] = kp_for(phi, tip_goal), phi
        track(goal["kp"] + tip_corr, kettle_rot(yaw, phi))

    t_end = S["t"] + 150.0
    while S["t"] < t_end and S["violation"] is None:
        V = S["kettle_V"]
        if variant == "full":
            target = tilt_for_head(brew, V, yaw, 0.045) if S["poured"] < 250e-6 else 0.0
            if S["poured"] >= 250e-6 and S["q"] < 0.1e-6:
                break
        elif variant == "once":
            if S["W"] > 140e-6:
                poured_once = True
            target = 0.0 if poured_once else tilt_for_head(brew, V, yaw, HEAD)
            if poured_once and S["q"] < 0.1e-6:
                break
        else:
            if S["brewed"] + S["q"] * 0.8 >= TARGET:
                mode = "done"
            elif mode == "pour" and S["W"] > FULL_HI:
                mode = "wait"
            elif mode == "wait" and S["W"] < FULL_LO:
                mode = "pour"
            target = tilt_for_head(brew, V, yaw, HEAD) if mode == "pour" else 0.0
            if mode == "done" and S["q"] < 0.1e-6 and phi < math.radians(2):
                break
        yield
        steer(target)

    # back to where it stood, down, and let go
    while phi > 0.01:
        yield
        steer(0.0)
    goal["kp"] = goal["kp"] + tip_corr
    yield from carry(goal["kp"] + [0, 0, 0.03], 0.0, 0.6)
    yield from carry(kp0 + [0, 0, 0.03], 0.0, 2.2)
    yield from carry(kp0 + [0, 0, 0.002], 0.0, 1.0)
    yield from hold(0.3)
    yield from R.set_grip(1.0, 0.5)
    yield from R.move(R.pos - 0.09 * a, R.rot, 0.8)
    yield from R.go_home(1.6)
    yield from wait(2.5)


def report(task):
    S = task._STATE
    return {"violation": S["violation"], "brewed_mL": round(S["brewed"] * 1e6, 1), "mug_mL": round(S["mug"] * 1e6, 1),
            "cone_mL": round(S["W"] * 1e6, 1), "poured_mL": round(S["poured"] * 1e6, 1),
            "max_cone_mL": round(S["max_W"] * 1e6, 1), "max_tilt_deg": round(S["max_tilt"], 1)}


if __name__ == "__main__":
    main(script, variants=("", "full", "once", "idle"), report=report)
