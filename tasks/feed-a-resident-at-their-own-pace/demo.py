"""Scripted full-score demo for feed-a-resident-at-their-own-pace, played in the yam_arena embodiment.

    TASK_LOT=A python demo.py task.py out                # full score (lots A, B, C)
    TASK_LOT=A python demo.py task.py out --variant eager   # feeds whenever the mouth opens: two bites at once
    TASK_LOT=A python demo.py task.py out --variant timer   # never looks; waits 50 s after every bite: too slow
    TASK_LOT=A python demo.py task.py out --variant idle    # does nothing
    TASK_LOT=A python demo.py task.py out --variant sweep   # pushes the held bite off the tongue with the next one

The left arm picks the spoon up by its handle (the gripper pitched 64 degrees from straight down, so that the spoon
can later be tipped in the mouth) and scoops a bite: the spoon, tip down 25 degrees, slides along the dish floor
and pushes the cube into the dish's far rim until the bevelled tip is under it, then swings tip-up under it. It
brings the spoon level to 5 cm in front of the lips, which makes the mouth open. Once open, the demo looks at the
tongue: if a bite is still there it takes the spoon back more than 8 cm, waits and comes again; if the tongue is
clear it slides the bowl in over the tongue, tips it 20 degrees forward so the bite slides off onto the tongue,
draws it out still tipped and levels it outside the lips, before the jaw closes. Where the spoon sits in the fingers
is measured after each lift; the cube and spoon poses and whether the tongue holds a bite are read from the simulator.
For a policy the same is in view of the spoon arm's (left) wrist camera, which looks straight into the open mouth as
the spoon comes up.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from demokit import down, main, wait, wait_until  # noqa: E402

GRASP_PITCH = 64.0     # deg from straight down
SCOOP_PITCH = 25.0     # deg, spoon tip down, sliding along the dish floor
SCOOP_UP = 10.0        # deg, spoon tip up as it swings under the bite
LEVEL = -3.0           # deg (tip slightly up) while carrying
TIP = 20.0             # deg, tipped forward so the bite slides off the tip onto the tongue
LIFT = 0.002           # m above the going-in height before tipping
X_IN = 0.039           # m, the spoon tip behind the lips' plane when fully in
APPROACH = 0.05        # m, the spoon tip in front of the lips: inside the 6 cm that opens the mouth
RETREAT = 0.10         # m, out of the way (more than 8 cm) while waiting
GAIN_IN = float(__import__('os').environ.get('GAIN_IN', '0.25'))
SLOW = float(__import__('os').environ.get('SLOW', '1.0'))
WAIT_HOLD = 50.0       # s, the "timer" variant's fixed wait


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _log_rot(R):
    """Rotation vector of a rotation matrix."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    return a / (2 * math.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _exp_rot(v):
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    u = v / a
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


class Held:
    """Where the spoon sits in the tool frame (measured): converts a wanted spoon pose to a tool pose. ``bias`` and
    ``rbias`` correct the arm's tracking error (it sags a little under load), fed back from the measured spoon
    pose by :func:`servo`."""

    def __init__(self, ctx, arm):
        st = ctx.states()["spoon"]
        Rs, ps = ctx.task.quat_matrix(st.quat_xyzw), np.asarray(st.pos, float)
        self.R = arm.rot.T @ Rs
        self.p = arm.rot.T @ (ps - arm.pos)
        self.bias = np.zeros(3)
        self.rbias = np.zeros(3)

    def tool(self, p_spoon, R_spoon):
        Rt = _exp_rot(self.rbias) @ R_spoon @ self.R.T
        return np.asarray(p_spoon, float) - Rt @ self.p + self.bias, Rt


def servo(ctx, H, pose, seconds, gain=0.4, rgain=0.0, bias_to=None):
    """A smooth straight move of the spoon to ``pose`` (from where it is), correcting the arm's tracking error as
    it goes: each step the measured spoon pose's lag behind the path is fed back into the bias."""
    from yam_arena import ik

    L, T = ctx.left, ctx.task
    st = ctx.states()["spoon"]
    p0, R0 = np.asarray(st.pos, float), T.quat_matrix(st.quat_xyzw)
    n = max(1, round(seconds * 20))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        p = p0 + s * (np.asarray(pose[0], float) - p0)
        R = ik.slerp_matrix(R0, pose[1], s)
        if bias_to is not None:          # the sag a tipped pose needed goes away as the spoon levels again
            if k == 1:
                b_from = H.bias.copy()
            H.bias = b_from + s * (bias_to - b_from)
        L.aim(*H.tool(p, R))
        yield
        st = ctx.states()["spoon"]
        H.bias = H.bias + gain * (p - np.asarray(st.pos, float))
        H.rbias = H.rbias + rgain * _log_rot(R @ T.quat_matrix(st.quat_xyzw).T)
        nb, nr = np.linalg.norm(H.bias), np.linalg.norm(H.rbias)
        if nb > 0.02:
            H.bias *= 0.02 / nb
        if nr > 0.25:
            H.rbias *= 0.25 / nr


def settle_on(ctx, H, pose, rounds=3):
    """Hold the spoon at ``pose`` and correct the command until the measured spoon position matches it."""
    for _ in range(rounds):
        yield from ctx.left.move(*H.tool(*pose), 0.3)
        yield from wait(0.25)
        err = pose[0] - np.asarray(ctx.states()["spoon"].pos, float)
        H.bias = H.bias + err
        if np.linalg.norm(err) < 0.0015:
            break


def tip_len(A) -> float:
    return A.BOWL_L / 2 + A.TIP_L


def pick_spoon(ctx):
    T, A, L = ctx.task, ctx.task.A, ctx.left
    st = ctx.states()["spoon"]
    Rs, ps = T.quat_matrix(st.quat_xyzw), np.asarray(st.pos, float)
    g = ps + Rs @ np.array([A.GRASP_X, 0.0, A.HANDLE_Z])
    G = rot_y(-math.radians(GRASP_PITCH)) @ down()
    yield from L.move(g + [0.0, 0.0, 0.07], G, 1.8, grip=1.0)
    yield from L.move(g + [0.0, 0.0, 0.002], G, 0.9)
    yield from L.set_grip(0.0, 0.5)
    yield from wait(0.2)
    H = Held(ctx, L)
    yield from L.move(*H.tool(ps + [0.0, 0.0, 0.08], np.eye(3)), 0.9)
    yield from wait(0.3)
    ctx.flags["H"] = Held(ctx, L)


def scoop(ctx, name):
    """Push cube ``name`` along +x into the dish's far rim with the spoon tip down, then swing under it."""
    T, A, L = ctx.task, ctx.task.A, ctx.left
    yield from wait(0.3)
    ctx.flags["H"] = H = Held(ctx, L)
    tip = tip_len(A)
    c = ctx.pos(name)
    floor = T.TOP + A.DISH_T
    R = rot_y(math.radians(SCOOP_PITCH))
    start = np.array([c[0] - 0.035 - tip * math.cos(math.radians(SCOOP_PITCH)), c[1],
                      floor - 0.0005 + tip * math.sin(math.radians(SCOOP_PITCH))])
    yield from L.move_speed(*H.tool(start + [0.0, 0.0, 0.03], R), speed=0.25, min_s=0.8)
    yield from settle_on(ctx, H, (start + [0.0, 0.0, 0.03], R))     # line the spoon up with the cube's lane
    yield from L.move(*H.tool(start, R), 0.6)
    wall_x = T.DISH_C[0] + A.DISH_L / 2 - A.DISH_WALL
    end = start + [wall_x - tip * math.cos(math.radians(SCOOP_PITCH)) - start[0] - 0.001, 0.0, 0.0]
    yield from L.move(*H.tool(end, R), 2.0)
    q = np.array([0.016, 0.0, 0.0])
    piv = end + R @ q
    n = 24
    for k in range(1, n + 1):
        Rk = rot_y(math.radians(SCOOP_PITCH + (-SCOOP_UP - SCOOP_PITCH) * k / n))
        yield from L.move(*H.tool(piv - Rk @ q + [-0.004 * k / n, 0.0, 0.004 * k / n], Rk), 0.05)
    yield from L.move(*H.tool(piv - Rk @ q + [-0.03, 0.0, 0.06], Rk), 1.5)
    yield from wait(0.3)
    ctx.flags["H"] = Held(ctx, L)


def in_spoon(ctx, name) -> bool:
    T = ctx.task
    st = ctx.states()
    Rs = T.quat_matrix(st["spoon"].quat_xyzw)
    return T.in_spoon(Rs.T @ (np.asarray(st[name].pos) - np.asarray(st["spoon"].pos)))


def tongue_holds_food(ctx) -> bool:
    """A bite lies on the tongue (what the left wrist camera sees in the open mouth)."""
    T = ctx.task
    st = ctx.states()
    return any(T.in_mouth(T.to_head(st[n].pos)) and not T.swallowed(T.to_head(st[n].pos)) for n in T.CUBE_NAMES)


def spoon_at(ctx, xtip, dz=0.0, pitch=LEVEL):
    T, A = ctx.task, ctx.task.A
    R = rot_y(math.radians(pitch))
    z = A.TONGUE_TOP + A.RIDGE + 0.003 + dz
    return T.HEAD_O + np.array([xtip, 0.0, z]) - R @ np.array([tip_len(A), 0.0, 0.0]), R


def offer(ctx, look=True, x_in=X_IN):
    """Bring the loaded spoon up to the lips until the mouth opens. With ``look``, put the bite in only if the
    tongue is clear (else back off and come again); without, put it in whenever the mouth opens."""
    T, L = ctx.task, ctx.left
    H = ctx.flags["H"]
    yield from L.move_speed(*H.tool(*spoon_at(ctx, -RETREAT, 0.02)), speed=0.25, min_s=0.8)
    yield from wait(0.4)
    ctx.flags["H"] = H = Held(ctx, L)
    yield from settle_on(ctx, H, spoon_at(ctx, -0.064))
    while True:
        yield from servo(ctx, H, spoon_at(ctx, -APPROACH), 0.5, gain=0.15)
        yield from wait_until(lambda: T.gap(T._STATE["jaw"]) > 0.0285, 3.0)
        if T.gap(T._STATE["jaw"]) <= 0.0285 or (look and tongue_holds_food(ctx)):
            # still chewing on the last one: back off past 8 cm, give it time, and offer again
            yield from L.move(*H.tool(*spoon_at(ctx, -RETREAT, 0.02)), 0.6)
            yield from wait_until(lambda: T._STATE["mouth"] == "rest", 6.0)
            yield from wait(1.0)
            H.bias = np.zeros(3)
            yield from settle_on(ctx, H, spoon_at(ctx, -0.064))
            continue
        break
    yield from servo(ctx, H, spoon_at(ctx, x_in), 0.9, gain=0.0)
    yield from L.move(*H.tool(*spoon_at(ctx, x_in, LIFT)), 0.1)
    c0, R0 = spoon_at(ctx, x_in, LIFT)
    piv = c0 + R0 @ np.array([0.0, 0.0, 0.004])
    Rr = rot_y(math.radians(TIP))
    b_level = H.bias.copy()
    yield from servo(ctx, H, (piv + Rr @ (c0 - piv), Rr @ R0), 0.8, gain=GAIN_IN)
    # draw it straight back, still tipped, until it is out past the lips (the bite stays on the tongue), then level
    yield from wait(0.25)                   # let the bite slide clear of the tip
    tipped = (piv + Rr @ (c0 - piv), Rr @ R0)
    yield from servo(ctx, H, (tipped[0] + [-(x_in + 0.012), 0.0, 0.0], tipped[1]), 1.2, gain=0.0, bias_to=b_level)
    H.bias = b_level
    yield from servo(ctx, H, spoon_at(ctx, -0.03, LIFT), 0.3, gain=0.0)
    yield from L.move(*H.tool(*spoon_at(ctx, -RETREAT, 0.02)), 0.5)
    print(f"t {T._STATE['t']:.1f} offered: {T.status()}", flush=True)


def feed_all(ctx, variant):
    T = ctx.task
    yield from pick_spoon(ctx)
    for i, name in enumerate(T.CUBE_NAMES):
        for _ in range(3):
            yield from scoop(ctx, name)
            if in_spoon(ctx, name):
                break
            print(f"t {T._STATE['t']:.1f} missed {name}, scooping again", flush=True)
        if variant == "timer" and i > 0:
            yield from wait_until(lambda: T._STATE["t"] >= ctx.flags["fed_at"] + WAIT_HOLD, 60.0)
        # sweep (a regression test): goes in 3 cm deeper with the 2nd bite to shove the held one off the tongue
        yield from offer(ctx, look=(variant == ""), x_in=X_IN + (0.03 if variant == "sweep" and i == 1 else 0.0))
        ctx.flags["fed_at"] = T._STATE["t"]
    yield from ctx.left.move(*ctx.flags["H"].tool(*spoon_at(ctx, -0.18, 0.05)), 1.0)


def script(ctx, variant):
    if variant == "idle":
        yield from wait(5.0)
        return
    yield from feed_all(ctx, variant)


if __name__ == "__main__":
    main(script, variants=("", "eager", "timer", "idle", "sweep"),
         report=lambda task: {"log": task._STATE["log"], "violation": task._STATE["violation"],
                              "fed": task.fed(), "lot": task._STATE["lot"],
                              "max_depth_mm": round(task._STATE["max_depth"] * 1000, 1)})
