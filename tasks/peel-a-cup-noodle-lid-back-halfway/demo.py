"""Scripted full-score demo for peel-a-cup-noodle-lid-back-halfway, played in the yam_arena embodiment.

    python demo.py task.py out                     # full score
    python demo.py task.py out --variant onehand   # peels the same way without holding the cup
    python demo.py task.py out --variant squeeze   # as the demo, but clamps the cup hard enough to dent it
    python demo.py task.py out --variant rip       # holds the cup, pulls the tab steadily back past the far rim
    python demo.py task.py out --variant nowatch   # holds the cup, pulls steadily to the halfway pose, not watching
    python demo.py task.py out --variant idle      # does nothing

The left gripper takes the cup round its body, coming in from the robot's side with its fingers either side of the
tab's line, so the pull can't pivot the cup in its grip. The right gripper comes in from outside the tab, tilted down
along it, and pinches it. It pulls the lid back up and away at about 45 deg, harder and harder until the tab's seal
lets go (about 6 N); the moment the front moves it eases off to just ahead of it, so the stored pull doesn't run the
peel on across the cup, and then follows the front a few millimetres at a time, watching where the lid still sticks
to the rim (the peel front). It stops once the front has passed the middle of the cup, then lets go of the tab and
both arms back away. The front's position is read from the task state (a
camera sees the same edge where the folded-back lid meets the part still on the rim).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from demokit import main, together, wait, wait_until  # noqa: E402

TARGET = 0.040            # m peeled: stop once the front is this far (the band is 38-57 mm of the 95 mm lid)
PULL_DEG = 45.0           # the flap's angle above the table, pulled back toward the right arm
STEP = 0.0010            # m the hand leads on by per control step while the front stays put
LEAD_MAX = 0.100
EASE = -0.010            # m: once the front has moved, the hand is put back this far behind where it would be
GRIP_TAB = 0.0            # closed on the 1.2 mm tab
PRESS = 0.020             # m the left hand's command is set below where it holds the cup
GRIP_HARD = 0.30          # squeeze: a clamping grip that would dent the paper cup
GRIP_CUP = 0.817          # round the 78 mm body


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


def script(ctx, variant):
    import task

    S = task._STATE
    A = task.A
    yield from wait(0.5)
    if variant == "idle":
        yield from wait(10.0)
        return
    L, R = ctx.left, ctx.right
    c = task.BASE
    lid_z = c[2] + 0.0005 + A.H + 0.0002 + A.TAB_T / 2
    tab = np.array([c[0], c[1] - A.LID_R - A.TAB_L / 2 - 0.001, lid_z])
    hold = np.array([c[0], c[1] + 0.0, c[2] + 0.050])
    a_hold = np.array([1.0, 0.0, 0.0])                       # left: in from the robot's side, fingers across y
    rot_hold = _frame(a_hold, np.array([0.0, 1.0, 0.0]))

    def take_cup():
        yield from L.move(hold - 0.12 * a_hold + [0, 0, 0.02], rot_hold, 1.6, grip=1.0)
        yield from L.move(hold - 0.12 * a_hold, rot_hold, 0.5)
        yield from L.move(hold, rot_hold, 1.2)
        yield from L.set_grip(GRIP_HARD if variant == "squeeze" else GRIP_CUP, 0.5)
        # press the held cup down onto the table, as a hand does: the soft arm then pushes down instead of giving
        yield from L.move(hold - [0, 0, PRESS], rot_hold, 0.6)

    def rot_pull(deg):
        """Fingers closing across the flap; the tool points along the flap toward the front."""
        d = math.radians(deg)
        a = np.array([0.0, math.cos(d), -math.sin(d)])
        return _frame(a, np.array([0.0, math.sin(d), math.cos(d)]))

    corr = np.zeros(3)
    hold_corr = [False]

    def aim_r(p, r):
        """Right arm to ``p`` with the sag the soft arm shows taken out (closed loop on the measured tool point)."""
        nonlocal corr
        tcp = S.get("tcp", {}).get("right")
        if tcp is not None and not hold_corr[0]:
            corr = np.clip(corr + 0.35 * (np.asarray(p) - tcp), -0.03, 0.03)
        R.aim(np.asarray(p) + corr, r)

    def move_r(p, r, seconds):
        p0 = R.pos - corr
        n = max(1, round(seconds * 20))
        for k in range(1, n + 1):
            s = 0.5 - 0.5 * math.cos(math.pi * k / n)
            aim_r(p0 + s * (np.asarray(p) - p0), r)
            yield

    def take_tab():
        up = math.degrees(A.TAB_UP)
        r0 = rot_pull(up)
        tip = S.get("tab_tip", tab)
        out = np.array([0.0, -math.cos(A.TAB_UP), math.sin(A.TAB_UP)])
        yield from R.move(tip + 0.10 * out + [0, 0, 0.02], r0, 1.6, grip=1.0)
        yield from R.set_grip(0.45, 0.3)
        yield from move_r(tip + 0.03 * out, r0, 0.8)
        yield from move_r(tip, r0, 0.8)
        yield from wait(0.3)
        hold_corr[0] = True                      # close on the tab where the hand is; don't push it with the loop
        yield from R.set_grip(GRIP_TAB, 0.6)
        hold_corr[0] = False

    if variant == "onehand":
        yield from take_tab()
    else:
        yield from together(take_cup(), take_tab())
    yield from wait(0.3)

    d = math.radians(PULL_DEG)
    back = np.array([0.0, -math.cos(d), math.sin(d)])
    edge_y = c[1] - A.LID_R

    def hand_at(p):
        """Where the tab is with the lid peeled ``p`` m and folded back up at PULL_DEG."""
        front = np.array([c[0], edge_y + p, lid_z])
        flap = p + 0.001 + task.GRASP_AT * A.TAB_L
        return front + flap * back

    if variant in ("rip", "nowatch"):
        # rip: one steady pull back past the far rim, as when taking a lid right off;
        # nowatch: the same steady pull, but only to where the hand would be with the lid peeled halfway
        hold_corr[0] = True
        end = 0.16 if variant == "rip" else TARGET
        n = round((2.0 if variant == "rip" else 3.0) * 20)
        for k in range(1, n + 1):
            aim_r(hand_at(end * k / n), rot_pull(PULL_DEG))
            yield
    else:
        p_cmd, last = 0.0, 0.0
        hold_corr[0] = True                      # the sag found on the free arm stays; the seal's pull is not fought
        while task.peeled_m(S["m"]) < TARGET - 1e-6 and S["violation"] is None:
            got = task.peeled_m(S["m"])
            if got > last + 1e-6:
                # the front moved: ease the pull off to just ahead of it, so the stored pull doesn't run it on
                p_cmd, last = got + EASE, got
            elif p_cmd < got + LEAD_MAX:
                p_cmd += STEP
            aim_r(hand_at(p_cmd), rot_pull(PULL_DEG))
            yield
            if ctx.now() > 60.0:
                break
    yield from wait(0.5)
    # let go of the tab and back away; the lid stays folded back on its crease
    yield from R.set_grip(1.0, 0.4)
    yield from R.move(R.pos + back * 0.05, None, 0.6)
    if variant != "onehand":
        yield from L.set_grip(1.0, 0.4)
        yield from L.move(hold - 0.12 * a_hold + [0, 0, 0.02], rot_hold, 0.8)
    yield from together(L.go_home(1.4), R.go_home(1.4))
    yield from wait(1.5)


def report(task):
    S = task._STATE
    return {"violation": S["violation"], "peeled_mm": round(task.peeled_m(S["m"]) * 1000, 1),
            "max_force_N": round(S["max_force"], 2), "tilt": round(S["tilt"], 1),
            "peel_log": S["peel_log"], "max_lift_mm": round(S["max_lift"] * 1000, 1),
            "max_squeeze_N": round(S["max_squeeze"], 1), "fmax": [round(float(x), 2) for x in S["fmax"]]}


if __name__ == "__main__":
    main(script, variants=("", "onehand", "squeeze", "rip", "nowatch", "idle"), report=report)
