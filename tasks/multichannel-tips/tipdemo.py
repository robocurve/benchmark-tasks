"""The scripted reference demo. Closed loop on what the cameras see: each control step it reads the poses of the
pipette and of the tip box (both in view of every camera) and its own arms' joints. It never reads a load, a tip's
depth or the scorer.

The right hand comes in from behind the pipette standing in its stand and grips it sideways across the handle, its pads just above the
manifold's shoulder, and brings the nozzle row over column 4. The left hand holds the box down by its rim, over the
used columns. The nozzles go into the tip mouths; then, as the manuals say, the hand presses and rocks the row side to
side: it rolls the row 6 deg each way to push the end tips on, then smaller rolls for the middle ones, each time
pressing to the depth at which a tip is fully on (from the tips' visible height and the pipette's own geometry). Then it
lifts the pipette 8 cm and holds it.

Modes (TASK_DEMO), for checking the scorer: ``full`` (the demo); ``straight`` (one level push to about the rocking's
peak force, no rocking); ``straight_hard`` (one level push to the depth that seats the middle tips); ``straight_max``
(one level push deep enough to seat all eight); ``offcol`` (the same rocking over column 5); ``nosteady`` (the full demo
with the left hand left off the box); ``touch`` (the left fingers press tip H4 on by hand first).
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import axis_angle_matrix

import demokit
import tiplaw as L
import tipkit as K
from demokit import _as_grip, down

MM = 1e-3
SIDE = _as_grip(np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]))  # tool along +x, fingers closing along y
KEEP_IN = 2.5  # mm: while rocking, the raised end's nozzle stays at least this far inside its tip
MARGIN = 1.3  # mm past full seat, for the tips' own heights, which no camera resolves
# the rocking: (row, roll in deg; +: the row-A end lower), in the order a hand would rock
ROCK = [(0, 6.0), (7, -6.0), (1, 6.0), (6, -6.0), (3, 0.5), (2, 2.75), (4, -0.5), (5, -2.75)]


def _pose(ctx, name):
    st = ctx.state(name)
    return np.asarray(st.pos, float), K.quat_matrix(st.quat_xyzw)


def _rx(deg: float) -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(deg))


def _ry(deg: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


def _rotvec(r: np.ndarray) -> np.ndarray:
    a = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(a))
    return v * a


class Hold:
    """Drives the pipette, not the TCP: holds the grasp offset seen when the hand closed, and integrates out what the
    soft arm gives under load (droop and wrist tilt), from the pipette's observed pose."""

    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.right
        self.base = np.asarray(station.BASES["right"], float)
        self.cp, self.cw = np.zeros(3), np.zeros(3)

    def learn(self):
        tp, tr = ik.tcp_pose(self.ctx.observed("right")[:6], self.base)
        pp, pr = _pose(self.ctx, "pipette")
        self.off_p = tr.T @ (pp - tp)
        self.off_r = tr.T @ pr

    def _command(self, p_d, r_d):
        r_t = r_d @ self.off_r.T
        r_cmd = axis_angle_matrix(self.cw / max(np.linalg.norm(self.cw), 1e-12), float(np.linalg.norm(self.cw))) @ r_t \
            if np.linalg.norm(self.cw) > 1e-9 else r_t
        p_t = p_d - r_t @ self.off_p
        self.arm._solve(p_t + self.cp, r_cmd)

    def go(self, target, t: float, settle: float = 0.0, gain: float = 0.1):
        """Follow ``target(s)`` (s from 0 to 1 over ``t`` seconds) for the pipette's (origin, rotation), then hold the end
        for ``settle`` seconds."""
        n, m = max(1, round(t * 20)), round(settle * 20)
        for k in range(1, n + m + 1):
            p_d, r_d = target(min(1.0, k / n))
            p_o, r_o = _pose(self.ctx, "pipette")
            ep, ew = p_d - p_o, _rotvec(r_d @ r_o.T)
            self.cp = np.clip(self.cp + gain * np.clip(ep, -0.003, 0.003), [-0.006, -0.006, -0.04], [0.006, 0.006, 0.04])
            self.cw = np.clip(self.cw + gain * np.clip(ew, -0.03, 0.03), -0.35, 0.35)
            self._command(p_d, r_d)
            yield


def script(ctx):
    mode = ctx.mode
    l, r = ctx.left, ctx.right
    pp, pr = _pose(ctx, "pipette")
    bp, br = _pose(ctx, "tip_box")
    grasp = pp + pr @ np.array([0.0, 0.0, K.GRIP_Z])
    rim_tcp = bp + br @ np.array([0.0475, K.BOX_W / 2 + 0.006, K.RIM_H]) + np.array([0.0, 0.0, 0.013])
    behind = grasp + np.array([-0.06, 0.0, 0.0])
    yield from ctx.together(r.move(behind + np.array([0.0, -0.08, 0.04]), grip=SIDE, finger=FINGER_OPEN, t=2.5, label="out to the side"),
                            l.move(rim_tcp + np.array([0.0, 0.0, 0.04]), grip=down(0.0), t=3.0, label="over the rim"))
    yield from ctx.together(r.move(behind, t=1.2, label="behind the handle"), l.close(t=0.6, label="fingers together"))
    yield from r.move(grasp, t=1.2, label="round the handle")
    yield from r.close(width=2 * K.HANDLE_R, force=40.0, label="grip the handle")
    if mode != "nosteady":
        yield from l.move(rim_tcp + np.array([0.0, 0.0, -0.012]), t=1.0, label="hold the box down")
    yield from r.wait(0.3)
    hold = Hold(ctx)
    hold.learn()

    col = 5 if mode == "offcol" else 4

    seen = {}

    def row_target(s_mid, roll=0.0, pitch=0.0):
        """The pipette's pose for the nozzle row's middle ``s_mid`` mm below the column's mouths, rolled and pitched.
        The box's pose is read until the nozzles go in, then kept: the column does not move with the hand."""
        bp, br = seen["box"] if "box" in seen else _pose(ctx, "tip_box")
        mouth = bp + br @ np.array([K.COL_X[col - 1] - K.BOX_XY[0], 0.0, K.TIP_TOP])
        rd = br @ _rx(-roll) @ _ry(pitch)
        end = mouth - br[:, 2] * (s_mid * MM)
        return end + rd @ np.array([0.0, 0.0, L.NOZZLE_LEN * MM]), rd

    def ramp(a, b):
        """From pose target a to b, both (s_mid, roll, pitch)."""
        def f(s):
            w = 0.5 - 0.5 * math.cos(math.pi * s)
            s_mid, roll, pitch = [(1 - w) * x + w * y for x, y in zip(a, b)]
            if min(a[0], b[0]) > 0.0:  # in the tips: keep the raised end's nozzle in its tip
                s_mid = max(s_mid, KEEP_IN + 31.5 * math.tan(math.radians(abs(roll))))
            return row_target(s_mid, roll, pitch)
        return f

    p0, r0 = _pose(ctx, "pipette")
    up = p0 + np.array([0.0, 0.0, 0.035])
    yield from hold.go(lambda s: (p0 + (up - p0) * s, r0), t=1.2)
    above = row_target(-15.0)
    yield from hold.go(lambda s: (up + (above[0] - up) * (0.5 - 0.5 * math.cos(math.pi * s)), above[1]), t=2.5, settle=1.0)
    yield from hold.go(ramp((-15.0, 0, 0), (-2.0, 0, 0)), t=1.0, settle=0.5)

    def off_axis():
        """The worst sideways offset (mm) of a nozzle end from its tip's axis, from the two visible poses."""
        pp, pr = _pose(ctx, "pipette")
        bp, br = _pose(ctx, "tip_box")
        worst = 0.0
        for y in K.ROW_Y:
            e = pp + pr @ np.array([0.0, y, -L.NOZZLE_LEN * MM])
            m = bp + br @ np.array([K.COL_X[col - 1] - K.BOX_XY[0], y, K.TIP_TOP])
            d = e - m
            worst = max(worst, float(np.linalg.norm(d - br[:, 2] * float(d @ br[:, 2]))) / MM)
        return worst

    good = 0
    for _ in range(80):  # line the nozzles up over the mouths before going in
        good = good + 1 if off_axis() < 0.3 else 0
        if _ % 10 == 0:
            pp, pr = _pose(ctx, "pipette")
            tgt = row_target(-2.0)
            print(f"[tipdemo] align: off {off_axis():.2f} mm, origin error {np.round((tgt[0] - pp) * 1000, 2).tolist()} mm, cp {np.round(hold.cp * 1000, 1).tolist()}", flush=True)
        if good >= 6:
            break
        yield from hold.go(lambda s: row_target(-2.0), t=0.05)
    seen["box"] = _pose(ctx, "tip_box")
    yield from hold.go(ramp((-2.0, 0, 0), (1.0, 0, 0)), t=1.5, settle=0.2)
    yield from hold.go(ramp((1.0, 0, 0), (L.S_CONTACT - 0.5, 0, 0)), t=1.0, settle=0.3)
    state = (L.S_CONTACT - 0.5, 0.0, 0.0)

    def depth_of(row):
        """How far nozzle ``row``'s end is below its tip's mouth (mm), from the two visible poses."""
        pp, pr = _pose(ctx, "pipette")
        bp, br = seen["box"]
        e = pp + pr @ np.array([0.0, K.ROW_Y[row], -L.NOZZLE_LEN * MM])
        m = bp + br @ np.array([K.COL_X[col - 1] - K.BOX_XY[0], K.ROW_Y[row], K.TIP_TOP])
        return -float((e - m) @ br[:, 2]) / MM

    def press(goal, t=0.8, settle=0.4):
        nonlocal state
        yield from hold.go(ramp(state, goal), t=t, settle=settle)
        state = goal

    if mode == "touch":  # the left fingers push tip H4 down: a gripper touching a tip
        bp, br = _pose(ctx, "tip_box")
        h4 = bp + br @ np.array([K.COL4_X - K.BOX_XY[0], K.ROW_Y[7], K.TIP_TOP]) + np.array([0.0, 0.0, 0.013])
        yield from l.move(h4 + np.array([0.0, 0.0, 0.03]), t=1.5)
        yield from l.move(h4 - np.array([0.0, 0.0, 0.002]), t=1.0)
        yield from l.move(h4 + np.array([0.0, 0.0, 0.03]), t=0.8)

    if mode.startswith("straight"):  # level pushes, no rocking: to about the rocking's peak force (12 N), to where the
        # middle tips seat, and to where all eight seat (the straight-push calibration)
        depth = {"straight": L.S_CONTACT + 0.9, "straight_hard": L.S_CONTACT + L.U_FULL - 0.2,
                 "straight_max": L.S_CONTACT + L.U_FULL + 0.4}[mode]
        yield from press((depth, 0.0, 0.0), t=1.5, settle=0.3)
        for _ in range(60):
            if min(depth_of(r) for r in range(8)) >= depth - 0.03:
                break
            yield from press((state[0] + 0.03, 0.0, 0.0), t=0.05, settle=0.0)
        yield from press(state, t=0.05, settle=1.0)
    else:
        for row, roll in ROCK:
            s_mid = L.S_CONTACT + L.U_FULL + MARGIN - L.ROW_Y[row] * math.tan(math.radians(roll))
            yield from press((s_mid - 1.0, roll, 0.0), t=1.2, settle=0.3)  # roll over, just short
            yield from press((s_mid, roll, 0.0), t=0.6, settle=0.2)  # and press
            want = s_mid + L.ROW_Y[row] * math.tan(math.radians(roll))  # this row's nozzle depth wanted
            for _ in range(70):  # press on until the nozzle is seen that deep (the arm gives under the load)
                if depth_of(row) >= want or state[0] > s_mid + 3.0:
                    break
                yield from press((state[0] + 0.03, roll, 0.0), t=0.05, settle=0.0)
            print(f"[tipdemo] row {row} roll {roll}: wanted {want:.2f} mm, seen {depth_of(row):.2f} mm, commanded mid {state[0]:.2f}", flush=True)
            yield from press(state, t=0.05, settle=0.3)
            yield from press((s_mid - 1.0, roll, 0.0), t=0.8, settle=0.3)  # ease off before the next roll
        yield from press((L.S_CONTACT + 0.3, 0.0, 2.0), t=0.8, settle=0.2)  # a small front-to-back rock
        yield from press((L.S_CONTACT + 0.3, 0.0, -2.0), t=0.8, settle=0.2)
    yield from press((L.S_CONTACT - 1.0, 0.0, 0.0), t=1.6, settle=0.6)  # ease off slowly: the soft arm springs back
    yield from press((-50.0, 0.0, 0.0), t=3.0, settle=0.3)  # slowly, straight up, until the tips are out of their holes
    # up and out to where the top camera sees the row past the hand: higher, a little forward, turned 35 deg so the
    # wrist is not in front of the tips; the left hand lets go of the box and moves up out of the view
    p0, r0 = _pose(ctx, "pipette")
    p1, r1 = row_target(-130.0)
    p1 = p1 + np.array([0.03, 0.0, 0.0])
    r1 = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(35.0)) @ r1

    def show(s):
        w = 0.5 - 0.5 * math.cos(math.pi * s)
        return p0 + (p1 - p0) * w, axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(35.0) * w) @ r0

    if mode != "nosteady":
        lp = l.at()
        yield from ctx.together(hold.go(show, t=1.4, settle=0.0), l.move(lp + np.array([-0.06, 0.06, 0.12]), t=1.4, label="let go of the box"))
    else:
        yield from hold.go(show, t=1.4, settle=0.0)
    yield from hold.go(lambda s: (p1, r1), t=0.1, settle=5.0)
