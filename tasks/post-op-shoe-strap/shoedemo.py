"""The scripted reference demo, open loop from the fixed start: Cartesian waypoints for each arm, played by the
harness's ``yam_scripted`` policy with IK on the arm's URDF. It reads nothing from the simulator while it runs.

1. The right hand pinches the shoe's heel tab and holds it while the left hand works (the task neither requires nor
   scores this).
2. The left hand pinches the strap's pull tab, draws it straight up (the strap runs back through its D-ring and the
   upper closes against its springback), swings it round the D-ring at about the tail's length so that the tail lies
   back over the strap, and presses the tab's end down.
3. It lets go, lifts clear, and comes down with its closed fingertips on the folded strap right at the D-ring, then
   smooths the strap down in one slide from the fold to its end, square to the upper (laying it down from the ring
   outward, as a person does).
4. Both hands let go and go home.

Modes (TASK_DEMO): success, and two scripted failures: failure (the strap is drawn snug and its end held down, then let
go without being smoothed down along its length) and loose (the tab is laid over and pressed down without first being
drawn up, so the upper is not closed).
"""

from __future__ import annotations

import math
import sys

import numpy as np

from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN, GRIPPER, READY
from yam_arena.demo import ArmPlan, Waypoint

TOP = station.TABLE_TOP


def allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation (world frame)."""
    from yam_arena import demo

    if getattr(demo, "_matrix_grips", False):
        return
    plain = demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    demo.grip_rotation = grip_rotation
    demo._matrix_grips = True


def grip(x, z) -> np.ndarray:
    """A TCP rotation whose fingers point along ``z`` and close along ``x`` (world)."""
    x, z = np.asarray(x, float), np.asarray(z, float)
    z = z / np.linalg.norm(z)
    x = x - np.dot(x, z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def finger_for(width: float, force: float = 0.0) -> float:
    """Finger joint target that closes the pads on something ``width`` across with about ``force`` N a finger."""
    return float(np.clip(-(width / 2 - force / GRIPPER["stiffness"]), FINGER_OPEN, 0.0))


DOWN_Y = grip((0, -1, 0), (0, 0, -1))  # straight down, closing along y


def tip_grip(deg: float) -> np.ndarray:
    """``DOWN_Y`` tipped ``deg`` about x, the fingertips toward -y (square to the upper's slope)."""
    a = math.radians(deg)
    return grip((0, -math.cos(a), math.sin(a)), (0, -math.sin(a), -math.cos(a)))


def heel_grip(deg: float = 20.0) -> np.ndarray:
    """Fingers pointing along +y and ``deg`` down (the right hand comes in from its own side), closing along x."""
    a = math.radians(deg)
    return grip((1, 0, 0), (0, math.cos(a), -math.sin(a)))


def home(side: str, t: float = 2.0) -> Waypoint:
    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[side])
    return Waypoint(t, tuple(float(v) for v in pos), FINGER_OPEN, "home", rot)


# Plan in the shoe's frame (origin: middle of the sole's underside; x toward the heel, y toward the left arm, z up).
TAB_Y, TAB_PINCH_Z = 0.0605, 0.074  # the pinch: the pads close on the upright tab below its bead
PAD = 0.013  # the pads' tips are this far past the TCP
# The strap leaves the pads at their tips. Their path (y, z): first straight up, drawing the strap through its D-ring
# until the upper is closed, then round the D-ring at about the tail's length, which lays the tail back over the strap.
OVER = ((0.046, 0.128), (0.010, 0.119), (-0.012, 0.099), (-0.020, 0.093))
PRESS_END = (-0.036, 0.077)  # pad tips aimed here, past the strap's end over the instep (the arm stops short of it)
# loose (scripted failure): the tab laid straight over and down, without the draw up
OVER_LOOSE = ((0.050, 0.100), (0.030, 0.106), (0.012, 0.100))
PRESS_END_LOOSE = (0.008, 0.094)
# the smoothing slide along the folded strap, from the fold at the D-ring to the strap's end: the top of the strap
# (y, z) and the finger tilt (deg) square to the upper there. The fingertips go FOLD_IN past the strap's top at the
# fold and ease to SMOOTH_IN over the first three points (the arm is soft: about 2.5 mm/N), SMOOTH_T s per point.
SMOOTH = ((0.0365, 0.0695, 40.0), (0.0329, 0.0733, 40.0), (0.0289, 0.0772, 40.0), (0.0247, 0.0808, 40.0),
          (0.0202, 0.0841, 40.0), (0.0154, 0.0869, 40.0), (0.0102, 0.0888, 30.0), (0.0050, 0.0908, 15.0),
          (-0.0035, 0.0985, 0.0), (-0.0090, 0.1005, 0.0))
SMOOTH_IN, FOLD_IN, SMOOTH_T = 0.012, 0.020, 0.4
OVER_DZ = 0.006  # the swing round the D-ring runs this much higher than the tail's length
OVER_T = 1.2  # s per point of the swing
HEEL = (0.144, -0.012, 0.108)  # the heel pull tab, pinched from the right arm's side


def plan(mode, w0, shoe) -> dict:
    """``mode``: success, failure or loose (module docstring)."""
    allow_matrix_grips()
    w0 = np.asarray(w0, float)

    def P(x, y, z):
        return tuple(float(v) for v in w0 + np.array((x, y, z)))

    open_f = finger_for(0.026)
    pinch_f = 0.0  # fully closed: the pads squeeze the 4 mm tab
    left = []
    t_left = 0.0

    def L(dur, pos, f, label, rot=DOWN_Y):
        nonlocal t_left
        left.append(Waypoint(dur, pos, f, label, rot))
        t_left += dur

    hx, hy, hz = HEEL
    right = [
        Waypoint(2.5, P(hx, hy - 0.07, hz + 0.01), finger_for(0.032), "right: beside the heel tab", heel_grip()),
        Waypoint(1.0, P(hx, hy, hz), finger_for(0.032), "right: round the heel tab", heel_grip()),
        Waypoint(0.6, P(hx, hy, hz), finger_for(0.008, 15.0), "right: pinch the heel tab", heel_grip()),
    ]
    L(2.5, P(shoe.straps[0].x, TAB_Y, 0.13), open_f, "left: over the tab")

    x = shoe.straps[0].x
    L(1.5, P(x, TAB_Y, TAB_PINCH_Z), open_f, "left: down round the tab")
    L(0.6, P(x, TAB_Y, TAB_PINCH_Z), pinch_f, "left: pinch the tab")
    over, end = (OVER_LOOSE, PRESS_END_LOOSE) if mode == "loose" else (OVER, PRESS_END)
    for y, z in over:
        L(OVER_T, P(x, y, z + PAD + OVER_DZ), pinch_f, "left: draw the strap back over")
    pe = P(x, end[0], end[1] + PAD)
    L(1.0, pe, pinch_f, "left: press the end of the strap down")
    L(0.8, pe, pinch_f, "left: hold")
    if mode == "failure":
        L(0.5, pe, open_f, "left: let go (careless: not pressed along)")
        L(0.8, P(x, end[0], 0.13), open_f, "left: up")
    else:
        L(0.3, pe, open_f, "left: let go of the tab")
        # one smoothing pass: the closed fingertips come down on the folded strap at the fold by the D-ring and slide
        # along it to its end, following the upper's curve, laying the hook patch down from the ring outward
        pts = [np.array((x, y, z)) for y, z, _ in SMOOTH]
        L(0.5, P(x, end[0], end[1] + PAD + 0.02), open_f, "left: up off the tab")
        y0, z0, a0 = SMOOTH[0]
        g0 = tip_grip(a0)
        # lift well clear before crossing to the fold: a low crossing catches the released tab and drags it back
        L(0.8, P(x, end[0], 0.135), 0.0, "left: up")
        L(1.2, P(*(pts[0] - g0[:, 2] * (PAD + 0.030))), 0.0, "left: over the fold", g0)
        L(1.0, P(*(pts[0] - g0[:, 2] * (PAD + 0.006))), 0.0, "left: over the fold", g0)
        L(0.6, P(*(pts[0] - g0[:, 2] * (PAD - FOLD_IN))), 0.0, "left: fingertips down on the strap at the fold", g0)
        L(0.4, P(*(pts[0] - g0[:, 2] * (PAD - FOLD_IN))), 0.0, "left: fingertips down on the strap at the fold", g0)
        for i, ((y, z, a), top) in enumerate(zip(SMOOTH[1:], pts[1:])):
            g = tip_grip(a)
            dep = FOLD_IN + (SMOOTH_IN - FOLD_IN) * min(1.0, (i + 1) / 3)
            L(SMOOTH_T, P(*(top - g[:, 2] * (PAD - dep))), 0.0, "left: smooth the strap down", g)
        y, z, a = SMOOTH[-1]
        L(0.6, P(x, y - 0.01, z + 0.03), 0.0, "left: off the strap", tip_grip(a))
        L(2.5, P(x, 0.06, 0.14), open_f, "left: up")
    left.append(home("left"))
    t_left += 2.0

    t_right = sum(w.duration for w in right)
    steady = max(1.0, t_left - 2.0 - 0.5 - t_right)
    right += [
        Waypoint(steady, P(hx, hy, hz), finger_for(0.008, 15.0), "right: steady the heel", heel_grip()),
        Waypoint(0.5, P(hx, hy, hz), finger_for(0.032), "right: let go", heel_grip()),
        Waypoint(1.0, P(hx, hy - 0.07, hz + 0.01), finger_for(0.032), "right: out", heel_grip()),
        home("right"),
    ]
    return {"left": left, "right": right}


if __name__ == "__main__":  # IK check, no simulator: python shoedemo.py [mode]
    import shoekit
    shoe = shoekit.Shoe(name="shoe", pos=(0.24, 0.0, TOP), size=(0.3, 0.12, 0.1))
    for mode in (sys.argv[1:] or ["success", "failure", "loose"]):
        p = plan(mode, (0.24, 0.0, TOP + 0.0005), shoe)
        for side, wps in p.items():
            ap = ArmPlan(list(wps), station.BASES[side])
            t, dt = 0.0, 0.05
            while t <= ap.duration:
                ap.action7(t)
                t += dt
            print(mode, side, f"duration {ap.duration:.1f} s, IK worst {ap.worst_ik_mm:.1f} mm {ap.worst_ik_deg:.1f} deg")
