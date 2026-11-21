"""The scripted demo for the tote task. Each lid starts lying flat (180 degrees) on its fixed support with its near end sticking out
10 cm past the support. Each arm pinches that near end of its own lid between its gripper pads (one pad above, one below the plate),
lifts the lid off the support, swings the lid up
past upright and down again to a few degrees short of flat (both arms on the same clock, so the lids come down together),
lets go, withdraws, and presses the lid's edge down lightly with the closed gripper so it sits flat on the rim; then both
arms withdraw and go home.

Open loop from the fixed starting layout: every target is computed from the tote's dimensions and mark in ``task.py`` and
``totekit.py``; nothing is read from the simulator. The arms' tool points are IK solutions on the URDF kinematics.

Environment variable ``TOTE_MODE`` selects control runs (default ``both``):
  both     the demo
  nopress  both lids lowered together, released, no final press
  stay     both lids lowered together and released, the grippers stay at the lids (never withdrawn) until the run ends
  seq      the left lid is closed and seated first, then the right lid is brought down after it
  left     the left arm only
  right    the right arm only
``TOTE_LAG`` (seconds, default 0) holds the right arm back that long before it starts lifting (angle-lag probe).

    python demo.py            # offline: solve every step, report the worst IK error

``yam_scripted`` plays the plan: this module swaps the policy's plan class for one that plays precomputed joint commands.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

HZ = 20
FULL_OPEN = 0.0939                  # m between the pads at grip command 1.0
APPROACH_GAP, RELEASE_GAP = 0.060, 0.016
U_PIN = 0.10                        # the lid is pinched this far out from its hinge axis ...
X_PIN = -0.259                      # ... at its near end (tote frame x), the pads reaching 1 cm in from the end
U_PRESS = 0.130                     # the closed gripper presses this far out from the hinge (on the full-thickness plate of either lid)
X_PRESS = {1: -0.20, -1: -0.08}     # tote-frame x of each arm's press (staggered so the hands keep apart)
PRESS_PITCH = math.radians(-45.0)
PRESS_YAW = math.radians(45.0)      # the pointer leans toward the middle of the tote
PRESS_DEPTH = 0.030                 # commanded this far into the plate (the arm stands off by its own compliance)
THETA_REL = math.radians(9.0)       # lids are let go at this opening
THETA0 = math.radians(180.0)
T_SWEEP = 9.0


def frame(approach, closing) -> np.ndarray:
    """Tool rotation whose approach axis (z) and finger-closing axis (x) point along these world directions."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float)
    x = x - z * (x @ z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def gap(width: float) -> float:
    return min(1.0, max(0.0, width / FULL_OPEN))


def ease(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def build(side: int, T, mode: str, delay: float = 0.0, lag: float = 0.0):
    """Plan for one arm: a list of (duration, fn(s) -> (tool point, rotation), grip)."""
    import totekit as K
    from yam_arena import ik
    from yam_arena.constants import READY

    o = T.start()
    base = np.array([0.0, 0.305 * side, 0.7475])
    p_ready, r_ready = ik.tcp_pose(np.array(READY, float), base)

    def pin_pose(theta, x=X_PIN):
        p = o + K.lid_point(side, theta, U_PIN, offset=-K.LID_T / 2, x=x)
        n = -side * np.array([0.0, side * math.sin(theta), math.cos(theta)])      # closing axis: the lid's face normal, signed for the wrist
        return p, frame((1.0, 0.0, 0.0), n)

    def line(a, b):
        (pa, ra), (pb, rb) = a, b
        return lambda s: (pa + s * (pb - pa), ik.slerp_matrix(ra, rb, s))

    plan = []

    def add(dur, fn, grip):
        plan.append((dur, fn, grip))

    def hold(pose, dur, grip):
        add(dur, lambda s: pose, grip)

    ready = (p_ready, r_ready)
    if delay > 0:
        hold(ready, delay, 1.0)
    pin0 = pin_pose(THETA0)
    pre = (pin0[0] + np.array([-0.04, 0.0, 0.0]), pin0[1])
    add(3.0, line(ready, pre), gap(APPROACH_GAP))
    add(1.2, line(pre, pin0), gap(APPROACH_GAP))
    add(0.8, lambda s: pin0, 0.0)
    if lag > 0 and side < 0:
        hold(pin0, lag, 0.0)

    def sweep(s):
        th = THETA0 + s * (THETA_REL - THETA0)
        return pin_pose(th)

    add(T_SWEEP, sweep, 0.0)
    rel = pin_pose(THETA_REL)
    add(0.6, lambda s: rel, gap(RELEASE_GAP))
    if mode == "stay":                                           # control: let go, then the grippers stay at the lids until the run ends
        hold(rel, 60.0, gap(RELEASE_GAP))
        return plan
    else:
        hold(rel, 0.8, gap(RELEASE_GAP))
        back = (rel[0] + np.array([-0.07, 0.0, 0.0]), rel[1])
        add(1.5, line(rel, back), gap(RELEASE_GAP))
    if mode != "nopress":
        xp = X_PRESS[side]
        ap = np.array([math.cos(PRESS_PITCH) * math.cos(PRESS_YAW), -side * math.cos(PRESS_PITCH) * math.sin(PRESS_YAW),
                       math.sin(PRESS_PITCH)])
        pr = frame(ap, (0.0, 0.0, 1.0))
        tip = 0.013                                              # pad tips reach this far past the tool point
        surf = o + K.lid_point(side, 0.0, U_PRESS, offset=-K.LID_T, x=xp)       # the plate's top face, shut
        touch = surf - tip * ap                                  # tool point when the pad tips just touch
        hover = touch + np.array([0.0, 0.0, 0.08])
        up = (back[0] + np.array([0.0, 0.0, 0.12]), back[1])
        add(1.2, line(back, up), 0.0)
        add(2.0, line(up, (hover, pr)), 0.0)
        add(1.2, line((hover, pr), (touch + np.array([0.0, 0.0, 0.02]), pr)), 0.0)
        pressed = touch + np.array([0.0, 0.0, -PRESS_DEPTH])
        add(1.5, line((touch + np.array([0.0, 0.0, 0.02]), pr), (pressed, pr)), 0.0)
        hold((pressed, pr), 2.0, 0.0)
        add(1.0, line((pressed, pr), (hover, pr)), 0.0)
        add(3.0, "home", gap(APPROACH_GAP))
    else:
        add(3.0, "home", gap(APPROACH_GAP))
    return plan


def solve_plan(side: int, T, mode: str = "both", delay: float = 0.0, lag: float = 0.0):
    """Joint commands (N x 7) for one arm, solved step by step from the ready pose."""
    from yam_arena import ik
    from yam_arena.constants import READY

    base = np.array([0.0, 0.305 * side, 0.7475])
    plan = build(side, T, mode, delay, lag)
    q = np.array(READY, float)
    cmds, worst_mm, worst_deg, bad = [], 0.0, 0.0, []
    grip = 1.0
    for si, (dur, fn, g1) in enumerate(plan):
        n = max(1, round(dur * HZ))
        g0 = grip
        if fn == "home":                                    # back to the ready pose in joint space
            q0 = q.copy()
            for k in range(1, n + 1):
                s = ease(k / n)
                q = q0 + s * (np.array(READY, float) - q0)
                grip = g0 + (g1 - g0) * s
                cmds.append(np.r_[q, grip])
            continue
        for k in range(1, n + 1):
            s = ease(k / n)
            pos, rot = fn(s)
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q, base_pos=base, iters=60)
            if pe > 0.005 or re > 0.05:
                bad.append((si, k, round(pe * 1000, 1), round(math.degrees(re), 1)))
            worst_mm, worst_deg = max(worst_mm, pe * 1000), max(worst_deg, math.degrees(re))
            grip = g0 + (g1 - g0) * ease(k / n)
            cmds.append(np.r_[q, grip])
    return np.array(cmds), worst_mm, worst_deg, bad


class Track:
    """A precomputed per-step command list; what ``yam_scripted`` plays as one arm's plan."""

    def __init__(self, cmds):
        self.cmds = np.asarray(cmds, float)
        self.duration = len(self.cmds) / HZ


_CACHE: dict = {}


def plans(T=None):
    if T is None:
        import task as T

    if not _CACHE:
        mode = os.environ.get("TOTE_MODE", "both")
        lag = float(os.environ.get("TOTE_LAG", "0"))
        arms = {"both": "left,right", "nopress": "left,right", "stay": "left,right", "seq": "left,right", "left": "left", "right": "right"}[mode]
        delay_right = 0.0
        if mode == "seq":
            delay_right = sum(d for d, _, _ in build(+1, T, "both")) + 1.0
        for name, side in (("left", +1), ("right", -1)):
            if name in arms.split(","):
                cmds, mm, deg, bad = solve_plan(side, T, mode, delay_right if side < 0 else 0.0, lag)
                print(f"DEMO {name}: {len(cmds)} steps, worst IK {mm:.1f} mm {deg:.1f} deg, {len(bad)} steps off {bad[:3]}", flush=True)
                _CACHE[name] = [Track(cmds)]
    return dict(_CACHE)


class TrackPlan:
    """Stands in for ``yam_arena.demo.ArmPlan``: plays the precomputed commands of a :class:`Track`."""

    def __init__(self, waypoints, base_pos, q_start=None, finger_start=None):
        self.track = waypoints[0]
        self.duration = self.track.duration

    def action7(self, t: float):
        n = len(self.track.cmds)
        k = min(max(int(round(t * HZ)) - 1, 0), n - 1)
        return self.track.cmds[k], "demo"


def install() -> None:
    """Make ``yam_scripted`` play :class:`TrackPlan` for this task file."""
    import yam_arena.scripted as S

    S.ArmPlan = TrackPlan


def hang_pinch_check(T, height=0.02):
    """Offline IK for pinching a lid that hangs at the hinge stop (THETA_STOP) by its free edge at the near end, ``height`` m above
    its lowest edge (the edge hangs a few mm above the table), the way the demo pinches the flat lid (approach +x, pads closing across the
    plate). Returns {side: (u, tool point z above the table, position error mm, rotation error deg)}."""
    import totekit as K
    from yam_arena import ik
    from yam_arena.constants import READY

    o = T.start()
    out = {}
    for side in (+1, -1):
        base = np.array([0.0, 0.305 * side, 0.7475])
        z0 = next(z for z in K.lid_zones(side) if z["x0"] <= X_PIN <= z["x1"])
        u = z0["u_end"]                                                              # the lid's free edge, along the plate
        th = K.THETA_STOP
        p = o + K.lid_point(side, th, u - height, offset=-K.LID_T / 2, x=X_PIN)
        n = -side * np.array([0.0, side * math.sin(th), math.cos(th)])
        q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(frame((1.0, 0.0, 0.0), n)), np.array(READY, float), base_pos=base, iters=300)
        out[side] = (round(u, 4), round(float(p[2] - T.TOP), 4), round(pe * 1000, 1), round(math.degrees(re), 2))
    return out


if __name__ == "__main__":
    import task as T

    mode = sys.argv[1] if len(sys.argv) > 1 else "both"
    if mode == "hang":
        print("hanging-lid pinch, 2 cm above the free edge (side: u, z above table, pos err mm, rot err deg):", hang_pinch_check(T))
        sys.exit(0)
    for name, side in (("left", +1), ("right", -1)):
        cmds, mm, deg, bad = solve_plan(side, T, mode)
        print(name, "steps", len(cmds), "worst", round(mm, 1), "mm", round(deg, 1), "deg", "bad", bad[:10], len(bad), flush=True)
