"""Scripted full-score demo (open loop: every waypoint comes from geometry.py, nothing is read from the simulation).

One arm holds the grinder's motor housing down on the table, as people do (the arm on the side of the motor). The other
arm (the one on the side of the head):
  1. pinches the red clamp lever by its tip and swings it up (the clamp lets the guard go);
  2. takes the guard's yellow lug between its fingers and walks it round the disc until the guard's closed side faces the
     OPERATOR mat (a half turn takes several grips: the wrist cannot turn that far in one);
  3. pinches the raised lever again and swings it down flat (the clamp holds the guard);
then both arms let go and go home.

    python demo.py task.py demo_out [--mode demo] [--max-steps 2400] [--no-record]

Modes: demo (1.0); settle (holds the grinder and does nothing else: 0); no_unlock (tries to turn the guard with the lever
shut: 0); too_little (turns the guard two thirds of the way, then shuts the lever: 0.50 in layout A); wrong_way (turns it
60 degrees the wrong way; measured 0.25 because the lever ends raised); forget_lock (turns it right but leaves the lever up: 0.50);
touch_disc (does everything right but first presses a fingertip on the disc: 0.25).
Set TASK_VARIANT=B for the mirrored layout. Writes one MP4 per camera, setup.png and result.json to the output folder.
The same script is played through ``--policy yam_scripted`` as the task's demo() (see plans() below; DEMO_MODE picks a mode).
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import CONTROL_HZ, Arm, run, together, wait  # noqa: E402

MODES = ("demo", "settle", "no_unlock", "too_little", "wrong_way", "forget_lock", "touch_disc")

# grip_rotation(yaw) closes the fingers along the gripper's x axis, which points along bearing (yaw - 90 degrees)
# (yam_arena.constants: "fingers closing along world Y" at yaw 0).
JAW_OFFSET = -math.pi / 2
ABOVE_MID = 0.006          # the tool point sits this far above the middle of what it grasps (the pads reach 13 mm below)
HOVER = 0.045              # approach height above the lug and the motor (lower is easier on the elbow)
HOVER_LEVER = 0.032        # approach height above the lever's pinch point (its tip is only 13 mm above it)
TRANSIT = 0.16             # the tool point's height over the table when it crosses the grinder (pads clear a raised lever by 12 mm)
RISE = 0.035               # how far the tool point rises after letting go of a flat lever
SIDE = 0.07                # how far out along its radius a raised lever is approached and left
LEVER_GRASP = 0.045        # the pinch point along the lever from its hinge
LEVER_UP = math.radians(88)
LUG_ABOVE = 0.010          # the tool point sits this far above the lug's middle: the pads (13 mm long) clear the skirt's rim by 5 mm
LEVER_ABOVE = 0.002        # the tool point sits this far above the lever's middle (the pads reach 13 mm below it)
HOLD_Y = -0.228            # the holding grasp, along the asset's y: over the black rear cap, 22.8 cm back from the spindle
TURN_STEP = math.radians(6)
ARM_ABOVE_DISC = 0.012     # touch_disc: the fingertips go this far below the disc's top face


def rot_for(yaw: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)


def jaw_yaw(bearing: float, near: float = 0.0) -> float:
    """The wrist yaw that puts the jaw axis along a bearing (the jaws are symmetric, so any multiple of pi will do):
    the one nearest ``near``."""
    y = bearing - JAW_OFFSET
    return y + math.pi * round((near - y) / math.pi)


def run_ik(arm, sequence, q_start=None):
    """A dry run of the arm's IK over a sequence of (tool point, yaw) poses (from ``q_start``, default where the arm is
    now): (the smallest joint margin in rad, or -1 if any pose is missed by more than 2 mm; the last joint solution).
    Pure geometry: nothing is read from the scene."""
    from yam_arena import ik

    q = (arm.q if q_start is None else q_start).copy()
    worst = 9.0
    for pos, yaw in sequence:
        q, err, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot_for(yaw)), q_init=q,
                             base_pos=arm.base, iters=80)
        if err * 1000 > 2.0:
            return -1.0, q
        worst = min(worst, float(min((q - ik.LOWER).min(), (ik.UPPER - q).min())))
    return worst, q


def margin_of(arm, sequence, q_start=None) -> float:
    return run_ik(arm, sequence, q_start)[0]


def choose_yaw(arm, y0: float, sequence_for, q_start=None) -> float:
    """The jaw-symmetric yaw (y0 plus a multiple of pi) whose whole motion keeps the wrist furthest from its limits."""
    best, best_m = y0, -9.0
    for k in (0, -1, 1, -2, 2):
        m = margin_of(arm, sequence_for(y0 + k * math.pi), q_start)
        if m > best_m + 1e-9:
            best, best_m = y0 + k * math.pi, m
    return best


def arms_of(ctx, L: dict):
    """(operating arm, holding arm): the working arm is on the side of the head (the spindle's side of the table)."""
    return (ctx.left, ctx.right) if L["spindle"][1] > 0 else (ctx.right, ctx.left)


def hold_point(L: dict) -> np.ndarray:
    """The holding grasp: 22.8 cm back from the spindle (over the rear cap), TCP 7 mm above the equator line."""
    yaw = L["yaw"]
    sx, sy = L["spindle"]
    return np.array([sx - HOLD_Y * math.sin(yaw), sy + HOLD_Y * math.cos(yaw), g.TOP + g.MOTOR_R + 0.007])


def hold_motor(arm, L: dict):
    p = hold_point(L)
    y = choose_yaw(arm, jaw_yaw(L["yaw"], 0.0), lambda yy: [(p + [0, 0, 0.10], yy), (p, yy)])
    rot = rot_for(y)                                  # jaws across the motor
    yield from arm.move(p + [0, 0, 0.10], rot, 2.0, grip=1.0)
    yield from arm.move(p, rot, 1.2)
    yield from arm.set_grip(0.0, 0.8)


def lever_point(L: dict, q: float, beta: float) -> np.ndarray:
    """Where the tool point pinches the lever when the guard is at joint angle q and the lever is raised beta."""
    h = np.array(g.lever_hinge(L, q))
    phi = L["yaw"] + q + g.LEVER_PHI
    r = np.array([math.cos(phi), math.sin(phi), 0.0])
    return h + LEVER_GRASP * (r * math.cos(beta) + np.array([0, 0, 1.0]) * math.sin(beta)) + [0, 0, LEVER_ABOVE]


def swing_lever(arm, L: dict, q: float, beta0: float, beta1: float):
    """Pinch the lever at raised angle beta0, swing it to beta1, let go and clear away. A raised lever is approached and
    left sideways (outwards along its radius: the skirt is below the pinch point), a flat one from above."""
    phi = L["yaw"] + q + g.LEVER_PHI
    out = np.array([math.cos(phi), math.sin(phi), 0.0])
    n = max(2, round(abs(beta1 - beta0) / math.radians(4)))
    betas = [beta0 + (beta1 - beta0) * k / n for k in range(n + 1)]
    side_in, side_out = beta0 > math.radians(45), beta1 > math.radians(45)
    pre = lever_point(L, q, beta0) + (out * SIDE if side_in else np.array([0, 0, HOVER_LEVER]))
    post = lever_point(L, q, beta1) + (out * SIDE if side_out else np.array([0, 0, RISE]))

    def seq(yy):
        return [(pre, yy)] + [(lever_point(L, q, bb), yy) for bb in betas] + [(post, yy)]

    rot = rot_for(choose_yaw(arm, jaw_yaw(phi + math.pi / 2, 0.0), seq))      # jaws across the lever's 14 mm width
    transit = g.TOP + TRANSIT                                                 # over a raised lever (its top is 13 cm up)
    if arm.pos[2] < transit - 0.001:
        yield from arm.move([arm.pos[0], arm.pos[1], transit], None, 1.0, grip=1.0)
    yield from arm.move([pre[0], pre[1], transit], rot, 1.8, grip=1.0)
    yield from arm.move(pre, rot, 1.0)
    yield from arm.move(lever_point(L, q, beta0), rot, 1.0)
    yield from arm.set_grip(0.0, 0.6)
    dt = 0.06 if beta1 > beta0 else 0.10                 # a lever being pushed shut is eased down
    for bb in betas[1:]:
        yield from arm.move(lever_point(L, q, bb), rot, dt)
    if beta1 < beta0:
        yield from wait(0.4)                              # hold it pressed flat for a moment
    yield from arm.set_grip(1.0, 0.5)
    yield from arm.move(post, rot, 0.8)


def lug_point(L: dict, bearing: float) -> np.ndarray:
    """The lug's middle (the tool point's xy, 6 mm above the lug's middle) when the lug is at a bearing."""
    r = g.SKIRT_R_IN + g.SKIRT_T + g.LUG_L / 2
    sx, sy = L["spindle"]
    return np.array([sx + r * math.cos(bearing), sy + r * math.sin(bearing),
                     g.TOP + g.LUG_MID + LUG_ABOVE])


def stage_sequence(L: dict, b: float, part: float, n: int, yy: float):
    return ([(lug_point(L, b) + [0, 0, HOVER], yy), (lug_point(L, b), yy)]
            + [(lug_point(L, b + part * k / n), yy + part * k / n) for k in range(1, n + 1)]
            + [(lug_point(L, b + part) + [0, 0, HOVER], yy + part)])


def plan_stages(arm, L: dict, b0: float, sweep: float, wanted: float = 0.08) -> int:
    """How many grips the turn needs: the fewest for which every grip can be done with every joint at least ``wanted``
    rad from its limit (dry-run IK, grip after grip, from where the arm is now)."""
    for stages in range(1, 8):
        part = sweep / stages
        n = max(2, round(abs(part) / TURN_STEP))
        q, ok = arm.q.copy(), True
        for i in range(stages):
            b = b0 + part * i
            y0 = choose_yaw(arm, jaw_yaw(b + math.pi / 2, 0.0), lambda yy, b=b: stage_sequence(L, b, part, n, yy), q)
            m, q = run_ik(arm, stage_sequence(L, b, part, n, y0), q)
            if m < wanted:
                ok = False
                break
        if ok:
            return stages
    return 7


def turn_guard(arm, L: dict, q: float, sweep: float):
    """Walk the lug round the disc by ``sweep`` radians, in as many grips as the wrist needs; the guard ends at q + sweep."""
    b = L["yaw"] + q + g.LUG_PHI
    stages = plan_stages(arm, L, b, sweep)
    for _ in range(stages):
        part = sweep / stages
        n = max(2, round(abs(part) / TURN_STEP))
        y0 = choose_yaw(arm, jaw_yaw(b + math.pi / 2, 0.0), lambda yy, b=b, part=part, n=n: stage_sequence(L, b, part, n, yy))
        rot = rot_for(y0)
        yield from arm.move(lug_point(L, b) + [0, 0, HOVER], rot, 1.6, grip=1.0)
        yield from arm.move(lug_point(L, b), rot, 0.9)
        yield from arm.set_grip(0.0, 0.6)
        for k in range(1, n + 1):
            rot = rot_for(y0 + part * k / n)
            yield from arm.move(lug_point(L, b + part * k / n), rot, 0.12)
        yield from arm.set_grip(1.0, 0.5)
        b += part
        yield from arm.move(arm.pos + [0, 0, HOVER], rot, 0.7)


def park_point(arm) -> np.ndarray:
    """Where an arm waits, clear of the grinder and above it: 26 cm out to its own side of the table, 16 cm above the
    table (the height every top-down plan on this table uses)."""
    return np.array([0.26, math.copysign(0.26, arm.base[1]), g.TOP + 0.16])


def park(arm, seconds: float = 2.5):
    yield from arm.move(park_point(arm), rot_for(0.0), seconds, grip=1.0)


def touch_disc(arm, L: dict):
    """touch_disc: put the closed fingertips on the disc's top face (a violation), then rise again."""
    sx, sy = L["spindle"]
    p = np.array([sx, sy, g.TOP + g.DISC_Z1 - ARM_ABOVE_DISC + 0.013])
    rot = rot_for(0.0)
    yield from arm.move(p + [0, 0, HOVER], rot, 2.0, grip=0.0)
    yield from arm.move(p, rot, 1.0)
    yield from wait(0.5)
    yield from arm.move(p + [0, 0, HOVER], rot, 1.0)


def script(ctx, mode: str = "demo", variant: str | None = None):
    L = g.layout(variant or os.environ.get("TASK_VARIANT", "A"))
    op, hold = arms_of(ctx, L)
    q0 = L["guard_q0"]
    yield from together(park(op), park(hold))                              # both arms come out of the ready pose
    yield from hold_motor(hold, L)
    if mode == "settle":
        yield from wait(4.0)
        return
    if mode == "touch_disc":
        yield from touch_disc(op, L)
    if mode != "no_unlock":
        yield from swing_lever(op, L, q0, 0.0, LEVER_UP)                  # raise the lever: the clamp lets go
    delta = g.wrap(L["operator_bearing"] - g.guard_bearing(L["yaw"], q0))   # how far the guard has to turn
    sweep = delta if abs(delta) < math.pi - 1e-6 else math.pi             # a half turn goes the near way round
    if mode == "too_little":
        sweep *= 2 / 3
    if mode == "wrong_way":                      # turned 60 degrees the wrong way (away from the operator, or sideways in layout A)
        sweep = math.radians(60) if abs(delta) >= math.pi - 1e-6 else -math.copysign(math.radians(60), delta)
    yield from turn_guard(op, L, q0, sweep)
    q1 = q0 + sweep
    if mode in ("demo", "too_little", "wrong_way", "touch_disc"):
        yield from swing_lever(op, L, q1, LEVER_UP - math.radians(2), -math.radians(3))   # shut the lever: clamp
    yield from park(op)
    yield from hold.set_grip(1.0, 0.8)
    yield from park(hold)
    yield from wait(3.0)


# -- the plan for ``--policy yam_scripted``: the same script, drained offline into per-arm Cartesian waypoints ------------------
DEMO_TIME_SCALE = 1.8      # every waypoint's duration is stretched by this, so the soft arms keep up (DEMO_TIME_SCALE=1 undoes it)


def mode_from_env() -> str:
    """The demo mode: DEMO_MODE (one of MODES; default demo)."""
    mode = os.environ.get("DEMO_MODE", "demo").strip().lower() or "demo"
    if mode not in MODES:
        raise ValueError(f"DEMO_MODE={mode!r}: pick one of {MODES}")
    return mode


class _Recorder(Arm):
    """An :class:`Arm` that also writes each move down as one waypoint (end pose, finger, duration), padding the time the
    other arm spends moving with a hold at its current pose. The script is the single source of the motion."""

    def __init__(self, side: str, clock: list):
        super().__init__(side)
        self.clock, self.t, self.waypoints = clock, 0, []

    def _wp(self, steps: int, label: str) -> None:
        from yam_arena.constants import FINGER_OPEN
        from yam_arena.demo import Waypoint, grip_rotation

        down = grip_rotation(0.0)
        m = self.rot @ down.T                                    # rot = Rz(yaw) @ down: the wrist yaw is its angle
        yaw = math.atan2(m[1, 0], m[0, 0])
        if np.abs(self.rot - grip_rotation(yaw)).max() > 1e-6:
            raise ValueError("a plan's poses are straight-down grips turned about the vertical (no other tilt)")
        self.waypoints.append(Waypoint(steps / CONTROL_HZ, tuple(float(v) for v in self.pos), FINGER_OPEN * self.grip,
                                       label, yaw))

    def move(self, pos, rot=None, seconds: float = 1.0, grip: float | None = None):
        idle = self.clock[0] - self.t
        if idle > 0:
            self._wp(idle, "wait")
            self.t += idle
        n = max(1, round(seconds * CONTROL_HZ))
        yield from super().move(pos, rot, seconds, grip)
        self.t += n
        self._wp(n, f"{self.side} move")


def plans(mode: str | None = None, variant: str | None = None, time_scale: float | None = None) -> dict:
    """``{"left": [Waypoint ...], "right": [...]}`` for ``mode`` in layout ``variant``, durations stretched by
    DEMO_TIME_SCALE (the environment variable overrides the default). Nothing is read from a simulation."""
    from dataclasses import replace

    mode = mode or mode_from_env()
    variant = variant or os.environ.get("TASK_VARIANT", "A")
    scale = time_scale if time_scale is not None else float(os.environ.get("DEMO_TIME_SCALE", DEMO_TIME_SCALE))
    clock = [0]
    ctx = SimpleNamespace(left=_Recorder("left", clock), right=_Recorder("right", clock))
    for _ in script(ctx, mode, variant):
        clock[0] += 1
    out = {}
    for side, arm in (("left", ctx.left), ("right", ctx.right)):
        out[side] = [replace(w, duration=w.duration * scale) for w in arm.waypoints]
    return out


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    p = S.get("pos", [0, 0, 0])
    return (f"pos {p[0]:.3f} {p[1]:.3f} yawd {math.degrees(S['yaw']):5.1f} tcp {S.get('tcp')}  err {math.degrees(S['err']):6.1f} deg  lever {math.degrees(S['raised']):5.1f} deg  shut {S['shut']}  "
            f"open {S['lever_open']}  locked {S['locked']}  moved {S['moved'] * 1000:5.1f} mm  clear {S['clear']}  why {S['why']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--mode", default="demo", choices=MODES)
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--log-every", type=int, default=20)
    ap.add_argument("--time-scale", type=float, default=float(os.environ.get("DEMO_TIME_SCALE", DEMO_TIME_SCALE)),
                    help="stretch every move and wait (default: the same DEMO_TIME_SCALE the yam_scripted plan uses)")
    a = ap.parse_args()
    if a.time_scale != 1.0:                       # play the script as slowly as the yam_scripted plan does
        _move, _wait = Arm.move, wait
        Arm.move = lambda self, pos, rot=None, seconds=1.0, grip=None: _move(self, pos, rot, seconds * a.time_scale, grip)
        wait = lambda seconds: _wait(seconds * a.time_scale)  # noqa: E731
    run(a.task, a.out, lambda ctx: script(ctx, a.mode), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=a.log_every)
