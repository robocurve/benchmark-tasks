"""Scripted full-score demo. The right arm takes the windlass rod from above by the end toward the arms and the hip,
and turns it anticlockwise (seen from above): past the first two times it lines up with the clip's hooks (30 and 78
degrees in, gauge about 90 and 140), to 194 degrees, where its wrist is near its limit. The left arm takes the other
end and holds it still while the right lets go, returns to its ready pose and takes the same end again, its wrist
turned the other way round; the left lets go. The right turns the rod on to 210 degrees, where the end it does not
hold lines up with the clip's hip-side hook, the gauge in the green. It rolls its hand so one finger is under the rod
and lifts its own end, tipping the rod about its middle until the other end goes down into the hook, then lets go and
backs off. Every move follows the rod's measured angle and tilt; the script reads the gauge's needle, not the band's
internal state.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--early-lock] [--hold] [--slip] [--keep-hold]

``--early-lock`` turns the rod the other way (clockwise), 102 degrees to where it first lines up with a hook (the
knee-side one, gauge about 160), and locks it there the same way (0.25). ``--hold`` turns it into the green but keeps
holding it instead of locking it (0.5). ``--slip`` lets go for the fresh grip with nothing holding the rod, so it
unwinds (0). ``--keep-hold`` locks it in the green but keeps the right hand on the rod (0.5).
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, turn, wait  # noqa: E402

AXIS = np.array([g.LEG_X, g.BAND_Y, g.TOP + g.ROD_Z])   # the rod's middle: it turns and tips about here
GRIP_R = 0.050            # take the rod 5 cm from its middle (the fingers are 6.8 cm wide along it)
ABOVE = 0.005             # the tool point this far above the rod's axis: the fingertips stay clear of the band
STEP = math.radians(2.0)  # how far the hand's angle advances each move
LEAD_MAX = math.radians(10.0)  # and how far it may run ahead of the rod's measured angle
EARLY_AT = g.PHI                           # 24, turning clockwise: the other end lines up with the knee-side hook
REGRASP_AT = math.radians(320.0)           # the right hand's wrist is near its limit here: it takes a fresh grip
LOCK_AT = 2 * math.pi - g.PHI              # 336: the other end lines up with the hip-side hook, gauge in the green
LEAN1 = 0.0               # the first grip, straight down
LEAN2 = 0.0               # and the second
HOLD_LEAN = -45.0         # the holding hand leans along the rod, which keeps the two arms apart
LOCK_ROLL = math.radians(12.0)   # while locking, the hand rolls this far about the rod's axis
DEBUG = bool(__import__("os").environ.get("TQ_DEBUG"))


class Rod:
    """The rod as the cameras see it: its direction (followed through every turn) and its tilt, plus the gauge's
    reading from its needle."""

    def __init__(self, ctx):
        self.ctx = ctx
        self.theta = g.THETA0
        self.last = g.THETA0

    def read(self):
        j = self.ctx.states()["tourniquet"].joints
        a = g.THETA0 + j["twist"]
        self.theta += math.remainder(a - self.last, 2 * math.pi)
        self.last = a
        return self.theta, j["press"]

    def gauge(self) -> float:
        needle = -self.ctx.states()["tourniquet"].joints["needle"]
        return (needle + g.NEEDLE_SWEEP / 2) / g.NEEDLE_SWEEP * g.GAUGE_MAX


def _tilted(theta: float, tilt: float) -> np.ndarray:
    """The rotation of the rod (its +x end along ``theta``) tipped ``tilt`` about its middle (+x end down)."""
    return turn([-math.sin(theta), math.cos(theta), 0.0], tilt)


def end(theta: float, s: int, tilt: float = 0.0, roll: float = 0.0) -> np.ndarray:
    """The tool point holding the rod's ``s`` end, the rod tipped ``tilt`` and the hand rolled ``roll`` about the
    rod's own axis."""
    d = np.array([math.cos(theta), math.sin(theta), 0.0])
    return AXIS + _tilted(theta, tilt) @ (s * GRIP_R * d + turn(d, roll) @ [0.0, 0.0, ABOVE])


def grip(theta: float, lean_deg: float = 0.0, branch: float = 0.0) -> np.ndarray:
    """Straight down with the fingers closing across the rod (its +x end along ``theta``), leaned ``lean_deg`` about
    the fingers' closing line, so both fingers stay level either side of the rod. ``branch`` (0 or pi) picks which way
    round the fingers straddle it."""
    from yam_arena.demo import grip_rotation

    return turn([-math.sin(theta), math.cos(theta), 0.0], math.radians(lean_deg)) @ grip_rotation(theta + branch)


def take(arm, p, rot):
    """Come down on the rod along the grip's approach, and close."""
    a = rot[:, 2]
    yield from arm.move_joints(p - a * 0.06, rot, 1.6, grip=1.0)
    yield from arm.move(p, rot, 0.8)
    yield from arm.set_grip(0.0, 0.5)


def sweep(rod, arm, s, goal, lean, branch=0.0):
    """Turn the rod to ``goal`` (either way). The hand's angle advances STEP a move and runs up to LEAD_MAX ahead of
    the rod's measured angle: the wrist gives under the band's pull, so the rod trails the hand. It stops once the rod
    is within half a degree of the goal. ``branch`` (0 or pi) picks which way round the fingers straddle the rod."""
    theta, _ = rod.read()
    d = 1.0 if goal > theta else -1.0
    cmd = theta
    for _ in range(800):
        if (goal - theta) * d <= math.radians(0.5):
            return cmd
        cmd += d * STEP
        if (cmd - theta) * d > LEAD_MAX:
            cmd = theta + d * LEAD_MAX
        if (cmd - goal) * d > LEAD_MAX:
            cmd = goal + d * LEAD_MAX
        yield from arm.move(end(cmd, s), grip(cmd, lean, branch), 0.1)
        theta, _ = rod.read()
    return cmd


def align(rod, arm, s, goal, cmd, lean, branch=0.0):
    """Hold the rod on ``goal``: the wrist gives under the band's pull, and springs on when a sweep stops, so correct the
    hand's angle by most of the measured error until the rod is within a quarter of a degree. Returns the hand's
    angle."""
    for _ in range(40):
        theta, _ = rod.read()
        err = goal - theta
        if abs(err) <= math.radians(0.25):
            break
        cmd += 0.7 * err
        if DEBUG:
            print(f"align: rod {math.degrees(theta):7.2f} tilt {math.degrees(_):5.1f} hand {math.degrees(cmd):7.2f} "
                  f"ik {arm.worst_mm:5.1f}", flush=True)
        yield from arm.move(end(cmd, s), grip(cmd, lean, branch), 0.15)
        yield from wait(0.1)
    return cmd


def release(arm, up=0.05):
    """Open and lift clear, staying near (for a fresh grasp)."""
    yield from arm.set_grip(1.0, 0.4)
    yield from arm.move(arm.pos + [0.0, 0.0, up], None, 0.6)


def let_go(arm, up=0.06):
    yield from release(arm, up)
    yield from arm.go_home(1.6)


def lock(rod, arm, s, goal, cmd, lean, branch=0.0):
    """Lift the held (far) end, rolling the hand about the rod so one finger comes under it and tipping the hand with
    the rod about its middle, until the rod's measured tilt puts its
    other end down past the hook's lips onto its floor (the arm gives under the strap's spring, so the hand leads the
    rod); the lips then hold it down. All the while keep the rod on the hook's line (``goal``), correcting the hand's
    angle ``cmd`` from the rod's measured one; the hook's lead-in guides the end into its channel on the way down."""
    theta, tilt = rod.read()
    sign = 1.0 if math.cos(goal) < 0 else -1.0        # a positive tilt tips the +x end down: which way dips the near end
    lift = roll = 0.0
    for _ in range(80):
        theta, tilt = rod.read()
        if sign * tilt >= g.LOCK_TILT + math.radians(0.3):
            break
        cmd += 0.5 * (goal - theta)
        roll = min(roll + math.radians(3.0), LOCK_ROLL)  # one finger comes under the rod: it lifts it, not friction
        if DEBUG:
            print(f"lock: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} hand {math.degrees(cmd):7.2f} "
                  f"lift {math.degrees(lift):5.1f} ik {arm.worst_mm:5.1f}", flush=True)
        lift = max(lift, min(lift + math.radians(1.0), sign * tilt + math.radians(14.0), g.SEAT_TILT + math.radians(12.0)))
        d = [math.cos(cmd), math.sin(cmd), 0.0]
        yield from arm.move(end(cmd, s, sign * lift, roll),
                            _tilted(cmd, sign * lift) @ turn(d, roll) @ grip(cmd, lean, branch), 0.15)
    yield from wait(0.3)


def phase(rod, R, L, label):
    if DEBUG:
        theta, tilt = rod.read()
        print(f"phase {label}: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} ik R {R.worst_mm:5.1f} "
              f"L {L.worst_mm:5.1f}", flush=True)


def script(ctx, mode="full"):
    R, L = ctx.right, ctx.left
    rod = Rod(ctx)
    theta, _ = rod.read()
    if mode == "early":
        # the right hand takes the same end the other way round and turns the rod clockwise, 102 degrees, to where the
        # other end lines up with the knee-side hook (gauge about 160), and locks it there as below
        yield from take(R, end(theta, 1), grip(theta, LEAN1, 0.0))
        cmd = yield from sweep(rod, R, 1, EARLY_AT, LEAN1, 0.0)
        cmd = yield from align(rod, R, 1, EARLY_AT, cmd, LEAN1, 0.0)
        yield from lock(rod, R, 1, EARLY_AT, cmd, LEAN1, 0.0)
        yield from let_go(R)
        yield from wait(2.0)
        return
    # grasp 1: the right hand on the rod's +x end (toward the arms and the hip), fingers straddling it the "far" way
    # round (branch pi)
    yield from take(R, end(theta, 1), grip(theta, LEAN1, math.pi))
    phase(rod, R, L, "took")
    yield from sweep(rod, R, 1, REGRASP_AT, LEAN1, math.pi)
    phase(rod, R, L, "sweep1")
    if mode == "slip":                                      # lets go for a fresh grip with nothing holding the rod
        yield from let_go(R)
        yield from wait(4.0)
        return
    # the left hand holds the other end still while the right takes a fresh grip, the other way round
    theta, _ = rod.read()
    yield from take(L, end(theta, -1), grip(theta, HOLD_LEAN))
    phase(rod, R, L, "left holds")
    yield from release(R)
    yield from R.go_home(1.6)                               # the wrist turns half round: start from the ready pose
    phase(rod, R, L, "right home")
    theta, _ = rod.read()
    yield from take(R, end(theta, 1), grip(theta, LEAN2))
    phase(rod, R, L, "right retook")
    yield from let_go(L)
    phase(rod, R, L, "left gone")
    # grasp 2: on until the other end lines up with the hip-side hook, the gauge in the green
    cmd = yield from sweep(rod, R, 1, LOCK_AT, LEAN2, 0.0)
    cmd = yield from align(rod, R, 1, LOCK_AT, cmd, LEAN2, 0.0)
    if rod.gauge() < g.GREEN:
        print(f"gauge {rod.gauge():.0f} mmHg at the lock angle: not green", flush=True)
    if mode == "hold":
        yield from wait(4.0)
        return
    yield from lock(rod, R, 1, LOCK_AT, cmd, LEAN2, 0.0)
    if mode == "keep":
        yield from wait(4.0)
        return
    yield from let_go(R)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"twist {S['twist']:7.1f}  {S['pressure']:5.1f} mmHg  aligned {S['aligned']}  dip {math.degrees(S['dip']):5.1f} deg  "
            f"locked {S['locked']}  arms {S['clearance'] * 100:5.1f} cm off  body {S['body_gap'] * 1000:5.1f} mm  "
            f"clear steps {S['clear_steps']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--early-lock", action="store_true", help="lock at the first chance, below the green")
    ap.add_argument("--hold", action="store_true", help="turn into the green but keep holding instead of locking")
    ap.add_argument("--slip", action="store_true", help="let go before the other hand has the rod")
    ap.add_argument("--keep-hold", action="store_true", help="lock in the green but keep a hand on the rod")
    a = ap.parse_args()
    mode = ("early" if a.early_lock else "hold" if a.hold else "slip" if a.slip else "keep" if a.keep_hold
            else "full")
    run(a.task, a.out, lambda ctx: script(ctx, mode), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=20)
