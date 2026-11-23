"""Scripted full-score demo. The right arm takes the windlass rod from above by the end toward the arms and the hip,
and turns it anticlockwise (seen from above), against the band's pull (up to about 2 N m): its hand runs up to 40
degrees ahead of the rod's measured angle, the wrist giving and the fingers splaying round the rod, and is tipped to
keep the rod level. It passes the first two times the rod lines up with the clip's hooks (30 and 78 degrees in, gauge
about 90 and 140) and goes to about 175 degrees, where its wrist is near its limit. The left arm takes the other end,
leaning out along the rod, and pushes on so it holds the rod while the right lets go, returns to its ready pose and
takes the same end again, its wrist turned the other way round, and takes up the pull; the left lets go. The right
turns the rod on to 210 degrees, where the end it does not hold lines up with the clip's hip-side hook, the gauge in
the green, and holds it on the hook's line. The left takes that end beyond the hook and tips it down, the right tipping
its own end up with the rod, until the end goes down into the hook, past its lips (10 N) and onto its floor. Both let
go and back off. Every move follows the rod's measured angle and tilt; the script reads the gauge's needle, not the
band's internal state.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--hold] [--slip] [--keep-hold]

``--hold`` turns it into the green but keeps holding it instead of locking it (0.5). ``--slip`` lets go for the
fresh grip with nothing holding the rod, so it unwinds (0). ``--keep-hold`` locks it in the green but keeps the right
hand on the rod (0.5).
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
from demokit import run, together, turn, wait  # noqa: E402

AXIS = np.array([g.LEG_X, g.BAND_Y, g.TOP + g.ROD_Z])   # the rod's middle: it turns and tips about here
GRIP_R = 0.050            # take the rod 5 cm from its middle (the fingers are 6.8 cm wide along it)
ABOVE = 0.005             # the tool point this far above the rod's axis: the fingertips stay clear of the band
STEP = math.radians(2.0)  # how far the hand's angle advances each move
LEAD_MAX = math.radians(40.0)  # and how far it may run ahead of the rod's measured angle: against the band's pull
#                                # (up to 2 N m) the wrist gives and the fingers splay round the rod, so the rod trails
YAW_LEAD = math.radians(40.0)  # the hand's own turn leads by as much: the wrist's twist does most of the turning
#                                # (a sideways push alone swings the hand on its wrist)
HOLD_PUSH = math.radians(30.0)  # a hand taking over the rod is set this far ahead of it, so it takes up the pull
REGRASP_AT = math.radians(300.0)           # the right hand's wrist is near its limit here: it takes a fresh grip
LOCK_AT = 2 * math.pi - g.PHI              # 336: the other end lines up with the hip-side hook, gauge in the green
LEAN1 = 0.0               # the first grip, straight down
LEAN2 = 0.0               # and the second
HOLD_LEAN = -45.0         # the holding hand leans along the rod, which keeps the two arms apart
PRESS_R = 0.073           # the pushing hand takes the rod's end this far out, its fingertips clear of the hook (end 7.5)
PRESS_LEAN = HOLD_LEAN    # leaning out along the rod, as when it holds the rod for the regrip
PRESS_LEAD = 12.0         # degrees: the pushing hand tips at most this far ahead of the rod's measured tilt
WALLS = (math.radians(8.0), math.radians(17.0))   # tipped this far, the rod's end is coming down between the hook's
ON_LINE = math.radians(0.5)                         # walls, so it goes on only within this of the hook's line
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


def joints_dbg(ctx, arm) -> str:
    """Commanded and measured joint angles of one arm (degrees), for debugging."""
    from scenekit import arm_joints

    meas = arm_joints(ctx.emb._env, arm.side, [f"joint{i}" for i in range(1, 7)])
    return " ".join(f"{math.degrees(c):6.1f}/{math.degrees(m):6.1f}" for c, m in zip(arm.q, meas))


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


def yaw_of(cmd: float, theta: float) -> float:
    """The hand's turn for a position command ``cmd``: the rod's measured angle, at most YAW_LEAD toward ``cmd``."""
    return theta + max(-YAW_LEAD, min(YAW_LEAD, cmd - theta))


def held(arm, cmd: float, theta: float, s: int, lean: float, branch: float):
    """The hand's pose for a position command ``cmd`` with the rod measured at ``theta``: its turn follows ``yaw_of``,
    and the whole hand is tipped about the rod's middle by ``arm.lift`` so its end is held level (the wrist gives under
    the twist and would push the end down)."""
    yaw = yaw_of(cmd, theta)
    tilt = -s * getattr(arm, "lift", 0.0)              # a positive tilt tips the +x end down
    return end(cmd, s, tilt), _tilted(yaw, tilt) @ grip(yaw, lean, branch)


def level(arm, s: int, tilt: float) -> None:
    """Raise (or lower) the held end by half its measured droop each move (kept within 0 to 15 degrees)."""
    arm.lift = min(max(getattr(arm, "lift", 0.0) + 0.5 * s * tilt, 0.0), math.radians(15.0))


def take(arm, p, rot):
    """Come down on the rod along the grip's approach, and close."""
    arm.lift = 0.0
    a = rot[:, 2]
    yield from arm.move_joints(p - a * 0.06, rot, 1.6, grip=1.0)
    yield from arm.move(p, rot, 0.8)
    yield from arm.set_grip(0.0, 0.5)


def sweep(rod, arm, s, goal, lean, branch=0.0, cmd=None):
    """Turn the rod to ``goal`` (either way). The hand's angle advances STEP a move and runs up to LEAD_MAX ahead of
    the rod's measured angle: the wrist gives under the band's pull, so the rod trails the hand. It stops once the rod
    is within half a degree of the goal. ``branch`` (0 or pi) picks which way round the fingers straddle the rod."""
    theta, tilt = rod.read()
    d = 1.0 if goal > theta else -1.0
    cmd = theta if cmd is None else cmd
    for _ in range(800):
        if (goal - theta) * d <= math.radians(0.5):
            return cmd
        cmd += d * STEP
        if (cmd - theta) * d > LEAD_MAX:
            cmd = theta + d * LEAD_MAX
        if (cmd - goal) * d > LEAD_MAX:
            cmd = goal + d * LEAD_MAX
        level(arm, s, tilt)
        yield from arm.move(*held(arm, cmd, theta, s, lean, branch), 0.1)
        theta, tilt = rod.read()
        if DEBUG:
            print(f"sweep: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} hand {math.degrees(cmd):7.2f} "
                  f"ik {arm.worst_mm:5.1f} q {joints_dbg(rod.ctx, arm)}", flush=True)
    return cmd


def align(rod, arm, s, goal, cmd, lean, branch=0.0):
    """Hold the rod on ``goal``: the wrist gives under the band's pull, and springs on when a sweep stops, so correct the
    hand's angle by most of the measured error until the rod is within a quarter of a degree. Returns the hand's
    angle."""
    for _ in range(40):
        theta, tilt = rod.read()
        err = goal - theta
        if abs(err) <= math.radians(0.25):
            break
        cmd += 0.7 * err
        if DEBUG:
            print(f"align: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} hand {math.degrees(cmd):7.2f} "
                  f"ik {arm.worst_mm:5.1f}", flush=True)
        level(arm, s, tilt)
        yield from arm.move(*held(arm, cmd, theta, s, lean, branch), 0.15)
        yield from wait(0.1)
    return cmd


def release(arm, up=0.05):
    """Open and lift clear, staying near (for a fresh grasp)."""
    yield from arm.set_grip(1.0, 0.4)
    yield from arm.move(arm.pos + [0.0, 0.0, up], None, 0.6)


def let_go(arm, up=0.06):
    yield from release(arm, up)
    yield from arm.go_home(1.6)


def press(rod, arm, goal, flags):
    """Push the rod's end toward the arms down into the hook it lines up with (at ``goal``), past the hook's lips: take
    that end beyond the hook, leaning out along the rod as when holding it for the regrip (so the hands keep apart), and
    tip the hand down with the rod, leading its measured tilt, until the rod seats (its measured tilt reaches the hook's
    floor). Sets ``flags["seated"]``; ``flags["tilt"]`` tells the other hand how far to tip with the rod."""
    near = 1 if math.cos(goal) < 0 else -1               # the end toward the arms; a tilt of ``near * dip`` dips it

    def pose(theta, dip):
        R = _tilted(theta, near * dip)
        dvec = np.array([math.cos(theta), math.sin(theta), 0.0])
        return AXIS + R @ (near * PRESS_R * dvec + np.array([0.0, 0.0, ABOVE])), R @ grip(theta, PRESS_LEAN)

    theta, tilt = rod.read()
    dip = max(near * tilt, 0.0)
    yield from take(arm, *pose(theta, dip))
    for _ in range(160):
        theta, tilt = rod.read()
        if near * tilt >= g.LOCK_TILT + math.radians(0.3):
            break
        if DEBUG:
            print(f"press: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} cmd {math.degrees(dip):5.1f} "
                  f"ik {arm.worst_mm:5.1f}", flush=True)
        dip = min(dip + math.radians(1.0), near * tilt + math.radians(PRESS_LEAD), g.SEAT_TILT + math.radians(6.0))
        if WALLS[0] <= near * tilt <= WALLS[1] and abs(theta - goal) > ON_LINE:
            dip = min(dip, near * tilt + math.radians(1.0))   # its end is near the hook's walls but off the channel's
            #                                                   line: wait for the other hand to bring it back on
        flags["tilt"] = near * dip                       # the holding hand tips its end up with the rod
        yield from arm.move(*pose(theta, dip), 0.15)
    flags["seated"] = True
    yield from wait(0.3)


def hold_on(rod, arm, s, goal, cmd, lean, branch, flags):
    """Keep the rod on ``goal`` against the band (as ``align``), the hand tipping with the rod's measured tilt, until
    ``flags["seated"]``."""
    while not flags.get("seated"):
        theta, tilt = rod.read()
        cmd += 0.5 * (goal - theta)
        target = flags.get("tilt", 0.0)                  # tip with the rod as the other hand pushes its end down
        arm.lift = min(max(getattr(arm, "lift", 0.0) + 0.5 * s * (tilt - target), 0.0), math.radians(8.0))
        t = target - s * arm.lift
        yaw = yaw_of(cmd, theta)
        if DEBUG:
            print(f"hold: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} target {math.degrees(target):5.1f} "
                  f"hand {math.degrees(cmd):7.2f} lift {math.degrees(arm.lift):4.1f}", flush=True)
        tip = _tilted(theta, t)                          # tip the level pose about the rod's own tilt axis, so tipping
        yield from arm.move(AXIS + tip @ (end(cmd, s) - AXIS), tip @ grip(yaw, lean, branch), 0.15)   # doesn't turn it


def phase(rod, R, L, label):
    if DEBUG:
        theta, tilt = rod.read()
        print(f"phase {label}: rod {math.degrees(theta):7.2f} tilt {math.degrees(tilt):5.1f} ik R {R.worst_mm:5.1f} "
              f"L {L.worst_mm:5.1f}", flush=True)


def script(ctx, mode="full"):
    R, L = ctx.right, ctx.left
    rod = Rod(ctx)
    theta, _ = rod.read()
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
    yield from L.move(end(theta + HOLD_PUSH, -1), grip(theta + HOLD_PUSH, HOLD_LEAN), 0.6)   # turn it on, as the right does
    phase(rod, R, L, "left holds")
    yield from release(R)
    yield from R.go_home(1.6)                               # the wrist turns half round: start from the ready pose
    phase(rod, R, L, "right home")
    theta, _ = rod.read()
    yield from take(R, end(theta, 1), grip(theta, LEAN2))
    cmd = theta + HOLD_PUSH                                 # take up the band's pull before the left lets go
    yield from R.move(*held(R, cmd, theta, 1, LEAN2, 0.0), 0.6)
    phase(rod, R, L, "right retook")
    yield from let_go(L)
    phase(rod, R, L, "left gone")
    # grasp 2: on until the other end lines up with the hip-side hook, the gauge in the green
    cmd = yield from sweep(rod, R, 1, LOCK_AT, LEAN2, 0.0, cmd)
    cmd = yield from align(rod, R, 1, LOCK_AT, cmd, LEAN2, 0.0)
    if rod.gauge() < g.GREEN:
        print(f"gauge {rod.gauge():.0f} mmHg at the lock angle: not green", flush=True)
    if mode == "hold":
        yield from wait(4.0)
        return
    flags = {}           # the right keeps the rod on the hook's line while the left pushes its other end down into it
    yield from together(hold_on(rod, R, 1, LOCK_AT, cmd, LEAN2, 0.0, flags), press(rod, L, LOCK_AT, flags))
    phase(rod, R, L, "seated")
    if mode == "keep":
        yield from let_go(L)
        yield from wait(4.0)
        return
    yield from let_go(R)
    yield from let_go(L)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"twist {S['twist']:7.1f}  {S['pressure']:5.1f} mmHg  band {S['torque']:5.2f} N m  lips {S['lip']}  "
            f"aligned {S['aligned']}  dip {math.degrees(S['dip']):5.1f} deg  "
            f"locked {S['locked']}  arms {S['clearance'] * 100:5.1f} cm off  body {S['body_gap'] * 1000:5.1f} mm  "
            f"clear steps {S['clear_steps']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hold", action="store_true", help="turn into the green but keep holding instead of locking")
    ap.add_argument("--slip", action="store_true", help="let go before the other hand has the rod")
    ap.add_argument("--keep-hold", action="store_true", help="lock in the green but keep a hand on the rod")
    a = ap.parse_args()
    mode = "hold" if a.hold else "slip" if a.slip else "keep" if a.keep_hold else "full"
    run(a.task, a.out, lambda ctx: script(ctx, mode), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=20)
