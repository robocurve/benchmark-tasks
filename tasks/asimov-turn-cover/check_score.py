"""The scorer on hand-built trials, with no simulator: python3 check_score.py.

Loads task.py with stand-ins for the harness's modules, then feeds its real Trial step by step with made-up states (the swivel's angle and the
fingertips) for the honest turn and each lazy or faulty route, and checks each score against the scoring text. Nothing here runs a simulation.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
import os
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
os.environ["ASIMOV_TRACE"] = "0"

# -- stand-ins for the harness, enough for task.py to load ------------------------------------------------------------------------------------------
ya = types.ModuleType("yam_arena")
tf = types.ModuleType("yam_arena.task_file")


@dataclasses.dataclass(frozen=True)
class Box:
    name: str
    pos: tuple = (0.0, 0.0, 0.0)
    size: tuple = (0.1, 0.1, 0.1)
    mass: float | None = None
    yaw_deg: float = 0.0
    start_quat: tuple | None = None
    solver_iters: int = 32


@dataclasses.dataclass(frozen=True)
class ObjectState:
    pos: np.ndarray
    quat_xyzw: np.ndarray
    lin_vel: np.ndarray
    ang_vel: np.ndarray
    low: np.ndarray
    high: np.ndarray


tf.Box, tf.ObjectState, tf.TABLE_TOP = Box, ObjectState, {"station": 0.7475}
ya.task_file = tf
sys.modules["yam_arena"], sys.modules["yam_arena.task_file"] = ya, tf
dk = types.ModuleType("demokit")
dk.TABLE_TOP, dk.DT, dk.CONTROL_HZ = 0.7475, 0.05, 20
sys.modules["demokit"] = dk
sys.path.insert(0, str(HERE))
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

spec = importlib.util.spec_from_file_location("task", HERE / "task.py")
T = importlib.util.module_from_spec(spec)
spec.loader.exec_module(T)
DT = T.DT
FAR = gk.tips(gk.hand(gk.PARKED["left"], opening=0.0), gk.hand(gk.PARKED["right"], opening=0.0))


def on_arm(ang, off=0.002):
    """All 24 fingertip points `off` behind the rear face of the body's left upper arm, with the swivel at `ang` (the catch)."""
    q = np.array([0.049 + off, -0.16, -0.10])
    c, s = math.cos(ang), math.sin(ang)
    p = np.array([T.HOOK[0] + c * q[0] - s * q[1], T.HOOK[1] + s * q[0] + c * q[1], T.COM_Z + q[2]])
    return np.repeat(p[None, :], 24, axis=0)


def at_gap(ang, gap):
    """Fingertip points `gap` straight out from the arm's rear face (along its normal)."""
    return on_arm(ang, off=gap)


class Joint:
    def __init__(self, a):
        self.a = a

    def joint(self, name):
        return self.a


def run(states) -> float:
    """states: a list of (swivel angle in radians, fingertips (24, 3))."""
    T.TRIAL.reset()
    cur = {}
    m.state = lambda env, name: Joint(cur["ang"])
    m.fingertips = lambda env: cur["tips"]
    for ang, tips in states:
        cur["ang"], cur["tips"] = ang, tips
        T.TRIAL.step(None)
    return T.score(None)


def trial(stop_at=math.radians(177.7), rate=math.radians(36.0), catch_steps=8, hold_steps=60, clear=True, catch_gap=0.002, overshoot=0.0,
          direction=1.0, extra_turn=0.0, free_stop=False, hard=False, end_rate=0.0, hand_on=False):
    """The honest turn: still 20 steps, a push (fingertips on the body) to `rate`, a free spin to the catch, slowed to rest at `stop_at` over
    `catch_steps` with a fingertip on the arm (`hard`: in 2 steps), optionally past the mark by `overshoot` and back, then the hand away
    (`clear`) or left on (`hand_on`), held `hold_steps`."""
    out, ang = [], 0.0
    for _ in range(20):
        out.append((ang, FAR))
    for k in range(8):  # the push: fingertips on the body as it speeds up
        ang += direction * rate * (k + 1) / 8 * DT
        out.append((ang, on_arm(ang)))
    target = direction * (stop_at + extra_turn + overshoot)
    while abs(ang) < abs(target) - rate * DT * catch_steps / 2:
        ang += direction * rate * DT
        out.append((ang, FAR))
    n = 0 if hard else catch_steps  # hard: stopped dead in one step, 36 deg/s to 0 (720 deg/s^2)
    left = target - ang
    for k in range(n):  # slowing evenly to rest at the target, a fingertip on the arm (or nothing, for a collision)
        ang += left * (2 * (n - k) - 1) / (n * n)
        out.append((ang, FAR if free_stop else on_arm(ang, catch_gap)))
    if overshoot:
        back, start = direction * (stop_at + extra_turn), ang
        for k in range(40):  # pushed back to the mark by the hand, easing in and out over 2 s
            u = (k + 1) / 40
            ang = start + (back - start) * (1 - math.cos(math.pi * u)) / 2
            out.append((ang, on_arm(ang)))
    for k in range(hold_steps):
        if end_rate:
            ang += direction * end_rate * DT
        tips = on_arm(ang, catch_gap) if hand_on else (at_gap(ang, min(0.12, 0.01 + 0.01 * k)) if clear else on_arm(ang, 0.02))
        out.append((ang, tips))
    return out


def coast(speeds, release_at=None, steps=800):
    """Pushes with no catch: each push (fingertips on the body for 6 steps) adds up to the next speed in `speeds`; the swivel's drag then slows it
    as the scene's does, viscously (time constant 20 s), hands away, to `steps`. `release_at`: (angle, rate): the body instead caught by a hand
    that slows it evenly to `rate` at `angle` and lets go there."""
    out, ang, rate = [(0.0, FAR)] * 20, 0.0, 0.0
    decay = math.exp(-DT / T.DRAG_TAU)
    for v in speeds:
        for k in range(6):
            rate = rate + (v - rate) / (6 - k)
            ang += rate * DT
            out.append((ang, on_arm(ang)))
        for _ in range(60):
            rate *= decay
            ang += rate * DT
            out.append((ang, FAR))
    if release_at is not None:
        a_rel, r_rel = release_at
        while ang < a_rel - 0.3:
            ang += rate * DT
            out.append((ang, FAR))
        n = 8
        for k in range(n):  # a hand slows it to r_rel at a_rel
            rate = rate + (r_rel - rate) / (n - k)
            ang += rate * DT
            out.append((ang, on_arm(ang)))
    while len(out) < steps:
        rate *= decay
        ang += rate * DT
        out.append((ang, FAR))
    return out

def split(rate=math.radians(36.0), frac=0.5, stop_at=math.radians(177.0)):
    """The honest push and spin, then stopped dead by a hand on the arm with the stop falling `frac` of the way through a step."""
    out, ang = [(0.0, FAR)] * 20, 0.0
    for k in range(8):
        ang += rate * (k + 1) / 8 * DT
        out.append((ang, on_arm(ang)))
    while ang < stop_at - rate * DT:
        ang += rate * DT
        out.append((ang, FAR))
    ang += rate * DT * frac  # the stop inside this step
    out.append((ang, on_arm(ang)))
    for k in range(60):
        out.append((ang, at_gap(ang, min(0.12, 0.01 + 0.01 * k))))
    return out


def slow(rate=math.radians(10.0)):
    """Turned slowly by a hand on the arm all the way (10 deg/s), stopped on the mark by it, then the hand away."""
    out, ang = [(0.0, FAR)] * 20, 0.0
    while ang < math.radians(178.0):
        ang += rate * DT
        out.append((ang, on_arm(ang)))
    for k in range(60):
        out.append((ang, at_gap(ang, min(0.12, 0.01 + 0.01 * k))))
    return out


def early_brake(then=math.radians(8.0)):
    """Pushed to 20 deg/s, braked by a hand to rest at 14 deg, then turned by the hand at `then` to the mark and stopped there, hands away."""
    out, ang, rate = [(0.0, FAR)] * 20, 0.0, math.radians(20.0)
    for k in range(8):
        ang += rate * (k + 1) / 8 * DT
        out.append((ang, on_arm(ang)))
    left = math.radians(14.0) - ang
    for k in range(8):
        ang += left * (2 * (8 - k) - 1) / 64
        out.append((ang, on_arm(ang)))
    for _ in range(20):
        out.append((ang, on_arm(ang)))
    while ang < math.radians(178.0):
        ang += then * DT
        out.append((ang, on_arm(ang)))
    for k in range(60):
        out.append((ang, at_gap(ang, min(0.12, 0.01 + 0.01 * k))))
    return out


def bounce(rebound, frac=0.5, rate=math.radians(36.0), block=math.radians(185.0), hands=1):
    """Spun at `rate` into a rigid block at `block` (a hand held still there), meeting it `frac` of the way through a step and bouncing back at
    `rebound`; the rebound then caught gently on the mark, hands away. `hands` 2: two hands held rigid 1 deg either side of the mark, the body
    rattling between them (restitution rebound / rate) until it rests."""
    out, ang = [(0.0, FAR)] * 20, 0.0
    for k in range(8):
        ang += rate * (k + 1) / 8 * DT
        out.append((ang, on_arm(ang)))
    walls = (block,) if hands == 1 else (math.pi - math.radians(1.0), math.pi + math.radians(1.0))
    hi = max(walls)
    while ang < hi - rate * DT:
        ang += rate * DT
        out.append((ang, FAR))
    v, e = rate, rebound / rate
    for _ in range(200):  # each step: travel, and bounce off whichever wall it meets
        nxt = ang + v * DT
        if v > 0 and nxt >= hi:
            t_in = (hi - ang) / (v * DT)
            v = -e * v
            nxt = hi + v * DT * (1 - t_in)
        elif hands == 2 and v < 0 and nxt <= min(walls):
            t_in = (ang - min(walls)) / (-v * DT)
            v = -e * v
            nxt = min(walls) + v * DT * (1 - t_in)
        ang = nxt
        out.append((ang, on_arm(ang)))
        if hands == 1 and v < 0:
            break
        if abs(v) < math.radians(1.0):
            break
    if hands == 1:  # the rebound caught gently on the mark
        target = math.radians(178.0)
        n = max(2, int(abs(target - ang) / (abs(v) * DT) * 2))
        left = target - ang
        for k in range(n):
            ang += left * (2 * (n - k) - 1) / (n * n)
            out.append((ang, on_arm(ang)))
    for k in range(60):
        out.append((ang, at_gap(ang, min(0.12, 0.01 + 0.01 * k))))
    return out


CASES = {
    "the honest turn: pushed to 36 deg/s, caught on the arm, at rest 2.3 deg short for 1 s": (trial(), 1.0),
    "the same turned the other way round": (trial(direction=-1.0), 1.0),
    "the same, a hand left on the body (2 mm off it)": (trial(hand_on=True), 1.0),
    "the same, a hand 2 cm off the body at the end": (trial(clear=False), 1.0),
    "at rest 8 deg short of the mark": (trial(stop_at=math.radians(172.0)), 1.0),
    "at rest 12 deg short of the mark": (trial(stop_at=math.radians(168.0)), 0.25),
    "past the mark by 20 deg, then brought back to it": (trial(overshoot=math.radians(20.0)), 1.0),
    "past the mark by 40 deg, then brought back to it": (trial(overshoot=math.radians(40.0)), 1.0),
    "one and a half turns (540 degrees net), at rest facing away": (trial(extra_turn=math.radians(360.0)), 0.25),
    "stopped at 120 deg, held": (trial(stop_at=math.radians(120.0)), 0.25),
    "stopped at 60 deg, held": (trial(stop_at=math.radians(60.0)), 0.0),
    "stopped by a collision (no fingertip on it) at the mark": (trial(free_stop=True), 0.0),
    "stopped dead by a hand in one step (36 deg/s to 0 within 0.05 s)": (trial(hard=True), 0.0),
    "still turning at 25 deg/s at the end": (trial(end_rate=math.radians(25.0), hold_steps=10), 0.0),
    "on the mark only 0.9 s at the end": (trial(hold_steps=18), 0.25),
    "nothing done": ([(0.0, FAR)] * 60, 0.0),
    "one push to 24.7 deg/s, no catch: it coasts on past the mark": (coast([math.radians(24.7)]), 0.25),
    "one push to 10.5 deg/s, no catch: it creeps through the mark and on": (coast([math.radians(10.5)]), 0.25),
    "one push to 11.5 deg/s, no catch: it creeps through the mark and on": (coast([math.radians(11.5)]), 0.25),
    "caught and let go on the mark still turning at 2.9 deg/s: it drifts on past": (coast([math.radians(36.0)], release_at=(math.radians(178.0), math.radians(2.9))), 0.25),
    "two nudges (18, then 16 deg/s), no catch, at rest short of the mark": (coast([math.radians(18.0), math.radians(16.0)]), 0.25),
    "stopped dead by a hand from 36 deg/s, the stop 30 % into a step": (split(frac=0.3), 0.0),
    "stopped dead by a hand from 36 deg/s, the stop 50 % into a step": (split(frac=0.5), 0.0),
    "stopped dead by a hand from 36 deg/s, the stop 70 % into a step": (split(frac=0.7), 0.0),
    "stopped dead by a hand from 43 deg/s, the stop 50 % into a step": (split(rate=math.radians(43.0)), 0.0),
    "turned slowly by hand at 10 deg/s and stopped on the mark": (slow(), 1.0),
    "the honest turn, caught with the palm (fingertips 3 cm off the arm)": (trial(catch_gap=0.03), 1.0),
    "braked to rest at 14 deg, then turned by hand at 8 deg/s to the mark": (early_brake(), 1.0),
    "braked to rest at 14 deg, then turned by hand at 4.9 deg/s to the mark": (early_brake(math.radians(4.9)), 1.0),
    "a rigid block at 185 deg from 36 deg/s, a 15 deg/s rebound caught on the mark (block 30 % into a step)": (bounce(math.radians(15.0), 0.3), 0.0),
    "the same, the block 50 % into a step": (bounce(math.radians(15.0), 0.5), 0.0),
    "the same, the block 80 % into a step": (bounce(math.radians(15.0), 0.8), 0.0),
    "the same, the block at the end of a step": (bounce(math.radians(15.0), 1.0), 0.0),
    "a rigid block with a 30 deg/s rebound": (bounce(math.radians(30.0)), 0.0),
    "an elastic reversal, 36 to -36 deg/s": (bounce(math.radians(36.0)), 0.0),
    "two hands held rigid 1 deg either side of the mark, the body rattling between them (restitution 0.5)": (bounce(math.radians(18.0), hands=2), 0.0),
}
bad = 0
for name, (states, want) in CASES.items():
    got = run(states)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (text: {want})")
print("all as the scoring text says" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
