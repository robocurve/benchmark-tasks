"""Limit joint speed through time parameterization. Free-space startup uses joint interpolation."""

from dataclasses import dataclass
from pathlib import Path
import math, numpy as np
from native.fixture import BASES


@dataclass
class Segment:
    start: float
    end: float
    q0: np.ndarray
    q1: np.ndarray
    pos0: np.ndarray
    pos1: np.ndarray
    label: str


def virtual(side, xyz):
    return (np.asarray(xyz) - BASES[side]) * [-1.0, -1.0, 1.0]


def orientation():
    from yam_arena.urdf import rpy_matrix
    from yam_arena.demo import grip_rotation
    return rpy_matrix(np.array((0.,math.radians(-20.),0.))) @ grip_rotation(0.)


def solve(pos,q0):
    from yam_arena import ik
    q,dp,dr = ik.solve(pos,ik.matrix_to_quat_xyzw(orientation()),q_init=q0,
                       iters=90,damping=.015,rot_weight=.22,tol=.00015)
    if dp>.0025 or dr>math.radians(2):
        raise ValueError(f"Unreachable TCP {pos}, error {dp} m, {dr} rad")
    return q


class Plan:
    def __init__(self, s):
        from yam_arena.constants import READY

        self.side = s
        self.duration = 0.0
        self.segments = []
        self.q = np.r_[READY, 1.0]

    def add(self, w):
        from yam_arena import ik
        from yam_arena.constants import FINGER_OPEN

        p0 = ik.tcp_pose(self.q[:6])[0]
        p1 = virtual(self.side, w.pos)
        count = (
            1
            if not self.segments
            else max(1, math.ceil(np.linalg.norm(p1 - p0) / 0.035))
        )
        f0 = self.q[-1]
        for i in range(1, count + 1):
            p = p0 + (p1 - p0) * i / count
            q = np.r_[
                solve(p, self.q[:6]), f0 + (w.finger / FINGER_OPEN - f0) * i / count
            ]
            delta = np.abs(q - self.q)
            dt = max(
                w.duration / count,
                0.05,
                1.51 * max(delta[:6]) / (0.45 if self.duration < 7 else 0.70),
                1.51 * delta[-1],
            )
            self.segments.append(
                Segment(
                    self.duration,
                    self.duration + dt,
                    self.q,
                    q,
                    ik.tcp_pose(self.q[:6])[0],
                    p,
                    w.label,
                )
            )
            self.q = q
            self.duration += dt

    def wait(self, t):
        if t <= self.duration:
            return
        from yam_arena import ik

        p = ik.tcp_pose(self.q[:6])[0]
        self.segments.append(
            Segment(self.duration, t, self.q, self.q, p, p, "wait for props")
        )
        self.duration = t

    def action7(self, t):
        from yam_arena import ik

        for b in self.segments:
            if t < b.end:
                u = ik.smoothstep((t - b.start) / (b.end - b.start))
                return b.q0 + u * (b.q1 - b.q0), b.label
        return self.q.copy(), "done"


def compile_plans(paths):
    plans = {s: Plan(s) for s in BASES}
    for s in ("left", "right"):
        for w in paths[s]:
            if w.label == "above valve":
                plans[s].wait(max(p.duration for p in plans.values()))
            plans[s].add(w)
    if max(p.duration for p in plans.values()) + 7.1 > 60:
        raise ValueError("Plan exceeds action ceiling")
    return plans


class ScriptClock:
    """Advance the scripted teacher only after measured grip and seating events."""

    def __init__(self, plans, mode):
        self.plans = plans
        self.mode = mode
        self.time = 0.
        self.fault = None

    def advance(self, qualification, dt):
        candidate = self.time + dt
        label = self.plans['right'].action7(candidate)[1]
        if self.mode in ('reference','alternate','late_clear') and label == 'turn valve' and not qualification.worker_clear:
            return self.time
        self.time = candidate
        return self.time


def install_task20_guard(entry):
    try:
        from yam_arena.scripted import YamScriptedPolicy
    except ModuleNotFoundError:
        import yam_arena

        if not hasattr(yam_arena, "__path__"):
            return
        raise
    if getattr(YamScriptedPolicy, "_cart_guard_entry", None) == entry:
        return
    original = YamScriptedPolicy.reset
    orig_act = getattr(YamScriptedPolicy, "act", None)

    def reset(policy, scene):
        if policy._task is None or Path(policy._task.path).resolve() != entry:
            return original(policy, scene)
        from native.cart_motion import demo

        policy._plans = compile_plans(demo())
        policy._end = max(p.duration for p in policy._plans.values())
        import os

        policy._cart_clock = ScriptClock(
            policy._plans, os.environ.get("TASK20_CONTROL", "reference")
        )

    def act(policy, obs):
        if policy._task is None or Path(policy._task.path).resolve() != entry:
            return orig_act(policy, obs)
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack, STATE_KEY

        clock = policy._cart_clock
        monitor = policy._task.module.monitor
        t = clock.advance(monitor.qualifier, policy._dt)
        if clock.fault:
            print("Script stopped on measured state: " + clock.fault, flush=True)
            cmd = obs.state[STATE_KEY]
        else:
            cmd = pack(*(policy._plans[s].action7(t)[0] for s in ("left", "right")))
        stop = bool(clock.fault or t > policy._end + policy.hold_s)
        return ActionChunk(
            actions=[Action(data=cmd, meta={"request_stop": True} if stop else {})],
            control_hz=1 / policy._dt,
        )

    YamScriptedPolicy.act = act
    YamScriptedPolicy.reset = reset
    YamScriptedPolicy._cart_guard_entry = entry
