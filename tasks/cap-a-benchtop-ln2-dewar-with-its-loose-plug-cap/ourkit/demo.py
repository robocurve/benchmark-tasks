"""ourkit.demo: closed-loop scripted demos on yam_arena's Waypoint lists, and cheat modes for tests.

The yam_scripted policy (GPU) and physcheck play each arm's waypoints with ``ArmPlan``, which reads a waypoint's
position only once the arm is moving toward it. A position here can be :class:`Live`: worked out from the object
states the kit keeps (``scene.LATEST``, updated after every control step) when the move starts ("once"), or re-read
every control step while the move runs and then frozen (``track=True``, a servo). A scripted demo may read the
simulator's state to plan; nothing is moved by script, every motion is the arm's.

    p = Plan("right")
    grasp(p, lambda st: st["box"].point("handle"), width=0.025, fallback=(0.30, -0.10, TOP + 0.05))
    ease(p, lambda st: st["box"].to_world((0, 0.10, 0)), speed=0.01, fallback=(0.30, 0.00, TOP + 0.05))
    p.release().home()
    return choose(demo=lambda: together(p, idle_plan("left", p.duration)), none=idle)

Cheat modes: ``KIT_DEMO=<mode>`` in the environment picks one of :func:`choose`'s builders (default "demo"); the
standard cheats are ``none`` (both arms stay home: tests that nothing happens without a command), ``slam`` (the
crux done fast or forced) and ``wrong_order``. physcheck passes the environment through, so

    KIT_DEMO=none .planning/pipeline/pc_pool.sh python3 .planning/pipeline/physcheck.py <task> --out <dir>

scores the cheat with the real scorer.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

from . import scene, tiltgrip

sys.modules.setdefault("tiltgrip", tiltgrip)  # primitives.py imports it by that name
from . import primitives as P  # noqa: E402
from .tiltgrip import Grip  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

HZ = 20.0
MODES = ("demo", "none", "slam", "wrong_order")


def mode() -> str:
    """The demo mode for this run: ``KIT_DEMO`` (default "demo")."""
    return (os.environ.get("KIT_DEMO") or "demo").strip()


def started() -> bool:
    return bool(scene.LATEST["steps"]) and scene.LATEST["states"] is not None


def now() -> float:
    """The policy's clock for the action being worked out now (s since the reset)."""
    return (scene.LATEST["steps"] + 1) / HZ


def _xyz(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in np.asarray(v, dtype=float).reshape(3))


class Live:
    """A waypoint position given by ``fn(states)``. Before the run has stepped it is ``fallback`` (planning and
    offline checks see the planned path). Once the run steps it is worked out when the arm starts toward it and kept,
    or, with ``track``, re-read every control step until its move ends (``Plan`` sets ``t1``), then kept."""

    def __init__(self, fn, fallback, track: bool = False):
        self.fn, self.track = fn, track
        self.fallback = fallback
        self.t1: float | None = None
        self.value, self.seen, self.trial = None, -1, None

    def _fallback(self):
        return _xyz(self.fallback() if callable(self.fallback) else self.fallback)

    def resolve(self) -> tuple[float, float, float]:
        if not started():
            return self._fallback()
        if self.trial != scene.LATEST.get("resets"):  # a new trial: forget the last one's value
            self.value, self.seen, self.trial = None, -1, scene.LATEST.get("resets")
        k = scene.LATEST["steps"]
        done = self.t1 is not None and now() > self.t1 + 1e-9
        if self.value is not None and (not self.track or k == self.seen or done):
            return self.value
        self.value, self.seen = _xyz(self.fn(scene.LATEST["states"])), k
        return self.value

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.resolve(), dtype=dtype or float)

    def __iter__(self):
        return iter(self.resolve())

    def __len__(self):
        return 3

    def __getitem__(self, i):
        return self.resolve()[i]


def live(fn, fallback, track: bool = False) -> Live:
    return Live(fn, fallback, track)


class Plan:
    """One arm's waypoints, built in order. Every method returns the plan, so calls chain. ``memo`` is a dict the
    live functions may share (a command worked out in one move and reused in the next)."""

    def __init__(self, arm: str):
        self.arm = arm
        self.ws: list[Waypoint] = []
        self.memo: dict = {}

    @property
    def duration(self) -> float:
        return float(sum(w.duration for w in self.ws))

    @property
    def last(self) -> Waypoint:
        if self.ws:
            return self.ws[-1]
        pos, g = P.home(self.arm)
        return Waypoint(0.0, pos, FINGER_OPEN, "start", g)

    def add(self, ws) -> Plan:
        """Append waypoints (a primitive's output, say); Live positions learn when their move ends."""
        for w in ws:
            t = self.duration + w.duration
            if isinstance(w.pos, Live):
                w.pos.t1 = t
            self.ws.append(w)
        return self

    def move(self, seconds: float, pos, *, finger=None, label: str = "move", grip=None, track: bool = False,
             fallback=None) -> Plan:
        """Go to ``pos`` (a point, or ``fn(states)`` with ``fallback``) in ``seconds``."""
        w = self.last
        if callable(pos):
            pos = Live(pos, fallback if fallback is not None else w.pos, track)
        return self.add([Waypoint(float(seconds), pos, w.finger if finger is None else finger, label,
                                  w.yaw if grip is None else grip)])

    def hold(self, seconds: float, label: str = "hold still") -> Plan:
        w = self.last
        return self.add([Waypoint(float(seconds), w.pos, w.finger, label, w.yaw)])

    def fingers(self, gap: float, seconds: float = 0.8, label: str = "fingers") -> Plan:
        """Open or close to ``gap`` m between the pads where the hand is."""
        w = self.last
        return self.add([Waypoint(float(seconds), w.pos, P.finger_for_gap(gap), label, w.yaw)])

    def release(self, extra: float = 2 * P.OPEN_CLEARANCE, seconds: float = P.T_RELEASE) -> Plan:
        """Open the fingers ``extra`` m wider than they hold now."""
        return self.fingers(P.gap_for_finger(self.last.finger) + extra, seconds, "let go")

    def back(self, dist: float = P.APPROACH, seconds: float | None = None, label: str = "back away") -> Plan:
        """Draw back ``dist`` m along the grip's approach axis from where the hand was sent last (live, if it was)."""
        w = self.last
        a = P.approach_axis(w.yaw)
        prev = w.pos
        target = Live(lambda st: np.asarray(prev, dtype=float) - dist * a, lambda: np.asarray(prev, dtype=float) - dist * a)
        return self.add([Waypoint(seconds or P._t(dist, P.T_LIFT), target, w.finger, label, w.yaw)])

    def home(self, seconds: float = P.HOME_S, hold: float = P.FINAL_HOLD_S, finger: float = FINGER_OPEN) -> Plan:
        """Back to the start pose and hold still (the score is read on a settled scene; the video ends on it)."""
        pos, g = P.home(self.arm)
        return self.add([Waypoint(seconds, pos, finger, "back to the start pose", g),
                         Waypoint(hold, pos, finger, "hold still", g)])

    def waypoints(self) -> list[Waypoint]:
        return list(self.ws)


def together(*plans, label: str = "hold still") -> dict:
    """{arm: waypoints} with every arm padded to end together."""
    out = {p.arm: p.waypoints() or Plan(p.arm).hold(0.1).waypoints() for p in plans}
    return P.sync(out, label)


def idle_plan(arm: str, seconds: float) -> Plan:
    """An arm that stays in its start pose for ``seconds``."""
    return Plan(arm).hold(seconds, "stay home")


def idle(seconds: float = 12.0) -> dict:
    """The ``none`` cheat: both arms stay home."""
    return together(idle_plan("left", seconds), idle_plan("right", seconds))


def choose(**builders) -> dict:
    """The plan for this run's mode: ``choose(demo=..., none=..., slam=...)``, each a function returning a plan dict.
    ``none`` defaults to :func:`idle`."""
    builders.setdefault("none", idle)
    m = mode()
    if m not in builders:
        raise ValueError(f"KIT_DEMO={m!r}: this task's demo modes are {sorted(builders)}")
    return builders[m]()


# -- moves on top of primitives.py, re-anchored to live object poses ---------------------------------------------------


def anchored(ws, ref_fallback, ref_fn, track: bool = False) -> list[Waypoint]:
    """A primitive's waypoints, planned round ``ref_fallback``, moved with a live reference point: each waypoint's
    position becomes ``ref_fn(states) + (its planned position - ref_fallback)``, read when that move starts (or every
    step with ``track``). Without ``ref_fn`` the waypoints are returned unchanged."""
    if ref_fn is None:
        return list(ws)
    ref0 = np.asarray(ref_fallback, dtype=float)
    out = []
    for w in ws:
        off = np.asarray(w.pos, dtype=float) - ref0
        fn = (lambda st, off=off: np.asarray(ref_fn(st), dtype=float) + off)
        out.append(Waypoint(w.duration, Live(fn, tuple(np.asarray(w.pos, dtype=float)), track), w.finger, w.label,
                            w.yaw))
    return out


def _split(where, fallback):
    """(planning point, live function or None) from a point or a function of the states."""
    if callable(where):
        if fallback is None:
            raise ValueError("a live target needs a fallback point for planning")
        return np.asarray(fallback, dtype=float), where
    return np.asarray(where, dtype=float), None


def grasp(plan: Plan, where, width: float, *, height: float | None = None, grip=None, fallback=None,
          lift: float | None = P.APPROACH, track: bool = False, **kw) -> Plan:
    """Grasp a rigid part at ``where`` (a point or ``fn(states)``): straight down (``grasp_down_rigid``, needs
    ``height``) or tilted (``grasp_tilted`` when ``grip`` is a tilted Grip), each waypoint re-aimed at the live part."""
    c, fn = _split(where, fallback)
    if grip is not None and P.tilt_deg(grip) > 1.0:
        ws = P.grasp_tilted(c, width, grip=grip, retreat=lift, **kw)
    else:
        if height is None:
            raise ValueError("a straight-down grasp needs the part's height")
        ws = P.grasp_down_rigid(c, width, height, grip=grip or 0.0, lift=lift, **kw)
    return plan.add(anchored(ws, c, fn, track))


def press(plan: Plan, where, force_n: float, *, fallback=None, **kw) -> Plan:
    """Press the fingertips onto ``where`` with about ``force_n`` (``press_with_overshoot``), aimed at the live point."""
    c, fn = _split(where, fallback)
    return plan.add(anchored(P.press_with_overshoot(c, force_n, **kw), c, fn))


def push(plan: Plan, where, direction, travel: float, *, surface_z: float, force_n: float = 0.0, fallback=None,
         **kw) -> Plan:
    """Push a slider ``travel`` m along ``direction`` (``push_slider``), aimed at the live face point."""
    c, fn = _split(where, fallback)
    return plan.add(anchored(P.push_slider(c, direction, travel, surface_z=surface_z, force_n=force_n, **kw), c, fn))


def ease(plan: Plan, where, speed: float, *, fallback=None, label: str = "ease", min_s: float = 1.0,
         track: bool = False) -> Plan:
    """Move to ``where`` slowly: the move takes the planned distance / ``speed`` (m/s; the smooth profile peaks at
    1.5 times it), at least ``min_s``."""
    c, fn = _split(where, fallback)
    dist = float(np.linalg.norm(c - np.asarray(plan.last.pos, dtype=float)))
    return plan.move(max(min_s, dist / speed), fn or tuple(c), label=label, track=track, fallback=tuple(c))


def follow(plan: Plan, fn, seconds: float, *, fallback, label: str = "follow") -> Plan:
    """A servo move: the target is re-read from the states every control step for ``seconds``."""
    return plan.move(seconds, fn, label=label, track=True, fallback=fallback)


def corrected(plan: Plan, seconds: float, want, have, *, key: str, base=None, max_fix: float = 0.040,
              label: str = "correct", fallback=None) -> Plan:
    """Send the hand where the part will be ``want(states)``, given that it is ``have(states)`` now while the hand
    was sent to ``plan.memo[base]`` (default: the last stored command): command + (want - have), each axis clipped to
    ``max_fix``. The result is stored as ``plan.memo[key]``. This is how a compliant arm's lag under load is taken out
    (the shade demo: 30 mm under a 6 N pull)."""
    w = plan.last
    fb = fallback if fallback is not None else w.pos

    def fn(st):
        cmd = np.asarray(plan.memo.get(base or "_last", fb), dtype=float)
        fix = np.clip(np.asarray(want(st), dtype=float) - np.asarray(have(st), dtype=float), -max_fix, max_fix)
        plan.memo[key] = plan.memo["_last"] = cmd + fix
        return plan.memo[key]

    return plan.move(seconds, fn, label=label, fallback=fb)


def remember(plan: Plan, key: str, fn):
    """Wrap a live function so its result is also stored as ``plan.memo[key]`` (and as the last command)."""

    def keep(st):
        v = np.asarray(fn(st), dtype=float)
        plan.memo[key] = plan.memo["_last"] = v
        return v

    return keep


def gap_finger(gap: float) -> float:
    return P.finger_for_gap(gap)


__all__ = ["Plan", "Live", "live", "together", "idle", "idle_plan", "choose", "mode", "now", "anchored", "grasp",
           "press", "push", "ease", "follow", "corrected", "remember", "gap_finger", "Grip", "Waypoint", "P",
           "FINGER_OPEN", "FINGER_CLOSED", "MODES", "math"]
