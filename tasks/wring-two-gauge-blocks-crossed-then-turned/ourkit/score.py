"""ourkit.score: scorer helpers. A task's ``score()`` is a :func:`ladder` over a few flags that its ``on_step`` hook keeps:

    WALL = Sticky()                     # a latched zero: moved off its marks, ever
    PULLED = Ever()                     # a milestone reached at some time
    END = Settle(3.0)                   # the end state held still, hands clear, for 3 s

    def score(states):
        return 0.0 if WALL else ladder((0.25, PULLED and below), (0.5, END.ok), (1.0, at_line))

Every threshold is a number a tape measure or a video checks; ``Settle`` gives "still for N s" and "hands clear by
X mm" from the grippers' collision hulls (read from the arm joints, in Isaac and in physcheck).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from . import scene

HZ = 20.0


class Sticky:
    """A latched zero: once tripped it stays tripped until the next reset (``reset()``). ``bool(x)`` is tripped."""

    def __init__(self):
        self.reason, self.t = None, None

    def trip(self, reason: str, t: float | None = None) -> None:
        if self.reason is None:
            self.reason, self.t = reason, t

    def reset(self) -> None:
        self.reason, self.t = None, None

    def __bool__(self) -> bool:
        return self.reason is not None

    def __repr__(self) -> str:
        return f"Sticky({self.reason!r} at {self.t})" if self else "Sticky(clear)"


class Ever:
    """A milestone: true from the first step its condition held until the next reset."""

    def __init__(self):
        self.t = None

    def update(self, cond: bool, t: float | None = None) -> bool:
        if cond and self.t is None:
            self.t = t if t is not None else 0.0
        return bool(self)

    def reset(self) -> None:
        self.t = None

    def __bool__(self) -> bool:
        return self.t is not None


class Settle:
    """The last ``seconds`` of the trial: each step ``push(values, ok)`` with the values that must stay still (m or rad,
    any number) and whether the step was otherwise fine (hands clear). ``ok`` is true once the window is full, every
    step in it was fine and no value changed by more than ``tol``."""

    def __init__(self, seconds: float, tol: float = 0.002, hz: float = HZ):
        self.n = int(round(seconds * hz)) + 1
        self.tol = tol
        self.rows: deque = deque(maxlen=self.n)

    def push(self, values, ok: bool = True) -> None:
        self.rows.append((np.atleast_1d(np.asarray(values, dtype=float)), bool(ok)))

    def reset(self) -> None:
        self.rows.clear()

    @property
    def spread(self) -> float:
        if not self.rows:
            return math.inf
        v = np.array([r[0] for r in self.rows])
        return float((v.max(axis=0) - v.min(axis=0)).max())

    @property
    def ok(self) -> bool:
        return len(self.rows) >= self.n and all(r[1] for r in self.rows) and self.spread < self.tol


def ladder(*rungs) -> float:
    """``ladder((0.25, a), (0.5, b), (1.0, c))``: the value of the highest rung whose condition holds along with every
    rung below it; 0 if the first fails. Rungs go up in value. Partial credit never skips a rung."""
    out = 0.0
    for value, cond in rungs:
        if not cond:
            break
        out = float(value)
    return out


def within(value: float, target: float, band: float) -> bool:
    """|value - target| <= band, with a 1e-9 allowance so a number on the band edge counts as stated."""
    return abs(float(value) - float(target)) <= float(band) + 1e-9


# -- the grippers ------------------------------------------------------------------------------------------------------

GRIPPER_LINKS = ["gripper", "tip_left", "tip_right"]


def gripper_hulls(q6, finger: float, base) -> list:
    """One gripper's collision hulls in the world for arm joints ``q6`` and finger joint ``finger`` (m)."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    q = {n: float(v) for n, v in zip(ARM_JOINTS, q6, strict=True)} | {j: float(finger) for j in FINGER_JOINTS}
    return analysis.world_hulls(analysis.fk_link_poses_world(q, base), GRIPPER_LINKS)


def grippers() -> list:
    """Both grippers' hulls now, from the arms' joints (object states don't include the arms); [] outside a sim."""
    e = scene.env()
    if e is None:
        return []
    from yam_arena import station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    out = []
    for arm in ("left", "right"):
        robot = e.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        joints = scene._np(robot.data.joint_pos)[0]
        finger = float(np.mean([joints[names.index(j)] for j in FINGER_JOINTS]))
        out.append(gripper_hulls(joints[[names.index(j) for j in ARM_JOINTS]], finger, station.BASES[arm]))
    return out


def clearance(points, grips, beyond: float = math.inf) -> float:
    """Metres from the nearest of ``points`` to the nearest gripper hull (negative inside). Hulls whose bounding box
    is ``beyond`` or more from the points' are skipped (they can't be nearer)."""
    points = np.atleast_2d(np.asarray(points, dtype=float))
    best = math.inf
    lo, hi = points.min(axis=0), points.max(axis=0)
    for hulls in grips:
        for _, h in hulls:
            v = getattr(h, "vertices", None)
            if v is not None and np.maximum(v.min(axis=0) - hi, lo - v.max(axis=0)).max() >= beyond:
                best = min(best, beyond)
                continue
            best = min(best, float((points @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min()))
    return best


def hands_clear(points, mm: float, grips=None) -> bool:
    """No gripper within ``mm`` of any of ``points``. Outside a simulator (no grippers) it is true."""
    grips = grippers() if grips is None else grips
    return not grips or clearance(points, grips, beyond=mm / 1000.0) >= mm / 1000.0 - 1e-9


def surface_points(state, part: str, boxes, step: float = 0.02) -> np.ndarray:
    """Points over the faces of a part's boxes (``boxes`` in the part's frame), every ``step`` m, in the world at the
    state's joint positions: what "hands clear of the part" is measured against."""
    from .scene import corners

    r, t = state.spec.frames(state.q)[part]
    rot = state.rotation()
    out = []
    for b in boxes:  # a turned box is covered by its bounding box
        c = corners(b)
        lo, hi = c.min(axis=0), c.max(axis=0)
        axes = [np.linspace(lo[i], hi[i], max(2, int(math.ceil((hi[i] - lo[i]) / step)) + 1)) for i in range(3)]
        g = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
        face = g[np.any(np.isclose(g, lo) | np.isclose(g, hi), axis=1)]
        out.append(state.pos + (rot @ (t + (r @ face.T).T).T).T)
    return np.concatenate(out)
