"""guardkit: shared scoring guards for the holes reviewers kept finding (Nov 11). Import after yamkit:
    import guardkit as gk

Parts are shapes in an object's own frame (origin at objects[obj].pos, turning with its quat), so a check follows the
object however it lies: gk.Seg(a, b, r) is a capsule (a handle, neck or rod of radius r; a == b gives a ball),
gk.Block(center, size) a box and gk.Ring(r_in, r_out, z0, z1) a ring or solid cylinder about the object's z axis. With parts=None a check uses the object's world box (low/high), which over-covers a
tilted object (strict for release checks, loose for grip checks: pass parts for those).

Stateless checks (call any time; they read only this step's objects and kit.opening):
    gk.holding(o, "scoop", PARTS)            -> grippers with the tool point within 4 cm of ANY listed part and the
                                                fingers closed on something (not wide open, not an empty fist)
    gk.grip_on(o, "jug", HANDLE, gk.band(0.025))  -> grippers whose opening is stopped by that part (a real grip)
    gk.released_clear(o, "jug", PARTS, 0.01) -> True when every tool point and pad tip is 1 cm clear of the outline
    gk.touching(o, "tart", PARTS)            -> grippers whose own shape (palm and both fingers, at their present
                                                opening) touches or overlaps a part (within CONTACT, 2 mm): contact,
                                                not nearness. Use this, not a distance zone, for "a hand on X".
    gk.hand_gap(o, "left_gripper", "tart", PARTS) -> the gap (m) from that gripper's own shape to the parts
Per-trial memory (reset when kit.episode() changes; updated once per control step even though success() and score()
both call them; pass a fresh objects dict per step in offline tests):
    FLIP = gk.Latch(steps=2)                 -> FLIP.feed(o, cond): an irreversible event, 2 steps in a row confirm it
    ONE = gk.Latch(steps=3, total=True)      -> counts steps in all: swapping hands every 2 steps doesn't dodge it
    HOME = gk.ArmsHome()                     -> HOME.feed(o) every step; HOME.home(o): back at the start pose, still
                                                (reviewers penalize scoring arm-home: do not use it in a score)
    SW = gk.Sweep()                          -> SW.crossed(o, "knife", tip, test, obj="board"): tests the points
                                                between last step's and this step's tip (fast motions)
Typical end-of-run rule: full credit only if no gripper is gk.touching(...) the object and not FAIL.on.
Reviewers reject proximity-as-violation ("within 4 cm scores zero even if it is not touching"): judge a hand on an
object by touching() or a real grip (grip_on), never by a distance zone alone.
Ship only the helpers a task uses: trim the task's copy of this file.
Scored geometry is the thing a real gauge reads (the pipe's own slope, the deck's center): put that in the task.
"""

import math
from dataclasses import dataclass

import numpy as np

import yamkit as kit

GRIPPERS = ("left_gripper", "right_gripper")
NEAR = 0.04  # m: tool point within 4 cm of a part = a hand still on it (the tool point sits between the pads)
HELD_OPEN = (0.02, 0.85)  # opening (0 closed .. 1 open) of fingers closed on something: an empty fist reads 0
GRIP_PAD = 0.01  # m: for grip_on, the tool point within 1 cm of the gripped part
BAND = 0.15  # band(): fingers may squeeze up to 15 % of full opening narrower than the part ...
OVER = 0.03  # ... and be at most 3 % wider (an open hand hovering around it is not a grip)


# -- geometry ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Seg:
    """A capsule from `a` to `b` (object frame, m) with radius `r`: a handle, neck, rim or rod. a == b is a ball."""

    a: tuple
    b: tuple
    r: float = 0.0


@dataclass(frozen=True)
class Ring:
    """A solid ring about the object's z axis (object frame, m): radii `r_in`..`r_out`, heights `z0`..`z1`, centered on
    `center`. r_in = 0 gives a solid cylinder (a tart, a disk, a piston)."""

    r_in: float
    r_out: float
    z0: float
    z1: float
    center: tuple = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Block:
    """A box centered at `center` with full `size` (object frame, m)."""

    center: tuple
    size: tuple


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def local(objects, obj: str, p) -> np.ndarray:
    """World point `p` in object `obj`'s own frame."""
    s = objects[obj]
    return _rot(s.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(s.pos, float))


def _dist_local(part, p) -> float:
    if isinstance(part, Seg):
        a, b = np.asarray(part.a, float), np.asarray(part.b, float)
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-18 else float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        return max(0.0, float(np.linalg.norm(p - (a + t * ab))) - part.r)
    d = np.abs(p - np.asarray(part.center, float)) - np.asarray(part.size, float) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def _proj_local(part, q: np.ndarray) -> np.ndarray:
    """The points of `part` nearest to object-frame points q (n x 3)."""
    if isinstance(part, Block):
        c, h = np.asarray(part.center, float), np.asarray(part.size, float) / 2
        return c + np.clip(q - c, -h, h)
    if isinstance(part, Ring):
        c = np.asarray(part.center, float)
        d = q - c
        rho = np.hypot(d[:, 0], d[:, 1])
        safe = np.where(rho > 1e-12, rho, 1.0)
        ux, uy = np.where(rho > 1e-12, d[:, 0] / safe, 1.0), np.where(rho > 1e-12, d[:, 1] / safe, 0.0)
        r = np.clip(rho, part.r_in, part.r_out)
        return c + np.stack([r * ux, r * uy, np.clip(d[:, 2], part.z0, part.z1)], axis=1)
    a, b = np.asarray(part.a, float), np.asarray(part.b, float)
    ab = b - a
    L2 = float(ab @ ab)
    t = np.zeros(len(q)) if L2 < 1e-18 else np.clip((q - a) @ ab / L2, 0.0, 1.0)
    x = a + t[:, None] * ab
    v = q - x
    n = np.linalg.norm(v, axis=1)
    return x + v * np.minimum(1.0, part.r / np.maximum(n, 1e-12))[:, None]


def _dist_local_any(part, p) -> float:
    if isinstance(part, Ring):
        return float(np.linalg.norm(p - _proj_local(part, p[None, :])[0]))
    return _dist_local(part, p)


def dist(objects, obj: str, p, parts=None) -> float:
    """Distance (m) from world point `p` to the nearest of `parts` of `obj` (0 inside); parts=None: the object's
    world box (low/high)."""
    if parts is None:
        s = objects[obj]
        d = np.maximum(np.asarray(s.low, float) - p, np.asarray(p, float) - np.asarray(s.high, float))
        return float(np.linalg.norm(np.maximum(d, 0.0)))
    q = local(objects, obj, p)
    return min(_dist_local_any(part, q) for part in (parts if isinstance(parts, (list, tuple)) else (parts,)))


def tool_points(objects, g: str) -> tuple:
    """A gripper's tool point (between the pads) and its pad tips' point."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    s = objects[g]
    tcp = np.asarray(s.pos, float)
    return tcp, tcp + _rot(s.quat_xyzw) @ np.array([0.0, 0.0, PAD_TIP_PAST_TCP])


def band(width: float, squeeze: float = BAND, over: float = OVER) -> tuple:
    """The opening band of fingers closed on a part `width` m across: from the width's opening minus `squeeze`
    (never below half of it, so an empty fist next to a thin part is not a grip) to plus `over`."""
    from yam_arena.constants import FINGER_OPEN

    want = width / (2 * abs(FINGER_OPEN))
    return want - min(squeeze, want / 2), want + over


# The YAM gripper's own shape in its tool-point frame (x: the fingers' closing axis, z: the approach, toward the pad
# tips), as boxes fitted to the collision meshes the simulator uses (yam.urdf collision/gripper, tip_left, tip_right):
# the palm, and for each finger its body, its pad and its narrower tip. `o` is each finger's inner face's distance
# from the tool point: 0 closed, 47 mm fully open (the pads open 94 mm apart).
PALM = Block((0.0, 0.0, -0.1075), (0.102, 0.065, 0.074))
FAR = 0.02  # m: hand_gap computes gaps up to this exactly
CONTACT = 0.002  # m: touching: the shapes within 2 mm (the solver's resting gap plus the box fit to the meshes)


def hand_boxes(o: float) -> list:
    out = [PALM]
    for s in (-1.0, 1.0):
        out += [Block((s * (o + 0.012), 0.0, -0.051), (0.024, 0.068, 0.060)),  # finger body, 6 cm long
                Block((s * (o + 0.005), 0.0, -0.012), (0.010, 0.040, 0.018)),  # pad, 1 cm thick, 4 cm wide
                Block((s * (o + 0.0032), 0.0, 0.0049), (0.0064, 0.0264, 0.0158))]  # tip, to 12.8 mm past the TCP
    return out


def _box_gap(cg, Rg, box, co, Ro, part, iters: int = 40) -> float:
    """Gap (m, 0 if they overlap) between a gripper box (gripper frame cg, Rg) and an object part (object frame co,
    Ro), by alternating projection from the box's center and 8 corners (exact for convex parts; the corners keep a
    ring's hollow from trapping it)."""
    c, h = np.asarray(box.center, float), np.asarray(box.size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)] + [[0, 0, 0]], float)
    p = cg + (c + corners * h) @ Rg.T  # world points in the gripper box
    q = p
    for _ in range(iters):
        q = co + _proj_local(part, (p - co) @ Ro) @ Ro.T  # nearest on the part
        p_new = cg + (c + np.clip((q - cg) @ Rg - c, -h, h)) @ Rg.T  # nearest on the box
        if np.max(np.abs(p_new - p)) < 1e-6:
            p = p_new
            break
        p = p_new
    return float(np.min(np.linalg.norm(p - q, axis=1)))


def _hand_frames(objects, g: str, obj: str):
    from yam_arena.constants import FINGER_OPEN

    op = kit.opening(objects, g)
    o = abs(FINGER_OPEN) * (1.0 if not np.isfinite(op) else float(np.clip(op, 0.0, 1.0)))
    s, so = objects[g], objects[obj]
    return (o, np.asarray(s.pos, float), _rot(s.quat_xyzw), np.asarray(so.pos, float), _rot(so.quat_xyzw))


def hand_gap(objects, g: str, obj: str, parts, upto: float = FAR) -> float:
    """Gap (m) between gripper g's own shape (palm and fingers at their present opening) and the nearest of `parts`.
    Gaps over `upto` are only a lower bound; the search stops at the first box found within `upto` if upto is small."""
    o, cg, Rg, co, Ro = _hand_frames(objects, g, obj)
    best = math.inf
    for part in (parts if isinstance(parts, (list, tuple)) else (parts,)):
        for box in hand_boxes(o):
            ctr = cg + Rg @ np.asarray(box.center, float)
            reach = float(np.linalg.norm(np.asarray(box.size, float))) / 2
            low = _dist_local_any(part, Ro.T @ (ctr - co)) - reach  # the box can come no closer than this
            if low >= best:
                continue
            best = min(best, low if low > upto else _box_gap(cg, Rg, box, co, Ro, part))
    return best


def touching(objects, obj: str, parts, tol: float = CONTACT, grippers=GRIPPERS) -> tuple:
    """Grippers in contact with `obj`: their own shape (palm and both fingers, opened as they are now) touches or
    overlaps one of `parts` (within `tol`, 2 mm by default). A hand hovering more than `tol` from a part is not
    touching it."""
    out = []
    for g in grippers:
        if g not in objects:
            continue
        o, cg, Rg, co, Ro = _hand_frames(objects, g, obj)
        hit = False
        for part in (parts if isinstance(parts, (list, tuple)) else (parts,)):
            for box in hand_boxes(o):
                ctr = cg + Rg @ np.asarray(box.center, float)
                reach = float(np.linalg.norm(np.asarray(box.size, float))) / 2
                if _dist_local_any(part, Ro.T @ (ctr - co)) - reach > tol:
                    continue
                if _box_gap(cg, Rg, box, co, Ro, part) <= tol:
                    hit = True
                    break
            if hit:
                break
        if hit:
            out.append(g)
    return tuple(out)


# -- stateless checks --------------------------------------------------------------------------------------------------


def holding(objects, obj: str, parts=None, near: float = NEAR, open_band: tuple = HELD_OPEN, grippers=GRIPPERS) -> tuple:
    """Grippers holding `obj`: the tool point within `near` of ANY of `parts` (not just the handle's center: a hand
    on the neck or bowl holds it too) and the fingers closed on something (opening inside `open_band`). Falsy if none."""
    return tuple(g for g in grippers if g in objects and open_band[0] <= kit.opening(objects, g) <= open_band[1]
                 and dist(objects, obj, objects[g].pos, parts) < near)


def grip_on(objects, obj: str, part, open_band: tuple, pad: float = GRIP_PAD, grippers=GRIPPERS) -> tuple:
    """Grippers with a real grip on `part` of `obj`: the opening is stopped by the part (inside `open_band`, see
    band(); a closed fist reads 0, an open hand ~1) and the tool point within `pad` of it. Falsy if none."""
    return tuple(g for g in grippers if g in objects and open_band[0] <= kit.opening(objects, g) <= open_band[1]
                 and dist(objects, obj, objects[g].pos, part) <= pad)


def released_clear(objects, obj: str, parts=None, margin: float = 0.01, open_ok: float | None = None,
                   grippers=GRIPPERS) -> bool:
    """Let go: every gripper's tool point and pad tip at least `margin` clear of `obj`'s outline (its parts, or its
    world box). With `open_ok` (e.g. 0.85) a gripper open wider than that is not counted (a wide-open hand resting
    against the object); by default even an open hand must be clear."""
    for g in grippers:
        if g not in objects or (open_ok is not None and kit.opening(objects, g) > open_ok):
            continue
        if any(dist(objects, obj, p, parts) < margin for p in tool_points(objects, g)):
            return False
    return True


# -- per-trial memory --------------------------------------------------------------------------------------------------


class _PerStep:
    """Resets when kit.episode() changes; _tick() is True the first time it is called in a control step (keyed on
    yamkit's step index in the sim, offline on the objects dict)."""

    def __init__(self):
        self._ep, self._key, self._objs, self.step = None, None, None, 0
        self._fresh()

    def _fresh(self):
        pass

    def _tick(self, objects) -> bool:
        ep, n = kit.episode(), kit._EPISODE.get("last")
        if ep != self._ep:
            self._ep, self._key, self._objs, self.step = ep, None, None, 0
            self._fresh()
        key = (ep, n) if n is not None else None
        if (key is not None and key == self._key) or (key is None and objects is self._objs):
            return False
        self._key, self._objs = key, objects
        self.step += 1
        return True


class Latch(_PerStep):
    """An irreversible event: once `cond` holds for `steps` control steps in a row (default 2: a one-step physics pop
    does not count, a quick flick undone at once does), it stays on for the rest of the trial. total=True counts the
    steps in all instead of in a row (swapping hands every 2 steps, or toggling, can't dodge it)."""

    def __init__(self, steps: int = 2, total: bool = False):
        self.steps, self.total = steps, total
        super().__init__()

    def _fresh(self):
        self.run, self.count, self.on, self.when = 0, 0, False, None

    def feed(self, objects, cond: bool) -> bool:
        """Feed this step's condition (once per step; repeats within a step are ignored). Returns whether latched."""
        if self._tick(objects):
            self.run = self.run + 1 if cond else 0
            self.count += bool(cond)
            if not self.on and (self.count if self.total else self.run) >= self.steps:
                self.on, self.when = True, self.step
        return self.on


class ArmsHome(_PerStep):
    """Both arms back where they started: each gripper's tool point within `tol` of its pose at the trial's first
    step and moving less than `still` m per step for `hold` steps in a row (so an arm passing through home or still
    retreating does not count)."""

    def __init__(self, tol: float = 0.03, still: float = 0.002, hold: int = 1, grippers=GRIPPERS):
        self.tol, self.still, self.hold, self.grippers = tol, still, hold, grippers
        super().__init__()

    def _fresh(self):
        self.start, self.last, self.still_run = None, None, 0

    def feed(self, objects) -> None:
        if not self._tick(objects):
            return
        now = {g: np.asarray(objects[g].pos, float) for g in self.grippers if g in objects}
        if self.start is None and len(now) == len(self.grippers):
            self.start = now
        moved = self.last is None or any(g not in self.last or np.linalg.norm(p - self.last[g]) >= self.still
                                         for g, p in now.items())
        self.still_run = 0 if moved else self.still_run + 1
        self.last = now

    def home(self, objects) -> bool:
        self.feed(objects)
        return (self.start is not None and self.still_run >= self.hold
                and all(g in objects and np.linalg.norm(np.asarray(objects[g].pos, float) - self.start[g]) < self.tol
                        for g in self.grippers))


def swept(p0, p1, test, step: float = 0.005) -> bool:
    """True if test(point) holds anywhere on the segment p0 -> p1, checked every `step` m (both ends included)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(1, int(math.ceil(np.linalg.norm(p1 - p0) / step)))
    return any(test(p0 + (p1 - p0) * k / n) for k in range(n + 1))


class Sweep(_PerStep):
    """Between-step contact for fast motions: a tool point moving 3 cm in one 0.05 s step can jump over a thin target.
    crossed() tests the points between where the point was last step and where it is now. With `obj`, both are taken
    in that object's frame (motion relative to a moving object; `test` then gets object-frame points). A jump longer
    than `max_jump` (a reset, a teleport, a physics pop) tests only the current point."""

    def __init__(self, step: float = 0.005, max_jump: float = 0.05):
        self.res, self.max_jump = step, max_jump
        super().__init__()

    def _fresh(self):
        self.prev, self.cur, self.hit = {}, {}, {}

    def crossed(self, objects, key: str, point, test, obj: str | None = None) -> bool:
        if self._tick(objects):
            self.prev, self.hit = self.cur, {}
            self.cur = {}
        if key in self.hit:
            return self.hit[key]
        p = local(objects, obj, point) if obj is not None else np.asarray(point, float)
        self.cur[key] = p
        q = self.prev.get(key)
        if q is None or np.linalg.norm(p - q) > self.max_jump:
            q = p
        self.hit[key] = swept(q, p, test, self.res)
        return self.hit[key]
