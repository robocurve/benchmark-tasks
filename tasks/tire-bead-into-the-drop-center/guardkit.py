"""guardkit: shared scoring guards, trimmed to the helpers this task uses. Import after yamkit:
    import guardkit as gk

Parts are shapes in an object's own frame (origin at objects[obj].pos, turning with its quat), so a check
follows the object however it lies.

    gk.Seg: A capsule from `a` to `b` (object frame, m) with radius `r`: a handle, neck, rim or rod. a == b is a
        ball.
    gk.Ring: A solid ring about the object's z axis (object frame, m): radii `r_in`..`r_out`, heights `z0`..`z1`,
        centered on `center`. r_in = 0 gives a solid cylinder (a tart, a disk, a piston).
    gk.local: World point `p` in object `obj`'s own frame.
    gk.touching: Grippers in contact with `obj`: their own shape (palm and both fingers, opened as they are now)
        touches or overlaps one of `parts` (within `tol`, 2 mm by default). A hand hovering more than `tol` from a
        part is not touching it.
    gk._PerStep: Resets when kit.episode() changes; _tick() is True the first time it is called in a control step
        (keyed on yamkit's step index in the sim, offline on the objects dict).
    gk.Latch: An irreversible event: once `cond` holds for `steps` control steps in a row (default 2: a one-step
        physics pop does not count, a quick flick undone at once does), it stays on for the rest of the trial.
        total=True counts the steps in all instead of in a row (swapping hands every 2 steps, or toggling, can't
        dodge it).
    gk.Sweep: Between-step contact for fast motions: a tool point moving 3 cm in one 0.05 s step can jump over a
        thin target. crossed() tests the points between where the point was last step and where it is now. With
        `obj`, both are taken in that object's frame (motion relative to a moving object; `test` then gets object-
        frame points). A jump longer than `max_jump` (a reset, a teleport, a physics pop) tests only the current
        point.
"""

import math
from dataclasses import dataclass
import numpy as np
import yamkit as kit


GRIPPERS = ("left_gripper", "right_gripper")


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


# The YAM gripper's own shape in its tool-point frame (x: the fingers' closing axis, z: the approach, toward the pad
# tips), as boxes fitted to the collision meshes the simulator uses (yam.urdf collision/gripper, tip_left, tip_right):
# the palm, and for each finger its body, its pad and its narrower tip. `o` is each finger's inner face's distance
# from the tool point: 0 closed, 47 mm fully open (the pads open 94 mm apart).
PALM = Block((0.0, 0.0, -0.1075), (0.102, 0.065, 0.074))


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
