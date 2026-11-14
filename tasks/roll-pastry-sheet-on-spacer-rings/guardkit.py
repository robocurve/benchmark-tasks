"""guardkit: shared scoring guards, trimmed to the helpers this task uses. Import after yamkit:
    import guardkit as gk

Parts are flat-ended cylinders (gk.Cyl) in an object's own frame (origin at objects[obj].pos, turning with its
quat), so a check follows the object however it lies.

    gk.Cyl: A solid cylinder with flat ends from `a` to `b` (object frame, m) and radius `r` (this task's copy adds
        it for the rolling pin's barrel, rings and handles).
    gk.touching: Grippers in contact with `obj`: their own shape (palm and both fingers, opened as they are now)
        touches or overlaps one of `parts` (within `tol`, 2 mm by default). A hand hovering more than `tol` from a
        part is not touching it.
"""

from dataclasses import dataclass
import numpy as np
import yamkit as kit


GRIPPERS = ("left_gripper", "right_gripper")


@dataclass(frozen=True)
class Cyl:
    """A solid cylinder with flat ends from `a` to `b` (object frame, m) and radius `r`: a barrel, a ring on it or a
    handle, where a capsule's rounded end would reach past the flat end face."""

    a: tuple
    b: tuple
    r: float


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


def _proj_local(part, q: np.ndarray) -> np.ndarray:
    """The points of cylinder `part` nearest to object-frame points q (n x 3)."""
    a, b = np.asarray(part.a, float), np.asarray(part.b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s_ax = d @ u
    w = d - s_ax[:, None] * u  # the radial part
    n = np.linalg.norm(w, axis=1)
    return a + np.clip(s_ax, 0.0, L)[:, None] * u + w * np.minimum(1.0, part.r / np.maximum(n, 1e-12))[:, None]


def _dist_local_any(part, p) -> float:
    return float(np.linalg.norm(p - _proj_local(part, p[None, :])[0]))


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


def _box_gap(cg, Rg, box, co, Ro, part, iters: int = 40, stop: float | None = None) -> float:
    """Gap (m, 0 if they overlap) between a gripper box (gripper frame cg, Rg) and an object part (object frame co,
    Ro), by alternating projection from the box's center and 8 corners (exact for convex parts; the corners keep a
    ring's hollow from trapping it). With `stop`, returns as soon as a pair of points is found within it (the gap
    only shrinks as the projection goes on, so the answer to "within stop?" is the same)."""
    c, h = np.asarray(box.center, float), np.asarray(box.size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)] + [[0, 0, 0]], float)
    p = cg + (c + corners * h) @ Rg.T  # world points in the gripper box
    q = p
    for _ in range(iters):
        q = co + _proj_local(part, (p - co) @ Ro) @ Ro.T  # nearest on the part
        p_new = cg + (c + np.clip((q - cg) @ Rg - c, -h, h)) @ Rg.T  # nearest on the box
        if stop is not None and np.min(np.linalg.norm(p_new - q, axis=1)) <= stop:
            return float(np.min(np.linalg.norm(p_new - q, axis=1)))
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
        key = (g, obj, parts, tol, o, cg.tobytes(), Rg.tobytes(), co.tobytes(), Ro.tobytes())
        if key in _TOUCH_MEMO:  # the same poses and opening asked again (success() and score() in one step)
            if _TOUCH_MEMO[key]:
                out.append(g)
            continue
        hit = False
        for part in (parts if isinstance(parts, (list, tuple)) else (parts,)):
            for box in hand_boxes(o):
                ctr = cg + Rg @ np.asarray(box.center, float)
                reach = float(np.linalg.norm(np.asarray(box.size, float))) / 2
                if _dist_local_any(part, Ro.T @ (ctr - co)) - reach > tol:
                    continue
                if _box_gap(cg, Rg, box, co, Ro, part, stop=tol) <= tol:
                    hit = True
                    break
            if hit:
                break
        if len(_TOUCH_MEMO) > 256:
            _TOUCH_MEMO.clear()
        _TOUCH_MEMO[key] = hit
        if hit:
            out.append(g)
    return tuple(out)


_TOUCH_MEMO = {}  # touching() results keyed on the exact poses and opening they were computed from
