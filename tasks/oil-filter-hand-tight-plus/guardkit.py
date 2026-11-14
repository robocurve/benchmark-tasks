"""guardkit: shared scoring guards, trimmed to the helpers this task uses. Import after yamkit:
    import guardkit as gk

Parts are shapes in an object's own frame (origin at objects[obj].pos, turning with its quat), so a check
follows the object however it lies.

    gk.Ring: A solid ring about the object's z axis (object frame, m): radii `r_in`..`r_out`, heights `z0`..`z1`,
        centered on `center`. r_in = 0 gives a solid cylinder (a tart, a disk, a piston).
    gk.touching: Grippers in contact with `obj`: their own shape (palm and both fingers, opened as they are now)
        touches or overlaps one of `parts` (within `tol`, 2 mm by default). A hand hovering more than `tol` from a
        part is not touching it.
"""

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
