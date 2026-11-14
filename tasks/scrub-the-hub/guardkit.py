"""guardkit (trimmed to what this task uses): a real grip, judged by contact between the gripper's own
shape and an object's parts.
Import after yamkit:
    import guardkit as gk

Parts are shapes in an object's own frame (origin at objects[obj].pos, turning with its quat), so a check follows the
object however it lies: gk.Block(center, size) is a box.

    gk.gripping(o, "jug", HANDLE, gk.band(0.025))
                                           -> grippers holding the part: both fingers touching it and the opening
                                              stopped by it (an empty closed hand, an open hand resting on it, or a
                                              hand touching it with one finger or its palm does not hold it)
"""

from dataclasses import dataclass

import numpy as np

import yamkit as kit

GRIPPERS = ("left_gripper", "right_gripper")
BAND = 0.15  # band(): fingers may squeeze up to 15 % of full opening narrower than the part ...
OVER = 0.03  # ... and be at most 3 % wider (an open hand hovering around it is not a grip)


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
    """The points of `part` nearest to object-frame points q (n x 3)."""
    if isinstance(part, Block):
        c, h = np.asarray(part.center, float), np.asarray(part.size, float) / 2
        return c + np.clip(q - c, -h, h)
    raise TypeError(part)


def _dist_local(part, p) -> float:
    """Distance (m) from object-frame point p to the part (0 inside)."""
    return float(np.linalg.norm(p - _proj_local(part, p[None, :])[0]))


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
CONTACT = 0.002  # m: touching: the shapes within 2 mm (the solver's resting gap plus the box fit to the meshes)


def hand_boxes(o: float) -> list:
    """[palm, then for the finger at -x and the one at +x: its body, pad and tip]."""
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


def _meets(frames, boxes, parts, tol: float) -> bool:
    """Any of the gripper's `boxes` within `tol` of any of `parts`."""
    _, cg, Rg, co, Ro = frames
    for part in (parts if isinstance(parts, (list, tuple)) else (parts,)):
        for box in boxes:
            ctr = cg + Rg @ np.asarray(box.center, float)
            reach = float(np.linalg.norm(np.asarray(box.size, float))) / 2
            if _dist_local(part, Ro.T @ (ctr - co)) - reach > tol:  # the box can come no closer than this
                continue
            if _box_gap(cg, Rg, box, co, Ro, part) <= tol:
                return True
    return False


def gripping(objects, obj: str, parts, open_band: tuple, tol: float = CONTACT, grippers=GRIPPERS) -> tuple:
    """Grippers holding `obj` by `parts`: both fingers touching them (each finger's body, pad or tip within `tol`) and
    the opening stopped by them (inside `open_band`, see band(); a closed fist reads 0, an open hand about 1). A hand
    touching the part with one finger or its palm, an open hand resting on it or an empty fist beside it does not
    hold it."""
    out = []
    for g in grippers:
        if g not in objects or not (open_band[0] <= kit.opening(objects, g) <= open_band[1]):  # nan: no reading
            continue
        f = _hand_frames(objects, g, obj)
        boxes = hand_boxes(f[0])
        if _meets(f, boxes[1:4], parts, tol) and _meets(f, boxes[4:7], parts, tol):
            out.append(g)
    return tuple(out)
