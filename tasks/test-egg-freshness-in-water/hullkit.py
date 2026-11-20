"""hullkit: contact between the robot and a task's parts, on the robot's own collision shapes.

The robot model and its collision hulls load at run time from the competition environment
(yam_arena.analysis.link_hulls: the convex hulls the URDF lists for each link); nothing robot-derived is stored here.
In the simulator the arm's link poses are read every step (yamkit stores them with the grippers' states); offline,
where only a gripper's tool point, turn and opening are known, the hand's links (palm and both fingertips) are placed
by the URDF's own finger joints.

    hk.gap(objects, "right_gripper", "cannula/plate", PARTS) -> m between that arm's links and the parts
    hk.touching(objects, "cannula/plate", PARTS)              -> the grippers whose arm is within CONTACT of the parts

Parts are guardkit shapes (Block, Seg) in the object's own frame. The distance between a convex hull and a convex
part is found by Frank-Wolfe on the hull's vertices with the part's exact nearest point; the iteration stops when its
duality gap certifies the distance to 0.1 mm."""

import math
from functools import lru_cache

import numpy as np

import guardkit as gk
import yamkit as kit

GRIPPERS = ("left_gripper", "right_gripper")
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
HAND_LINKS = ("gripper", "tip_left", "tip_right")
CONTACT = 0.001  # m: touching = the robot's collision shape within 1 mm of the part (the solver rests at 0 gap)
_TOL = 1e-4  # m: distance accuracy


@lru_cache(maxsize=1)
def _hulls() -> dict:
    """{link: [(vertices (n, 3) in the link frame, local AABB lo, hi)]}, loaded from the competition environment."""
    from yam_arena.analysis import link_hulls

    out = {}
    for name, hs in link_hulls().items():
        if name in ARM_LINKS:
            out[name] = [(np.asarray(h.vertices, float), np.asarray(h.vertices, float).min(0),
                          np.asarray(h.vertices, float).max(0)) for h in hs]
    return out


@lru_cache(maxsize=64)
def _finger_frames(opening_mm: int) -> dict:
    """The fingertip links' poses in the gripper link's frame at an opening (0.1 mm steps), by the URDF's joints."""
    from yam_arena import YAM_URDF
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import forward_kinematics

    q = opening_mm / 10000.0 / abs(FINGER_OPEN) * FINGER_OPEN
    fk = forward_kinematics(YAM_URDF, {"joint7": q, "joint8": q}, root="gripper")
    return {k: fk[k] for k in ("tip_left", "tip_right")}


def link_poses(objects, g: str) -> dict:
    """{link: (position, rotation)} of arm g's links (world, as the scorer's positions)."""
    live = kit._LINKS.get(g)
    if live:
        return live
    if kit._IN_SIM["on"]:  # in a run the arm's links are always read; a missing one is an error, never a fallback
        raise RuntimeError(f"[hullkit] no arm link poses for {g} from the simulator")
    from yam_arena.constants import FINGER_OPEN, TCP_OFFSET

    s = objects[g]
    R = gk._rot(s.quat_xyzw)
    p = np.asarray(s.pos, float) - R @ np.asarray(TCP_OFFSET, float)
    op = kit.opening(objects, g)
    o = abs(FINGER_OPEN) * (1.0 if not np.isfinite(op) else float(np.clip(op, 0.0, 1.0)))
    out = {"gripper": (p, R)}
    for k, T in _finger_frames(int(round(o * 10000))).items():
        out[k] = (p + R @ T[:3, 3], R @ T[:3, :3])
    return out


def _part_box(part) -> tuple:
    if isinstance(part, gk.Block):
        c, h = np.asarray(part.center, float), np.asarray(part.size, float) / 2
        return c - h, c + h
    a, b = np.asarray(part.a, float), np.asarray(part.b, float)
    return np.minimum(a, b) - part.r, np.maximum(a, b) + part.r


def _hull_part_gap(V: np.ndarray, part, iters: int = 60) -> float:
    """Distance (m, 0 if they overlap) between the convex hull of points V and a convex part (same frame)."""
    proj = lambda q: gk._proj_local(part, q)  # noqa: E731
    d0 = np.linalg.norm(V - proj(V), axis=1)
    x = V[int(np.argmin(d0))].copy()
    for _ in range(iters):
        px = proj(x[None, :])[0]
        g = x - px
        f = float(np.linalg.norm(g))
        if f < 1e-9:
            return 0.0
        v = V[int(np.argmin(V @ g))]
        gap = float(g @ (x - v))
        if gap <= _TOL * f:  # f(x) - f* <= gap / f: certified to _TOL
            return max(0.0, f - gap / f)
        lo, hi = 0.0, 1.0
        for _ in range(24):  # the distance is convex along the step: golden-section search
            a, b = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
            ya, yb = x + a * (v - x), x + b * (v - x)
            fa = np.linalg.norm(ya - proj(ya[None, :])[0])
            fb = np.linalg.norm(yb - proj(yb[None, :])[0])
            lo, hi = (lo, b) if fa <= fb else (a, hi)
        x = x + 0.5 * (lo + hi) * (v - x)
    return float(np.linalg.norm(x - proj(x[None, :])[0]))


def gap(objects, g: str, obj: str, parts, links=ARM_LINKS, upto: float = 0.02) -> float:
    """Distance (m) between arm g's collision hulls (`links`) and the nearest of `parts` of `obj` (exact to 0.1 mm
    below `upto`; beyond it only `upto` or more is reported)."""
    so = objects[obj]
    co, Ro = np.asarray(so.pos, float), gk._rot(so.quat_xyzw)
    parts = parts if isinstance(parts, (list, tuple)) else (parts,)
    boxes = [_part_box(p) for p in parts]
    best = math.inf
    H = _hulls()
    for name, (p, R) in link_poses(objects, g).items():
        if name not in links or name not in H:
            continue
        A = Ro.T @ R  # link frame -> object frame
        t = Ro.T @ (np.asarray(p, float) - co)
        for V, lo, hi in H[name]:
            c, h = (lo + hi) / 2, (hi - lo) / 2
            cw, hw = A @ c + t, np.abs(A) @ h  # the hull's box in the object frame
            for part, (plo, phi) in zip(parts, boxes):
                sep = float(np.max(np.maximum(plo - (cw + hw), (cw - hw) - phi)))
                if sep > min(best, upto):
                    continue
                best = min(best, _hull_part_gap(V @ A.T + t, part))
    return best


def hand_gap(tcp, R, opening: float, parts, links=HAND_LINKS, upto: float = 0.02) -> float:
    """Distance (m) between a hand posed at tool point `tcp` with rotation R (world), its fingers `opening` m from the
    tool point, and world-frame `parts` (a planned pose: its palm and fingertips placed by the URDF's finger joints)."""
    from yam_arena.constants import TCP_OFFSET

    R = np.asarray(R, float)
    p = np.asarray(tcp, float) - R @ np.asarray(TCP_OFFSET, float)
    poses = {"gripper": (p, R)}
    for k, T in _finger_frames(int(round(abs(opening) * 10000))).items():
        poses[k] = (p + R @ T[:3, 3], R @ T[:3, :3])
    parts = parts if isinstance(parts, (list, tuple)) else (parts,)
    boxes = [_part_box(q) for q in parts]
    best = math.inf
    H = _hulls()
    for name, (lp, lR) in poses.items():
        if name not in links or name not in H:
            continue
        for V, lo, hi in H[name]:
            c, h = (lo + hi) / 2, (hi - lo) / 2
            cw, hw = lR @ c + lp, np.abs(lR) @ h
            for part, (plo, phi) in zip(parts, boxes):
                if float(np.max(np.maximum(plo - (cw + hw), (cw - hw) - phi))) > min(best, upto):
                    continue
                best = min(best, _hull_part_gap(V @ lR.T + lp, part))
    return best


def touching(objects, obj: str, parts, tol: float = CONTACT, grippers=GRIPPERS, links=ARM_LINKS) -> tuple:
    """The grippers whose arm (`links`) is within `tol` of one of `parts`: contact, not nearness."""
    return tuple(g for g in grippers if g in objects and gap(objects, g, obj, parts, links, upto=tol) <= tol)
