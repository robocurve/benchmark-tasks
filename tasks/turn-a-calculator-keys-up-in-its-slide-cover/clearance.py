"""How far apart the two arms stay: the least distance between any link of the left arm and any link of the right arm,
from the URDF's collision hulls (yam_arena.analysis), for given joints. No simulator.

The distance between two convex hulls is the largest gap between their shadows on a line, over all lines (for
separated hulls; 0 or less when they touch or overlap). Every line gives a gap no larger than the distance, so the
figure below is never more than the true distance: the lines tried are the faces' normals of both hulls and the line
between their middles, then nearby lines round the best one. (yam_arena's ``hull_set_distance`` sets vertices against
faces, which can overstate the distance where two edges pass close by.)

Each link is taken as ONE convex shell, the convex hull of all its collision pieces: it holds the link, so the distance
between two shells is no more than the distance between the links (and the pieces, up to 107 a link, would take
minutes for a whole plan).
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np

WRIST = ("link4", "link5", "gripper", "tip_left", "tip_right")  # the hand and the wrist
_RNG = np.random.default_rng(0)


def hull_gap(a, b, refine: bool = False) -> float:
    """A lower bound on the distance (m) between convex hulls ``a`` and ``b`` (yam_arena.analysis.Hull, world frame)."""
    av, bv = a.vertices, b.vertices
    toward = bv.mean(axis=0) - av.mean(axis=0)
    lines = np.vstack([a.planes[:, :3], -b.planes[:, :3], toward / max(np.linalg.norm(toward), 1e-12)])
    gaps = (bv @ lines.T).min(axis=0) - (av @ lines.T).max(axis=0)
    best_gap, best = float(gaps.max()), lines[int(gaps.argmax())]
    if refine:
        for scale in (0.3, 0.1, 0.03, 0.01, 0.003, 0.001):
            for _ in range(4):
                trial = best + scale * _RNG.normal(size=(96, 3))
                trial /= np.linalg.norm(trial, axis=1, keepdims=True)
                gaps = (bv @ trial.T).min(axis=0) - (av @ trial.T).max(axis=0)
                if gaps.max() > best_gap:
                    best_gap, best = float(gaps.max()), trial[int(gaps.argmax())]
    return best_gap


@lru_cache(maxsize=1)
def link_shells() -> dict:
    """Each link's convex shell (yam_arena.analysis.Hull in the link's frame): the hull of all its collision pieces."""
    from scipy.spatial import ConvexHull
    from yam_arena import analysis

    shells = {}
    for name, pieces in analysis.link_hulls().items():
        points = np.vstack([h.vertices for h in pieces])
        hull = ConvexHull(points)
        planes = np.unique(np.c_[hull.equations[:, :3], -hull.equations[:, 3]].round(9), axis=0)  # n . x <= d inside
        shells[name] = analysis.Hull(vertices=points[hull.vertices], planes=planes)
    return shells


def arm_hulls(q, base, fingers: float) -> list:
    """[(link, shell in the world)] of the arm at arm joints ``q`` (rad), the fingers opened ``fingers`` (0 closed, 1
    open)."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN

    joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, q)}, **dict.fromkeys(FINGER_JOINTS, float(fingers) * FINGER_OPEN)}
    out = []
    for name, pose in analysis.fk_link_poses_world(joints, base).items():
        shell = link_shells().get(name)
        if shell is None:
            continue
        r, p = pose[:3, :3], pose[:3, 3]
        normals = shell.planes[:, :3] @ r.T
        out.append((name, analysis.Hull(vertices=shell.vertices @ r.T + p, planes=np.c_[normals, shell.planes[:, 3] + normals @ p])))
    return out


def arms_apart(left, right, links=None, near: float = 0.15, refine: bool = False) -> tuple[float, tuple[str, str] | None]:
    """(least distance in m, (left link, right link)) between two arms' hulls, from :func:`arm_hulls`; ``links``: only
    these links of both arms. Pairs whose boxes lie more than ``near`` apart are skipped (they are farther than that)."""
    best, pair = near, None
    for name_a, ha in left:
        if links is not None and name_a not in links:
            continue
        lo_a, hi_a = ha.vertices.min(axis=0), ha.vertices.max(axis=0)
        for name_b, hb in right:
            if links is not None and name_b not in links:
                continue
            lo_b, hi_b = hb.vertices.min(axis=0), hb.vertices.max(axis=0)
            if np.maximum(lo_b - hi_a, lo_a - hi_b).max() > best:
                continue
            gap = hull_gap(ha, hb, refine=refine)
            if gap < best:
                best, pair = gap, (name_a, name_b)
    return best, pair


def hand_under(rot, fingers: float = 1.0) -> float:
    """How far (m) the hand (gripper and fingertips, opened ``fingers``) reaches under its grasp point when turned as
    ``rot`` (columns: closing axis, pads' width axis, approach)."""
    from yam_arena import analysis
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN, READY, TCP_OFFSET

    joints = {**{n: float(v) for n, v in zip(ARM_JOINTS, READY)}, **dict.fromkeys(FINGER_JOINTS, float(fingers) * FINGER_OPEN)}
    poses = analysis.fk_link_poses_world(joints, (0.0, 0.0, 0.0))
    g = poses["gripper"]
    corners = np.vstack([(h.vertices - g[:3, 3]) @ g[:3, :3] - np.array(TCP_OFFSET) for _, h in analysis.world_hulls(poses, ["gripper", "tip_left", "tip_right"])])
    return float(-(corners @ np.asarray(rot, dtype=float).T)[:, 2].min())


def plan_apart(moves: dict, bases: dict, hz: float = 20.0, tail_s: float = 1.0, every: int = 2, close: float = 0.030) -> dict:
    """Both arms' moves (side -> list of demo_kit.Move) played through the IK at ``hz`` as a run plays them: the least
    distance between the two arms over the whole plan, any links and the wrists only (``WRIST``), with when and which
    links. The closest moments are measured again with a finer search of lines."""
    from demo_kit import Arm

    arms = {side: Arm(side, bases[side], list(moves[side])) for side in ("left", "right")}
    end = max(arm.seconds for arm in arms.values()) + tail_s
    least = {key: (np.inf, None, None, None, None) for key in ("arms", "wrists")}
    near_moves: dict = {}
    for i in range(int(round(end * hz)) + 1):
        t = i / hz
        actions, labels = {}, {}
        for side, arm in arms.items():  # every step through the IK, as a run seeds it
            actions[side], labels[side] = arm.action(t)
        if i % every:
            continue
        hulls = {side: arm_hulls(arm.q, bases[side], actions[side][6]) for side, arm in arms.items()}  # the IK's pose, without a push's offsets
        for key, links in (("arms", None), ("wrists", WRIST)):
            gap, pair = arms_apart(hulls["left"], hulls["right"], links)
            if gap < least[key][0]:
                least[key] = (gap, t, pair, (labels["left"], labels["right"]), hulls)
            if key == "arms" and gap < close:  # every pair of moves during which the arms come closer than ``close``
                seg = near_moves.setdefault((labels["left"], labels["right"]), [gap, t, t, pair])
                if gap < seg[0]:
                    seg[0], seg[3] = gap, pair
                seg[2] = t
    out = {}
    for key, (gap, t, pair, labels, hulls) in least.items():  # gap: no less than this anywhere in the plan (150 mm: farther)
        fine, _ = arms_apart(hulls["left"], hulls["right"], None if key == "arms" else WRIST, refine=True)
        out[key] = {"at least (mm)": round(1000 * gap, 1), "at (s)": round(t, 2), "links": pair, "moves": labels, "there, finer search (mm)": round(1000 * fine, 1)}
    out[f"moves closer than {1000 * close:.0f} mm"] = [{"moves": k, "least (mm)": round(1000 * v[0], 1), "from (s)": round(v[1], 2), "to (s)": round(v[2], 2), "links": v[3]}
                                                       for k, v in near_moves.items()]  # fmt: skip
    return out
