"""On the station table, in front of the arms: a 19-inch two-post open-frame rack (8U) clamped to the table, its front
facing the arms. A 2U server fills U1-U2, a 1U switch U3 and a 1U patch panel U8; U4 to U7 are empty. A 1U tool-less
blanking panel stands upright in a slotted stand between the arms and the rack.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Fit the blanking panel at U6."

TOP = g.TOP

# -- objects --------------------------------------------------------------------------------------------------------
RACK = UsdBody("rack", usd="assets/rack.usda", pos=(0.0, 0.0, TOP), size=(0.08, 2 * g.POST_OUT, g.TOP_Z + g.BAR_H))
STAND = UsdBody("stand", usd="assets/stand.usda", pos=(0.0, 0.0, TOP), size=(0.05, 0.4, g.STAND_H))
PANEL = UsdBody("panel", usd="assets/panel.usda", pos=(g.STAND_X, 0.0, TOP + g.PANEL_START_Z),
                size=(g.PANEL_T, g.PANEL_W, g.PANEL_H), mass=g.PANEL_MASS, friction=(0.5, 0.4))
OBJECTS = [RACK, STAND, PANEL]

# -- monitor ---------------------------------------------------------------------------------------------------------
_STATE: dict = {}
HOLES = np.array([g.hole_z(h) for h in range(1, 3 * g.NU + 1)])
HOLD_STEPS = 20           # control steps (1 s) fitted with the arms clear before it counts as fitted and let go
PANEL_BOX = np.array([g.PANEL_T + g.PEG_L, g.PANEL_W, g.PANEL_H])   # the plate and its clips, as one box


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(ends={}, clearance=1.0, clear=True, clear_steps=0)


_reset_state()


def _ends(state):
    """Per end of the panel ("left" is the robot's left): the hole its clip is level with (within HOLE_TOL, or None),
    whether the clip is in that hole (within 1.75 mm of its centre across and up), and whether the end is pushed
    home (the clip in its hole and the panel's back against the post: no more than 2 mm in front, 1 mm behind)."""
    R, pos = state.rotation(), np.asarray(state.pos, float)
    out = {}
    for s, side in ((1, "left"), (-1, "right")):
        back = pos + R @ np.array([0.0, s * g.PY, 0.0])              # the back face where this end's clip leaves it
        clip = pos + R @ np.array([g.PEG_L / 2, s * g.PY, 0.0])       # the clip's middle
        z = clip[2] - TOP
        k = int(np.argmin(np.abs(HOLES - z)))
        level = abs(HOLES[k] - z) <= g.HOLE_TOL
        across = clip[1] - s * g.PY
        gap = g.RX - back[0]                                         # the panel's back in front of the post
        in_hole = bool(level and abs(HOLES[k] - z) <= g.CLIP_TOL and abs(across) <= g.CLIP_TOL and gap <= g.PEG_L)
        out[side] = {"hole": k + 1 if level else None, "in_hole": in_hole, "across": float(across), "gap": float(gap),
                     "home": bool(in_hole and -g.BEHIND_TOL <= gap <= g.SEAT_TOL)}
    return out


def _edge_points(size, step: float = 0.005) -> np.ndarray:
    """Points every 5 mm or less along a box's 12 edges, in its frame (origin at its centre)."""
    half, pts = np.asarray(size) / 2, []
    for a in range(3):
        b, c = [i for i in range(3) if i != a]
        t = np.linspace(-half[a], half[a], int(np.ceil(size[a] / step)) + 1)
        for sb in (-1.0, 1.0):
            for sc in (-1.0, 1.0):
                p = np.zeros((len(t), 3))
                p[:, a], p[:, b], p[:, c] = t, sb * half[b], sc * half[c]
                pts.append(p)
    return np.concatenate(pts)


EDGES = _edge_points(PANEL_BOX)


def _distance(hulls, R, centre) -> float:
    """The smallest gap between convex hulls and the panel's box (rotation, centre): from every hull vertex to the box,
    and from points along the box's edges to each nearby hull (by the hull's face planes, which never overestimate
    it). Negative when they overlap."""
    from yam_arena.analysis import box_signed_distance

    if not hulls:
        return 1.0
    verts = np.concatenate([h.vertices for h in hulls])
    best = float(box_signed_distance(verts, centre, R, PANEL_BOX).min())
    edges = centre + EDGES @ R.T
    lo, hi = edges.min(axis=0) - 0.02, edges.max(axis=0) + 0.02
    h_lo = np.array([h.vertices.min(axis=0) for h in hulls])
    h_hi = np.array([h.vertices.max(axis=0) for h in hulls])
    for i in np.flatnonzero(np.all(h_hi >= lo, axis=1) & np.all(h_lo <= hi, axis=1)):
        p = hulls[i].planes
        best = min(best, float((edges @ p[:, :3].T - p[:, 3]).max(axis=1).min()))
    return best


def _clearance(environment, state) -> float:
    """How far either arm is from the panel (its plate and clips, as one box), using the URDF's collision hulls of
    every link, the gripper body and both fingers at their simulated poses."""
    from yam_arena.analysis import world_hulls

    from scenekit import robot_link_poses

    hulls = [h for poses in robot_link_poses(environment).values() for _, h in world_hulls(poses)]
    R, pos = state.rotation(), np.asarray(state.pos, float)
    return _distance(hulls, R, pos + R @ np.array([(g.PEG_L - g.PANEL_T) / 2, 0.0, 0.0]))


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    state = object_states(environment, ["panel"])["panel"]
    S = _STATE
    S["ends"] = _ends(state)
    S["clearance"] = _clearance(environment, state)
    S["clear"] = S["clearance"] >= g.CLEAR
    S["clear_steps"] = S["clear_steps"] + 1 if _fitted_at(g.TARGET_HOLE) and S["clear"] else 0


def _fitted_at(hole: int) -> bool:
    e = _STATE["ends"]
    return bool(e) and all(e[s]["hole"] == hole and e[s]["home"] for s in ("left", "right"))


def success(objects) -> bool:
    """The panel at U6: both clips in U6's middle hole and both ends pushed home, with no part of either arm within
    1 cm of it (yam_arena ends the trial once this has held for SUCCESS_HOLD_S)."""
    return _fitted_at(g.TARGET_HOLE) and _STATE["clear"]


def score(objects) -> float:
    """At the end of the trial (success scores 1.0): 1.0 fitted at U6 with the arms clear for the last 1 s; 0.5 fitted
    at U6 but not let go for the last 1 s, or with one end pushed home and the other end's clip lined up in front of
    its hole (level, within 1.75 mm across, no more than 15 mm out); 0 anything else, including one hole off (the
    panel straddling two units) and the two ends in different holes."""
    e = _STATE["ends"]
    if not e:
        return 0.0
    if _fitted_at(g.TARGET_HOLE):
        return 1.0 if _STATE["clear_steps"] >= HOLD_STEPS else 0.5
    home = [s for s in ("left", "right") if e[s]["home"] and e[s]["hole"] == g.TARGET_HOLE]
    if len(home) == 1:
        o = e["right" if home[0] == "left" else "left"]
        if o["hole"] == g.TARGET_HOLE and abs(o["across"]) <= g.CLIP_TOL and -g.BEHIND_TOL <= o["gap"] <= g.OUT_TOL:
            return 0.5
    return 0.0
