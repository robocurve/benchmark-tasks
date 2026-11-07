"""On the station table, in front of the arms: a tote holding four LTO tape cartridges standing on edge, and behind it
a five-slot tape magazine clamped to the table, its slots facing the arms. From the robot's left the tote holds
AI0003L9, AI0001L9, AI0002L9 and AI0004L9; AI0002L9 stands turned round, its label facing the magazine.

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

INSTRUCTION = "Load the four tapes into the magazine in barcode order, starting at slot 1."

TOP = g.TOP
TAPES = [f"tape_{k}" for k in range(1, 5)]


def _tape(k: int) -> UsdBody:
    i = g.TOTE_ORDER.index(k)
    turned = k == g.TURNED
    return UsdBody(f"tape_{k}", usd=f"assets/tape_{k}.usda",
                   pos=(g.TOTE_X + (g.TAPE_L if turned else 0.0), g.BAY_Y[i], TOP + g.TOTE_FLOOR),
                   rot_xyzw=(0.0, 0.0, 1.0, 0.0) if turned else (0.0, 0.0, 0.0, 1.0),
                   size=(g.TAPE_L, g.TAPE_T, g.TAPE_H), mass=g.TAPE_MASS, friction=(0.5, 0.4))


# -- objects --------------------------------------------------------------------------------------------------------
MAGAZINE = UsdBody("magazine", usd="assets/magazine.usda", pos=(0.0, 0.0, TOP), size=(g.MAG_D, 0.16, 0.14),
                   friction=(0.3, 0.25))
TOTE = UsdBody("tote", usd="assets/tote.usda", pos=(0.0, 0.0, TOP), size=(0.12, 0.25, g.TOTE_WALL))
OBJECTS = [MAGAZINE, TOTE] + [_tape(k) for k in range(1, 5)]

# -- monitor ---------------------------------------------------------------------------------------------------------
_STATE: dict = {}
HOLD_STEPS = 20           # control steps (1 s) all loaded with the arms clear before it counts as let go
HOME_X = g.MAG_X + g.SLOT_IN
TAPE_SIZE = np.array([g.TAPE_L, g.TAPE_T, g.TAPE_H])


def _edge_points(step: float = 0.005) -> np.ndarray:
    """Points every 5 mm or less along a cartridge's 12 edges, in its frame (origin at its centre)."""
    half, pts = TAPE_SIZE / 2, []
    for a in range(3):
        b, c = [i for i in range(3) if i != a]
        t = np.linspace(-half[a], half[a], int(np.ceil(TAPE_SIZE[a] / step)) + 1)
        for sb in (-1.0, 1.0):
            for sc in (-1.0, 1.0):
                p = np.zeros((len(t), 3))
                p[:, a], p[:, b], p[:, c] = t, sb * half[b], sc * half[c]
                pts.append(p)
    return np.concatenate(pts)


EDGES = _edge_points()


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(slots={}, tapes={}, clearance=1.0, clear=True, clear_steps=0)


_reset_state()


def _tape_state(state):
    """Where a cartridge is: the slot it stands in (or None), whether its label faces the arms, and whether it is pushed
    home (its label face within HOME_TOL of the magazine's front)."""
    R, pos = state.rotation(), np.asarray(state.pos, float)
    centre = pos + R @ np.array([g.TAPE_L / 2, 0.0, g.TAPE_H / 2])
    facing = float(-(R @ np.array([-1.0, 0.0, 0.0]))[0])        # 1 when the label faces the arms (-x)
    label_out = facing >= g.LABEL_OUT and R[2, 2] > 0.95
    slot = None
    if centre[0] > g.MAG_X and g.MAG_FLOOR - 0.01 < pos[2] - TOP < g.MAG_FLOOR + 0.01:
        k = int(round((g.slot_y(1) - centre[1]) / g.SLOT_PITCH)) + 1
        if 1 <= k <= g.SLOTS and abs(centre[1] - g.slot_y(k)) <= g.IN_SLOT_TOL:
            slot = k
    near = centre[0] - g.TAPE_L / 2                               # the face toward the arms, when square to x
    home = bool(slot is not None and label_out and abs(near - HOME_X) <= g.HOME_TOL)
    return {"slot": slot, "label_out": bool(label_out), "home": home, "near": float(near)}


def _distance(hulls, boxes) -> float:
    """The smallest gap between convex hulls and cartridges, each (rotation, centre): from every hull vertex to each
    cartridge's box, and from points along each box's edges to each nearby hull (by the hull's face planes, which never
    overestimate it). Negative when they overlap."""
    from yam_arena.analysis import box_signed_distance

    if not boxes or not hulls:
        return 1.0
    verts = np.concatenate([h.vertices for h in hulls])
    h_lo = np.array([h.vertices.min(axis=0) for h in hulls])
    h_hi = np.array([h.vertices.max(axis=0) for h in hulls])
    best = 1.0
    for R, centre in boxes:
        best = min(best, float(box_signed_distance(verts, centre, R, TAPE_SIZE).min()))
        edges = centre + EDGES @ R.T
        lo, hi = edges.min(axis=0) - 0.02, edges.max(axis=0) + 0.02
        for i in np.flatnonzero(np.all(h_hi >= lo, axis=1) & np.all(h_lo <= hi, axis=1)):
            p = hulls[i].planes
            best = min(best, float((edges @ p[:, :3].T - p[:, 3]).max(axis=1).min()))
    return best


def _clearance(environment, states) -> float:
    """How far either arm is from the nearest cartridge in a slot (1 m if none is in a slot), using the URDF's
    collision hulls of every link, the gripper body and both fingers at their simulated poses."""
    from yam_arena.analysis import world_hulls

    from scenekit import robot_link_poses

    hulls = [h for poses in robot_link_poses(environment).values() for _, h in world_hulls(poses)]
    boxes = []
    for st in states.values():
        R, pos = st.rotation(), np.asarray(st.pos, float)
        boxes.append((R, pos + R @ np.array([g.TAPE_L / 2, 0.0, g.TAPE_H / 2])))
    return _distance(hulls, boxes)


def _in_order() -> int:
    """How many slots, from slot 1 on, hold the right cartridge (AI0001L9 in slot 1, ...) label out and pushed home."""
    n = 0
    for k in range(1, 5):
        t = _STATE["slots"].get(k)
        if t is None or t["id"] != k or not t["home"]:
            break
        n += 1
    return n


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    states = object_states(environment, TAPES)
    S = _STATE
    S["tapes"] = {name: _tape_state(st) for name, st in states.items()}
    S["slots"] = {}
    for name, t in S["tapes"].items():
        if t["slot"] is not None:
            S["slots"].setdefault(t["slot"], {**t, "id": int(name.split("_")[1])})
    loaded = {n: st for n, st in states.items() if S["tapes"][n]["slot"] is not None}
    S["clearance"] = _clearance(environment, loaded)
    S["clear"] = S["clearance"] >= g.CLEAR
    S["clear_steps"] = S["clear_steps"] + 1 if _in_order() == 4 and S["clear"] else 0


def success(objects) -> bool:
    """Slots 1 to 4 hold AI0001L9 to AI0004L9, each label out and pushed home, with no part of either arm within 1 cm
    of them (yam_arena ends the trial once this has held for SUCCESS_HOLD_S)."""
    return _in_order() == 4 and _STATE["clear"]


def score(objects) -> float:
    """At the end of the trial (success scores 1.0): 0.25 for each slot, counting from slot 1, that holds the right
    cartridge label out and pushed home, stopping at the first slot that does not (wrong cartridge, turned round, not
    home, or empty); all four but still touched by the robot in the last 1 s scores 0.75."""
    n = _in_order()
    if n == 4:
        return 1.0 if _STATE["clear_steps"] >= HOLD_STEPS else 0.75
    return 0.25 * n
