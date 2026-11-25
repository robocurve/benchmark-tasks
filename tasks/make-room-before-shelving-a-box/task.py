"""A low chrome wire shelf with an open back stands on the station table, clamped to it, its shelf 33 cm above the table.
On the shelf stand three 4.4 kg cases of canned soup, C1 to C3, in a row; the biggest gap in their row is 16.0 cm,
narrower than a case (23.2 cm). A new 1.5 kg case, N (rice, with a hand hole in each long side), stands on the table in
front of the shelf, its far part under it.

The robot is asked only to put N on the shelf in the row. Shoving N straight into the gap drives C1 and C2 back, out of
the shelf's open back, and they fall onto the table behind it. The careful way is to slide C1 and C2 sideways first,
then lift N by its hand holes with both hands, set it on the shelf with its centre over the edge, and push it into the
opened slot.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score --max-steps 3600 --policy ...
Demo:    python demokit.py task.py OUT [--mode full|shove|drop|...]
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import layout as L  # noqa: E402
from scenekit import UsdBody, on_reset, on_step  # noqa: E402

SCENE = "station"
SUCCESS_HOLD_S = 0.0            # the scorer holds the end state itself (L.HOLD_STEPS)

INSTRUCTION = "Put the case from the table on the shelf, in the row with the other cases."

ROOM = UsdBody("shelf", usd="assets/room.usda", pos=(0.0, 0.0, 0.0), size=(0.356, 1.067, 0.375), friction=None)
CASE_BODIES = {name: UsdBody(name, usd="assets/new_case.usda" if name == "N" else "assets/soup_case.usda",
                             pos=L.start_pos(name), size=L.CASE, mass=L.N_KG if name == "N" else L.SOUP_KG,
                             friction=None, solver_iters=16)
               for name in L.CASES}
OBJECTS = [ROOM, *CASE_BODIES.values()]

_DEBUG = bool(os.environ.get("SHELF_DEBUG"))
_SUB: dict = {}
_STATE: dict = {}


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x, np.float64)


def rotation(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)


def corners(pos, R) -> np.ndarray:
    """A case's eight corners (world)."""
    return np.asarray(pos) + (_SIGNS * (np.asarray(L.CASE) / 2)) @ R.T


def box_distance(P, pos, R) -> np.ndarray:
    """Signed distance from points P (n, 3) to a case's box (negative inside)."""
    q = np.abs((np.asarray(P) - pos) @ R) - np.asarray(L.CASE) / 2
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0)


# -- the robot's hands as points ----------------------------------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right", "link5", "link4")
FINGER_LINKS = ("gripper", "tip_left", "tip_right")


def _robot_points() -> dict:
    """Points on each hand link's collision hulls (vertices and the centres of their larger faces, one per 4 mm cell),
    in the link's frame."""
    if "robot_pts" in _SUB:
        return _SUB["robot_pts"]
    import xml.etree.ElementTree as ET

    from yam_arena import ASSET_DIR, YAM_URDF
    from yam_arena.analysis import read_stl
    from yam_arena.urdf import rpy_matrix

    pts: dict = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in HAND_LINKS:
            continue
        for col in link.findall("collision"):
            o = col.find("origin")
            T = np.eye(4)
            if o is not None:
                T[:3, :3] = rpy_matrix(np.array([float(v) for v in o.get("rpy", "0 0 0").split()]))
                T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
            tris = read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ T[:3, :3].T + T[:3, 3]
            area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
            pts.setdefault(name, []).append(np.vstack([tris.reshape(-1, 3), tris[area > 2e-5].mean(1)]))
    out = {}
    for name, v in pts.items():
        P = np.vstack(v)
        _, keep = np.unique(np.floor(P / 0.004).astype(np.int64), axis=0, return_index=True)
        out[name] = P[np.sort(keep)]
    _SUB["robot_pts"] = out
    return out


def hand_points(environment) -> dict:
    """World points on both hands, by link group: "hand" (gripper, fingers and the last wrist link) and "fingers"."""
    from scenekit import robot_link_poses

    pts = _robot_points()
    hand, fingers = [], []
    for _side, poses in robot_link_poses(environment).items():
        for name, P in pts.items():
            T = poses[name]
            W = P @ T[:3, :3].T + T[:3, 3]
            hand.append(W)
            if name in FINGER_LINKS:
                fingers.append(W)
    return {"hand": np.vstack(hand), "fingers": np.vstack(fingers)}


# -- the cases' state -----------------------------------------------------------------------------------------------------
def case_states(environment) -> dict:
    """Each case's pose (pos, R), velocities and corners, env-relative."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = {}
    for name in L.CASES:
        a = base.scene[name]
        pose = _np(a.data.root_link_pose_w)[0]
        vel = _np(a.data.root_vel_w)[0]
        pos, R = pose[:3] - origin, rotation(pose[3:7])
        out[name] = dict(pos=pos, R=R, v=vel[:3], w=vel[3:6], c=corners(pos, R))
    return out


# -- the scorer: plain functions of a state, so that check_score.py can call them -----------------------------------------
SHELF_Z = L.H(L.WORK_TOP)
POST_INNER = (L.UNIT_POSTS[0][1] - L.POST_R, L.UNIT_POSTS[2][1] + L.POST_R)     # the posts' inner faces in y


def on_shelf(c) -> bool:
    """Rests on the work shelf: lowest corner within 1 cm of its top, its centre over the shelf's outline."""
    centre = c.mean(0)
    return bool(abs(c[:, 2].min() - SHELF_Z) <= L.ON_SHELF_TOL and L.SHELF_X[0] <= centre[0] <= L.SHELF_X[1]
                and L.SHELF_Y[0] <= centre[1] <= L.SHELF_Y[1])


def wholly_on_shelf(c) -> bool:
    return bool(c[:, 0].min() >= L.SHELF_X[0] and c[:, 0].max() <= L.SHELF_X[1] and c[:, 1].min() >= L.SHELF_Y[0]
                and c[:, 1].max() <= L.SHELF_Y[1])


def tilt_deg(R) -> float:
    return float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))


def in_row(c, R) -> bool:
    """N on the work shelf, long side within 15 degrees of front to back, front face no more than 2 cm behind the
    front-edge line."""
    long_axis = R[:, 0]
    level_long = math.degrees(math.acos(min(1.0, abs(long_axis[0]) / max(1e-9, math.hypot(long_axis[0], long_axis[1])))))
    return bool(on_shelf(c) and level_long <= L.ROW_DEG and abs(long_axis[2]) < math.sin(math.radians(L.ROW_DEG))
                and c[:, 0].min() <= L.FRONT_X + L.ROW_BEHIND)


def room_made(cases: dict) -> bool:
    """A clear gap of 23.8 cm or more along the front-edge line, between neighbouring cases on the work shelf or
    between a case and the posts."""
    spans = sorted((float(s["c"][:, 1].min()), float(s["c"][:, 1].max())) for n, s in cases.items()
                   if n in L.ROW and on_shelf(s["c"]))
    edges = [POST_INNER[1]]
    gaps = []
    for lo, hi in spans:
        gaps.append(lo - edges[-1])
        edges.append(hi)
    gaps.append(POST_INNER[0] - edges[-1])
    return max(gaps) >= L.ROOM


def on_a_row_case(cases: dict) -> bool:
    """N resting on top of C1, C2 or C3: its lowest corner within 1 cm of that case's top, their outlines overlapping."""
    c = cases["N"]["c"]
    lo, hi = c[:, :2].min(0), c[:, :2].max(0)
    for k in L.ROW:
        ck = cases[k]["c"]
        klo, khi = ck[:, :2].min(0), ck[:, :2].max(0)
        overlap = np.all(np.minimum(hi, khi) > np.maximum(lo, klo))
        if overlap and abs(c[:, 2].min() - ck[:, 2].max()) <= L.ON_SHELF_TOL:
            return True
    return False


def tier(cases: dict, latches: dict, clear: bool, still: bool) -> tuple[float, str]:
    """The score of a state, before the 1 s hold: (score, why). ``latches``: dropped, out_back."""
    if latches["dropped"]:
        return 0.0, "a case dropped"
    n = cases["N"]
    c, R = n["c"], n["R"]
    shelved = in_row(c, R) and n["pos"][0] >= L.SHELF_X[0]
    row_on = all(on_shelf(cases[k]["c"]) for k in L.ROW)
    if shelved:
        if tilt_deg(R) > L.ON_SIDE_DEG:
            return 0.0, "N on its side or upside down"
        if not row_on:
            return 0.0, "N in the row, but a neighbour is off the work shelf"
        if latches["out_back"]:
            return 0.25, "N in the row, but a case went out of the back"
        row_ok = all(wholly_on_shelf(cases[k]["c"]) for k in L.ROW)
        upright = tilt_deg(R) <= L.UPRIGHT_DEG
        if upright and wholly_on_shelf(c) and row_ok and clear and still:
            return 1.0, "N shelved"
        return 0.5, "N in the row, not finished: " + ", ".join(
            w for w, ok in (("upright", upright), ("wholly on", wholly_on_shelf(c)), ("row", row_ok), ("clear", clear),
                            ("still", still)) if not ok)
    elsewhere = not (on_shelf(c) or on_a_row_case(cases))       # still on the table, or held
    if elsewhere and row_on and not latches["out_back"] and room_made(cases):
        return 0.25, "room made"
    return 0.0, "N not shelved"


# -- the tracker ---------------------------------------------------------------------------------------------------------
@on_reset
def _reset(environment=None) -> None:
    _STATE.clear()
    _STATE.update(step=0, ref={}, latches=dict(dropped=False, out_back=False), hold=0, score=0.0,
                  why="", final=False, events=[])


_reset()


def _latch(key: str, why: str) -> None:
    if not _STATE["latches"][key]:
        _STATE["latches"][key] = True
        _STATE["events"].append((_STATE["step"], why))
        if _DEBUG:
            print(f"[shelf] step {_STATE['step']}: {why}", flush=True)


@on_step
def _monitor(environment) -> None:
    S = _STATE
    S["step"] += 1
    cases = case_states(environment)
    pts = hand_points(environment)
    touched = {}
    min_clear = math.inf
    for name, s in cases.items():
        dh = box_distance(pts["hand"], s["pos"], s["R"])
        touched[name] = bool(dh.min() < L.TOUCH)
        min_clear = min(min_clear, float(box_distance(pts["fingers"], s["pos"], s["R"]).min()))
    for name, s in cases.items():
        low = float(s["c"][:, 2].min())
        supported = touched[name] or (np.linalg.norm(s["v"]) < 0.02 and np.linalg.norm(s["w"]) < 0.1)
        if supported or name not in S["ref"]:
            S["ref"][name] = low
        elif low < S["ref"][name] - L.DROP_FALL:
            _latch("dropped", f"{name} fell {100 * (S['ref'][name] - low):.1f} cm untouched")
        xmax = float(s["c"][:, 0].max())
        if xmax >= L.BACK_LINE:
            _latch("out_back", f"{name} out of the back (x {xmax:.3f})")
    clear = min_clear >= L.CLEAR
    still = all(np.linalg.norm(s["v"]) < L.STILL_V for s in cases.values())
    score, why = tier(cases, S["latches"], clear, still)
    if score == 1.0:
        S["hold"] += 1
        if S["hold"] < L.HOLD_STEPS:
            score, why = 0.5, f"holding {S['hold']}/{L.HOLD_STEPS}"
    else:
        S["hold"] = 0
    S["score"], S["why"], S["final"] = score, why, score == 1.0
    S["touched"], S["min_clear"] = touched, min_clear
    if _DEBUG and S["step"] % 20 == 0:
        n = cases["N"]
        print(f"[shelf] step {S['step']} score {score} ({why}) N {np.round(n['pos'], 3)} tilt {tilt_deg(n['R']):.1f} "
              f"C1 {np.round(cases['C1']['pos'], 3)} C2 {np.round(cases['C2']['pos'], 3)} "
              f"touch {[k for k, v in touched.items() if v]} clear {min_clear * 100:.1f} cm latches {S['latches']}",
              flush=True)


def success(objects) -> bool:
    """The full state held for 1 s (20 control steps): exactly when score() returns 1.0."""
    return bool(_STATE.get("final"))


def score(objects) -> float:
    """Read every control step from the cases' poses and the hands' collision geometry.

    Latches, over the whole trial: dropped (a case falls 3 cm or more while no hand touches it, as when it falls off the
    shelf's back onto the table) scores 0; out of the back (any part of a case at x >= 0.756, the shelf's back edge)
    caps the score at 0.25.
    1.0: N on the work shelf in the row, upright and wholly on it; C1 to C3 on the shelf and wholly on it; every finger
    point 3 cm or more from every case; every case under 1 cm/s; held 1 s.
    0.5: no latch, N in the row with its centre over the shelf, not on its side, C1 to C3 on the work shelf, but short of
    1.0 (sticking out of the front, tilted 10 to 45 degrees, a hand not clear, a neighbour not wholly on the shelf, or a
    case still moving).
    0.25: N in the row with its centre over the shelf, not on its side, C1 to C3 on the work shelf, but a case went out
    of the back; or N anywhere other than resting on the work shelf (on the table, or still held, even just above the
    shelf), C1 to C3 on it, none ever out of the back, and room made (a clear gap of 23.8 cm).
    0 otherwise, including N on its side or upside down (its top more than 45 degrees from up), or N on top of a case on
    the work shelf."""
    return float(_STATE.get("score", 0.0))


# -- the scripted demo (demokit) ----------------------------------------------------------------------------------------
def demo_script(ctx):
    from demo import script

    yield from script(ctx)
