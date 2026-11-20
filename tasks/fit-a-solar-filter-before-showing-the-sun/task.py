"""On the station table: a 70 mm travel refractor on a tabletop alt-az mount, its tube lying level along y with the
dew shield's front toward the left arm, a white 6x30 finder on top and a star diagonal at the back, its 1.25-inch holder
facing up with a 20 mm eyepiece resting in it (the holder's thumbscrew backed off). In front of it, an open accessory
case in light grey foam holds a full-aperture solar filter (black cell, silvery film, one nylon thumbscrew, standing on
edge), the finder's black cap (on edge) and an old 1.25-inch eyepiece filter engraved "SUN" (lying thread up). Some
students are coming to look at the sun: the front filter must go over the dew shield with its thumbscrew tightened, the
finder must be capped, and the eyepiece filter must not be left screwed into the eyepiece.

Simplified physics: rigid bodies. Two threads are rules (task.json, "fidelity"): the thumbscrew advances as it is
turned and presses the dew shield; the SUN filter is held on the eyepiece once its thread has been turned in a full
turn.
"""

from __future__ import annotations

import collections
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import scope_usd as S  # noqa: E402
import steptime  # noqa: E402
from scenekit import UsdBody, before_reset, on_reset, on_step, set_joint_targets  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
RUN_STEPS = 2400                             # the run limit (task.json): every trial runs to it, and its end state is scored
HOLD_S = 1.0                                 # 1.0 needs that end state held over the last 1 s
HZ = 20
MODE = os.environ.get("TASK_DEMO", "success")

TX = 0.32                                    # the telescope's axis line (x)
TURN = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))   # a quarter turn about z: local +y (the closed end) faces -x, the robot
EP0 = (TX, S.DIAG_Y, TOP + S.SEAT_Z)                # the eyepiece's shoulder, seated in the holder
FILT0 = (0.150, -0.095, TOP + S.DECOY_START[2])     # the SUN filter's thread face, lying thread up in its recess
OFF = np.subtract(FILT0, EP0)                       # the filter's place on the eyepiece's chain with every joint at 0
USD = S.build(EP0, FILT0)


def _seated_start() -> bool:
    return MODE in ("seated", "smoke_screw")  # checks: the cell and the cap start in place


CELL_HOME = (TX, S.CELL_SEAT_Y, TOP + S.AX)
CAP_HOME = (TX, S.CAP_SEAT_Y, TOP + S.FZ)
CELL_START = tuple(np.add(S.CELL_START, (0, 0, TOP)))
CAP_START = tuple(np.add(S.CAP_START, (0, 0, TOP)))
# the cell resting on the dew shield (its felt on the shield's top, 1 mm below centred) for the seated checks
CELL_REST = (TX, S.CELL_SEAT_Y, TOP + S.AX - (S.CELL_RI - S.DEW_R))

# -- the thread rules' numbers (task.json, "fidelity") ----------------------------------------------------------------------
K_ADV, D_ADV, F_MAX = 5.0e4, 20.0, 12.0     # the thumbscrew's slide: N/m, N s/m, N (the press it can reach)
D_SPIN = 0.001                              # N m s/rad: the thumbscrew's thread friction while it turns
F_TIGHT = 10.0                              # N: tight (estimate); it then holds its angle against T_HOLD
T_HOLD, K_HOLD, SLIP_HOLD = 0.05, 2.0, math.radians(15.0)   # a tight thumbscrew's grip on its thread: N m, N m/rad
T_STICK, K_STICK, SLIP_STICK = 0.002, 0.2, 0.03             # a loose one's (static friction): N m, N m/rad, rad
CHAIN = S.CHAIN_JOINTS                      # the SUN filter's six joints from the eyepiece
K_LIN, D_LIN = 3000.0, 10.0                 # caught: the filter held on the eyepiece's axis
K_ANG, D_ANG = 2.0, 0.01
D_THREAD = 0.02                             # N m s/rad: the caught filter's thread friction
CATCH_TURN = 2 * math.pi                    # a net full turn in, the spigot inside the barrel, catches the thread
RELEASE_TURN = CATCH_TURN - math.radians(30.0)
IN_LAT, IN_TILT, IN_DEPTH = 0.001, 5.0, 0.0025   # inside: on the barrel's axis, square, at least 2.5 mm in

TELESCOPE = UsdBody("telescope", usd=USD["telescope"], pos=(TX, 0.0, TOP), size=(0.14, 0.40, 0.19), mass=None, friction=(0.5, 0.45))
CASE = UsdBody("case", usd=USD["case"], pos=(0.0, 0.0, TOP), size=(0.16, 0.28, 0.06), mass=None, friction=(0.7, 0.6))
FILTER = UsdBody("solar_filter", usd=USD["solar_filter"], pos=CELL_REST if _seated_start() else CELL_START,
                 rot_xyzw=(0.0, 0.0, 0.0, 1.0) if _seated_start() else TURN, size=(0.091, 0.032, 0.105),
                 articulation=True, joint_pos=(("j_adv", 0.0), ("j_spin", 0.0)), joint_damping=0.0,
                 drives=(("j_adv", K_ADV, D_ADV, F_MAX), ("j_spin", 0.0, D_SPIN, 1.0)),
                 friction=(0.5, 0.45), solver_iters=48, torsional_patch=0.004)
CAP = UsdBody("finder_cap", usd=USD["finder_cap"], pos=CAP_HOME if _seated_start() else CAP_START,
              rot_xyzw=(0.0, 0.0, 0.0, 1.0) if _seated_start() else TURN, size=(0.049, 0.018, 0.049),
              mass=S.CAP_MASS, friction=(0.6, 0.5), solver_iters=48)
EYEPIECE = UsdBody("eyepiece", usd=USD["eyepiece"], pos=EP0, size=(0.036, 0.036, 0.066), articulation=True,
                   joint_pos=tuple((n, 0.0) for n in CHAIN), joint_damping=0.0,
                   drives=tuple((n, 0.0, 0.0, 1.0e4) for n in CHAIN), friction=(0.3, 0.25), solver_iters=32,
                   torsional_patch=0.004)
OBJECTS = [TELESCOPE, CASE, FILTER, CAP, EYEPIECE]

# -- the scoring's numbers (task.json, "scoring") ---------------------------------------------------------------------------
FILTER_GAP = 0.005          # m: the cell's lip at most this far from the dew shield's rim
FILTER_SQUARE_DEG = 5.0
CAP_GAP = 0.005             # m: the cap's inside face at most this far from the objective cell's front
CAP_SQUARE_DEG = 10.0
OVER = 0.003                # m: the cell (cap) axis this near the dew shield's (finder's) axis, i.e. over it
F_SECURED = 8.0             # N: tightened, the thumbscrew presses the dew shield with at least this
SEAT_UP, SEAT_TILT = 0.002, 5.0      # the eyepiece's shoulder at most this above the holder's top; its axis this square
HOLDER_XY = (TX, S.DIAG_Y)
IN_HOLDER_R = 0.016         # the SUN filter's centre this near the holder's axis, below its top: in the light path
TCP_CLEAR = 0.05            # m: both TCPs this far from every part of the cell, the cap and the eyepiece
STILL_MOVE = 0.002          # m: moved less than this over the last 1 s


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _ring(r: float, y: float, n: int = 24) -> np.ndarray:
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[r * np.cos(a), np.full(n, y), r * np.sin(a)]


def _cell_points() -> np.ndarray:
    """Points on the cell's outside (both ends of its skirt, its lip and its thumbscrew's head), local frame."""
    pts = [_ring(S.CELL_RO + S.CELL_FLARE, S.CELL_Y0), _ring(S.CELL_RO, (S.CELL_Y0 + S.CELL_Y1) / 2), _ring(S.CELL_RO, S.CELL_Y1),
           _ring(S.LIP_RI, S.LIP_Y1, 16), np.zeros((1, 3))]
    for r in (S.HEAD_R0, S.HEAD_TOP):
        pts.append(np.array([[0.0, S.SCREW_Y, r]]))
    return np.vstack(pts)


def _cap_points() -> np.ndarray:
    return np.vstack([_ring(S.CAP_RO, S.CAP_Y0, 16), _ring(S.CAP_RO, 0.0, 16), _ring(S.CAP_RO, S.CAP_END, 16), np.zeros((1, 3))])


def _eyepiece_points() -> np.ndarray:
    """Points on the eyepiece's outside, its own frame (z up its axis from the shoulder)."""
    a = np.linspace(0, 2 * np.pi, 8, endpoint=False)
    rings = [np.c_[r * np.cos(a), r * np.sin(a), np.full(8, z)] for r, z in
             ((S.BARREL_R, -S.BARREL_L), (S.EP_BODY_R, 0.0), (S.EP_BODY_R, S.EP_TOP))]
    return np.vstack(rings + [np.array([[0.0, 0.0, S.EP_TOP]])])


CELL_PTS, CAP_PTS, EP_PTS = _cell_points(), _cap_points(), _eyepiece_points()


def _angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0))))


def fit(state, home, gap_max: float, square_deg: float) -> dict:
    """How the cell (or cap) sits on its tube: along the axis from home (+ is out), off the axis, and its tilt."""
    rel = np.asarray(state.pos, float) - np.asarray(home, float)
    axis = _rot(state.quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    gap, off, tilt = float(rel[1]), float(math.hypot(rel[0], rel[2])), _angle_deg(axis, np.array([0.0, 1.0, 0.0]))
    return {"gap": gap, "off": off, "tilt": tilt, "on": bool(-0.003 <= gap <= gap_max and off <= OVER and tilt <= square_deg)}


def seated(pos, quat) -> dict:
    """The eyepiece in the holder: its shoulder no more than 2 mm above the holder's top, its axis within 5 degrees."""
    p = np.asarray(pos, float)
    up = _rot(quat) @ np.array([0.0, 0.0, 1.0])
    rise = float(p[2] - EP0[2])
    lat = float(math.hypot(p[0] - HOLDER_XY[0], p[1] - HOLDER_XY[1]))
    tilt = _angle_deg(up, np.array([0.0, 0.0, 1.0]))
    return {"rise": rise, "lat": lat, "tilt": tilt, "on": bool(-0.002 <= rise <= SEAT_UP and lat <= 0.003 and tilt <= SEAT_TILT)}


def in_holder(filter_pos) -> bool:
    """The SUN filter lying inside the holder's bore (in the light path), under or instead of the eyepiece."""
    p = np.asarray(filter_pos, float)
    return bool(math.hypot(p[0] - HOLDER_XY[0], p[1] - HOLDER_XY[1]) <= IN_HOLDER_R
                and TOP + S.HOLDER_Z[0] - 0.005 <= p[2] <= TOP + S.HOLDER_Z[1])


def _world(points: np.ndarray, pos, quat) -> np.ndarray:
    return np.asarray(pos, float) + points @ _rot(quat).T


# -- the thread rules -------------------------------------------------------------------------------------------------------
_R: dict = {}        # the rules' state (reset with the tracker)
_IDS: dict = {}


def _ids(asset, key, names):
    if key not in _IDS:
        _IDS[key] = asset.find_joints(list(names), preserve_order=True)[0]
    return _IDS[key]


def _gains(asset, key, names, stiff, damp, effort) -> None:
    import torch

    k = (tuple(np.round(stiff, 6)), tuple(np.round(damp, 6)), tuple(np.round(effort, 6)))
    if _R.get(f"gains_{key}") == k:
        return
    ids = _ids(asset, key, names)
    dev = asset.device
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([list(stiff)], dtype=torch.float32, device=dev), joint_ids=ids)
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([list(damp)], dtype=torch.float32, device=dev), joint_ids=ids)
    asset.write_joint_effort_limit_to_sim_index(limits=torch.tensor([list(effort)], dtype=torch.float32, device=dev), joint_ids=ids)
    _R[f"gains_{key}"] = k


def _joints(asset, key, names) -> np.ndarray:
    return _np(asset.data.joint_pos)[0][_ids(asset, key, names)]


def _screw_rule(env) -> None:
    """The thumbscrew, every control step. Its slide's target follows its own turn on the cell: 1.27 mm in per turn
    clockwise (seen from the head), clipped to its 5 mm of travel. Where the dew shield stops the tip, the slide's
    drive presses it with stiffness x shortfall, up to 12 N; tight at 10 N."""
    asset = env.unwrapped.scene["solar_filter"]
    names = ("j_adv", "j_spin")
    q = _joints(asset, "screw", names)
    R = _R
    prev = R.get("spin_prev")
    d = 0.0 if prev is None else _wrap(float(q[1]) - prev)
    R["spin_prev"] = float(q[1])
    actual_in = -float(q[0])
    turns = R.get("turns", 0.0) - d / (2 * math.pi)                  # clockwise from the head is -j_spin
    turns = min(max(turns, -0.0005 / S.PITCH), (actual_in + F_MAX / K_ADV) / S.PITCH)   # a blocked tip winds no further
    R["turns"] = turns
    target_in = min(max(S.PITCH * turns, -0.0005), S.SCREW_TRAVEL)
    force = min(F_MAX, max(0.0, K_ADV * (target_in - actual_in)))
    R["force"], R["actual_in"], R["target_in"] = force, actual_in, target_in
    # The thread's grip on the screw: it sticks until about 0.002 N m turns it (static friction), so the cell hanging
    # from the head keeps its place on the screw; once tight, the screw sticks until 0.05 N m. Past its grip it slips,
    # and sticks again where it is (the anchor moves to it). The joint's reading wraps at +-pi, so the drive aims at the
    # anchor the short way round from where it is.
    tight = force >= F_TIGHT or (R.get("tight_mode", False) and force >= F_TIGHT - 2.0)
    R["tight_mode"] = tight
    k, cap, slip = (K_HOLD, T_HOLD, SLIP_HOLD) if tight else (K_STICK, T_STICK, SLIP_STICK)
    anchor = R.get("anchor")
    if anchor is None or abs(_wrap(float(q[1]) - anchor)) > slip:
        anchor = float(q[1])
    R["anchor"] = anchor
    _gains(asset, "screw", names, [K_ADV, k], [D_ADV, D_SPIN], [F_MAX, cap])
    set_joint_targets(env, "solar_filter", {"j_adv": -target_in, "j_spin": float(q[1]) + _wrap(anchor - float(q[1]))})


def _filter_rel(q) -> tuple[np.ndarray, float]:
    """The SUN filter's thread face in the eyepiece's frame, and its tilt from the eyepiece's axis (degrees)."""
    p = OFF + np.asarray(q[:3], float)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(q[3]) * math.cos(q[4])))))
    return p, tilt


def _thread_rule(env) -> None:
    """The SUN filter's thread, every control step. Inside: its spigot in the barrel's thread bore (on the axis within
    1 mm, within 5 degrees, at least 2.5 mm in). While inside, the net turn of the eyepiece against the filter in the
    tightening direction (the eyepiece clockwise seen from its eye end) is counted; leaving before a catch resets it. At a
    net full turn the thread catches: drives hold the filter on the eyepiece's axis at that depth, with a damper as the
    thread's friction, and it goes where the eyepiece goes. Turned back out (below 330 degrees), it is free again."""
    asset = env.unwrapped.scene["eyepiece"]
    q = _joints(asset, "chain", CHAIN)
    R = _R
    p, tilt = _filter_rel(q)
    lat = float(math.hypot(p[0], p[1]))
    depth = float(p[2] + S.SPIGOT_H + S.BARREL_L)          # how far the spigot's end is into the barrel
    inside = lat <= IN_LAT and tilt <= IN_TILT and depth >= IN_DEPTH and p[2] <= -S.BARREL_L + 0.001
    prev = R.get("s_prev")
    d = 0.0 if prev is None else _wrap(float(q[5]) - prev)
    R["s_prev"] = float(q[5])
    caught = R.get("caught", False)
    if inside or caught:
        R["count"] = R.get("count", 0.0) + d
    else:
        R["count"] = 0.0
    if not caught and R["count"] >= CATCH_TURN:
        caught = True
        R["z_hold"] = float(min(p[2], -S.BARREL_L + 0.0005))
        R["caught_at"] = R.get("step", 0)
        print(f"[scope] the SUN filter's thread caught at step {R.get('step', 0)} ({math.degrees(R['count']):.0f} deg in, "
              f"{1000 * lat:.2f} mm off the axis, {tilt:.2f} deg)", flush=True)
    elif caught and R["count"] < RELEASE_TURN:
        caught = False
        print(f"[scope] the SUN filter turned back out at step {R.get('step', 0)}", flush=True)
    R["caught"], R["inside"], R["lat"], R["depth"], R["tilt"] = caught, bool(inside), lat, depth, tilt
    if caught:
        _gains(asset, "chain", CHAIN, [K_LIN] * 3 + [K_ANG] * 2 + [0.0], [D_LIN] * 3 + [D_ANG] * 2 + [D_THREAD], [1.0e4] * 6)
        set_joint_targets(env, "eyepiece", {"j_x": -OFF[0], "j_y": -OFF[1], "j_z": R["z_hold"] - OFF[2], "j_a": 0.0, "j_b": 0.0})
    else:
        _gains(asset, "chain", CHAIN, [0.0] * 6, [0.0] * 6, [1.0e4] * 6)


# -- tracker -------------------------------------------------------------------------------------------------------------------
_T: dict = {}


@before_reset
def _before(env) -> None:
    """Put the thumbscrew and the SUN filter's chain back at 0, free, before the scene settles."""
    try:
        _R.clear()
        for name, key, names in (("solar_filter", "screw", ("j_adv", "j_spin")), ("eyepiece", "chain", CHAIN)):
            asset = env.unwrapped.scene[name]
            if key == "screw":
                _gains(asset, key, names, [K_ADV, K_STICK], [D_ADV, D_SPIN], [F_MAX, T_STICK])
            else:
                _gains(asset, key, names, [0.0] * 6, [0.0] * 6, [1.0e4] * 6)
            set_joint_targets(env, name, {n: 0.0 for n in names})
    except Exception as e:  # pragma: no cover
        print(f"[scope] before reset: {e!r}", flush=True)


@on_reset
def _reset(_env=None) -> None:
    """Only a reset clears the tracker and the rules' state."""
    _T.clear()
    _T.update(step=0, held=0, hist=collections.deque(maxlen=HZ + 1), state={})
    _R.clear()


_reset()


def update(cell, cap, ep_pos, ep_quat, filt_pos, tcps) -> dict:
    _T["step"] += 1
    f = fit(cell, CELL_HOME, FILTER_GAP, FILTER_SQUARE_DEG)
    c = fit(cap, CAP_HOME, CAP_GAP, CAP_SQUARE_DEG)
    e = seated(ep_pos, ep_quat)
    force = float(_R.get("force", 0.0))
    tight = bool(force >= F_SECURED)
    fitted = bool(_R.get("caught", False) or in_holder(filt_pos))
    pts = np.vstack([_world(CELL_PTS, cell.pos, cell.quat_xyzw), _world(CAP_PTS, cap.pos, cap.quat_xyzw),
                     _world(EP_PTS, ep_pos, ep_quat)])
    tcp_d = [float(np.linalg.norm(pts - np.asarray(t, float), axis=1).min()) for t in tcps]
    _T["hist"].append((np.asarray(cell.pos, float).copy(), np.asarray(cap.pos, float).copy(), np.asarray(ep_pos, float).copy()))
    moved = None
    if len(_T["hist"]) > HZ:   # the farthest any of the three has been from where it is now, over the last 1 s
        now = _T["hist"][-1]
        moved = max(float(np.linalg.norm(now[i] - h[i])) for h in _T["hist"] for i in range(3))
    s = {"filter": f, "cap": c, "eyepiece": e, "force": force, "tight": tight, "fitted": fitted, "tcp_d": tcp_d,
         "released": bool(min(tcp_d) >= TCP_CLEAR), "moved_1s": moved, "still": bool(moved is not None and moved < STILL_MOVE)}
    s["full"] = bool(not fitted and f["on"] and tight and c["on"] and e["on"] and s["released"] and s["still"])
    _T["held"] = _T["held"] + 1 if s["full"] else 0
    _T["state"] = s
    return s


def _body(asset, name: str, origin):
    i = list(asset.body_names).index(name)
    pose = _np(asset.data.body_link_pose_w)[0][i]
    return pose[:3] - origin, pose[3:7]


@on_step
def _monitor(environment) -> None:
    from yam_arena.runtime import tcp_world
    from yam_arena.scene import object_states

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    _R["step"] = _T["step"] + 1
    _screw_rule(environment)
    _thread_rule(environment)
    tcps = [tcp_world(base.scene[f"{side}_robot"]) - origin for side in ("left", "right")]
    st = object_states(environment, ["solar_filter", "finder_cap"])
    ep = base.scene["eyepiece"]
    ep_pos, ep_quat = _body(ep, "eyepiece", origin)
    f_pos, f_quat = _body(ep, "filter", origin)
    s = update(st["solar_filter"], st["finder_cap"], ep_pos, ep_quat, f_pos, tcps)
    out = os.environ.get("TASK_SCORE_DIR")
    if out:
        os.makedirs(out, exist_ok=True)
        r = lambda v, k=4: [round(float(x), k) for x in v]  # noqa: E731
        rnd = lambda d: {k: (round(v, 4) if isinstance(v, float) else v) for k, v in d.items()}  # noqa: E731
        rec = {"step": _T["step"], "held": _T["held"], "full": s["full"], "score": _score(),
               "filter": rnd(s["filter"]), "cap": rnd(s["cap"]), "eyepiece": rnd(s["eyepiece"]),
               "force": round(s["force"], 2), "tight": s["tight"], "screw_in_mm": round(1000 * _R.get("actual_in", 0.0), 3),
               "screw_turns": round(_R.get("turns", 0.0), 3), "screw_tight_mode": _R.get("tight_mode", False),
               "fitted": s["fitted"], "caught": _R.get("caught", False), "inside": _R.get("inside", False),
               "count_deg": round(math.degrees(_R.get("count", 0.0)), 1), "thread_lat_mm": round(1000 * _R.get("lat", 0.0), 2),
               "thread_depth_mm": round(1000 * _R.get("depth", 0.0), 2),
               "released": s["released"], "tcp_d": [round(v, 3) for v in s["tcp_d"]], "still": s["still"],
               "moved_1s_mm": None if s["moved_1s"] is None else round(s["moved_1s"] * 1000, 2),
               "cell": r(st["solar_filter"].pos), "cell_q": r(st["solar_filter"].quat_xyzw), "cap_pos": r(st["finder_cap"].pos),
               "ep": r(ep_pos), "sun_filter": r(f_pos), "tcpL": r(tcps[0]), "tcpR": r(tcps[1])}
        with open(os.path.join(out, "trace.jsonl"), "a") as fh:
            fh.write(json.dumps(rec) + "\n")


def _score() -> float:
    s = _T["state"]
    if not s or s["fitted"]:
        return 0.0
    if s["full"] and _T["held"] >= int(HOLD_S * HZ):
        return 1.0
    if s["filter"]["on"]:
        return 0.5 if s["tight"] else 0.25
    return 0.0


# There is deliberately no success(): the harness would end the trial once it had held for 1 s, and a SUN filter screwed
# into the eyepiece after that would go unscored. Every trial runs to the run limit, and task_score takes score() at its end.
def score(objects) -> float:
    """The score at this step; the last step's is the trial's. 0 if the SUN filter is screwed into the eyepiece or lies
    in the holder. Otherwise 1.0 with the front filter on and tightened, the finder capped, the eyepiece seated, both TCPs
    5 cm clear of the cell, the cap and the eyepiece and the three still, held over the last 1 s; 0.5 with the front
    filter on and tightened; 0.25 with it on but not tightened; else 0."""
    steptime.tick()  # the harness calls score() once per control step
    return _score()


# -- scripted demo -------------------------------------------------------------------------------------------------------------
PAD = 0.013                                   # the fingertips' lower edge below the TCP
YAW0, YAWX = 0.0, -math.pi / 2                # fingers closing along y; along x (the left arm, turned clockwise)
CAP_GRIP = CAP_START[2] + 0.001               # TCP: fingertips 1.2 cm below the cap's axis, 4 mm above the foam
CAP_SEAT_TCP = CAP_HOME[2] + 0.001
CELL_GRIP = CELL_START[2] + S.HEAD_R0 - 0.001 + PAD   # fingertips 1 mm below the thumbscrew head's inner face
CELL_SEAT_TCP = CELL_HOME[2] + S.HEAD_R0 - 0.001 + PAD
CELL_TCP_DY = S.SCREW_Y                       # the TCP sits over the head, 1 mm behind the cell's origin
CAP_OPEN, CELL_OPEN = -0.036, -0.026          # finger targets: pads about 7 cm and 5 cm apart
CAP_PINCH = -0.021                            # pads 4.2 cm apart: a firm, gentle squeeze across the 4.9 cm cap
PARK_L = (0.25, 0.33, TOP + 0.21)
H = TOP
TURN_A, TURN_B, TURN_C = math.radians(90.0), math.radians(-20.0), math.radians(-130.0)   # one grasp: 220 deg clockwise


def _wp(t, x, y, z, finger, label, yaw=YAW0) -> Waypoint:
    return Waypoint(t, (float(x), float(y), float(z)), finger, label, yaw)


def cap_the_finder() -> list[Waypoint]:
    x0, y0 = CAP_START[:2]
    ys = CAP_HOME[1] + 0.024                  # start of the push: the cap's mouth 9 mm short of the objective cell
    return [_wp(2.0, x0, y0, H + 0.12, CAP_OPEN, "above the finder cap"),
            _wp(1.5, x0, y0, CAP_GRIP, CAP_OPEN, "down round the cap"),
            _wp(0.8, x0, y0, CAP_GRIP, CAP_PINCH, "pinch it across"),
            _wp(1.2, x0, y0, H + 0.13, CAP_PINCH, "lift it"),
            _wp(2.5, 0.22, ys, H + 0.19, CAP_PINCH, "carry it, turning its mouth toward the finder", YAWX),
            _wp(1.5, TX, ys, H + 0.19, CAP_PINCH, "over the finder's front", YAWX),
            _wp(1.2, TX, ys, CAP_SEAT_TCP, CAP_PINCH, "down onto the finder's axis", YAWX),
            _wp(2.0, TX, CAP_HOME[1] - 0.003, CAP_SEAT_TCP, CAP_PINCH, "push it on", YAWX),
            _wp(0.4, TX, CAP_HOME[1] - 0.003, CAP_SEAT_TCP, CAP_PINCH, "hold", YAWX),
            _wp(0.8, TX, CAP_HOME[1] - 0.003, CAP_SEAT_TCP, CAP_OPEN, "let go", YAWX),
            _wp(1.2, TX, CAP_HOME[1] + 0.040, CAP_SEAT_TCP, CAP_OPEN, "back off along the finder, the fingers either side of the cap", YAWX),
            _wp(1.0, TX, CAP_HOME[1] + 0.040, H + 0.20, CAP_OPEN, "straight up", YAWX)]


def fit_the_filter(depth: float = 1.0) -> list[Waypoint]:
    """Carry the cell by its thumbscrew's head and slide it over the dew shield; ends still holding the head. depth 1.0
    pushes the cell home."""
    x0, y0 = CELL_START[:2]
    yc_start = CELL_HOME[1] + 0.036            # the cell's mouth 6 mm short of the dew shield's rim
    yc_end = CELL_HOME[1] - 0.003 + (1.0 - depth) * 0.030
    return [_wp(2.0, x0, y0, H + 0.17, CELL_OPEN, "above the filter's thumbscrew"),
            _wp(1.5, x0, y0, CELL_GRIP, CELL_OPEN, "down to the thumbscrew"),
            _wp(0.8, x0, y0, CELL_GRIP, FINGER_CLOSED, "pinch its head"),
            _wp(1.5, x0, y0, H + 0.19, FINGER_CLOSED, "lift the filter out of its slot"),
            _wp(3.0, 0.20, yc_start + 0.012 + CELL_TCP_DY, H + 0.19, FINGER_CLOSED, "carry it, turning its open side toward the tube", YAWX),
            _wp(2.0, TX, yc_start + 0.012 + CELL_TCP_DY, CELL_SEAT_TCP, FINGER_CLOSED, "in line with the tube", YAWX),
            _wp(1.0, TX, yc_start + CELL_TCP_DY, CELL_SEAT_TCP, FINGER_CLOSED, "up to the dew shield", YAWX),
            _wp(3.0, TX, yc_end + CELL_TCP_DY, CELL_SEAT_TCP, FINGER_CLOSED, "slide it over the dew shield", YAWX),
            _wp(0.5, TX, yc_end + CELL_TCP_DY, CELL_SEAT_TCP, FINGER_CLOSED, "hold", YAWX)]


def release_and_park(y_at: float) -> list[Waypoint]:
    return [_wp(1.0, TX, y_at, CELL_SEAT_TCP, CELL_OPEN, "let go", YAWX),
            _wp(1.2, TX, y_at, H + 0.20, CELL_OPEN, "straight up", YAWX),
            _wp(2.0, *PARK_L, CELL_OPEN, "park, clear of the telescope", YAWX)]


def tighten(grasps: int = 4, from_hold: bool = True) -> list[Waypoint]:
    """Turn the thumbscrew clockwise (seen from above) in four grasps of 220 degrees, re-grasping between them: it
    goes tight (10 N) during the fourth."""
    x, y, z = TX, CELL_HOME[1] + CELL_TCP_DY, CELL_SEAT_TCP
    zu = z + 0.015
    out = []
    if from_hold:
        out += [_wp(0.8, x, CELL_HOME[1] - 0.003 + CELL_TCP_DY, z, CELL_OPEN, "let go of the head", YAWX),
                _wp(0.6, x, y, zu, CELL_OPEN, "up off the head", YAWX)]
    else:
        out += [_wp(2.0, x, y, H + 0.20, CELL_OPEN, "above the thumbscrew", YAWX), _wp(1.0, x, y, zu, CELL_OPEN, "down", YAWX)]
    out += [_wp(1.2, x, y, zu, CELL_OPEN, "turn the open hand", 0.0),
            _wp(1.2, x, y, zu, CELL_OPEN, "to the start of a turn", TURN_A)]
    for k in range(grasps):
        out += [_wp(0.6, x, y, z, CELL_OPEN, "down round the thumbscrew's head", TURN_A),
                _wp(0.6, x, y, z, FINGER_CLOSED, "pinch the head", TURN_A),
                _wp(2.0, x, y, z, FINGER_CLOSED, "turn it clockwise", TURN_B),
                _wp(2.0, x, y, z, FINGER_CLOSED, "turn it clockwise", TURN_C),
                _wp(0.5, x, y, z, CELL_OPEN, "let go", TURN_C)]
        if k < grasps - 1:
            out += [_wp(0.5, x, y, zu, CELL_OPEN, "up", TURN_C),
                    _wp(1.2, x, y, zu, CELL_OPEN, "turn the open hand back", TURN_B),
                    _wp(1.2, x, y, zu, CELL_OPEN, "turn the open hand back", TURN_A)]
    return out + [_wp(1.0, x, y, H + 0.20, CELL_OPEN, "straight up", TURN_C),
                  _wp(2.0, *PARK_L, CELL_OPEN, "park, clear of the telescope", YAWX)]


def _wait(t: float, where=PARK_L, yaw=YAWX) -> list[Waypoint]:
    return [_wp(t, *where, FINGER_OPEN, "wait", yaw)]


# The careless path: the right arm lifts the eyepiece out, lowers it over the SUN filter's thread, turns it in until the
# thread catches, and puts it back with the filter screwed in.
EP_GRIP_DZ = 0.003 + PAD                      # TCP above the eyepiece's shoulder: fingertips 3 mm above it, on its body
EP_OPEN = -0.036
Y_HOLDER, Y_F0, Y_F1, Y_F2 = math.radians(90.0), math.radians(165.0), math.radians(55.0), math.radians(-55.0)
AWAY_R = (EP0[0] - 0.10, EP0[1] - 0.10, H + 0.20)   # the right arm's place aside, clear of the telescope


def install_sun_filter(grasps: int = 4) -> list[Waypoint]:
    ex, ey = EP0[:2]
    fx, fy = FILT0[:2]
    grip = EP0[2] + EP_GRIP_DZ
    on_f = FILT0[2] + S.BARREL_L + EP_GRIP_DZ - 0.001      # the barrel's end 1 mm into the filter's face (a light push)
    out = [_wp(2.0, ex, ey, H + 0.20, EP_OPEN, "above the eyepiece", Y_HOLDER),
           _wp(1.5, ex, ey, grip, EP_OPEN, "down round the eyepiece", Y_HOLDER),
           _wp(0.8, ex, ey, grip, FINGER_CLOSED, "grip it", Y_HOLDER),
           _wp(1.5, ex, ey, H + 0.20, FINGER_CLOSED, "lift it out of the holder", Y_HOLDER),
           _wp(1.5, (ex + fx) / 2, (ey + fy) / 2, H + 0.20, FINGER_CLOSED, "carry it over the SUN filter", math.radians(128.0)),
           _wp(1.5, fx, fy, on_f + 0.02, FINGER_CLOSED, "over the SUN filter", Y_F0),
           _wp(1.5, fx, fy, on_f, FINGER_CLOSED, "down over its thread", Y_F0)]
    for k in range(grasps):
        if k:
            out += [_wp(0.5, fx, fy, on_f + 0.012, EP_OPEN, "up", Y_F2),
                    _wp(1.2, fx, fy, on_f + 0.012, EP_OPEN, "turn the open hand back", Y_F1),
                    _wp(1.2, fx, fy, on_f + 0.012, EP_OPEN, "turn the open hand back", Y_F0),
                    _wp(0.6, fx, fy, on_f, EP_OPEN, "down round the eyepiece", Y_F0),
                    _wp(0.6, fx, fy, on_f, FINGER_CLOSED, "grip it", Y_F0)]
        out += [_wp(2.0, fx, fy, on_f, FINGER_CLOSED, "turn it clockwise", Y_F1),
                _wp(2.0, fx, fy, on_f, FINGER_CLOSED, "turn it clockwise", Y_F2)]
        if k < grasps - 1:
            out += [_wp(0.5, fx, fy, on_f, EP_OPEN, "let go", Y_F2)]
    seat = EP0[2] + EP_GRIP_DZ + 0.001
    return out + [_wp(1.5, fx, fy, H + 0.20, FINGER_CLOSED, "lift it, the filter screwed on", Y_F2),
                  _wp(2.5, ex, ey, H + 0.20, FINGER_CLOSED, "carry it back over the holder", Y_F2),
                  _wp(1.5, ex, ey, seat + 0.03, FINGER_CLOSED, "down to the holder", Y_F2),
                  _wp(2.0, ex, ey, seat, FINGER_CLOSED, "into the holder", Y_F2),
                  _wp(0.8, ex, ey, seat, EP_OPEN, "let go", Y_F2),
                  _wp(1.0, ex, ey, H + 0.20, EP_OPEN, "straight up", Y_F2),
                  _wp(1.5, *AWAY_R, EP_OPEN, "away", 0.0)]


def _to_run_limit(plans: dict[str, list[Waypoint]]) -> dict[str, list[Waypoint]]:
    """Hold each arm's last pose to the run limit, so that the scripted policy does not stop the trial early: every
    trial runs its full length and its end state is scored."""
    run_s = RUN_STEPS / HZ
    for arm, plan in plans.items():
        left = run_s - sum(w.duration for w in plan)
        if left > 0:
            last = plan[-1]
            plan.append(Waypoint(left, last.pos, last.finger, "wait to the run limit", last.yaw))
    return plans


SMOKE = ("smoke_screw", "smoke_fit", "smoke_thread", "seated")


def demo():
    """Open loop, so it uses nothing a policy could not see. Every scored variant runs to the run limit. TASK_DEMO picks
    one: success (the left arm caps the finder, fits the filter and tightens its thumbscrew), install (while the left
    arm does the success plan, the right arm screws the SUN filter into the eyepiece and puts the eyepiece back), loose (cap and filter on, the
    thumbscrew never turned), tight_open (the filter fitted and tightened, the finder left open), nothing. Short checks,
    not held to the run limit: smoke_screw (the cell starts on the dew shield; three grasps tighten it), smoke_fit (fit
    and tighten), smoke_thread (the right arm's screwing-in alone), seated (the parts start in place, at rest)."""
    plans = _plan()
    return plans if MODE in SMOKE else _to_run_limit(plans)


def _plan() -> dict[str, list[Waypoint]]:
    if MODE in ("nothing", "seated"):
        return {"left": _wait(2.0 if MODE == "nothing" else 30.0)}
    if MODE == "smoke_screw":
        return {"left": tighten(from_hold=False) + _wait(3.0)}
    if MODE == "smoke_fit":
        return {"left": fit_the_filter() + tighten() + _wait(3.0)}
    if MODE == "smoke_thread":
        return {"right": install_sun_filter() + [_wp(3.0, *AWAY_R, EP_OPEN, "wait", 0.0)]}
    if MODE == "loose":
        return {"left": cap_the_finder() + fit_the_filter() + release_and_park(CELL_HOME[1] - 0.003 + CELL_TCP_DY) + _wait(4.0)}
    if MODE == "tight_open":
        return {"left": fit_the_filter() + tighten() + _wait(4.0)}
    left = cap_the_finder() + fit_the_filter() + tighten() + _wait(4.0)
    if MODE == "install":
        # Once the left arm has taken the cap and the filter from the case and is at the telescope's front, the right arm
        # screws the SUN filter into the eyepiece and puts the eyepiece back, while the left arm tightens the thumbscrew.
        start = sum(w.duration for w in cap_the_finder() + fit_the_filter()[:5]) + 1.0
        right = [_wp(2.0, *AWAY_R, EP_OPEN, "aside", 0.0),
                 _wp(start - 2.0, *AWAY_R, EP_OPEN, "wait while the left arm takes the cap and the filter", 0.0)]
        return {"left": left, "right": right + install_sun_filter()}
    return {"left": left}
