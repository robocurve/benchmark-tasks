"""On a polished dark granite kitchen worktop stands a white marble mortar with four lugs round its rim, a small heap of
black peppercorns in its bowl. Its beech pestle stands on its broad end on a folded linen tea towel to the right.

Task: grind the peppercorns. Hold the mortar still with one hand (by one of its lugs); with the other take the pestle
and grind round and round at the bottom of the bowl until every peppercorn is ground to powder, then stand the pestle
back on its tea towel and take both arms back to where they started.

Why two hands: the worktop is polished and the mortar slides on it at about 0.4 N, while the pestle pressed onto the
bowl's floor and moved round drags it with a few newtons. Ground with nothing holding the mortar, the mortar slides and
turns along with the pestle, and the pestle only rubs the peppercorns while it moves against a still mortar.

Run it:  inspect-robots run --embodiment yam_arena -E task=grind_peppercorns_in_a_mortar_holding_it_still.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy,  # noqa: E402
                     on_reset, on_step, plate, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the worktop ------------------------------------------------------------------------------------------------------
WX0, WX1, WY0, WY1, WT_H = 0.160, 0.480, -0.265, 0.200, 0.010
CT = TOP + WT_H
# -- the mortar: origin at the middle of its foot ---------------------------------------------------------------------
MX, MY = 0.330, -0.050
M_RO, M_RI, M_H, M_FLOOR = 0.078, 0.058, 0.055, 0.015  # outer and inner radius, height, the bowl floor's height
LUG_L, LUG_W, LUG_T = 0.022, 0.024, 0.020
LUG_R = M_RO + LUG_L / 2 - 0.004  # a lug's middle, from the mortar's axis
LUG_A = math.pi / 4  # the lugs point front-left, back-left, back-right and front-right
N_CORNS = 12
POWDER_R = (0.015, 0.021, 0.027, 0.033, 0.039)
HEAP = 0.012  # the heap (and the grinding) sits this far from the middle, away from the held lug
HEAP_C = (HEAP * math.cos(-math.pi / 4), HEAP * math.sin(-math.pi / 4))
# -- the pestle: origin at the middle of its broad end ----------------------------------------------------------------
P_HEAD_R, P_HEAD_H, P_NECK_R, P_HANDLE_R, P_LEN = 0.019, 0.034, 0.016, 0.0115, 0.110
# -- the tea towel ----------------------------------------------------------------------------------------------------
TX, TY, T_SIZE, T_H = 0.290, -0.200, 0.100, 0.004
# -- the rules -------------------------------------------------------------------------------------------------------
TOUCH_Z = 0.008  # the pestle's end within this of the bowl's floor: grinding
STILL_MM, STILL_DEG = 1.5, 0.75  # a step's rubbing counts only if the mortar moved less than this in that step
ROUND_M = 2 * math.pi * 0.015  # one round of grinding: this much rubbing along the bowl's floor
CIRCLES = 5  # rounds of grinding to grind every peppercorn
TOWEL_R = 0.045

GRANITE, GRANITE_V = (0.11, 0.11, 0.12), (0.28, 0.28, 0.30)
MARBLE, MARBLE_D, VEIN = (0.93, 0.93, 0.91), (0.82, 0.82, 0.80), (0.62, 0.63, 0.66)
BEECH, BEECH_D = (0.80, 0.62, 0.40), (0.62, 0.45, 0.27)
PEPPER, POWDER = (0.10, 0.08, 0.07), (0.42, 0.36, 0.30)
LINEN, STRIPE = (0.90, 0.87, 0.80), (0.70, 0.22, 0.20)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _worktop() -> Body:
    shapes = [BoxShape((0.0, 0.0, WT_H / 2), (WX1 - WX0, WY1 - WY0, WT_H), GRANITE)]
    rng = np.random.default_rng(3)
    for _ in range(40):  # pale flecks in the polished granite
        shapes.append(BoxShape((rng.uniform(-0.15, 0.15), rng.uniform(-0.21, 0.21), WT_H + 0.0002),
                               (rng.uniform(0.004, 0.012), rng.uniform(0.003, 0.008), 0.0004), GRANITE_V,
                               collide=False, yaw=rng.uniform(0, math.pi)))
    return Body.build("worktop", pos=((WX0 + WX1) / 2, (WY0 + WY1) / 2, TOP), mass=1.0, kinematic=True,
                      color=GRANITE, shapes=shapes, friction=(0.10, 0.08), slick=True)


def _towel() -> Body:
    shapes = [BoxShape((0.0, 0.0, T_H / 2), (T_SIZE, T_SIZE, T_H), LINEN)]
    for s in (-1.0, 1.0):
        shapes.append(BoxShape((s * 0.036, 0.0, T_H + 0.0002), (0.008, T_SIZE, 0.0004), STRIPE, collide=False))
    return Body.build("towel", pos=(TX, TY, CT), mass=0.1, kinematic=True, color=LINEN, shapes=shapes,
                      friction=(0.9, 0.8))


_SHAPE: dict = {"corns": [], "powder": []}


def _mortar() -> Body:
    """A white marble mortar: a solid foot, a thick round wall, four lugs at the rim, peppercorns in the bowl."""
    shapes = [CylShape((0.0, 0.0, M_FLOOR / 2), M_RO, M_FLOOR, MARBLE)]
    n = 32
    rm = (M_RO + M_RI) / 2
    for k in range(n):
        a = 2 * math.pi * (k + 0.5) / n
        nrm = np.array([math.cos(a), math.sin(a), 0.0])
        shapes.append(plate(rm * nrm + [0, 0, M_FLOOR - 0.001], rm * nrm + [0, 0, M_H],
                            2 * M_RO * math.tan(math.pi / n) + 0.001, M_RO - M_RI, nrm, MARBLE))
    for k in range(4):  # rounded lugs
        a = LUG_A + k * math.pi / 2
        shapes.append(BoxShape(((LUG_R - 0.004) * math.cos(a), (LUG_R - 0.004) * math.sin(a), M_H - LUG_T / 2),
                               (LUG_L - 0.008, LUG_W, LUG_T), MARBLE, yaw=a))
        shapes.append(CylShape(((LUG_R + LUG_L / 2 - LUG_W / 2) * math.cos(a), (LUG_R + LUG_L / 2 - LUG_W / 2)
                                * math.sin(a), M_H - LUG_T / 2), LUG_W / 2, LUG_T, MARBLE))
    rng = np.random.default_rng(11)
    for _ in range(9):  # grey veins on the outside of the marble
        a = rng.uniform(0, 2 * math.pi)
        z = rng.uniform(0.01, M_H - 0.008)
        shapes.append(BoxShape(((M_RO + 0.0003) * math.cos(a), (M_RO + 0.0003) * math.sin(a), z),
                               (0.0006, rng.uniform(0.012, 0.030), 0.0012), VEIN, collide=False,
                               quat_xyzw=_quat_zx(a, rng.uniform(-0.6, 0.6))))
    shapes.append(CylShape((0.0, 0.0, M_FLOOR + 0.0002), M_RI - 0.001, 0.0004, MARBLE_D, collide=False))  # the floor
    for i, r in enumerate(POWDER_R):  # the ground pepper, shown as it is ground (largest lowest)
        _SHAPE["powder"].append(len(shapes))
        shapes.append(CylShape((HEAP_C[0] * 0.5, HEAP_C[1] * 0.5, M_FLOOR + 0.0006 + 0.0005 * (len(POWDER_R) - i)), r,
                               0.0012 + 0.0004 * i, POWDER, collide=False))
    for k in range(N_CORNS):  # the peppercorns, a small heap in the middle of the bowl
        a = k * 2.4
        r = 0.006 + 0.0022 * k
        _SHAPE["corns"].append(len(shapes))
        shapes.append(SphereShape((HEAP_C[0] + r * math.cos(a), HEAP_C[1] + r * math.sin(a),
                                   M_FLOOR + 0.0035 + (0.003 if k < 3 else 0.0)),
                                  0.0034, PEPPER, collide=False))
    return Body.build("mortar", pos=(MX, MY, CT), mass=0.45, color=MARBLE, shapes=shapes, friction=(1.0, 0.9),
                      com=(0.0, 0.0, 0.020))


def _quat_zx(yaw: float, tilt: float):
    """A quaternion turning ``yaw`` about z, then tilting ``tilt`` about the turned x axis."""
    cz, sz, cx, sx = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
    return (cz * sx, sz * sx, sz * cx, cz * cx)


def _pestle() -> Body:
    """A beech pestle standing on its broad end: a thick head, a tapering neck, a handle and a rounded top."""
    shapes = [CylShape((0.0, 0.0, P_HEAD_H / 2), P_HEAD_R, P_HEAD_H, BEECH),
              CylShape((0.0, 0.0, 0.0015), P_HEAD_R - 0.0025, 0.003, BEECH_D, collide=False),
              CylShape((0.0, 0.0, P_HEAD_H + 0.006), P_NECK_R, 0.012, BEECH),
              CylShape((0.0, 0.0, (P_HEAD_H + 0.012 + P_LEN) / 2), P_HANDLE_R, P_LEN - P_HEAD_H - 0.012, BEECH),
              SphereShape((0.0, 0.0, P_LEN - 0.004), P_HANDLE_R + 0.001, BEECH),
              CylShape((0.0, 0.0, P_HEAD_H + 0.018), P_HANDLE_R + 0.0005, 0.003, BEECH_D, collide=False)]
    return Body.build("pestle", pos=(TX, TY, CT + T_H), mass=0.12, color=BEECH, shapes=shapes, friction=(0.30, 0.25),
                      com=(0.0, 0.0, 0.030))


_PATHS: dict = {}
_author_sliders_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    """bodykit's sliders, then: note where the mortar's peppercorns and powder are, and hide the powder."""
    _author_sliders_plain(stage, prim_path, spec)
    if spec.name == "mortar":
        from pxr import UsdGeom

        _PATHS["corns"] = [f"{prim_path}/body/s{i}" for i in _SHAPE["corns"]]
        _PATHS["powder"] = [f"{prim_path}/body/s{i}" for i in _SHAPE["powder"]]
        for p in _PATHS["powder"]:
            UsdGeom.Imageable(stage.GetPrimAtPath(p)).MakeInvisible()


bodykit._author_sliders = _author_sliders

OBJECTS = [_worktop(), _towel(), _mortar(), _pestle()]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"rubbed": 0.0, "circles": 0, "last": None, "shown": -1, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("MORTAR_DEBUG")), "steps": 0}


def _show(path: str, on: bool) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    prim = get_current_stage().GetPrimAtPath(path)
    if prim:
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()


def _show_ground(n: int) -> None:
    """Show the bowl after ``n`` rounds of grinding: fewer peppercorns, more powder."""
    if n == _EP["shown"] or "corns" not in _PATHS:
        return
    left = N_CORNS - round(N_CORNS * min(n, CIRCLES) / CIRCLES)
    for i, p in enumerate(_PATHS["corns"]):
        _show(p, i >= N_CORNS - left)
    for i, p in enumerate(_PATHS["powder"]):
        _show(p, i < min(n, CIRCLES))
    _EP["shown"] = n


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _show_ground(0)


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pm, rm = _pose(base, origin, "mortar")
    pp, rp = _pose(base, origin, "pestle")
    yaw = math.degrees(math.atan2(rm[1, 0], rm[0, 0]))
    q = rm.T @ (pp - pm)  # the pestle's end in the mortar's frame
    upright = float(rp[:, 2] @ rm[:, 2]) > math.cos(math.radians(35.0))
    grinding = q[2] < M_FLOOR + TOUCH_Z and math.hypot(q[0], q[1]) < M_RI and upright
    moved = turned = 0.0
    if _EP["last"] is not None:
        lq, lp, lyaw = _EP["last"]
        moved = float(np.linalg.norm((pm - lp)[:2])) * 1000.0
        turned = abs((yaw - lyaw + 180.0) % 360.0 - 180.0)
        if grinding and lq is not None and moved < STILL_MM and turned < STILL_DEG:
            _EP["rubbed"] += float(np.linalg.norm((q - lq)[:2]))  # rubbed along the floor of a still mortar
    _EP["last"] = (q.copy() if grinding else None, pm.copy(), yaw)
    _EP["circles"] = int(_EP["rubbed"] / ROUND_M)
    _show_ground(_EP["circles"])
    tilt_m = math.degrees(math.acos(max(-1.0, min(1.0, float(rm[2, 2])))))
    on_towel = (math.hypot(pp[0] - TX, pp[1] - TY) < TOWEL_R and pp[2] < CT + T_H + 0.01
                and float(rp[2, 2]) > math.cos(math.radians(20.0)))  # standing on its broad end on the towel
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"ground": _EP["circles"] >= CIRCLES, "some": _EP["circles"] >= 2, "on_towel": on_towel,
                "mortar_ok": tilt_m < 10.0 and WX0 < pm[0] < WX1 and WY0 < pm[1] < WY1}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0:
            mm = lambda v: np.round((np.asarray(v) - [0, 0, CT]) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            print(f"[mortar] t={_DEBUG['steps'] / 20:5.1f}s fingers={[round(v, 1) for v in fj]} mortar={mm(pm)}"
                  f" yaw={yaw:.1f} tilt={tilt_m:.1f} pestle={mm(pp)} end_in_bowl={np.round(q * 1000).astype(int).tolist()}"
                  f" grind={grinding} rubbed={_EP['rubbed'] * 1000:.0f}mm rounds={_EP['circles']}"
                  f" step_moved={moved:.1f}mm/{turned:.2f}deg towel={on_towel} L={mm(tl)} R={mm(tr)} score={score(None)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: every peppercorn ground, the pestle standing back on its tea towel, the mortar upright on the worktop and
    both arms back where they started; 0.5: every peppercorn ground; 0.25: some ground (two rounds); else 0.0. All
    judged at the end."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["ground"]:
        return 1.0 if s["on_towel"] and s["mortar_ok"] and _arms_home() else 0.5
    return 0.25 if s["some"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the mortar and the pestle at rest: the trial ends there."""
    return score(objects) >= 1.0 and objects["mortar"].resting() and objects["pestle"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand takes the mortar's left lug from above, its wrist leaning out to the left, and holds it. The right takes
# the pestle by its handle, lifts it over the rim, lowers it onto the bowl's floor and grinds round and round, pressing
# down. Then it lifts the pestle out and stands it back on its tea towel; the left lets go of the lug and both arms go
# back to where they started.

LUG_FINGER = -0.016  # pre-open only a little wider than the lug: a wide grip shoves the light mortar
LUG_ABOVE = 0.003  # the TCP this far above the lug's top: the lug pinched about 1 cm deep, near the fingertips
LUG_HOLD_R = LUG_R + 0.002
HOLD_A = LUG_A + math.pi / 2  # the left holds the back-left lug
LEFT_LEAN = math.radians(25.0)  # turned about its closing axis: the left's wrist leans out, back and left
HANDLE_FINGER = -0.024
GRIP_UP = 0.064  # where the right takes the pestle, from its broad end: its fingertips just above the neck's flare
PRESS = 0.0025  # aim the pestle's end this far into the floor: it presses down
ROUND_R = 0.018
ROUND_PTS, ROUND_S, ROUNDS = 16, 2.2, 8
OVER = 0.060
LEAD_S, LEAD_STEPS = 3.0, 15


def _about(axis, a: float) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot(arm: str) -> np.ndarray:
    """The right's grip closes along x, straight down; the left's closes across the back-left lug, its pads square
    to the lug's sides and its wrist leaning out (back and to the left), clear of the right."""
    if arm == "left":  # closing across the back-left lug, then leaned about the closing axis
        g = grip_rotation(HOLD_A - math.pi)
        return _about(g[:, 0], LEFT_LEAN) @ g
    return grip_rotation(math.pi / 2)


def _tcp_for(point, depth: float, arm: str) -> np.ndarray:
    return np.asarray(point, float) - depth * _rot(arm)[:, 2]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    up = np.array([0.0, 0.0, OVER])

    def go(arm, t, pos, finger, label, n=1):
        p0 = cur[arm][0]
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label,
                                      _rot(arm)))
        cur[arm] = (p1, finger)

    def wait(arm, t, label="wait"):
        p, f = cur[arm]
        plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, _rot(arm)))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def pestle_tcp(end):  # the right's TCP with the pestle's broad end at ``end``
        return np.asarray(end, float) + [0.0, 0.0, GRIP_UP]

    lug_top = np.array([MX + LUG_HOLD_R * math.cos(HOLD_A), MY + LUG_HOLD_R * math.sin(HOLD_A), CT + M_H])
    hold = _tcp_for(lug_top, LUG_ABOVE, "left")
    above = hold - 0.06 * _rot("left")[:, 2]
    plan["left"] += _joint_path("left", np.array(READY, float), _good_ik("left", above, _rot("left")),
                                "over the mortar's back-left lug", LUG_FINGER)
    cur["left"] = (above, LUG_FINGER)
    go("left", 1.2, hold, LUG_FINGER, "down round the lug")
    go("left", 0.6, hold, FINGER_CLOSED, "hold the mortar by its lug")

    stand = np.array([TX, TY, CT + T_H])
    take = pestle_tcp(stand)
    plan["right"] += _joint_path("right", np.array(READY, float), _good_ik("right", take + up, _rot("right")),
                                 "over the pestle", HANDLE_FINGER)
    cur["right"] = (take + up, HANDLE_FINGER)
    go("right", 1.2, take, HANDLE_FINGER, "down round its handle")
    go("right", 0.6, take, FINGER_CLOSED, "take the pestle")
    lifted = pestle_tcp(stand + [0.0, 0.0, M_H + 0.014])
    go("right", 1.2, lifted, FINGER_CLOSED, "lift it")
    over = pestle_tcp([MX + HEAP_C[0], MY + HEAP_C[1], CT + M_H + 0.014])
    go("right", 1.8, over, FINGER_CLOSED, "carry it over the mortar", n=3)
    wait("right", max(0.0, dur("left") - dur("right")), "wait for the mortar to be held")
    floor = np.array([MX + HEAP_C[0], MY + HEAP_C[1], CT + M_FLOOR - PRESS])
    go("right", 1.4, pestle_tcp(floor), FINGER_CLOSED, "lower it onto the bowl's floor and press down", n=2)
    go("right", 0.6, pestle_tcp(floor + [ROUND_R, 0.0, 0.0]), FINGER_CLOSED, "out to the edge of the heap")
    for k in range(ROUNDS):
        for j in range(1, ROUND_PTS + 1):
            a = 2 * math.pi * j / ROUND_PTS
            go("right", ROUND_S / ROUND_PTS,
               pestle_tcp(floor + [ROUND_R * math.cos(a), ROUND_R * math.sin(a), 0.0]), FINGER_CLOSED,
               f"grind round the bowl ({k + 1} of {ROUNDS})")
    go("right", 0.6, pestle_tcp(floor + [0.0, 0.0, 0.006]), FINGER_CLOSED, "ease off in the middle")
    go("right", 1.4, over, FINGER_CLOSED, "lift the pestle out of the mortar", n=2)
    go("right", 1.8, lifted, FINGER_CLOSED, "carry it back over its tea towel", n=3)
    go("right", 1.2, pestle_tcp(stand + [0.0, 0.0, 0.003]), FINGER_CLOSED, "stand it on its broad end")
    go("right", 0.5, pestle_tcp(stand + [0.0, 0.0, 0.003]), HANDLE_FINGER, "let go")
    go("right", 0.8, take + up, HANDLE_FINGER, "rise clear")
    wait("left", dur("right") - 2.6 - dur("left"), "hold the mortar")
    go("left", 0.5, hold, LUG_FINGER, "let go of the lug")
    go("left", 0.8, above, LUG_FINGER, "rise clear")
    wait("right", max(0.0, dur("left") - dur("right")))
    for arm in ("left", "right"):
        p, f = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, _rot(arm)), np.array(READY, float), "back to where it started",
                                 f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
