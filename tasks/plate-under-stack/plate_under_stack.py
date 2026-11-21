"""Take the dinner plate out from under the side plates stacked on it, holding them up meanwhile.

An open wall-cabinet carcass (no door) stands on the counter, its open front facing the robot. On its floor sits a
dinner plate with three side plates stacked on it, and two nested cereal bowls beside it. The rest of the floor is
too narrow for a side plate, and the counter mat in front of the cabinet is for the dinner plate. The task: put the
dinner plate on the mat and leave the three side plates stacked where they were, without ever setting them down
anywhere else or dropping them. One hand has to hold the side plates up while the other draws the dinner plate out.

Frame: x forward from the arm bases, y left, z up, metres. Plate bodies have their origin at the centre of the foot
ring's bottom, z along the plate's axis.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies are simulated)
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from bodykit import _quat_from_matrix  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0

WHITE, WHITE2 = (0.95, 0.95, 0.93), (0.90, 0.90, 0.88)
CARCASS, MAT_C = (0.93, 0.92, 0.89), (0.30, 0.42, 0.52)

# -- the cabinet: IKEA METOD wall cabinet frame 60 x 37 x 40 cm, 18 mm white particleboard, no door ---------------
X0 = 0.37  # its open front (world x)
CAB_W, CAB_D, CAB_H, BOARD, BACK = 0.60, 0.37, 0.40, 0.018, 0.016
SHELF = TOP + BOARD  # the cabinet floor's top face
IN_HALF_W = CAB_W / 2 - BOARD  # 0.282
IN_DEPTH = CAB_D - BACK  # 0.354


def _cabinet() -> Body:
    shapes = [BoxShape((CAB_D / 2, 0.0, BOARD / 2), (CAB_D, CAB_W, BOARD), CARCASS),
              BoxShape((CAB_D / 2, 0.0, CAB_H - BOARD / 2), (CAB_D, CAB_W, BOARD), CARCASS)]
    for s in (1.0, -1.0):
        shapes.append(BoxShape((CAB_D / 2, s * (CAB_W / 2 - BOARD / 2), CAB_H / 2), (CAB_D, BOARD, CAB_H - 2 * BOARD),
                               CARCASS))
    shapes.append(BoxShape((CAB_D - BACK / 2, 0.0, CAB_H / 2), (BACK, CAB_W - 2 * BOARD, CAB_H - 2 * BOARD), CARCASS))
    return Body.build("cabinet", pos=(X0, 0.0, TOP), mass=10.0, shapes=shapes, color=CARCASS, kinematic=True,
                      friction=(0.35, 0.30))


# -- the plates and bowls: Corelle Winter Frost White (Vitrelle glass), sizes and masses from the maker's pages -----
# name: (outer radius, foot radius, flat well radius, foot height, wall thickness, height at the rim's edge, mass)
DISH = {
    "dinner": (0.130, 0.080, 0.095, 0.003, 0.003, 0.019, 0.340),  # 10.25 in plate, 0.75 in tall, 12 oz
    "side": (0.0855, 0.050, 0.055, 0.003, 0.003, 0.022, 0.130),  # 6.75 in plate, 0.87 in tall, 4.6 oz
    "bowl": (0.0795, 0.040, 0.050, 0.006, 0.0035, 0.051, 0.180),  # 18 oz bowl, 6.25 in across, 2 in tall, 6.3 oz
}
N_RIM = {"dinner": 20, "side": 16, "bowl": 14}
MU_GLASS = (0.45, 0.40)


def rim_slope(kind: str) -> float:
    R, rf, rw, hf, t, he, m = DISH[kind]
    return math.atan2(he - (hf + t), R - rw)


def well_top(kind: str) -> float:
    """Height of the flat well's top face above the foot's bottom."""
    _, _, _, hf, t, _, _ = DISH[kind]
    return hf + t


def rim_top(kind: str, r: float) -> float:
    """Height of the rim's top face at radius r above the foot's bottom (on the flat well inside it)."""
    R, rf, rw, hf, t, he, m = DISH[kind]
    return hf + t + max(0.0, r - rw) * math.tan(rim_slope(kind))


def _dish_shapes(kind: str, color) -> list:
    """Foot, flat well and a ring of flat facets rising to the rim: the real profile, not a flat disc."""
    R, rf, rw, hf, t, he, m = DISH[kind]
    a = rim_slope(kind)
    n = N_RIM[kind]
    length = (R - rw) / math.cos(a)
    width = 2 * math.pi * R / n * 1.04
    shapes = [CylShape((0.0, 0.0, hf / 2), rf, hf, color), CylShape((0.0, 0.0, hf + t / 2), rw + 0.001, t, color)]
    for k in range(n):
        ph = 2 * math.pi * k / n
        e1 = np.array([math.cos(ph) * math.cos(a), math.sin(ph) * math.cos(a), math.sin(a)])
        e2 = np.array([-math.sin(ph), math.cos(ph), 0.0])
        e3 = np.cross(e1, e2)
        rm = (rw + R) / 2
        top_mid = np.array([rm * math.cos(ph), rm * math.sin(ph), (hf + t + he) / 2])
        c = top_mid - t / 2 * e3
        shapes.append(BoxShape(tuple(c.tolist()), (length, width, t), color,
                               quat_xyzw=tuple(_quat_from_matrix(np.column_stack([e1, e2, e3])))))
    return shapes


SIDES = ("side1", "side2", "side3")  # bottom to top
BOWLS = ("bowl1", "bowl2")  # bottom, top
SIDE_PITCH = well_top("side")  # a side plate's foot stands on the well of the one below: 6 mm per plate
BOWL_PITCH = well_top("bowl")  # 9.5 mm per bowl

# -- the counter mat: 28 x 28 cm, 2 mm silicone, drawn only (the plate is set on the counter through it) -----------
MAT = 0.28
MAT_NOM = (0.20, 0.0)

# -- nominal start (the setup text's layout; every seeded trial moves it, see layout()) -------------------------
DINNER_NOM = (X0 + 0.21, 0.065)  # bowls on the right
BOWL_GAP = (0.020, 0.030)  # bowl rim to dinner-plate rim


def _build() -> list:
    objs = [_cabinet(),
            Body.build("mat", pos=(MAT_NOM[0], MAT_NOM[1], TOP), mass=0.1, kinematic=True, color=MAT_C,
                       shapes=[BoxShape((0.0, 0.0, 0.0006), (MAT, MAT, 0.001), MAT_C, collide=False)]),
            Body.build("dinner", pos=(DINNER_NOM[0], DINNER_NOM[1], SHELF), mass=DISH["dinner"][6],
                       shapes=_dish_shapes("dinner", WHITE), color=WHITE, friction=MU_GLASS, solver_iterations=32)]
    zs = SHELF + well_top("dinner")
    for i, n in enumerate(SIDES):
        objs.append(Body.build(n, pos=(DINNER_NOM[0], DINNER_NOM[1], zs + i * SIDE_PITCH), mass=DISH["side"][6],
                               shapes=_dish_shapes("side", WHITE2 if i % 2 else WHITE), color=WHITE,
                               friction=MU_GLASS, solver_iterations=32))
    yb = DINNER_NOM[1] - (DISH["dinner"][0] + 0.025 + DISH["bowl"][0])
    for i, n in enumerate(BOWLS):
        objs.append(Body.build(n, pos=(DINNER_NOM[0], yb, SHELF + i * BOWL_PITCH), mass=DISH["bowl"][6],
                               shapes=_dish_shapes("bowl", WHITE2 if i else WHITE), color=WHITE, friction=MU_GLASS,
                               solver_iterations=32))
    return objs


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
DISHES = ["dinner", *SIDES, *BOWLS]

# -- thresholds (the scoring text, number for number) -------------------------------------------------------------
ZONE_M = 0.05  # a side plate's centre may never be more than 5 cm (horizontally) from where it started
STACK_APART_M = 0.025  # ... nor more than 2.5 cm from the side plate under it
FALL_TILT = 15.0  # degrees: a side plate tipped this far has fallen
DROP_VZ = 0.5  # m/s downward: a plate let fall about 1.3 cm or more
DROP_DZ = 0.012  # or one that sinks 12 mm within 0.1 s
CLEAR_M = 0.235  # the dinner plate's centre this far from the side plates' start: fully out from under them
MAT_TOL = 0.06  # the dinner plate's centre within 6 cm of the mat's centre
FLAT = 5.0  # degrees
REST_DINNER = 0.010  # its foot within 1 cm of the counter top
SPOT_M = 0.04  # each side plate within 4 cm of its start
STACK_M = 0.015  # and within 1.5 cm of the one below
REST_SIDE = 0.005  # the bottom side plate's foot within 5 mm of the cabinet floor
HANDS_CLEAR = 0.03  # every finger point at least 3 cm from every plate at the end
BOWL_MOVE = 0.01
POINTS = {"out": 0.35, "dinner_on_mat": 0.25, "sides_back": 0.25, "bowls": 0.15}

# -- episode state --------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["PLATE_DEBUG"], "w") if os.environ.get("PLATE_DEBUG") else None


def _blank() -> None:
    start, mat = _EP.get("start"), _EP.get("mat")
    _EP.clear()
    _EP.update(step=0, hands={}, report={}, zhist={n: [] for n in DISHES}, left_zone=None, apart=None, tipped=None,
               side_drop=None, dinner_drop=None, out=None, now=None, start=start, mat=mat or MAT_NOM, max_off=0.0,
               max_tilt=0.0)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def tilt_deg(state) -> float:
    return math.degrees(state.tilt())


def _hand_points(base) -> dict:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out[arm] = finger_points(p, r, half)
    return out


def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on both fingers of a gripper (TCP at p, rotation r, pads half m from the centre line)."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def dish_clearance(state, kind: str, points) -> float:
    """Smallest distance (m) from the points to a dish's bounding cylinder (radius R, foot to rim height)."""
    R, he = DISH[kind][0], DISH[kind][5]
    best = float("inf")
    for p in points:
        loc = state.to_local(p)
        dr = max(0.0, math.hypot(loc[0], loc[1]) - R)
        dz = max(0.0, -loc[2], loc[2] - he)
        best = min(best, math.hypot(dr, dz))
    return best


def _xy(state) -> np.ndarray:
    return np.asarray(state.pos, float)[:2]


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["now"] = st
    k = _EP["step"]
    start = _EP.get("start")
    for n in DISHES:
        h = _EP["zhist"][n]
        h.append(float(st[n].pos[2]))
        if len(h) > 3:
            h.pop(0)
    if start is not None and k >= 2:
        for i, n in enumerate(SIDES):
            s = st[n]
            off = float(np.linalg.norm(_xy(s) - start[n][0][:2]))
            _EP["max_off"] = max(_EP["max_off"], off)
            _EP["max_tilt"] = max(_EP["max_tilt"], tilt_deg(s))
            if off > ZONE_M and _EP["left_zone"] is None:
                _EP["left_zone"] = (k, n, round(off, 3))
            if i > 0 and np.linalg.norm(_xy(s) - _xy(st[SIDES[i - 1]])) > STACK_APART_M and _EP["apart"] is None:
                _EP["apart"] = (k, n)
            if tilt_deg(s) > FALL_TILT and _EP["tipped"] is None:
                _EP["tipped"] = (k, n, round(tilt_deg(s), 1))
            h = _EP["zhist"][n]
            if (float(s.lin_vel[2]) < -DROP_VZ or (len(h) == 3 and h[-1] - h[0] < -DROP_DZ)) and _EP["side_drop"] is None:
                _EP["side_drop"] = (k, n, round(float(s.lin_vel[2]), 3))
        d = st["dinner"]
        h = _EP["zhist"]["dinner"]
        if (float(d.lin_vel[2]) < -DROP_VZ or (len(h) == 3 and h[-1] - h[0] < -DROP_DZ)) and _EP["dinner_drop"] is None:
            _EP["dinner_drop"] = (k, round(float(d.lin_vel[2]), 3))
        if _EP["out"] is None and np.linalg.norm(_xy(d) - start["side1"][0][:2]) >= CLEAR_M and not _spoilt():
            _EP["out"] = k
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    _EP["step"] += 1
    if _DBG is not None and k % 2 == 0:
        _debug(st)


def _spoilt():
    """The first rule broken with the side plates (None if none): left their spot, came apart, tipped or dropped."""
    for key in ("left_zone", "apart", "tipped", "side_drop"):
        if _EP.get(key) is not None:
            return key
    return None


# -- scoring ----------------------------------------------------------------------------------------------------------
def report(objects) -> dict:
    start = _EP.get("start")
    pts = [p for arm in _EP.get("hands", {}).values() for p in arm]
    out: dict = {"score": 0.0, "parts": {}}
    if start is None:
        return out
    spoilt = _spoilt()
    out["side_plates_spoilt"] = spoilt and {spoilt: _EP[spoilt]}
    out["dinner_out_at_step"] = _EP.get("out")
    if spoilt or _EP.get("out") is None:
        return out
    parts = {"out": POINTS["out"]}
    d = objects["dinner"]
    mat = np.asarray(_EP["mat"], float)
    clr_d = dish_clearance(d, "dinner", pts) if pts else float("inf")
    dm = {"centre_to_mat_m": round(float(np.linalg.norm(_xy(d) - mat)), 3), "tilt": round(tilt_deg(d), 1),
          "foot_above_counter_m": round(float(d.pos[2]) - TOP, 4), "dropped": _EP.get("dinner_drop"),
          "hands_m": round(clr_d, 3)}
    if (dm["centre_to_mat_m"] <= MAT_TOL and dm["tilt"] < FLAT and abs(dm["foot_above_counter_m"]) <= REST_DINNER
            and dm["dropped"] is None and clr_d >= HANDS_CLEAR):
        parts["dinner_on_mat"] = POINTS["dinner_on_mat"]
    sm: dict = {"plates": []}
    ok = True
    for i, n in enumerate(SIDES):
        s = objects[n]
        e = {"from_start_m": round(float(np.linalg.norm(_xy(s) - start[n][0][:2])), 3), "tilt": round(tilt_deg(s), 1),
             "hands_m": round(dish_clearance(s, "side", pts) if pts else float("inf"), 3)}
        if i > 0:
            e["from_below_m"] = round(float(np.linalg.norm(_xy(s) - _xy(objects[SIDES[i - 1]]))), 3)
        ok &= e["from_start_m"] <= SPOT_M and e["tilt"] < FLAT and e.get("from_below_m", 0.0) <= STACK_M
        ok &= e["hands_m"] >= HANDS_CLEAR
        sm["plates"].append(e)
    sm["bottom_foot_above_floor_m"] = round(float(objects[SIDES[0]].pos[2]) - SHELF, 4)
    ok &= abs(sm["bottom_foot_above_floor_m"]) <= REST_SIDE
    if ok:
        parts["sides_back"] = POINTS["sides_back"]
    bm = []
    bok = True
    for n in BOWLS:
        b = objects[n]
        mv = float(np.linalg.norm(_xy(b) - start[n][0][:2]))
        bm.append({"moved_m": round(mv, 4), "tilt": round(tilt_deg(b), 1)})
        bok &= mv < BOWL_MOVE and tilt_deg(b) < FLAT
    nest = float(np.linalg.norm(_xy(objects[BOWLS[1]]) - _xy(objects[BOWLS[0]])))
    bok &= nest <= BOWL_MOVE
    if bok:
        parts["bowls"] = POINTS["bowls"]
    out.update(parts=parts, dinner=dm, sides=sm, bowls=bm, bowl_nest_m=round(nest, 4))
    out["score"] = round(min(1.0, sum(parts.values())), 2)
    return out


def score(objects) -> float:
    """0 if a side plate ever left its spot (5 cm), came apart from the one below (2.5 cm), tipped past 15 degrees or
    dropped, or if the dinner plate never came fully out from under them. Otherwise 0.35, +0.25 dinner plate flat on the
    mat and let go, +0.25 side plates stacked back at their spot on the cabinet floor and let go, +0.15 bowls untouched."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(np.linalg.norm(_xy(objects["dinner"]) - np.asarray(_EP["mat"], float)))


def _debug(st) -> None:
    rep = _EP.get("report") or {}
    d = st["dinner"]
    parts = [f"{_EP['step']:4d} din=({d.pos[0]:.3f},{d.pos[1]:.3f},{d.pos[2]:.4f}) t={tilt_deg(d):4.1f}"]
    for n in SIDES:
        s = st[n]
        parts.append(f"{n[-1]}=({s.pos[0]:.3f},{s.pos[1]:.3f},{s.pos[2]:.4f}) t={tilt_deg(s):4.1f}")
    b = st[BOWLS[0]]
    parts.append(f"b1=({b.pos[0]:.3f},{b.pos[1]:.3f}) spoilt={_spoilt()} out={_EP.get('out')} "
                 f"maxoff={_EP['max_off']:.3f} maxtilt={_EP['max_tilt']:.1f} ddrop={_EP.get('dinner_drop')} "
                 f"parts={','.join(rep.get('parts', {}))} score={rep.get('score')}")
    _DBG.write(" ".join(parts) + "\n")
    _DBG.flush()


# -- the seeded start -----------------------------------------------------------------------------------------------
VARY_X = 0.010  # the dinner plate's centre moves up to this in x
VARY_Y = 0.020  # and in y, within its side
SIDE_OFF = 0.010  # the side-plate stack sits up to 1 cm off the dinner plate's centre, in any direction
MAT_VARY = 0.03


def layout(seed: int | None) -> dict:
    """Start poses (position, quaternion xyzw) of every loose body for a seed (None or 0: the setup text's nominal
    layout), plus "_mat", the mat's centre, and "_bowls_side" (+1 left, -1 right)."""
    rng = np.random.default_rng(seed) if seed else None
    u = (lambda a, b: float(rng.uniform(a, b))) if rng is not None else (lambda a, b: 0.5 * (a + b))
    s = -1.0 if rng is None else float(rng.choice((-1.0, 1.0)))  # which side the bowls stand
    xc = DINNER_NOM[0] + u(-VARY_X, VARY_X)
    yc = -s * DINNER_NOM[1] + u(-VARY_Y, VARY_Y)
    if rng is None:
        off = np.zeros(2)
    else:
        a, r = u(0.0, 2 * math.pi), SIDE_OFF * math.sqrt(u(0.0, 1.0))
        off = np.array([r * math.cos(a), r * math.sin(a)])
    gap = u(*BOWL_GAP)
    yb = yc + s * (DISH["dinner"][0] + gap + DISH["bowl"][0])
    yb = max(-(IN_HALF_W - DISH["bowl"][0] - 0.004), min(IN_HALF_W - DISH["bowl"][0] - 0.004, yb))
    xb = xc + u(-0.02, 0.02)
    q0 = (0.0, 0.0, 0.0, 1.0)
    pose = {"dinner": (np.array([xc, yc, SHELF + 0.0005]), _zq(u(0, 2 * math.pi)))}
    for i, n in enumerate(SIDES):
        pose[n] = (np.array([xc + off[0], yc + off[1], SHELF + well_top("dinner") + i * SIDE_PITCH + 0.0008 * (i + 1)]),
                   _zq(u(0, 2 * math.pi)))
    for i, n in enumerate(BOWLS):
        pose[n] = (np.array([xb, yb, SHELF + i * BOWL_PITCH + 0.0008 * (i + 1)]), q0)
    mat = (MAT_NOM[0] + u(-MAT_VARY, MAT_VARY), MAT_NOM[1] + u(-MAT_VARY, MAT_VARY))
    pose["mat"] = (np.array([mat[0], mat[1], TOP]), q0)
    pose["_mat"] = mat
    pose["_bowls_side"] = s
    return pose


def _zq(a: float) -> tuple:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


@bodykit.on_reset_env
def _lay_out(emb, seed):
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    lay = layout(seed)
    for name, (pos, quat) in ((k, v) for k, v in lay.items() if not k.startswith("_")):
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(max(1.0, getattr(ie, "SETTLE_S", 0.5)) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    st = _scene.object_states(emb._env, NAMES)
    _EP["start"] = {n: (np.asarray(st[n].pos, float).copy(), np.asarray(st[n].quat_xyzw, float).copy()) for n in DISHES}
    _EP["mat"] = lay["_mat"]
    _EP["bowls_side"] = lay["_bowls_side"]
    _blank()
    if _DBG is not None:
        _DBG.write(f"seed {seed}: bowls {'left' if lay['_bowls_side'] > 0 else 'right'} "
                   + " ".join(f"{n}=({v[0][0]:.4f},{v[0][1]:.4f},{v[0][2]:.4f})" for n, v in _EP["start"].items())
                   + f" mat=({lay['_mat'][0]:.3f},{lay['_mat'][1]:.3f})\n")
    return emb._observation()


# -- the scripted demo (reads the plates' poses from the simulator to plan; moves them only through the grippers) ----
TIP = 0.013  # pad tips past the TCP
CLR = 0.002  # each pad this far off the glass while the fingers slide round it
SIDE_TH = float(os.environ.get("PLATE_SIDE_TH", "55"))  # polar angle of the side-plate grip (deg from -x toward +y)
SIDE_R = float(os.environ.get("PLATE_SIDE_R", "0.066"))  # fingertip radius on the side plates
DIN_R = float(os.environ.get("PLATE_DIN_R", "0.110"))  # fingertip radius on the dinner plate's rim
LIFT = float(os.environ.get("PLATE_LIFT", "0.045"))  # the side plates are held this far up
PULL_TO = 0.270  # the dinner plate's centre ends this far in front of the side plates' start centre
PRESS = float(os.environ.get("PLATE_PRESS", "0.006"))  # the hands are sent this far below the counter when setting down
FAIL = os.environ.get("PLATE_FAIL", "")  # wrong outcomes on purpose (README): fixed, yank, drop, nomat
LOOKS = 2


def _rim_frame(theta_deg: float, kind: str, normal=np.array([0.0, 0.0, 1.0]), ref=None):
    """Directions for a pinch on a rim at polar angle theta round the dish (0: the side facing the robot, +: toward
    +y): outward v, the pinch axis n (perpendicular to the rim, up and in) and the tool axis z_t (down the rim, in).
    ``normal`` is the dish's axis; ``ref`` the in-plane direction of theta = 0 (default: -x projected)."""
    nz = np.asarray(normal, float) / np.linalg.norm(normal)
    r0 = np.array([-1.0, 0.0, 0.0]) if ref is None else np.asarray(ref, float)
    r0 = r0 - (r0 @ nz) * nz
    r0 /= np.linalg.norm(r0)
    r90 = np.cross(nz, r0)  # +90 degrees (toward +y when the dish is level and r0 is -x)
    if r90[1] < 0:
        r90 = -r90
    th = math.radians(theta_deg)
    v = math.cos(th) * r0 + math.sin(th) * r90
    a = rim_slope(kind)
    u = -v
    n = u * math.sin(a) + nz * math.cos(a)
    zt = u * math.cos(a) - nz * math.sin(a)
    return v, n, zt, nz


def _rot(n, zt, flip: bool) -> np.ndarray:
    x = -n if flip else n
    return np.column_stack([x, np.cross(zt, x), zt])


def pinch(origin, theta_deg: float, kind: str, r_tip: float, layers: int = 1, normal=(0, 0, 1), ref=None,
          flip: bool = False):
    """TCP position and rotation that put the fingertips' midpoint at the middle of a pack of ``layers`` nested
    rims at radius r_tip, closing across the rims; and the pack's half thickness along the pinch axis."""
    v, n, zt, nz = _rim_frame(theta_deg, kind, np.asarray(normal, float), ref)
    t = DISH[kind][4]
    p_top = np.asarray(origin, float) + v * r_tip + nz * rim_top(kind, r_tip)
    lower = p_top - n * t
    upper = p_top + nz * (layers - 1) * well_top(kind)
    centre = (lower + upper) / 2
    half = float((upper - lower) @ n) / 2
    return centre - TIP * zt, _rot(n, zt, flip), zt, half


def _wp(sec, pos, f, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(float(v) for v in pos), float(f), label, rot)


def ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, sec: float, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return _wp(sec, p, FINGER_OPEN, label, r)


def _pad(left: list, right: list) -> None:
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    if tl < tr - 1e-9:
        left.append(park("left", tr - tl, "wait"))
    elif tr < tl - 1e-9:
        right.append(park("right", tl - tr, "wait"))


def _rot_y(b: float) -> np.ndarray:
    return np.array([[math.cos(b), 0.0, math.sin(b)], [0.0, 1.0, 0.0], [-math.sin(b), 0.0, math.cos(b)]])


def lean_angle(cx: float) -> float:
    """Tilt (rad) at which a dinner plate whose centre is at x = cx, past the cabinet's edge, rests on that edge and
    on the counter with its outer rim (2 mm pressed in)."""
    drop = BOARD + DISH["dinner"][5] - DISH["dinner"][4] + 0.002
    return math.atan2(drop, DISH["dinner"][0] + (X0 - cx))


def _flip_for(arm: str, make) -> bool:
    """Which of the two equal pinches (the hand turned half a turn about its axis) the arm reaches best from the
    ready pose: ``make(flip)`` gives (tcp, rot)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    best = None
    for flip in (False, True):
        tcp, rot = make(flip)
        q, pe, re = ik.solve(tcp, ik.matrix_to_quat_xyzw(rot), READY, base_pos=station.BASES[arm], iters=300)
        key = (round(pe + 0.05 * re, 3), abs(float(q[5])))
        if best is None or key < best[0]:
            best = (key, flip)
    return best[1]


def stage0(start: dict) -> dict:
    """The hand on the open side of the cabinet lifts the three side plates by their rims; the other hand draws the
    dinner plate out from under them to the cabinet's edge and leans it there (front rim on the counter); the first
    hand sets the side plates back down. (With the plates in the cabinet's left half the left forearm would meet the
    cabinet's left wall, so the hands swap jobs, as a person's would.)"""
    os_ = start["side1"][0]
    od = start["dinner"][0]
    hold, pull = ("right", "left") if os_[1] > 0.0 else ("left", "right")
    th = SIDE_TH if hold == "left" else -SIDE_TH
    up = np.array([0.0, 0.0, 1.0])
    # the holding hand: pinch the three side-plate rims together
    fl = _flip_for(hold, lambda f: pinch(os_, th, "side", SIDE_R, layers=3, flip=f)[:2])
    tcp_h, rot_h, zt_h, half_h = pinch(os_, th, "side", SIDE_R, layers=3, flip=fl)
    f_h = -(half_h + CLR)
    H = [_wp(2.5, tcp_h - 0.12 * zt_h + 0.03 * up, -0.03, "over the side plates", rot_h),
         _wp(1.5, tcp_h - 0.05 * zt_h, f_h, "line up with the rims", rot_h),
         _wp(2.0, tcp_h, f_h, "slide a finger under the side plates", rot_h),
         _wp(1.0, tcp_h, 0.0, "pinch the three rims", rot_h),
         _wp(0.8, tcp_h, 0.0, "pinch the three rims", rot_h),
         _wp(2.0, tcp_h + LIFT * up, 0.0, "lift the side plates", rot_h)]
    t_lifted = sum(w.duration for w in H)
    # the pulling hand: pinch the dinner plate's front rim, draw it out, lean it on the cabinet's edge
    fp = _flip_for(pull, lambda f: pinch(od, 0.0, "dinner", DIN_R, flip=f)[:2])
    tcp_p, rot_p, zt_p, half_p = pinch(od, 0.0, "dinner", DIN_R, flip=fp)
    f_p = -(half_p + CLR)
    P = [_wp(2.5, tcp_p - 0.12 * zt_p + 0.03 * up, -0.03, "over the dinner plate's front rim", rot_p),
         _wp(1.5, tcp_p - 0.05 * zt_p, f_p, "line up with the rim", rot_p)]
    P.append(_wp(t_lifted - 4.0, tcp_p - 0.05 * zt_p, f_p, "wait for the side plates to go up", rot_p))
    P += [_wp(2.0, tcp_p, f_p, "slide a finger under the rim", rot_p),
          _wp(1.0, tcp_p, 0.0, "pinch the rim", rot_p),
          _wp(0.5, tcp_p, 0.0, "pinch the rim", rot_p)]
    dx = (os_[0] - PULL_TO) - od[0]
    pulled = tcp_p + np.array([dx, 0.0, 0.0])
    P.append(_wp(5.0, pulled, 0.0, "draw the dinner plate out", rot_p))
    pivot = np.array([X0, od[1], SHELF])
    tilt = _rot_y(-lean_angle(os_[0] - PULL_TO))
    leaned = pivot + tilt @ (pulled - pivot)
    rot_lean = tilt @ rot_p
    P += [_wp(2.0, leaned, 0.0, "lean it on the cabinet's edge", rot_lean),
          _wp(0.8, leaned, f_p - 0.003, "let go", rot_lean),
          _wp(1.2, leaned - 0.07 * (tilt @ zt_p), f_p - 0.003, "back off", rot_lean),
          park(pull, 2.0)]
    t_leaned = sum(w.duration for w in P[:-3])
    if FAIL == "yank":  # wrong on purpose: nobody holds the side plates up
        H = [park(hold, 1.0, "wait")]
    else:
        H.append(_wp(max(0.5, t_leaned - sum(w.duration for w in H)), tcp_h + LIFT * up, 0.0,
                     "hold the side plates up", rot_h))
        if FAIL == "drop":  # wrong on purpose: let go of the side plates while they are still up
            H += [_wp(0.8, tcp_h + LIFT * up, f_h, "let go up high", rot_h)]
        else:
            down = tcp_h - well_top("dinner") * up + 0.001 * up
            H += [_wp(2.0, down, 0.0, "set the side plates down", rot_h),
                  _wp(0.6, down, 0.0, "set the side plates down", rot_h),
                  _wp(0.8, down, f_h - 0.002, "let go", rot_h)]
        H += [_wp(1.5, np.asarray(H[-1].pos) - 0.08 * zt_h, H[-1].finger, "back off", rot_h), park(hold, 2.0)]
    plans = {hold: H, pull: P}
    _pad(plans["left"], plans["right"])
    return plans


def stage1(dinner_state) -> dict:
    """Both hands pinch the leaning dinner plate's left and right rims, level it, carry it to the mat, set it down."""
    od = np.asarray(dinner_state.pos, float)
    rd = dinner_state.rotation()
    nz = rd[:, 2]
    ref = np.array([-1.0, 0.0, 0.0])
    mat = np.asarray(_EP["mat"], float)
    level_c = np.array([od[0], od[1], TOP + 0.045])
    # the leaning plate's frame -> the level plate's frame (same heading, axis straight up)
    e0 = ref - (ref @ nz) * nz
    e0 /= np.linalg.norm(e0)
    old = np.column_stack([e0, np.cross(nz, e0), nz])
    new = np.column_stack([ref, np.cross([0.0, 0.0, 1.0], ref), [0.0, 0.0, 1.0]])
    turn = new @ old.T
    out = {}
    for arm, th, flip in (("left", 90.0, False), ("right", -90.0, True)):
        tcp, rot, zt, half = pinch(od, th, "dinner", DIN_R, normal=nz, ref=ref, flip=flip)
        f = -(half + CLR)
        lev = level_c + turn @ (tcp - od)
        lrot = turn @ rot
        carry = lev + np.array([mat[0] - od[0], mat[1] - od[1], 0.0])
        place = carry - np.array([0.0, 0.0, 0.045 + PRESS])
        zt_l = turn @ zt
        out[arm] = [_wp(2.5, tcp - 0.10 * zt + np.array([0, 0, 0.09]), -0.03, "high over the dinner plate's rim", rot),
                    _wp(1.5, tcp - 0.10 * zt + np.array([0, 0, 0.03]), -0.03, "over the dinner plate's rim", rot),
                    _wp(1.5, tcp - 0.05 * zt, f, "line up with the rim", rot),
                    _wp(2.0, tcp, f, "slide a finger under the rim", rot),
                    _wp(1.0, tcp, 0.0, "pinch the rim", rot),
                    _wp(0.6, tcp, 0.0, "pinch the rim", rot),
                    _wp(2.0, lev, 0.0, "lift it level", lrot),
                    _wp(3.5, carry, 0.0, "carry it to the mat", lrot),
                    _wp(2.5, place, 0.0, "set it down", lrot),
                    _wp(0.6, place, 0.0, "set it down", lrot),
                    _wp(0.8, place, f - 0.003, "let go", lrot),
                    _wp(1.5, place - 0.08 * zt_l + np.array([0, 0, 0.03]), f - 0.003, "back off", lrot),
                    park(arm, 2.0)]
    return out


def _fixed_start() -> dict:
    lay = layout(None)
    return {n: (lay[n][0] + np.array([0.0, 0.0, 0.0]), lay[n][1]) for n in DISHES}


class _Lean:
    """The leaning dinner plate's pose as a plan made for the nominal layout would expect it (FAIL=fixed)."""

    def __init__(self, start):
        os_, od = start["side1"][0], start["dinner"][0]
        c = np.array([os_[0] - PULL_TO, od[1], SHELF])
        pivot = np.array([X0, od[1], SHELF])
        self._r = _rot_y(-lean_angle(c[0]))
        self.pos = pivot + self._r @ (c - pivot)

    def rotation(self):
        return self._r


def plan_stage(stage: int, img, module) -> dict:
    start = _fixed_start() if FAIL == "fixed" else _EP["start"]
    if stage == 0:
        print(f"platedemo: side plates at ({start['side1'][0][0]:.4f}, {start['side1'][0][1]:.4f}), dinner plate at "
              f"({start['dinner'][0][0]:.4f}, {start['dinner'][0][1]:.4f})", flush=True)
        return stage0(start)
    if FAIL == "nomat":  # wrong on purpose: the dinner plate is left leaning on the cabinet's edge
        return {}
    d = _Lean(start) if FAIL == "fixed" else _EP["now"]["dinner"]
    print(f"platedemo: dinner plate leaning at ({d.pos[0]:.4f}, {d.pos[1]:.4f}, {d.pos[2]:.4f}), tilted "
          f"{math.degrees(math.acos(min(1.0, d.rotation()[2, 2]))):.1f} deg", flush=True)
    return stage1(d)


def placeholder_plans() -> dict:
    return {arm: [park(arm, 1.0, "look")] for arm in ("left", "right")}


def demo():
    return placeholder_plans()


import platedemo  # noqa: E402  (the demo's control hooks for yam_scripted)

platedemo.install()
platedemo_task = True
