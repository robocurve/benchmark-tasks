"""On the table, two 25-slot 2-inch wafer cassettes stand upright side by side, open tops up, slots running left to
right: the near one holds three wafers in adjacent slots, the far one two wafers with one empty slot between them. To
the left of the cassettes a pair of wafer tweezers stands tips down in a small stand. Move the middle wafer of the
near cassette into the empty slot of the far one with the tweezers, holding it only by its top edge, then stand the
tweezers back in their stand.

Layout: trial k uses LAYOUTS[k mod 3] (set TRIAL=k; WAFER_LAYOUT=A, B or C overrides). The source and target slots are
(9, 13), (13, 17) and (17, 9).
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import artkit as ak  # noqa: E402
import parts as P  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
LAYOUTS = {"A": (9, 13), "B": (13, 17), "C": (17, 9)}
_key = os.environ.get("WAFER_LAYOUT") or "ABC"[int(os.environ.get("TRIAL", "0")) % 3]
LAYOUT = _key.upper()
SRC, DST = LAYOUTS[LAYOUT]

NEAR_XY = (0.22, 0.15)  # source cassette, centre of its footprint
FAR_XY = (0.30, 0.15)  # target cassette
STAND_XY = (0.22, 0.33)
Q_Z90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # 90 deg about z: tweezer leaves part along y
SEAT_Z = TOP + P.seated_center_z()

# scoring thresholds (all checkable with a ruler or the trial video)
SEATED_TOL = 0.002  # top edge within 2 mm of its neighbours' top edges
DISPLACED = 0.002  # another wafer moved or lifted 2 mm or more
CLEAR = 0.050  # tweezer tips at least 50 mm from the wafer before any credit
RELEASE_HIGH = 0.003  # parted from the tips with its top edge over 3 mm (half a tip's width) above its neighbours'
PARTED_STEPS = 3  # free of the tips this many control steps in a row = parted (ignores one-step contact flicker)
ROBOT_TOUCH = 1e-4  # N: any reported contact between a robot link and a wafer
UPRIGHT_DEG = 5.0
RUN_LIMIT_STEPS = 2400
SCORE_OVER = "end"


def slot_xy(c, k):
    return np.array([c[0], c[1] + P.slot_y(k)])


def seat_pos(c, k):
    x, y = slot_xy(c, k)
    return (float(x), float(y), SEAT_Z)


WAFER_KW = dict(mass=P.WAFER_MASS, size=(P.WAFER_D, P.WAFER_T, P.WAFER_D), kind="rigid")
ASSETS = P.asset_paths(HERE / os.environ.get("ASSET_DIR", "assets"))
NEIGHBOURS = {"nb_n1": (NEAR_XY, SRC - 1), "nb_n2": (NEAR_XY, SRC + 1), "nb_f1": (FAR_XY, DST - 1),
              "nb_f2": (FAR_XY, DST + 1)}

OBJECTS = [
    ak.UsdPart("near_cassette", pos=(*NEAR_XY, TOP), size=(P.CAS_W, P.CAS_L, P.CAS_H), usd=str(ASSETS["cassette"]),
               kind="kinematic"),
    ak.UsdPart("far_cassette", pos=(*FAR_XY, TOP), size=(P.CAS_W, P.CAS_L, P.CAS_H), usd=str(ASSETS["cassette"]),
               kind="kinematic"),
    ak.UsdPart("stand", pos=(*STAND_XY, TOP), size=P.STAND, usd=str(ASSETS["stand"]), kind="kinematic"),
    ak.UsdPart("tweezers", pos=(*STAND_XY, TOP + P.TW_LEN + 0.0005), mass=2 * P.TW_MASS_HALF, size=(0.01, 0.01, P.TW_LEN),
               usd=str(ASSETS["tweezers"]), kind="articulation", rot_xyzw=Q_Z90, joint_pos0=(("hinge", 0.0),),
               springs=(("hinge", P.TW_SPRING, P.TW_SPRING_DAMPING),)),
    ak.UsdPart("wafer", pos=seat_pos(NEAR_XY, SRC), usd=str(ASSETS["wafer"]), **WAFER_KW),
    *[ak.UsdPart(n, pos=seat_pos(c, k), usd=str(ASSETS["wafer"]), **WAFER_KW) for n, (c, k) in NEIGHBOURS.items()],
]

ak.sensor("grip", "wafer", ["tweezers/half_a", "tweezers/half_b"])
_LINKS = "Geometry/base/link1/link2/link3/link4"
_ROBOT = [f"{arm}/{_LINKS}{tail}" for arm in ("LeftArm", "RightArm")
          for tail in ("", "/link5", "/link5/gripper", "/link5/gripper/tip_left", "/link5/gripper/tip_right")]
ak.sensor("robot_wafer", "wafer", _ROBOT)  # the robot itself (not the tweezers) touching the moved wafer
ak.sensor("robot_nb", "nb_.*", _ROBOT)  # ... or any other wafer

# -- measurement (simulator) -> pure scoring ---------------------------------------------------------------------------

S: dict = {}


@ak.on_reset
def _reset():
    S.clear()
    S.update(was_held=False, parted_pos=None, checked=False, dropped=False, displaced=False, table=False,
             robot=False, entered=False, free_steps=0, events=[])


_reset()


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def disk_points(c, n):
    """Lowest point height and the two side edges (along the wafer's in-plane horizontal) of a disk."""
    n = n / np.linalg.norm(n)
    low = c[2] - P.WAFER_R * math.sqrt(max(0.0, 1 - n[2] ** 2))
    h = np.cross(n, [0.0, 0.0, 1.0])
    h = h / (np.linalg.norm(h) or 1.0)
    return low, c + h * P.WAFER_R, c - h * P.WAFER_R


def disk_dist(p, c, n):
    v = p - c
    a = float(v @ n)
    r = float(np.linalg.norm(v - n * a))
    if r <= P.WAFER_R:
        return abs(a) - P.WAFER_T / 2
    return math.hypot(r - P.WAFER_R, max(0.0, abs(a) - P.WAFER_T / 2))


def slot_index(c, y):
    """Which slot of the cassette centred at c a y position falls in (nearest pocket)."""
    return int(round((y - (c[1] + P.slot_y(1))) / P.PITCH)) + 1


def in_footprint(c, p):
    return abs(p[0] - c[0]) < P.CAS_W / 2 and abs(p[1] - c[1]) < P.CAS_L / 2


def tip_points():
    out = []
    for half in ("half_a", "half_b"):
        p, q = ak.body_pose("tweezers", half)
        out.append(p + _rot(q) @ np.array(P.half_point(half, P.TW_HINGE_GAP / 2 + P.TIP_T / 2, -P.TW_LEN)))
    return out


def measure(objects) -> dict:
    """Everything the scorer needs, from the simulator state (and the contact sensor)."""
    w = objects["wafer"]
    n = _rot(w.quat_xyzw)[:, 1]  # disc axis (faces' normal) is the body's y
    low, e1, e2 = disk_points(w.pos, n)
    m = dict(wafer_pos=np.asarray(w.pos, float), wafer_vz=float(w.lin_vel[2]), wafer_speed=float(np.linalg.norm(w.lin_vel)),
             wafer_low=low, edge_y=(float(e1[1]), float(e2[1])), edge_x=(float(e1[0]), float(e2[0])),
             upright_deg=math.degrees(math.acos(min(1.0, abs(n[1])))))
    m["nb_moved"] = max(float(np.linalg.norm(np.asarray(objects[k].pos) - np.array(seat_pos(c, j))))
                        for k, (c, j) in NEIGHBOURS.items())
    env = ak.env() is not None
    m["grip_N"] = ak.force("grip") if env else 0.0
    m["robot_touch_N"] = max(ak.force("robot_wafer"), ak.force("robot_nb")) if env else 0.0
    if env:
        tips = tip_points()
        m["tips_to_wafer"] = min(disk_dist(t, w.pos, n) for t in tips)
        mid = (tips[0] + tips[1]) / 2
        _, qa = ak.body_pose("tweezers", "half_a")
        axis = _rot(qa) @ np.array([0.0, 0.0, -1.0])
        m["tw_in_stand"] = bool(abs(mid[0] - STAND_XY[0]) < 0.008 and abs(mid[1] - STAND_XY[1]) < 0.009
                                and mid[2] < TOP + P.STAND[2] and axis[2] < -math.cos(math.radians(15))
                                and objects["tweezers"].resting(0.01, 0.2))
        m["robot_to_tweezers"] = _robot_to_tweezers(mid, axis)
    else:
        m.update(tips_to_wafer=1.0, tw_in_stand=False, robot_to_tweezers=1.0)
    return m


def _robot_to_tweezers(mid, axis):
    """Smallest distance from either arm's TCP to the tweezers' axis (tips to back end)."""
    from yam_arena.runtime import tcp_world

    scene = ak.env().unwrapped.scene
    origin = ak._t(scene.env_origins)[0]
    best = 1.0
    for arm in ("left_robot", "right_robot"):
        p = np.asarray(tcp_world(scene[arm]), float) - origin
        v = p - mid
        t = float(np.clip(v @ -axis, 0.0, P.TW_LEN))
        best = min(best, float(np.linalg.norm(v + axis * t)))
    return best


def ladder(m: dict, s: dict) -> dict:
    """Pure scoring from one step's measurements m; s carries the latched events between steps."""
    held = m["grip_N"] > 0.02
    p = m["wafer_pos"]
    if held:
        s["was_held"], s["checked"], s["free_steps"] = True, False, 0
    else:
        s["free_steps"] += 1
    # the wafer must already rest in a slot when it parts from the tweezer tips (tips opened, or it slipped out)
    if s["was_held"] and s["free_steps"] == 1:
        s["parted_pos"] = p.copy()
    if s["was_held"] and s["free_steps"] == PARTED_STEPS and not s["checked"] and not s["dropped"]:
        s["checked"] = True
        q = s["parted_pos"]
        in_cassette = (in_footprint(NEAR_XY, q) or in_footprint(FAR_XY, q)) and q[2] < TOP + P.CAS_H
        if not in_cassette or q[2] - SEAT_Z > RELEASE_HIGH:
            s["dropped"] = True
            s["events"].append("dropped: left the tips before resting in a slot")
    if m["robot_touch_N"] > ROBOT_TOUCH and not s["robot"]:
        s["robot"] = True
        s["events"].append("the robot touched a wafer (not with the tweezers)")
    if m["nb_moved"] >= DISPLACED and not s["displaced"]:
        s["displaced"] = True
        s["events"].append("another wafer moved or lifted 2 mm or more")
    if m["wafer_low"] < TOP + 0.001 and not s["table"]:
        s["table"] = True
        s["events"].append("the wafer touched the table")
    slots = tuple(slot_index(FAR_XY, y) for y in m["edge_y"])
    in_far = in_footprint(FAR_XY, p) and m["wafer_low"] < TOP + P.CAS_H
    in_target = in_far and slots == (DST, DST) and m["upright_deg"] < UPRIGHT_DEG
    if in_target:
        s["entered"] = True
    resting = m["wafer_speed"] < 0.005 and s["free_steps"] >= 10
    seated = in_target and abs(p[2] - SEAT_Z) <= SEATED_TOL and resting
    clear = (not held) and m["tips_to_wafer"] >= CLEAR
    zero = s["dropped"] or s["displaced"] or s["table"] or s["robot"]
    if zero or not clear:
        score = 0.0
    elif seated and m["tw_in_stand"] and m["robot_to_tweezers"] >= 0.03:
        score = 1.0
    elif seated:
        score = 0.75
    elif in_target and resting:
        score = 0.5
    else:
        score = 0.0
    return dict(score=score, held=held, clear=clear, in_target=in_target, seated=seated, slots=slots,
                tw_in_stand=m["tw_in_stand"], events=list(s["events"]))


_LAST: dict = {"objects": None, "result": None}


def evaluate(objects) -> dict:
    """One evaluation per control step: score() and success() are both called on the same states."""
    if _LAST["objects"] is objects:
        return _LAST["result"]
    res = ladder(measure(objects), S)
    _LAST.update(objects=objects, result=res)
    return res


def score(objects) -> float:
    return evaluate(objects)["score"]


def success(objects) -> bool:
    return evaluate(objects)["score"] == 1.0
