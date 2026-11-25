"""A fixed 1U rack server front with two slotted chassis ears stands on the station table. A 430 mm
plastic security bezel rests upright in front of it. Two tabs at its right end fit the right ear;
a spring returned latch at its left end runs over a ramp on the left ear. Fit the bezel flush.
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque
from typing import Any

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robocurve_ext as rx  # noqa: E402

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, ObjectState  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20.0
INSTRUCTION = "Fit the security bezel on the front of the server so it sits flush and clicks shut."

SERVER_WIDTH = 0.4826
SERVER_HEIGHT = 0.0432
BEZEL_WIDTH = 0.430
BEZEL_HEIGHT = 0.041
BEZEL_DEPTH = 0.025
BEZEL_MASS = 0.350
EAR_FACE_X = 0.420
BEZEL_FINAL_X = EAR_FACE_X + BEZEL_DEPTH / 2
BEZEL_START_X = 0.255
EAR_INNER_Y = 0.2185  # 3.5 mm side play each end: the tilted bezel's front corner clears the ear on insertion
EAR_OUTER_Y = SERVER_WIDTH / 2
SLOT_Z = (0.0105, 0.0325)
SLOT_HEIGHT = 0.012
SLOT_INNER_Y = -EAR_INNER_Y
SLOT_OUTER_Y = -0.2393
TAB_Y = -0.2235
TAB_X_LOCAL = 0.005
TAB_SIZE = (0.015, 0.017, 0.006)  # back face flush with the bezel's, clear of the steel panel
LATCH_LOCAL = (-0.0015, 0.221, 0.022)
LATCH_SIZE = (0.012, 0.012, 0.010)
LATCH_STROKE = 0.014
LATCH_SPRING_N_PER_M = 350.0
LATCH_DAMPING_NS_PER_M = 0.4
CATCH_END_X = 0.419
CATCH_INNER_Y = EAR_INNER_Y
CATCH_OUTER_Y = EAR_INNER_Y + 0.013
LATCH_WINDOW_H = 0.014  # left-ear window behind the catch; the extended latch springs into it
PLASTIC_ON_STEEL = rx.Friction(0.25, 0.25)  # acetal latch on steel catch, not yam_arena's rubber 1.2

# End-state measurements. The time, drop and force tests latch for the entire episode.
FLUSH_MM = 2.0
YAW_DEG = 2.0
FINAL_REST_S = 2.0
TAB_FREE_PLAY_MM = 3.0
LATCH_OVERLAP_MM = 1.5
LATCH_RETURN_MAX_MM = 2.0
FLOOR_DROP_Z = TOP - 0.20
FORCE_FAULT_N = 100.0
GRIPPER_CLEAR_M = 0.025

STEEL = (0.44, 0.47, 0.50)
EAR = (0.62, 0.65, 0.68)
PLASTIC = (0.12, 0.15, 0.17)
CATCH = (0.74, 0.76, 0.78)


def f3(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in v)


def piece(x0, x1, y0, y1, z0, z1, centre, color=None):
    return rx.Piece(size=f3((x1 - x0, y1 - y0, z1 - z0)),
                    offset=f3(((x0 + x1) / 2 - centre[0], (y0 + y1) / 2 - centre[1],
                               (z0 + z1) / 2 - centre[2])), color=color)


# The steel 1U panel is recessed behind the removable plastic face. Each ear is a built-up
# shape with two real openings, not a visual slot painted on a solid collider.
SERVER_C = (0.453, 0.0, SERVER_HEIGHT / 2)
server_pieces = [piece(0.446, 0.486, -EAR_INNER_Y, EAR_INNER_Y, 0, SERVER_HEIGHT, SERVER_C, STEEL)]
# Right ear: two tab slots. Left ear: one latch window. Each opening is open toward the bezel
# and closed by a 2 mm outer wall.
for ya, yb, wall, holes in ((-EAR_OUTER_Y, -EAR_INNER_Y, (-EAR_OUTER_Y, SLOT_OUTER_Y),
                             [(zc, SLOT_HEIGHT) for zc in SLOT_Z]),
                            (EAR_INNER_Y, EAR_OUTER_Y, (-SLOT_OUTER_Y, EAR_OUTER_Y),
                             [(LATCH_LOCAL[2], LATCH_WINDOW_H)])):
    edges = [0.0] + [z for zc, h in holes for z in (zc - h / 2, zc + h / 2)] + [SERVER_HEIGHT]
    for za, zb in zip(edges[::2], edges[1::2]):
        server_pieces.append(piece(EAR_FACE_X, 0.470, ya, yb, za, zb, SERVER_C, EAR))
    for zc, h in holes:
        server_pieces.append(piece(EAR_FACE_X, 0.470, *wall, zc - h / 2, zc + h / 2, SERVER_C, EAR))

SERVER = Box("server_front", pos=f3((SERVER_C[0], SERVER_C[1], TOP + SERVER_C[2])),
             size=(0.066, SERVER_WIDTH, SERVER_HEIGHT), color=STEEL)
CATCH_PART = Box("left_catch", pos=f3((0.405, (CATCH_INNER_Y + CATCH_OUTER_Y) / 2, TOP + 0.022)),
                 size=(0.028, 0.013, 0.010), color=CATCH)


# The bezel has a continuous face sheet, full-depth edge rails, end returns and three ribs.
# The upper rail is 25 mm deep, so each gripper can pinch a real top edge over >=15 mm.
BC = (BEZEL_START_X, 0.0, BEZEL_HEIGHT / 2)
bx0, bx1 = BEZEL_START_X - BEZEL_DEPTH / 2, BEZEL_START_X + BEZEL_DEPTH / 2
bezel_pieces = [
    piece(bx0, bx0 + 0.004, -BEZEL_WIDTH / 2, BEZEL_WIDTH / 2, 0, BEZEL_HEIGHT, BC),
    piece(bx0, bx1, -BEZEL_WIDTH / 2, BEZEL_WIDTH / 2, 0, 0.005, BC),
    piece(bx0, bx1, -BEZEL_WIDTH / 2, BEZEL_WIDTH / 2,
          BEZEL_HEIGHT - 0.005, BEZEL_HEIGHT, BC),
]
for y in (-0.210, -0.145, 0.0, 0.145, 0.210):
    bezel_pieces.append(piece(bx0 + 0.004, bx1, y - 0.003, y + 0.003,
                              0.005, BEZEL_HEIGHT - 0.005, BC))
for zc in SLOT_Z:
    bezel_pieces.append(piece(BEZEL_START_X + TAB_X_LOCAL - TAB_SIZE[0] / 2,
                              BEZEL_START_X + TAB_X_LOCAL + TAB_SIZE[0] / 2,
                              TAB_Y - TAB_SIZE[1] / 2, TAB_Y + TAB_SIZE[1] / 2,
                              zc - TAB_SIZE[2] / 2, zc + TAB_SIZE[2] / 2, BC))

_lo = np.min([np.subtract(p.offset, np.divide(p.size, 2)) for p in bezel_pieces], axis=0)
_hi = np.max([np.add(p.offset, np.divide(p.size, 2)) for p in bezel_pieces], axis=0)
# The sim's ObjectState.pos is this local bounds centre (the tabs make it -8.5 mm in y), not the bezel origin.
BEZEL_BOUNDS_C = (_lo + _hi) / 2

BEZEL = Box("bezel", pos=f3((BEZEL_START_X, 0, TOP + BEZEL_HEIGHT / 2)),
            size=(BEZEL_DEPTH, BEZEL_WIDTH + 0.017, BEZEL_HEIGHT),
            mass=BEZEL_MASS - 0.025, color=PLASTIC)
LATCH = Box("latch", pos=f3((BEZEL_START_X + LATCH_LOCAL[0], LATCH_LOCAL[1],
                              TOP + LATCH_LOCAL[2])),
            size=LATCH_SIZE, mass=0.025, color=(0.25, 0.30, 0.34))
LATCH_JOINT = rx.Prismatic(anchor=(0.0, 0.0, 0.0), axis="Y", lower=-LATCH_STROKE,
                           upper=0.0, target=0.0, stiffness=LATCH_SPRING_N_PER_M,
                           damping=LATCH_DAMPING_NS_PER_M, parent="bezel")
rx.install({"server_front": rx.Compound(tuple(server_pieces)),
            "left_catch": (rx.Mesh(os.path.join(HERE, "catch_ramp.obj"), approximation="none"),
                           PLASTIC_ON_STEEL),
            "bezel": rx.Compound(tuple(bezel_pieces)), "latch": (LATCH_JOINT, PLASTIC_ON_STEEL)})
OBJECTS = [SERVER, CATCH_PART, BEZEL, LATCH]


def _rot(obj: ObjectState) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(obj.quat_xyzw, float))


def _origin(bezel: ObjectState) -> np.ndarray:
    return np.asarray(bezel.pos, float) - _rot(bezel) @ BEZEL_BOUNDS_C


def _local_point(bezel: ObjectState, p) -> np.ndarray:
    return _origin(bezel) + _rot(bezel) @ np.asarray(p, float)


def yaw_deg(obj: ObjectState) -> float:
    r = _rot(obj)
    return abs(math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0]))))


def flush_mm(obj: ObjectState) -> float:
    r = _rot(obj)
    # Both ends and both depth faces must agree with the steel ear's front datum.
    dx = [abs(_local_point(obj, (-BEZEL_DEPTH / 2, y, 0))[0] - EAR_FACE_X)
          for y in (-BEZEL_WIDTH / 2, 0, BEZEL_WIDTH / 2)]
    return float(max(dx) * 1000)


def tabs_seated(obj: ObjectState) -> bool:
    for zc in SLOT_Z:
        p = _local_point(obj, (TAB_X_LOCAL, TAB_Y, zc - BEZEL_HEIGHT / 2))
        if not (0.425 <= p[0] <= 0.455 and SLOT_OUTER_Y + 0.001 <= p[1] <= SLOT_INNER_Y - 0.001
                and abs(p[2] - TOP - zc) <= 0.003):
            return False
    return True


def latch_travel_mm(objects: dict[str, ObjectState]) -> float:
    """Compression in the joint's parent frame (metres converted to millimetres).

    This maximal-coordinate joint is not an articulation DOF. Installed PhysX
    explicitly ignores JointStateAPI on standalone joints; use the tested pose
    projection instead of an authored, non-live USD position attribute.
    """
    b, l = objects["bezel"], objects["latch"]
    relative = _rot(b).T @ (np.asarray(l.pos, float) - _origin(b))
    return float(max(0.0, (LATCH_LOCAL[1] - relative[1]) * 1000))


def catch_overlap_mm(objects: dict[str, ObjectState]) -> float:
    b, l = objects["bezel"], objects["latch"]
    nose_front_x = float(l.pos[0] - LATCH_SIZE[0] / 2)
    if nose_front_x < CATCH_END_X + 0.001:
        return 0.0
    return float(max(0, min(CATCH_OUTER_Y, l.pos[1] + LATCH_SIZE[1] / 2) - CATCH_INNER_Y) * 1000)


def latched(objects: dict[str, ObjectState]) -> bool:
    return bool(tabs_seated(objects["bezel"]) and
                latch_travel_mm(objects) <= LATCH_RETURN_MAX_MM and
                catch_overlap_mm(objects) >= LATCH_OVERLAP_MM)


def _embodiment():
    f = sys._getframe(1)
    while f is not None:
        s = f.f_locals.get("self")
        if s is not None and getattr(s, "_base", None) is not None and hasattr(s, "_t"):
            return s
        f = f.f_back
    return None


def _numpy(a: Any) -> np.ndarray:
    if hasattr(a, "numpy"):
        return np.asarray(a.numpy(), float)
    if hasattr(a, "detach"):
        return a.detach().cpu().numpy().astype(float)
    return np.asarray(a, float)


_CONTACT = {"view": None, "error": None}


def robot_contact_force_n(base) -> float:
    """Largest net contact load on any rigid robot body, including palms and wrists."""
    if base is None:
        return 0.0
    if _CONTACT["view"] is None and _CONTACT["error"] is None:
        try:
            import carb.settings
            from pxr import Usd, UsdPhysics

            carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
            paths = []
            stage = base.scene.stage
            for root_name in ("left_robot", "right_robot", "LeftArm", "RightArm"):
                root = stage.GetPrimAtPath(f"/World/envs/env_0/{root_name}")
                if root.IsValid():
                    paths.extend(p.GetPath().pathString for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                                 if p.HasAPI(UsdPhysics.RigidBodyAPI))
            paths = sorted(set(paths))
            if not paths:
                raise RuntimeError("no robot rigid links found")
            _CONTACT["view"] = base.sim.physics_sim_view.create_rigid_contact_view(
                paths, max_contact_data_count=256)
            print(f"BEZEL_CONTACT_LINKS n={len(paths)}", flush=True)
        except Exception as exc:
            _CONTACT["error"] = f"{type(exc).__name__}: {exc}"
            print(f"BEZEL_CONTACT_ERROR {_CONTACT['error']}", flush=True)
            return math.inf
    try:
        v = _CONTACT["view"]
        net = _numpy(v.get_net_contact_forces(float(base.sim.get_physics_dt()))).reshape(v.sensor_count, 3)
        return float(np.linalg.norm(net, axis=1).max())
    except Exception as exc:
        _CONTACT["error"] = f"{type(exc).__name__}: {exc}"
        print(f"BEZEL_CONTACT_ERROR {_CONTACT['error']}", flush=True)
        return math.inf


def _tcps(emb) -> list[np.ndarray]:
    if emb is None:
        return []
    from yam_arena import runtime

    return [ik.tcp_pose(runtime.arm_q(emb._base.scene[f"{side}_robot"]), station.BASES[side])[0]
            for side in ("left", "right")]


def _clear(objects, tcps) -> bool:
    b = objects["bezel"]
    r = _rot(b)
    h = np.array([BEZEL_DEPTH / 2, BEZEL_WIDTH / 2, BEZEL_HEIGHT / 2])
    for tcp in tcps:
        p = r.T @ (np.asarray(tcp) - _origin(b))
        if np.linalg.norm(p - np.clip(p, -h, h)) < GRIPPER_CLEAR_M:
            return False
    return True


_STATE = {"last_t": None, "fault": False, "history": deque(), "step": 0, "logged_t": None}


def _measure(objects, tcps=None, force_n=None):
    emb = _embodiment()
    t = None if emb is None else float(emb._t)
    if t is not None and _STATE["last_t"] is not None and t < _STATE["last_t"]:
        _STATE.update(last_t=None, fault=False, history=deque(), step=0, logged_t=None)
    b = objects["bezel"]
    if force_n is None:
        force_n = robot_contact_force_n(None if emb is None else emb._base)
    dropped = float(b.high[2]) < FLOOR_DROP_Z
    if dropped or force_n > FORCE_FAULT_N:
        _STATE["fault"] = True
    flush_now = (flush_mm(b) <= FLUSH_MM and yaw_deg(b) <= YAW_DEG and b.resting())
    if t is None:
        flush_2s = flush_now
    else:
        _STATE["last_t"] = t
        h = _STATE["history"]
        h.append((t, flush_now))
        while h and h[0][0] < t - FINAL_REST_S:
            h.popleft()
        flush_2s = bool(h and t - h[0][0] >= FINAL_REST_S - 1 / CONTROL_HZ - 1e-6
                        and all(ok for _, ok in h))
    if tcps is None:
        tcps = _tcps(emb)
    clear = _clear(objects, tcps)
    tab = tabs_seated(b)
    latch = latched(objects)
    value = 0.0 if _STATE["fault"] or not clear else float(0.25 * tab + 0.25 * flush_2s + 0.50 * latch)
    if t is not None and t != _STATE["logged_t"]:
        _STATE["logged_t"] = t
        _STATE["step"] += 1
        # One numeric record per control step, including the full final 2 s.
        print(f"BEZEL_STEP step={_STATE['step']} t={t:.2f} tab={int(tab)} flush={int(flush_2s)} "
              f"latch={int(latch)} latch_travel_mm={latch_travel_mm(objects):.2f} "
              f"overlap_mm={catch_overlap_mm(objects):.2f} yaw_deg={yaw_deg(b):.2f} "
              f"flush_mm={flush_mm(b):.2f} rest={int(b.resting())} clear={int(clear)} "
              f"force_n={force_n:.1f} fault={int(_STATE['fault'])} score={value:.2f} "
              f"bezel_xyz={','.join(f'{v:.5f}' for v in b.pos)} "
              f"tcps={';'.join(','.join(f'{v:.5f}' for v in p) for p in tcps)}", flush=True)
    return value


def score(objects: dict[str, ObjectState], tcps=None, *, force_n=None) -> float:
    return _measure(objects, tcps, force_n)


def success(objects: dict[str, ObjectState], tcps=None, *, force_n=None) -> bool:
    return bool(score(objects, tcps, force_n=force_n) == 1.0)


def distance(objects: dict[str, ObjectState]) -> float:
    b = objects["bezel"]
    return float(abs(_local_point(b, (-BEZEL_DEPTH / 2, 0, 0))[0] - EAR_FACE_X)
                 + math.radians(yaw_deg(b)) * BEZEL_WIDTH / 2)


# Both hands pinch the continuous upper rim of the real bezel. The right hand places
# the tabs first; the left hand then follows the circular swing toward the ear.
GRIP_Y = 0.185
GRIP_Z = TOP + 0.0385
LIFT_Z = GRIP_Z + 0.025
FINGER_EDGE = -0.0075
PARK_Z = TOP + 0.145
OPEN_YAW = math.radians(8.0)
FINGER_RELEASE = -0.015


def _wp(dt, x, y, z, finger, label, yaw):
    return Waypoint(dt, f3((x, y, z)), finger, label, yaw)


def demo() -> dict[str, list[Waypoint]]:
    plans = {}
    for side, sign in (("left", 1), ("right", -1)):
        y = GRIP_Y if side == "left" else -0.140
        jaw_yaw = -sign * math.pi / 2
        # Rotate both real grip points about the tab centre, including wrist yaw.
        open_x = 0.446 - TAB_X_LOCAL * math.cos(OPEN_YAW) - (y - TAB_Y) * math.sin(OPEN_YAW)
        open_y = (TAB_Y - TAB_X_LOCAL * math.sin(OPEN_YAW)
                  + (y - TAB_Y) * math.cos(OPEN_YAW))
        stage_x = open_x - 0.060
        insert_yaw = jaw_yaw + OPEN_YAW
        common = [
            _wp(1.6, BEZEL_START_X, y, TOP + 0.095, FINGER_OPEN, "approach", jaw_yaw),
            _wp(1.2, BEZEL_START_X, y, GRIP_Z, FINGER_OPEN, "at_upper_edge", jaw_yaw),
            _wp(1.3, BEZEL_START_X, y, GRIP_Z, FINGER_EDGE, "pinch_upper_edge", jaw_yaw),
            _wp(1.4, BEZEL_START_X, y, LIFT_Z, FINGER_EDGE, "lift", jaw_yaw),
            _wp(2.0, stage_x, open_y, LIFT_Z, FINGER_EDGE, "carry_to_slot_entry", insert_yaw),
            _wp(1.0, stage_x, open_y, GRIP_Z, FINGER_EDGE, "lower_before_slots", insert_yaw),
            _wp(2.0, open_x, open_y, GRIP_Z, FINGER_EDGE, "insert_right_tabs", insert_yaw),
            _wp(0.5, open_x, open_y, GRIP_Z, FINGER_EDGE, "seat_tabs", insert_yaw),
        ]
        if side == "right":
            plans[side] = common + [
                _wp(0.5, open_x, open_y, GRIP_Z, FINGER_RELEASE, "release_seated_tabs", insert_yaw),
                _wp(0.8, open_x, open_y, GRIP_Z + 0.045, FINGER_RELEASE, "rise_from_tabs", insert_yaw),
                _wp(1.2, 0.20, sign * 0.28, PARK_Z, FINGER_OPEN, "park", jaw_yaw),
                _wp(10.6, 0.20, sign * 0.28, PARK_Z, FINGER_OPEN, "done", jaw_yaw),
            ]
        else:
            push_start_x = open_x - 0.045
            # The wrist is a ~0.15 N/mm position spring (BEZEL_STEP logs): ~5 N of push needs ~35 mm of error.
            press_x = 0.450
            plans[side] = common + [
                _wp(1.3, open_x, open_y, GRIP_Z, FINGER_EDGE, "wait_right_clear", insert_yaw),
                _wp(0.4, open_x, open_y, GRIP_Z, FINGER_RELEASE, "release_left_edge", insert_yaw),
                _wp(0.7, open_x, open_y, GRIP_Z + 0.045, FINGER_RELEASE, "rise_from_left_edge", insert_yaw),
                _wp(0.6, push_start_x, y, GRIP_Z + 0.045, 0.0, "prepare_left_push", jaw_yaw),
                _wp(0.8, push_start_x, y, GRIP_Z, 0.0, "outside_left_end", jaw_yaw),
                _wp(5.0, press_x, y, GRIP_Z, 0.0, "swing_in_press_latch_ramp", jaw_yaw),
                _wp(1.5, press_x, y, GRIP_Z, 0.0, "hold_click", jaw_yaw),
                _wp(0.8, press_x - 0.035, y, GRIP_Z + 0.045, 0.0, "retreat", jaw_yaw),
                _wp(1.0, 0.20, sign * 0.28, PARK_Z, FINGER_OPEN, "park", jaw_yaw),
                _wp(2.0, 0.20, sign * 0.28, PARK_Z, FINGER_OPEN, "done", jaw_yaw),
            ]
    return plans
