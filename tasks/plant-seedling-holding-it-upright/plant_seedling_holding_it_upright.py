"""On the station table, in front of the arms: a low potting bench with a square planting hole in it. A seedling stands in
a small nursery cell on the bench, and a tray of soil clods lies on the bench with its open front at the hole's edge.

Task: plant the seedling. Take it out of its cell, stand it upright in the middle of the planting hole, tip the soil
into the hole around it, and let go so that it stands on its own. The seedling's root plug has a round bottom: on the
hole's flat floor the seedling falls over unless it is held, and only once soil is packed around the plug does it stand
by itself. One hand holds it upright while the other tips the soil in.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- bench: fixed, 5 cm tall, from x 0.18 to 0.46 and y -0.14 to 0.16, with a planting hole 10 x 10 cm and 4.8 cm deep
  centred at (0.34, 0.03).
- nursery cell: fixed, 4.4 x 4.4 cm inside and 4.5 cm deep, on the bench at (0.26, 0.08).
- seedling: a root plug 4 cm across and 4.5 cm tall with a round bottom, a stem 0.8 cm square up to 15 cm, and a crown
  of three leaves 5 cm long at its top; 80 g, most of it in the plug.
- tray: 8 cm wide, 10 cm deep, 2.5 cm walls, open at the front, 60 g, with a handle post 4 cm tall just behind its
  back wall; its open front overhangs the hole's near edge by 1 cm.
- soil: 45 clods, 1.4 cm cubes of 3 g, in two layers in the tray.

Full score: the seedling upright (within 10 degrees) in the hole, its plug in the middle (within 2 cm) and down on the
hole's floor, standing on its own (both grippers 3.5 cm or more from it), with at least 32 of the 45 clods in the hole;
still for 2 s.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset, on_step,  # noqa: E402
                     rotation)
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "best"  # partial credit: the best reached during the run

# -- layout (world frame) ---------------------------------------------------------------------------------------------
BENCH_X, BENCH_Y, BENCH_H = (0.18, 0.46), (-0.14, 0.16), 0.05
HOLE_XY, HOLE_HALF, HOLE_FLOOR = (0.34, 0.03), 0.05, 0.002  # 10 x 10 cm, 4.8 cm deep
CELL_XY, CELL_IN, CELL_T, CELL_H = (0.26, 0.08), 0.044, 0.003, 0.045
BENCH_TOP = TOP + BENCH_H
FLOOR_TOP = TOP + HOLE_FLOOR

# -- the seedling (origin: the bottom of its root plug) ------------------------------------------------------------------
PLUG_R, PLUG_H = 0.020, 0.045  # a round bottom (a sphere) under a short cylinder
STEM_W, STEM_TOP = 0.008, 0.150
LEAF = (0.050, 0.018, 0.003)
LEAF_Z = 0.147
SEEDLING_KG = 0.08
PLUG_C, STEM_C, LEAF_C = (0.36, 0.25, 0.15), (0.33, 0.55, 0.22), (0.30, 0.64, 0.26)

# -- the tray (origin: the middle of its floor's underside; its open front towards +y) ------------------------------------
TRAY_W, TRAY_L, TRAY_FLOOR, TRAY_WALL, TRAY_T = 0.080, 0.100, 0.004, 0.025, 0.003
TRAY_POST = ((0.0, -TRAY_L / 2 - 0.012, 0.045), (0.020, 0.012, 0.040))  # a handle post 1.2 cm behind the back wall
TRAY_BRIDGE = ((0.0, -TRAY_L / 2 - 0.004, 0.030), (0.020, 0.010, 0.010))  # joining it to the wall
TRAY_GRIP = (0.0, -TRAY_L / 2 - 0.012, 0.052)  # the TCP on the post: the pads cover its upper 2.6 cm
TRAY_KG = 0.06
OVERHANG = 0.01  # the tray's open front reaches this far over the hole's near edge
TRAY_XY = (HOLE_XY[0], HOLE_XY[1] - HOLE_HALF + OVERHANG - TRAY_L / 2)
TRAY_C, HANDLE_C = (0.20, 0.42, 0.62), (0.15, 0.15, 0.16)

# -- the soil ---------------------------------------------------------------------------------------------------------
CLOD, CLOD_KG, N_CLODS = 0.014, 0.003, 45
CLOD_MU = 0.3  # crumbly soil: 0.45 jammed the clods at the tray's mouth (11 of 45 in at 43 degrees)
SOIL_C, BENCH_C, FLOOR_C, CELL_C = (0.40, 0.29, 0.19), (0.66, 0.52, 0.36), (0.28, 0.20, 0.14), (0.14, 0.14, 0.15)


def _bench() -> list:
    (x0, x1), (y0, y1) = BENCH_X, BENCH_Y
    hx, hy = HOLE_XY
    hx0, hx1, hy0, hy1 = hx - HOLE_HALF, hx + HOLE_HALF, hy - HOLE_HALF, hy + HOLE_HALF
    z = TOP + BENCH_H / 2

    def block(name, xa, xb, ya, yb):
        return Box(name, pos=((xa + xb) / 2, (ya + yb) / 2, z), size=(xb - xa, yb - ya, BENCH_H), color=BENCH_C)

    return [block("bench_near", x0, hx0, y0, y1), block("bench_far", hx1, x1, y0, y1),
            block("bench_right", hx0, hx1, y0, hy0), block("bench_left", hx0, hx1, hy1, y1),
            Box("hole_floor", pos=(hx, hy, TOP + HOLE_FLOOR / 2), size=(2 * HOLE_HALF, 2 * HOLE_HALF, HOLE_FLOOR),
                color=FLOOR_C)]


def _cell() -> list:
    cx, cy = CELL_XY
    a, t, z = CELL_IN / 2 + CELL_T / 2, CELL_T, BENCH_TOP + CELL_H / 2
    long = CELL_IN + 2 * CELL_T
    return [Box("cell_front", pos=(cx + a, cy, z), size=(t, long, CELL_H), color=CELL_C),
            Box("cell_back", pos=(cx - a, cy, z), size=(t, long, CELL_H), color=CELL_C),
            Box("cell_left", pos=(cx, cy + a, z), size=(CELL_IN, t, CELL_H), color=CELL_C),
            Box("cell_right", pos=(cx, cy - a, z), size=(CELL_IN, t, CELL_H), color=CELL_C)]


def _seedling() -> Body:
    shapes = [SphereShape((0.0, 0.0, PLUG_R), PLUG_R, PLUG_C),
              CylShape((0.0, 0.0, (PLUG_R + PLUG_H) / 2), PLUG_R, PLUG_H - PLUG_R, PLUG_C),
              BoxShape((0.0, 0.0, (PLUG_H + STEM_TOP) / 2), (STEM_W, STEM_W, STEM_TOP - PLUG_H), STEM_C)]
    shapes += [BoxShape((0.0, 0.0, LEAF_Z), LEAF, LEAF_C, yaw=k * math.pi / 3) for k in range(3)]
    return Body.build("seedling", pos=(*CELL_XY, BENCH_TOP + 0.0005), mass=SEEDLING_KG, shapes=shapes,
                      friction=(0.8, 0.7), damping=(0.05, 0.2), color=STEM_C)


def _tray() -> Body:
    hw, hl = TRAY_W / 2, TRAY_L / 2
    shapes = [BoxShape((0.0, 0.0, TRAY_FLOOR / 2), (TRAY_W, TRAY_L, TRAY_FLOOR), TRAY_C),
              BoxShape((hw - TRAY_T / 2, 0.0, TRAY_FLOOR + TRAY_WALL / 2), (TRAY_T, TRAY_L, TRAY_WALL), TRAY_C),
              BoxShape((-hw + TRAY_T / 2, 0.0, TRAY_FLOOR + TRAY_WALL / 2), (TRAY_T, TRAY_L, TRAY_WALL), TRAY_C),
              BoxShape((0.0, -hl + TRAY_T / 2, TRAY_FLOOR + 0.015), (TRAY_W, TRAY_T, 0.030), TRAY_C),
              BoxShape(TRAY_BRIDGE[0], TRAY_BRIDGE[1], HANDLE_C), BoxShape(TRAY_POST[0], TRAY_POST[1], HANDLE_C)]
    return Body.build("tray", pos=(*TRAY_XY, BENCH_TOP + 0.0005), mass=TRAY_KG, shapes=shapes, friction=(0.7, 0.6),
                      color=TRAY_C)


def _clods() -> list[Body]:
    tx, ty = TRAY_XY
    pitch = CLOD + 0.0005
    xs = [(i - 2) * pitch for i in range(5)]
    ys0 = -TRAY_L / 2 + TRAY_T + 0.002 + CLOD / 2
    spots = [(x, ys0 + j * pitch, 0) for j in range(6) for x in xs]  # 30 on the floor, from the back wall forward
    spots += [(x, ys0 + j * pitch, 1) for j in range(3) for x in xs]  # 15 more on top, at the back
    z0 = BENCH_TOP + 0.0005 + TRAY_FLOOR + 0.0005
    shape = BoxShape((0.0, 0.0, CLOD / 2), (CLOD, CLOD, CLOD), SOIL_C)
    return [Body.build(f"clod_{i:02d}", pos=(tx + x, ty + y, z0 + layer * (CLOD + 0.0005)), mass=CLOD_KG,
                       color=SOIL_C, shapes=[shape], friction=(CLOD_MU, CLOD_MU - 0.05), damping=(0.05, 0.3))
            for i, (x, y, layer) in enumerate(spots[:N_CLODS])]


SEEDLING, TRAY = _seedling(), _tray()
CLODS = _clods()
CLOD_NAMES = [c.name for c in CLODS]
OBJECTS = [*_bench(), *_cell(), SEEDLING, TRAY, *CLODS]

# -- scoring --------------------------------------------------------------------------------------------------------
UPRIGHT = math.radians(10)
CENTRED = 0.02
DOWN = 0.015  # the plug's bottom at most this far above the hole's floor
CLEAR = 0.035  # "on its own": both grippers at least this far from the seedling
PACKED, SOME = 32, 10  # the demo packs 36 of 45; 32 leaves a margin
_DEBUG = os.environ.get("PLANT_DEBUG") == "1"
_EP = {"step": 0}
_TCP: dict[str, np.ndarray] = {}
_ARM_IDS: dict[str, list[int]] = {}


@on_reset
def _reset():
    _EP["step"] = 0
    _TCP.clear()


@on_step
def _track_grippers(env) -> None:
    """Each gripper's TCP from its arm's joint positions (yam_arena's own URDF kinematics)."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    base = getattr(env, "unwrapped", env)
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0, _ARM_IDS[side]]
        _TCP[side] = np.asarray(ik.tcp_pose(q, station.BASES[side])[0], float)


def _seedling_pose(o):
    s = o["seedling"]
    r = rotation(s.quat_xyzw)
    return np.asarray(s.pos, float), r


def _clear_of_grippers(o) -> bool:
    """Both TCPs at least CLEAR from the seedling's stem and plug (a segment along its axis)."""
    if len(_TCP) < 2:
        return False
    p, r = _seedling_pose(o)
    a, b = p + r[:, 2] * PLUG_R, p + r[:, 2] * STEM_TOP
    for tcp in _TCP.values():
        t = float(np.clip(np.dot(tcp - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0))
        if np.linalg.norm(tcp - (a + t * (b - a))) < CLEAR:
            return False
    return True


def _in_hole(o, name) -> bool:
    x, y, z = o[name].pos
    hx, hy = HOLE_XY
    return bool(abs(x - hx) < HOLE_HALF and abs(y - hy) < HOLE_HALF and FLOOR_TOP - 0.005 < z < BENCH_TOP)


def _tally(o) -> dict:
    p, r = _seedling_pose(o)
    tilt = float(math.acos(max(-1.0, min(1.0, r[2, 2]))))
    centre = p + r[:, 2] * PLUG_R  # the round bottom's centre
    planted = (tilt < UPRIGHT and math.hypot(centre[0] - HOLE_XY[0], centre[1] - HOLE_XY[1]) <= CENTRED
               and p[2] - FLOOR_TOP <= DOWN)
    return {"tilt": tilt, "planted": planted, "soil": sum(_in_hole(o, n) for n in CLOD_NAMES),
            "alone": _clear_of_grippers(o)}


def _points(t) -> float:
    if not t["planted"]:
        return 0.0
    if t["soil"] >= PACKED:
        return 1.0 if t["alone"] else 0.5
    return 0.25 if t["soil"] >= SOME else 0.0


def _debug(o, t) -> None:
    _EP["step"] += 1
    if _EP["step"] % 10 != 1:
        return
    p, _ = _seedling_pose(o)
    tray = o["tray"]
    tcp = {k: np.round(v, 3).tolist() for k, v in _TCP.items()}
    print(f"DBG step={_EP['step']} seedling=({p[0]:.3f},{p[1]:.3f},{p[2] - TOP:.3f}) tilt={math.degrees(t['tilt']):.0f} "
          f"tray=({tray.pos[0]:.3f},{tray.pos[1]:.3f},{tray.pos[2] - TOP:.3f}) "
          f"tray_tilt={math.degrees(math.acos(max(-1.0, min(1.0, rotation(tray.quat_xyzw)[2, 2])))):.0f} "
          f"soil={t['soil']} planted={t['planted']} alone={t['alone']} tcp={tcp} pts={_points(t)}", flush=True)


def score(objects) -> float:
    """Partial credit (success scores 1.0; the best over the run counts): 0.5 when the seedling stands upright in the
    middle of the hole with at least 32 of the 45 clods in the hole but a gripper still holds it; 0.25 when it stands
    upright there with at least 10 clods in; 0 otherwise."""
    return min(_points(_tally(objects)), 0.5)


def success(objects) -> bool:
    t = _tally(objects)
    if _DEBUG:
        _debug(objects, t)
    return _points(t) >= 1.0 and objects["seedling"].resting()


# -- scripted demo --------------------------------------------------------------------------------------------------
# The left hand takes the seedling's stem from its own side, level, lifts it out of the cell, carries it over the hole
# and lowers it until the plug sits on the hole's floor, and holds it there. The right hand takes the tray's handle from
# above and lifts the tray's back, so that the tray turns about its front edge on the bench and the clods slide into
# the hole around the plug; it lowers the tray back and lets go. Then the left hand lets go and draws back.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


STEM_ROT = _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2)  # left, level, pointing -y, fingers closing front and back
TRAY_ROT = grip_rotation(math.pi / 2)  # right, from above, fingers closing along x (the tray turns between them)
OPEN = -0.02
STEM_GRIP = float(os.environ.get("PLANT_STEM_GRIP", "-0.002"))  # 4 mm between the fingers round the 8 mm stem
POST_GRIP = float(os.environ.get("PLANT_POST_GRIP", "-0.006"))  # 1.2 cm round the 2 cm post
STEM_UP = 0.115  # grip the stem this far above the plug's bottom: the fingers' roots reach 3.4 cm lower
LIFT = 0.055  # lift the seedling this much: the plug clears the cell's 4.5 cm walls
TILT = math.radians(float(os.environ.get("PLANT_TILT", "50")))  # commanded; the tray reaches ~41 deg, the clods slide out
PIVOT = np.array([HOLE_XY[0], HOLE_XY[1] - HOLE_HALF, BENCH_TOP])  # the bench edge the tray turns about
HANDLE_AT = np.array([TRAY_XY[0], TRAY_XY[1] + TRAY_GRIP[1], BENCH_TOP + TRAY_GRIP[2]])


def _good_ik(arm: str, pos, rot):
    import itertools

    from yam_arena import ik, station

    best = None
    for sd in itertools.product((0.0,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2), (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list[Waypoint]:
    """From READY along a straight line in joint space (the big turn of the wrist stays on one IK branch)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


LEFT_TIMES = [1.5, 1.0, 0.4, 1.5, 2.0, 1.5, 0.6]  # slide in, grip, steady, lift, carry, lower, settle
RIGHT_TIMES = [1.0, 1.0, 0.4]  # down onto the handle, grip, steady
TIP_S, HOLD_S, BACK_S = 2.5, 3.5, 2.0


def _tray_handle(theta: float) -> np.ndarray:
    """The TCP on the handle post with the tray's back lifted so that the tray turns ``theta`` about its front edge."""
    rel = HANDLE_AT - PIVOT
    c, s = math.cos(theta), math.sin(theta)
    return PIVOT + np.array([rel[0], rel[1] * c + rel[2] * s, -rel[1] * s + rel[2] * c])


def _right() -> list[Waypoint]:
    def tcp(theta):
        return _tray_handle(theta)

    out = _lead_in("right", tcp(0.0) + [0.0, 0.0, 0.05], TRAY_ROT, OPEN, "over the tray's handle")
    out += [_w(RIGHT_TIMES[0], tcp(0.0), OPEN, "down round the handle", TRAY_ROT),
            _w(RIGHT_TIMES[1], tcp(0.0), POST_GRIP, "grip the handle", TRAY_ROT),
            _w(RIGHT_TIMES[2] + sum(LEFT_TIMES) + 3.0 - sum(RIGHT_TIMES), tcp(0.0), POST_GRIP,
               "wait until the seedling is held upright in the hole", TRAY_ROT)]
    n = 6
    out += [_w(TIP_S / n, tcp(TILT * k / n), POST_GRIP, "lift the tray's back: the soil slides into the hole", TRAY_ROT)
            for k in range(1, n + 1)]
    out.append(_w(HOLD_S, tcp(TILT), POST_GRIP, "hold it tipped", TRAY_ROT))
    out += [_w(BACK_S / n, tcp(TILT * (n - k) / n), POST_GRIP, "lower the tray", TRAY_ROT) for k in range(1, n + 1)]
    out += [_w(0.8, tcp(0.0), OPEN, "let go", TRAY_ROT), _w(1.0, tcp(0.0) + [0.0, 0.0, 0.06], OPEN, "rise", TRAY_ROT)]
    return out


def _left() -> list[Waypoint]:
    in_cell = np.array([CELL_XY[0], CELL_XY[1], BENCH_TOP + STEM_UP])
    beside = in_cell + [0.0, 0.03, 0.0]
    lifted = in_cell + [0.0, 0.0, LIFT]
    over = np.array([HOLE_XY[0], HOLE_XY[1], lifted[2]])
    planted = np.array([HOLE_XY[0], HOLE_XY[1], FLOOR_TOP + STEM_UP + 0.002])
    out = _lead_in("left", beside, STEM_ROT, OPEN, "level, beside the seedling's stem")
    out += [_w(LEFT_TIMES[0], in_cell, OPEN, "slide in round the stem", STEM_ROT),
            _w(LEFT_TIMES[1], in_cell, STEM_GRIP, "grip the stem", STEM_ROT),
            _w(LEFT_TIMES[2], in_cell, STEM_GRIP, "steady", STEM_ROT),
            _w(LEFT_TIMES[3], lifted, STEM_GRIP, "lift it out of its cell", STEM_ROT),
            _w(LEFT_TIMES[4], over, STEM_GRIP, "carry it over the hole", STEM_ROT),
            _w(LEFT_TIMES[5], planted, STEM_GRIP, "lower it into the hole, upright", STEM_ROT),
            _w(LEFT_TIMES[6] + 3.0, planted, STEM_GRIP, "hold it upright", STEM_ROT),
            _w(TIP_S + HOLD_S + BACK_S + 0.5, planted, STEM_GRIP, "keep holding it while the soil goes in", STEM_ROT),
            _w(0.8, planted, OPEN, "let go: it stands in the soil", STEM_ROT),
            _w(1.0, planted + [0.0, 0.05, 0.0], OPEN, "draw back", STEM_ROT),
            _w(1.0, planted + [0.0, 0.05, 0.06], OPEN, "rise", STEM_ROT),
            _w(2.5, planted + [0.0, 0.05, 0.06], OPEN, "wait", STEM_ROT)]
    return out


def demo():
    return {"left": _left(), "right": _right()}
