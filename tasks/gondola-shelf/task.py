"""Installing two hook-in gondola shelf brackets at a marked planogram height and seating a flanged steel shelf.

A powder-coated steel retail gondola upright stands fixed at the back of the bench with two vertical slotted standards
240 mm apart. Each standard has five rectangular slots on a 36 mm vertical pitch, forming four bracket levels numbered 1
to 4; level 3 is marked in yellow across both standards. Two loose stamped-steel shelf brackets start upright in side
cradles on the left and right, and a white flanged steel shelf pan starts flat on low risers in front. Each bracket has
an upper rear J-hook and a lower alignment tongue spaced one slot pitch below it: pushing a level bracket straight at the
upright drives its lower tongue into the steel bridge between slots, so each arm must pitch its bracket nose-up (~28 deg),
pass the upper hook through slot 3, drop it behind the web, and swing the bracket down flat so the lower tongue seats in
slot 2. Both arms then lift the 320 mm shelf by its side fences and lower it flat across both brackets so its front and
rear lips capture them.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY as _READY  # noqa: E402
from yam_arena import ik as _ik  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Colours (sRGB)
SLATE = (0.24, 0.26, 0.29)
DARK = (0.16, 0.17, 0.19)
GALV_STEEL = (0.68, 0.71, 0.74)
BRACKET_GRIP_C = (0.22, 0.44, 0.72)
SHELF_WHITE = (0.93, 0.94, 0.95)
PLAN_YELLOW = (0.95, 0.78, 0.10)
LABEL_WHITE = (0.95, 0.95, 0.95)

X = np.array([1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])
Z = np.array([0.0, 0.0, 1.0])


def Ry(a: float) -> np.ndarray:
    return axis_angle_matrix(Y, a)


def Rz(a: float) -> np.ndarray:
    return axis_angle_matrix(Z, a)


def _boxes_mesh(boxes, colour):
    """Drawn-only mesh (collide='none') from (centre, size) tuples."""
    pts, tris = [], []
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    for c, s in boxes:
        c, h = np.asarray(c, float), np.asarray(s, float) / 2.0
        b = len(pts)
        pts += [c + h * [sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        tris += [(b + i, b + j, b + k) for i, j, k in faces]
    return labkit.Mesh(points=np.array(pts, float), tris=np.array(tris), color=colour, collide="none")


SEG = {"1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc"}


def _digits_front(text: str, x_face: float, y_c: float, z_c: float, h: float, colour):
    """Seven-segment numerals on a vertical face facing -x (towards the robots)."""
    w, t = 0.55 * h, 0.14 * h
    out = []
    segs = {"a": (0, h / 2, w, t), "g": (0, 0, w, t), "d": (0, -h / 2, w, t),
            "f": (w / 2, h / 4, t, h / 2), "b": (-w / 2, h / 4, t, h / 2),
            "e": (w / 2, -h / 4, t, h / 2), "c": (-w / 2, -h / 4, t, h / 2)}
    for n, ch in enumerate(text):
        yc = y_c - n * (w + 0.35 * h)
        for s in SEG.get(ch, ""):
            dy, dz, sy, sz = segs[s]
            out.append(((x_face - 0.0005, yc + dy, z_c + dz), (0.001, sy, sz)))
    return _boxes_mesh(out, colour)


# -- 1. Slotted retail gondola upright (fixed) ------------------------------------------------------------------------
UP_X0 = 0.410          # front face of both slotted standards
WEB_T = 0.0045         # 4.5 mm slotted steel front web
CAV_D = 0.026          # 26 mm hollow channel cavity behind the slots
BACK_T = 0.006         # 6 mm rear channel backplate
COL_Y = {"left_bracket": 0.120, "right_bracket": -0.120}  # 240 mm centre-to-centre
SLOT_W = 0.011         # 11 mm slot width (5 mm bracket blade -> +/-3 mm lateral clearance)
SLOT_H = 0.019         # 19 mm vertical slot opening
PITCH = 0.036          # 36 mm vertical slot pitch (17 mm solid bridge between slots)
SLOT_SILLS = tuple(0.044 + i * PITCH for i in range(5))   # 5 slots: sills at 44, 80, 116, 152, 188 mm
# Bracket levels 1..4 hook their upper J-hook onto slots 1..4 (sills 80, 116, 152, 188 mm) and lower tongue in 0..3:
TARGET_LEVEL = 3
SILL_3 = SLOT_SILLS[TARGET_LEVEL]  # 0.152 m above table
UP_H = 0.232
UP_W = 0.340           # total upright width (-0.170 to +0.170 m)
BASE_H = 0.008


def _build_upright():
    boxes = [
        # Heavy steel baseplate anchored on the bench (x 0.365..0.445, y -0.175..0.175)
        ((UP_X0 + 0.015, 0.0, BASE_H / 2), (0.080, UP_W + 0.010, BASE_H), DARK),
        # Full-width rear backplate behind the channel cavities
        ((UP_X0 + WEB_T + CAV_D + BACK_T / 2, 0.0, UP_H / 2), (BACK_T, UP_W, UP_H), SLATE),
        # Central infill panel between the two slotted columns (y -0.095..+0.095), recessed 3 mm behind web front
        ((UP_X0 + 0.003 + (WEB_T + CAV_D - 0.003) / 2, 0.0, (BASE_H + UP_H) / 2),
         (WEB_T + CAV_D - 0.003, 0.190, UP_H - BASE_H), SLATE),
        # Outer side walls of the upright frame
        *[((UP_X0 + (WEB_T + CAV_D) / 2, sy * (UP_W / 2 - 0.010), (BASE_H + UP_H) / 2),
           (WEB_T + CAV_D, 0.020, UP_H - BASE_H), SLATE) for sy in (-1, 1)],
    ]
    # Build each slotted column web at y = +/-0.120: solid side cheeks + horizontal bridges between slots
    cheek_w = 0.020
    for cy in COL_Y.values():
        for sy in (-1, 1):
            yc = cy + sy * (SLOT_W / 2 + cheek_w / 2)
            boxes.append(((UP_X0 + WEB_T / 2, yc, (BASE_H + UP_H) / 2),
                          (WEB_T, cheek_w, UP_H - BASE_H), DARK))
            # Internal side walls of the channel cavity
            boxes.append(((UP_X0 + WEB_T + CAV_D / 2, cy + sy * 0.018, (BASE_H + UP_H) / 2),
                          (CAV_D, 0.005, UP_H - BASE_H), DARK))
        # Horizontal steel bridges across the slot column: below slot 0, between slots i and i+1, and above slot 4
        z_cursor = BASE_H
        for sill in SLOT_SILLS:
            bh = sill - z_cursor
            if bh > 1e-4:
                boxes.append(((UP_X0 + WEB_T / 2, cy, z_cursor + bh / 2), (WEB_T, SLOT_W, bh), DARK))
            z_cursor = sill + SLOT_H
        if UP_H - z_cursor > 1e-4:
            boxes.append(((UP_X0 + WEB_T / 2, cy, z_cursor + (UP_H - z_cursor) / 2),
                          (WEB_T, SLOT_W, UP_H - z_cursor), DARK))

    # Visual markings: bright yellow planogram band across level 3 + level numerals 1..4
    y_band = [
        # Yellow marker tabs flanking slot 3 on both columns + central level-3 alignment stripe
        *[((UP_X0 - 0.0004, cy + sy * (SLOT_W / 2 + 0.009), SILL_3 + SLOT_H / 2),
           (0.0008, 0.014, SLOT_H)) for cy in COL_Y.values() for sy in (-1, 1)],
        ((UP_X0 + 0.0026, 0.0, SILL_3 + 0.003), (0.0008, 0.180, 0.006)),
    ]
    meshes = [_boxes_mesh(y_band, PLAN_YELLOW)]
    for lvl in (1, 2, 3, 4):
        zc = SLOT_SILLS[lvl] + SLOT_H / 2
        col = PLAN_YELLOW if lvl == TARGET_LEVEL else LABEL_WHITE
        for cy, off in ((COL_Y["left_bracket"], -0.032), (COL_Y["right_bracket"], 0.032)):
            meshes.append(_digits_front(str(lvl), UP_X0 + 0.003, cy + off, zc, 0.017, col))
    return part("gondola_upright", pos=(0.0, 0.0, TOP), fixed=True, friction=0.45, boxes=boxes, meshes=meshes)


UPRIGHT = _build_upright()


# -- 2. Left & right stamped-steel gondola shelf brackets (loose, 0.16 kg each) ---------------------------------------
# Part frame: x = 0 at rear vertical shoulder (meets UP_X0 when seated), z = 0 at underside of upper hook neck (rests
# on SILL_3 when seated). Extends towards -x (towards the robots).
BLADE_T = 0.005
FLANGE_W = 0.018
FLANGE_X0, FLANGE_X1 = -0.143, -0.019   # 124 mm top support flange (centre at -0.081 m)
FLANGE_TOP = 0.005                      # top bearing plane z_local = +5 mm
BRK_BOT = -0.035                        # mid/front web bottom edge z_local = -35 mm (rests on cradle floor)
HEEL_BOT = -0.048                       # rear lower bearing heel z_local = -48 mm (bears on bridge below slot 2)
GRIP_X0, GRIP_X1 = -0.188, -0.160       # 28 mm front grip tab, stepped below and clear in front of the shelf lip
GRIP_W = 0.012                          # 12 mm grip thickness along y
GRIP_Z0, GRIP_Z1 = -0.032, -0.007       # 25 mm tall grip tab stepped 12 mm below flange top

CRADLE_Z = 0.042                        # cradle floor height above table (lifts bracket origin to z_rel = 0.077 m)
BRK_START_X = 0.365                     # rear shoulder x in cradle (grip centre at x = 0.191 m, clear of upright)
BRK_START_Y = 0.245                     # +/-245 mm outboard, clear of upright and shelf


def brk_start(name: str) -> np.ndarray:
    sgn = 1.0 if name == "left_bracket" else -1.0
    return np.array([BRK_START_X, sgn * BRK_START_Y, TOP + CRADLE_Z - BRK_BOT + 0.0003])


def _make_bracket(name: str):
    boxes = [
        # Upper J-hook neck (x -3..10.5 mm, z 0..4.5 mm) and downward locking tooth behind web (x 6.5..10.5, z -6.5..0 mm)
        ((0.00375, 0.0, 0.00225), (0.0135, BLADE_T, 0.0045), GALV_STEEL),
        ((0.0085, 0.0, -0.00325), (0.0040, BLADE_T, 0.0065), GALV_STEEL),
        # Lower alignment tongue 1 slot pitch down (x 0..6.5 mm, z -29..-24 mm)
        ((0.00325, 0.0, -0.0265), (0.0065, BLADE_T, 0.0050), GALV_STEEL),
        # Rear hook riser (-3..0 mm, z -48..0 mm), rear lower heel (-24..-3 mm, z -48..-35 mm), and stepped lower web
        ((-0.0015, 0.0, HEEL_BOT / 2), (0.003, BLADE_T, -HEEL_BOT), GALV_STEEL),
        ((-0.0135, 0.0, (HEEL_BOT + BRK_BOT) / 2), (0.021, BLADE_T, BRK_BOT - HEEL_BOT), GALV_STEEL),
        ((-0.0815, 0.0, (BRK_BOT - 0.007) / 2), (0.157, BLADE_T, -0.007 - BRK_BOT), GALV_STEEL),
        # Web riser directly beneath the 124 mm shelf-bearing flange (-143..-19 mm, z -7..0 mm)
        (((FLANGE_X0 + FLANGE_X1) / 2, 0.0, -0.0035), (FLANGE_X1 - FLANGE_X0, BLADE_T, 0.007), GALV_STEEL),
        # Top horizontal shelf-bearing flange (x -143..-19 mm, y +/-9 mm, z 0..5 mm)
        (((FLANGE_X0 + FLANGE_X1) / 2, 0.0, FLANGE_TOP / 2),
         (FLANGE_X1 - FLANGE_X0, FLANGE_W, FLANGE_TOP), GALV_STEEL),
        # Front blue vinyl-sleeved grip tab (x -188..-160 mm, y +/-6 mm, z -32..-7 mm) + bottom keeper ledge
        (((GRIP_X0 + GRIP_X1) / 2, 0.0, (GRIP_Z0 + GRIP_Z1) / 2),
         (GRIP_X1 - GRIP_X0, GRIP_W, GRIP_Z1 - GRIP_Z0), BRACKET_GRIP_C),
        (((GRIP_X0 + GRIP_X1) / 2, 0.0, GRIP_Z0 - 0.0012),
         (GRIP_X1 - GRIP_X0, GRIP_W + 0.008, 0.0024), BRACKET_GRIP_C),
    ]
    return part(name, pos=tuple(brk_start(name)), mass=0.160, friction=0.85, boxes=boxes)


LEFT_BRACKET = _make_bracket("left_bracket")
RIGHT_BRACKET = _make_bracket("right_bracket")

# Fixed U-channel cradles holding both brackets upright at the start (supporting mid web ahead of the rear lower heel)
CRADLE_BOXES = []
for _sgn in (-1.0, 1.0):
    _cy = _sgn * BRK_START_Y
    _cx = BRK_START_X - 0.088
    CRADLE_BOXES += [
        ((_cx, _cy, CRADLE_Z / 2), (0.104, 0.034, CRADLE_Z), DARK),
        *[((_cx, _cy + e * 0.0095, CRADLE_Z + 0.007), (0.100, 0.010, 0.014), DARK) for e in (-1, 1)],
    ]
BRACKET_CRADLES = part("bracket_cradles", pos=(0.0, 0.0, TOP), fixed=True, friction=0.5, boxes=CRADLE_BOXES)


# -- 3. Flanged white powder-coated steel shelf pan (loose, 0.42 kg) --------------------------------------------------
# Part frame: z = 0 at underside of deck plate (rests on FLANGE_TOP when seated across both brackets).
SHELF_D = 0.146         # 146 mm front-to-back outer depth
SHELF_W = 0.320         # 320 mm left-to-right outer width (-0.160..+0.160 m; brackets sit at +/-0.120 m)
DECK_T = 0.004          # 4 mm deck plate
LIP_T = 0.004           # 4 mm front & rear turndown lips (inner pocket = 138 mm over 124 mm bracket flange -> +/-7 mm)
LIP_DROP = 0.007        # 7 mm downward captive lip front and rear (5 mm vertical clearance above stepped -7 mm web)
FENCE_W = 0.008         # 8 mm thick upright side fences at y = +/-0.156 m for bimanual grasping
FENCE_H = 0.028         # 28 mm tall side fences above deck underside

RISER_Z = 0.056         # starting riser height above table
SHELF_START = np.array([0.138, 0.0, TOP + RISER_Z + 0.0003])  # rear edge at 0.213 m, clear in front of 0.230 m brackets
SHELF_GOAL_X = UP_X0 + (FLANGE_X0 + FLANGE_X1) / 2   # 0.410 - 0.081 = 0.329 m
SHELF_GOAL_Z = SILL_3 + FLANGE_TOP                   # 0.152 + 0.005 = 0.157 m above table

SHELF = part(
    "shelf",
    pos=tuple(SHELF_START),
    mass=0.420,
    friction=0.90,
    boxes=[
        # Main flat deck
        ((0.0, 0.0, DECK_T / 2), (SHELF_D, SHELF_W, DECK_T), SHELF_WHITE),
        # Front and rear captive turndown lips (capture bracket flanges front-to-back)
        *[((sx * (SHELF_D / 2 - LIP_T / 2), 0.0, -LIP_DROP / 2), (LIP_T, SHELF_W, LIP_DROP), SHELF_WHITE)
          for sx in (-1, 1)],
        # Left and right upright end-fences + 4 mm outward bottom grip keeper ledges
        *[((0.0, sy * (SHELF_W / 2 - FENCE_W / 2), FENCE_H / 2), (0.110, FENCE_W, FENCE_H), SHELF_WHITE)
          for sy in (-1, 1)],
        *[((0.0, sy * (SHELF_W / 2 + 0.002), 0.0015), (0.110, 0.004, 0.003), SHELF_WHITE)
          for sy in (-1, 1)],
    ],
)

SHELF_REST = part(
    "shelf_rest",
    pos=(0.0, 0.0, TOP),
    fixed=True,
    friction=0.6,
    boxes=[((SHELF_START[0], sy * 0.095, RISER_Z / 2), (0.110, 0.024, RISER_Z), DARK) for sy in (-1, 1)],
)

OBJECTS = [UPRIGHT, LEFT_BRACKET, RIGHT_BRACKET, BRACKET_CRADLES, SHELF, SHELF_REST]


# -- Scoring ----------------------------------------------------------------------------------------------------------
BRK_NAMES = ("left_bracket", "right_bracket")
LOOSE_NAMES = ("left_bracket", "right_bracket", "shelf")
IMPACT_SPEED = 0.50      # > 0.50 m/s pre-impact downward collider-vertex speed at contact latches 0
TABLE_X0, TABLE_X1 = -0.19, 0.91   # station.TABLE_CENTER[0] = 0.36 +/- 0.55 (station.TABLE_SIZE = 1.10 x 1.30)
TABLE_Y_HALF = 0.65                # station.TABLE_SIZE[1] / 2
FLOOR_Z = -0.0125                  # station.FLOOR_Z
CONTACT_TOL = 0.0008               # 0.8 mm surface contact shell (matches labkit.CONTACT_OFFSET = 0.5 mm, restOffset = 0)
PENETRATION_FLOOR = -0.004         # allow up to 4 mm transient solver penetration at impact
CONTROL_DT = 0.05                  # 20 Hz nominal control step

BRK_POS_TOL = 0.007      # seated bracket shoulder within 7 mm of (UP_X0, col_y, TOP + SILL_3)
BRK_ANG_TOL = 7.0        # seated bracket pitch, roll and yaw within 7 deg
SHELF_X_TOL = 0.012      # seated shelf within 12 mm front-to-back of bracket flange centre
SHELF_Y_TOL = 0.015      # seated shelf within 15 mm of midline
SHELF_Z_TOL = 0.010      # seated shelf underside within 10 mm of bracket flange top
SHELF_SUPPORT_Z_TOL = 0.005  # shelf underside within 5 mm vertically of each seated bracket flange top
SHELF_LEVEL_TOL = 5.0    # seated shelf tilt <= 5 deg, yaw <= 6 deg
HANDS_CLEAR = 0.080      # all four fingertips >= 8 cm from every physical collider surface of scored loose parts
SETTLE_STEPS = 20        # 1.0 s at 20 Hz (21 consecutive samples)
SETTLE_TOL = 0.001       # < 1 mm 3D Euclidean motion across all physical collider vertices over the last 20 steps


def _part_box_colliders_and_verts(spec):
    boxes = []
    verts = []
    for c, s, _k, (yaw, pitch) in spec.boxes:
        c_arr = np.asarray(c, dtype=float)
        h_arr = 0.5 * np.asarray(s, dtype=float)
        R_box = labkit._rot((0.0, pitch, yaw))
        boxes.append((c_arr, h_arr, R_box))
        for dx in (-h_arr[0], h_arr[0]):
            for dy in (-h_arr[1], h_arr[1]):
                for dz in (-h_arr[2], h_arr[2]):
                    verts.append(c_arr + R_box @ np.array([dx, dy, dz], dtype=float))
    arr = np.asarray(verts, dtype=float)
    _, idx = np.unique(np.round(arr, 6), axis=0, return_index=True)
    return tuple(boxes), arr[np.sort(idx)]


_PART_BOX_SPECS = {}
_PART_COLLIDER_VERTS = {}
for _p_spec in (LEFT_BRACKET, RIGHT_BRACKET, SHELF):
    _PART_BOX_SPECS[_p_spec.name], _PART_COLLIDER_VERTS[_p_spec.name] = _part_box_colliders_and_verts(_p_spec)

_state = {}
_hist = []


@on_reset
def _reset_state(env):
    _state.clear()
    _state.update(
        step=0,
        viol=set(),
        prev_verts={},
        prev_down={},
        brk_ok={},
        shelf_ok=False,
        clear_brk={},
        clear_all=False,
        settled=False,
    )
    _hist.clear()


def _tips(env):
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7"], preserve_order=True)[0]
        half = abs(float(robot.data.joint_pos.torch[0, ids].cpu().numpy()[0]))
        out += [mid + half * g[:3, 0], mid - half * g[:3, 0]]
    return out


def _world_verts(name: str, st) -> np.ndarray:
    p = np.asarray(st.pos, dtype=float)
    R = st.rotation()
    return _PART_COLLIDER_VERTS[name] @ R.T + p[None, :]


def _min_tip_dist_to_part(name: str, st, tips) -> float:
    """Exact minimum 3D Euclidean distance from all 4 fingertips to any physical box collider of ``name``."""
    p = np.asarray(st.pos, dtype=float)
    R = st.rotation()
    tips_local = (np.asarray(tips, dtype=float) - p[None, :]) @ R
    best = float("inf")
    for c_box, h_box, R_box in _PART_BOX_SPECS[name]:
        rel = (tips_local - c_box[None, :]) @ R_box
        outside = np.maximum(np.abs(rel) - h_box[None, :], 0.0)
        d = float(np.min(np.linalg.norm(outside, axis=-1)))
        if d < best:
            best = d
    return best


def _bracket_seated_l3(st, col_y: float) -> bool:
    p = np.asarray(st.pos, float)
    R = st.rotation()
    # Check origin at (UP_X0, col_y, TOP + SILL_3)
    if (abs(p[0] - UP_X0) > BRK_POS_TOL or abs(p[1] - col_y) > BRK_POS_TOL
            or abs(p[2] - (TOP + SILL_3)) > BRK_POS_TOL):
        return False
    # Check upper hook tooth is physically behind the 4.5 mm steel web, and lower tongue is inside slot 2
    tooth_w = st.to_world((0.0085, 0.0, -0.00325))
    tongue_w = st.to_world((0.00325, 0.0, -0.0265))
    sill_2 = TOP + SLOT_SILLS[TARGET_LEVEL - 1]
    if tooth_w[0] < UP_X0 + WEB_T + 0.001 or tooth_w[2] > TOP + SILL_3 + 0.002:
        return False
    if not (tongue_w[0] >= UP_X0 - 0.001 and sill_2 <= tongue_w[2] <= sill_2 + SLOT_H):
        return False
    # Orientation within 7 deg of upright & square (R close to identity)
    ang_x = math.degrees(math.acos(max(-1.0, min(1.0, float(R[0, 0])))))
    ang_y = math.degrees(math.acos(max(-1.0, min(1.0, float(R[1, 1])))))
    ang_z = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
    return max(ang_x, ang_y, ang_z) <= BRK_ANG_TOL


def _shelf_seated(sh, bs) -> bool:
    if not all(_bracket_seated_l3(bs[n], COL_Y[n]) for n in BRK_NAMES):
        return False
    p = np.asarray(sh.pos, float)
    R = sh.rotation()
    yaw_err = abs(math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0]))))
    if not (abs(p[0] - SHELF_GOAL_X) <= SHELF_X_TOL
            and abs(p[1]) <= SHELF_Y_TOL
            and abs(p[2] - (TOP + SHELF_GOAL_Z)) <= SHELF_Z_TOL
            and sh.tilt_deg() <= SHELF_LEVEL_TOL
            and yaw_err <= 6.0):
        return False
    # Verify both seated bracket flanges lie inside the shelf's 138 mm captive lip pocket and support the deck underside
    pocket_half_x = 0.5 * SHELF_D - LIP_T
    inner_half_y = 0.5 * SHELF_W - FENCE_W
    for n in BRK_NAMES:
        for fx in (FLANGE_X0, FLANGE_X1):
            fw = np.asarray(bs[n].to_world((fx, 0.0, FLANGE_TOP)), dtype=float)
            fl = R.T @ (fw - p)
            if (abs(float(fl[0])) > pocket_half_x
                    or abs(float(fl[1])) > inner_half_y
                    or abs(float(fl[2])) > SHELF_SUPPORT_Z_TOL):
                return False
    return True


def _check_part_impact(name: str, part_st, verts_w: np.ndarray, st: dict, dt: float) -> None:
    """Latch a violation only when a physical collider vertex contacts the tabletop or floor after > 0.50 m/s descent."""
    pos = np.asarray(part_st.pos, dtype=float)
    lin_v = np.asarray(part_st.lin_vel, dtype=float)
    ang_v = np.asarray(part_st.ang_vel, dtype=float)
    vel_verts = lin_v[None, :] + np.cross(ang_v[None, :], verts_w - pos[None, :])
    down_now = -vel_verts[:, 2]

    prev_v = st["prev_verts"].get(name)
    prev_d = st["prev_down"].get(name)
    if prev_v is not None and prev_d is not None and dt > 0.0:
        fd_down = (prev_v[:, 2] - verts_w[:, 2]) / dt
        for i in range(len(verts_w)):
            x, y, z = float(verts_w[i, 0]), float(verts_w[i, 1]), float(verts_w[i, 2])
            on_table = (
                (TABLE_X0 <= x <= TABLE_X1)
                and (abs(y) <= TABLE_Y_HALF)
                and (PENETRATION_FLOOR <= (z - TOP) <= CONTACT_TOL)
            )
            on_floor = z <= (FLOOR_Z + CONTACT_TOL)
            if on_table or on_floor:
                z_surf = FLOOR_Z if on_floor else TOP
                incident_down = max(float(down_now[i]), float(fd_down[i]))
                prev_z = float(prev_v[i, 2])
                if prev_z > (z_surf + CONTACT_TOL) and (prev_z - float(prev_d[i]) * dt) <= (z_surf + CONTACT_TOL):
                    incident_down = max(incident_down, float(prev_d[i]))
                if incident_down > IMPACT_SPEED:
                    surf = "floor" if on_floor else "table"
                    st["viol"].add(f"{name} impacted {surf} at {incident_down:.2f} m/s")
                    break
    st["prev_verts"][name] = verts_w
    st["prev_down"][name] = down_now


@on_step
def _watch(env):
    S = labkit._SPECS
    st = _state
    st["step"] += 1
    dt = float(getattr(getattr(env, "unwrapped", None), "step_dt", CONTROL_DT))
    bs = {n: labkit._part_state(env, S[n]) for n in BRK_NAMES}
    sh = labkit._part_state(env, S["shelf"])

    verts_w = {n: _world_verts(n, bs[n]) for n in BRK_NAMES}
    verts_w["shelf"] = _world_verts("shelf", sh)

    for n, b in bs.items():
        _check_part_impact(n, b, verts_w[n], st, dt)
    _check_part_impact("shelf", sh, verts_w["shelf"], st, dt)

    tips = _tips(env)
    clear_brk = {n: _min_tip_dist_to_part(n, bs[n], tips) >= HANDS_CLEAR for n in BRK_NAMES}
    clear_sh = _min_tip_dist_to_part("shelf", sh, tips) >= HANDS_CLEAR

    raw_brk = {n: _bracket_seated_l3(bs[n], COL_Y[n]) for n in BRK_NAMES}
    st["brk_ok"] = {n: raw_brk[n] and clear_brk[n] for n in BRK_NAMES}
    st["shelf_ok"] = _shelf_seated(sh, bs)
    st["clear_all"] = clear_brk["left_bracket"] and clear_brk["right_bracket"] and clear_sh

    pts = np.vstack([verts_w["left_bracket"], verts_w["right_bracket"], verts_w["shelf"]])
    _hist.append(pts)
    del _hist[:-(SETTLE_STEPS + 1)]
    st["settled"] = (len(_hist) > SETTLE_STEPS
                     and float(np.max(np.linalg.norm(np.asarray(_hist) - _hist[-1], axis=-1))) < SETTLE_TOL)


def score(objects) -> float:
    st = _state
    if st.get("viol") or not st.get("settled", False):
        return 0.0
    n_brk = sum(bool(v) for v in st.get("brk_ok", {}).values())
    if n_brk == 2 and st.get("shelf_ok", False) and st.get("clear_all", False):
        return 1.0
    return round(0.25 * n_brk, 3)


SUCCESS_HOLD_S = 1.0


def success(objects) -> bool:
    return score(objects) == 1.0


def distance(objects) -> float:
    return float(np.linalg.norm(np.asarray(objects["shelf"].pos, float)
                                - np.array([SHELF_GOAL_X, 0.0, TOP + SHELF_GOAL_Z])))


# -- Scripted demo (uses only observable object poses; zero internal latch/force reads) -------------------------------
TIP_ABOVE_TCP = 0.013


class Orient(float):
    """Waypoint yaw carrying a full 3x3 TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot", False):
    def _grip(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _prev_grip(yaw)

    _grip._rot = True
    _demo.grip_rotation = _grip


def _ik_err(pe, re):
    return pe * 1000.0 + math.degrees(re)


def _endpoints(plan):
    rng = np.random.default_rng(7)
    seeds = [np.array(_READY, float)] + [np.array(_READY, float) + rng.normal(0.0, 0.8, 6) for _ in range(10)]
    q_prev, out, switch = plan.q.copy(), [], set()
    for i, w in enumerate(plan.waypoints):
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, q_prev, base_pos=plan.base_pos, iters=300)
        if _ik_err(pe, re) > 1.5:
            switch.add(i)
            best = (_ik_err(pe, re), 0.0, q)
            for s in [q_prev + rng.normal(0.0, 0.3, 6) for _ in range(4)] + seeds:
                qs, ps, rs = _ik.solve(pos, quat, s, base_pos=plan.base_pos, iters=200)
                e, d = _ik_err(ps, rs), float(np.max(np.abs(qs - q_prev)))
                if (e < 1.0 and (best[0] >= 1.0 or d < best[1])) or (best[0] >= 1.0 and e < best[0]):
                    best = (e, d, qs)
            q = best[2]
        elif (float(np.max(np.abs(q[3:5] - q_prev[3:5]))) > math.radians(70.0)
              or float(np.max(np.abs(q - q_prev))) > math.radians(100.0)):
            switch.add(i)
        out.append(q)
        q_prev = q
    return out, switch


def _seg_idx(plan, t):
    acc = 0.0
    for i, w in enumerate(plan.waypoints):
        if t < acc + w.duration:
            return i, _ik.smoothstep((t - acc) / w.duration)
        acc += w.duration
    return len(plan.waypoints) - 1, 1.0


def _action7(self, t):
    _demo._plan_clock = float(t)
    if getattr(self, "_qends", None) is None:
        self._qends, self._switch = _endpoints(self)
    pos, rot, finger, label = self.target(t)
    quat = _ik.matrix_to_quat_xyzw(rot)
    i, s = _seg_idx(self, t)
    q0 = self._qends[i - 1] if i > 0 else self._q_start
    seed = q0 + s * (self._qends[i] - q0)
    if i in self._switch:
        self.q = seed
        grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
        return np.r_[seed, grip], label
    q, pe, re = _ik.solve(pos, quat, self.q, base_pos=self.base_pos, iters=80)
    if _ik_err(pe, re) > 2.0:
        q2, p2, r2 = _ik.solve(pos, quat, seed, base_pos=self.base_pos, iters=80)
        if _ik_err(p2, r2) < _ik_err(pe, re) - 1.0:
            q, pe, re = q2, p2, r2
    self.q = q
    self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
    self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))
    grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
    return np.r_[q, grip], label


_prev_init = _demo.ArmPlan.__init__
if not getattr(_prev_init, "_gs", False):
    def _init(self, *a, _prev=_prev_init, **k):
        _prev(self, *a, **k)
        self._q_start, self._qends = self.q.copy(), None

    _init._gs = True
    _demo.ArmPlan.__init__ = _init
    _demo.ArmPlan.action7 = _action7


def _appr(R: np.ndarray) -> np.ndarray:
    return R @ _prev_grip(0.0).T @ np.array([0.0, 0.0, -1.0])


def _tcp(tip, R: np.ndarray, past: float = 0.004) -> np.ndarray:
    a = _appr(R)
    return np.asarray(tip, float) + a * (past - TIP_ABOVE_TCP)


class _Plan:
    def __init__(self):
        self.L, self.R = [], []
        self.t = 0.0

    def hold(self, plan, dur, label="hold"):
        w = plan[-1]
        plan.append(Waypoint(dur, w.pos, w.finger, label, w.yaw))

    def phase(self, left=(), right=()):
        tl, tr = sum(w[0] for w in left), sum(w[0] for w in right)
        dur = max(tl, tr)
        for plan, ws, tot in ((self.L, left, tl), (self.R, right, tr)):
            for w in ws:
                plan.append(Waypoint(*w))
            if dur - tot > 1e-9:
                self.hold(plan, dur - tot)
        t0 = self.t
        self.t += dur
        return t0

    def window(self, t0, t1, key, fn, watch):
        _ct["wins"].append((t0, t1, key, fn, watch))


_ct = {"trim": {}, "wins": [], "last": {}}


@on_reset
def _reset_ct(env):
    _ct["trim"] = {}
    _ct["last"] = {}


def _seen(env, name: str) -> np.ndarray:
    return np.asarray(labkit._part_state(env, labkit._SPECS[name]).pos, float)


@on_step
def _ctrack(env):
    t = getattr(_demo, "_plan_clock", 0.0)
    for t0, t1, key, fn, watch in _ct["wins"]:
        if not t0 <= t < t1:
            continue
        p = _seen(env, watch)
        last = _ct["last"].get(key)
        _ct["last"][key] = p
        if last is None or np.max(np.abs(p - last)) > 0.0005:
            continue
        e = np.asarray(fn(env), float)
        tr = _ct["trim"].get(key, np.zeros(3))
        _ct["trim"][key] = np.clip(tr + np.clip(0.45 * e, -0.0015, 0.0015), -0.025, 0.025)


class _Dyn:
    """TCP target plus live visual-pose trim ``key``."""

    def __init__(self, tcp, key):
        self.tcp, self.key = np.asarray(tcp, float), key

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.tcp + _ct["trim"].get(self.key, np.zeros(3)), dtype=dtype or float)


# Grip geometry for brackets: hand pitched 44 deg forward, closing along y across the 12 mm blue grip tab
BRK_OPEN = -0.028
BRK_HOLD = 0.0
R_BRK0 = Ry(-math.radians(44.0)) @ _prev_grip(0.0)
GRIP_LOCAL = np.array([(GRIP_X0 + GRIP_X1) / 2, 0.0, (GRIP_Z0 + GRIP_Z1) / 2 + 0.002])
D_TCP_LOCAL = _tcp(GRIP_LOCAL, R_BRK0, past=0.006)  # TCP in bracket local frame when gripped at deg = 0


def _brk_pose(col_y: float, dx_mm: float, dz_mm: float, pitch_deg: float):
    """(tcp_pos, Orient) placing a held bracket's origin at (UP_X0 + dx, col_y, TOP + SILL_3 + dz) pitched nose-up."""
    Rb = Ry(math.radians(pitch_deg))
    origin = np.array([UP_X0 + dx_mm * 0.001, col_y, TOP + SILL_3 + dz_mm * 0.001])
    return origin + Rb @ D_TCP_LOCAL, Orient(Rb @ R_BRK0)


# Grip geometry for the shelf: hands pitched 22 deg forward, closing along y across the 8 mm upright side fences
SH_OPEN = -0.026
SH_HOLD = 0.0
R_SH = Ry(-math.radians(22.0)) @ _prev_grip(0.0)
O_SH = Orient(R_SH)


def _sh_tcp(shelf_pos, side_sgn: float, up_extra: float = 0.0) -> np.ndarray:
    tip = np.asarray(shelf_pos, float) + np.array([0.0, side_sgn * (SHELF_W / 2 - FENCE_W / 2), 0.016 + up_extra])
    return _tcp(tip, R_SH, past=0.006)


LEFT_PARK = np.array([0.11, 0.28, TOP + 0.22])
RIGHT_PARK = np.array([0.11, -0.28, TOP + 0.22])


def demo():
    _ct["wins"].clear()
    pl = _Plan()

    # Phase 1a: both arms pick up their brackets from the side cradles and carry them pitched 28 deg nose-up in front of slot 3
    def _brk_pick_and_carry(name: str, key: str):
        p0 = brk_start(name)
        cy = COL_Y[name]
        a0 = _appr(R_BRK0)
        o0 = Orient(R_BRK0)
        tcp_pick = p0 + D_TCP_LOCAL
        p_pre, o_pre = _brk_pose(cy, -22.0, 14.0, 28.0)
        return [
            (1.8, tcp_pick - 0.055 * a0 + 0.045 * Z, BRK_OPEN, f"over {name} grip", o0),
            (1.0, tcp_pick - 0.025 * a0, BRK_OPEN, f"approach {name}", o0),
            (0.9, tcp_pick, BRK_OPEN, f"around {name} grip", o0),
            (0.6, tcp_pick, BRK_HOLD, f"grasp {name}", o0),
            (1.2, tcp_pick + 0.075 * Z, BRK_HOLD, f"lift {name} from cradle", o0),
            (2.0, p_pre - 0.025 * X, BRK_HOLD, f"carry {name} pitched 28 deg to column", o_pre),
            (1.0, _Dyn(p_pre, key), BRK_HOLD, f"line {name} hook with level 3 slot", o_pre),
        ]

    pl.phase(left=_brk_pick_and_carry("left_bracket", "bl"),
             right=_brk_pick_and_carry("right_bracket", "br"))

    # Visual alignment hold: observe actual held bracket pose in front of slot 3 before entering the 11 mm slot
    p_pre_l, o_pre_l = _brk_pose(COL_Y["left_bracket"], -22.0, 14.0, 28.0)
    p_pre_r, o_pre_r = _brk_pose(COL_Y["right_bracket"], -22.0, 14.0, 28.0)
    tw_b = pl.phase(left=[(1.4, _Dyn(p_pre_l, "bl"), BRK_HOLD, "hold left_bracket seen in line", o_pre_l)],
                    right=[(1.4, _Dyn(p_pre_r, "br"), BRK_HOLD, "hold right_bracket seen in line", o_pre_r)])
    for name, key in (("left_bracket", "bl"), ("right_bracket", "br")):
        tgt_pre = np.array([UP_X0 - 0.022, COL_Y[name], TOP + SILL_3 + 0.014])
        pl.window(tw_b + 0.25, tw_b + 1.4, key, lambda env, n=name, g=tgt_pre: g - _seen(env, n), name)

    # Phase 1b: thread upper J-hooks through slot 3 at 28 deg, drop teeth behind web, and swing down flat into slot 2
    def _brk_insert(name: str, key: str):
        cy = COL_Y[name]
        p_in1, o_in1 = _brk_pose(cy, -6.0, 14.0, 28.0)
        p_in2, o_in2 = _brk_pose(cy, 2.8, 14.0, 28.0)
        p_drp, o_drp = _brk_pose(cy, 2.0, 6.8, 28.0)
        p_sw1, o_sw1 = _brk_pose(cy, 0.5, 4.2, 16.0)
        p_sw2, o_sw2 = _brk_pose(cy, -0.4, 1.8, 5.0)
        p_seat, o_seat = _brk_pose(cy, -0.5, 0.5, 0.0)
        a_seat = _appr(o_seat.rot)
        return [
            (1.1, _Dyn(p_in1, key), BRK_HOLD, f"advance {name} hook to slot mouth", o_in1),
            (1.3, _Dyn(p_in2, key), BRK_HOLD, f"pass {name} upper hook through slot 3", o_in2),
            (1.3, _Dyn(p_drp, key), BRK_HOLD, f"drop {name} tooth behind upright web", o_drp),
            (1.3, _Dyn(p_sw1, key), BRK_HOLD, f"swing {name} down towards slot 2", o_sw1),
            (1.2, _Dyn(p_sw2, key), BRK_HOLD, f"seat {name} lower tongue into slot 2", o_sw2),
            (1.0, _Dyn(p_seat, key), BRK_HOLD, f"settle {name} flat on level 3 sill", o_seat),
            (0.6, _Dyn(p_seat, key), BRK_OPEN, f"release {name}", o_seat),
            (1.0, p_seat - 0.045 * a_seat + 0.030 * Z, BRK_OPEN, f"back off {name}", o_seat),
        ]

    pl.phase(left=_brk_insert("left_bracket", "bl"), right=_brk_insert("right_bracket", "br"))

    # Phase 2: both arms grasp the shelf's upright side fences, lift it clear in front of the seated brackets, carry
    # it over the brackets, visually verify alignment, and lower its captive lips onto both brackets simultaneously
    a_sh = _appr(R_SH)
    sh0 = SHELF_START
    sh_lift = np.array([SHELF_START[0] - 0.005, 0.0, TOP + SHELF_GOAL_Z + 0.050])
    sh_over = np.array([SHELF_GOAL_X, 0.0, TOP + SHELF_GOAL_Z + 0.050])
    sh_near = np.array([SHELF_GOAL_X, 0.0, TOP + SHELF_GOAL_Z + 0.012])
    sh_seat = np.array([SHELF_GOAL_X, 0.0, TOP + SHELF_GOAL_Z + 0.001])

    def _flange_over_goal(env) -> np.ndarray:
        S = labkit._SPECS
        fl_c = (0.5 * (FLANGE_X0 + FLANGE_X1), 0.0, FLANGE_TOP)
        bl = np.asarray(labkit._part_state(env, S["left_bracket"]).to_world(fl_c), float)
        br = np.asarray(labkit._part_state(env, S["right_bracket"]).to_world(fl_c), float)
        return 0.5 * (bl + br) + np.array([0.0, 0.0, 0.050])

    def _sh_pick_and_carry(sgn: float, side: str):
        g0 = _sh_tcp(sh0, sgn)
        g_lift = _sh_tcp(sh_lift, sgn)
        g_over = _sh_tcp(sh_over, sgn)
        return [
            (1.6, g0 - 0.045 * a_sh + 0.050 * Z, SH_OPEN, f"{side} over shelf fence", O_SH),
            (1.0, g0 - 0.020 * a_sh, SH_OPEN, f"{side} approach shelf fence", O_SH),
            (0.8, g0, SH_OPEN, f"{side} around shelf fence", O_SH),
            (0.6, g0, SH_HOLD, f"{side} grasp shelf fence", O_SH),
            (1.6, g_lift, SH_HOLD, f"{side} lift shelf level above brackets", O_SH),
            (2.0, _Dyn(g_over, "sh"), SH_HOLD, f"{side} carry shelf over seated brackets", O_SH),
        ]

    pl.phase(left=_sh_pick_and_carry(+1.0, "left"), right=_sh_pick_and_carry(-1.0, "right"))
    tw_s = pl.phase(left=[(1.4, _Dyn(_sh_tcp(sh_over, +1.0), "sh"), SH_HOLD, "left hold shelf seen over brackets", O_SH)],
                    right=[(1.4, _Dyn(_sh_tcp(sh_over, -1.0), "sh"), SH_HOLD, "right hold shelf seen over brackets", O_SH)])
    pl.window(tw_s + 0.25, tw_s + 1.4, "sh", lambda env: _flange_over_goal(env) - _seen(env, "shelf"), "shelf")

    def _sh_place_and_park(sgn: float, side: str):
        g_near = _sh_tcp(sh_near, sgn)
        g_seat = _sh_tcp(sh_seat, sgn)
        park = LEFT_PARK if sgn > 0 else RIGHT_PARK
        return [
            (1.2, _Dyn(g_near, "sh"), SH_HOLD, f"{side} align captive lips over flanges", O_SH),
            (1.4, _Dyn(g_seat, "sh"), SH_HOLD, f"{side} seat shelf onto level-3 brackets", O_SH),
            (0.6, _Dyn(g_seat, "sh"), SH_OPEN, f"{side} release shelf", O_SH),
            (1.0, g_seat - 0.045 * a_sh + 0.040 * Z, SH_OPEN, f"{side} lift clear of shelf", O_SH),
            (1.8, park, SH_OPEN, f"{side} park clear", O_SH),
            (2.5, park, SH_OPEN, f"{side} hold still", O_SH),
        ]

    pl.phase(left=_sh_place_and_park(+1.0, "left"), right=_sh_place_and_park(-1.0, "right"))
    return {"left": pl.L, "right": pl.R}
