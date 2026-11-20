"""A steam-table section with three 1/3-size hotel pans, two with a serving ladle in them: cover every pan.

On the table:
- a stainless steam-table section (a 34 x 55 cm base with a 4 mm stainless surround) holding three 1/3-size stainless
  hotel pans (325 x 176 mm flange to flange, 100 mm deep) side by side, their long sides toward the robot (x), full of
  food (rice, 4.5 cm deep). The left and right pans each hold a stainless 30 cm-handled ladle with its bowl in the food
  and its handle leaning on the far rim and pointing away from the robot; the middle pan has no ladle
- three stainless 1/3-size pan covers lying on the table beside the section, each with a strap handle on top: a solid
  one (left of the section, next to it) and two notched ones (a 5 x 6 cm notch in one short end, for a ladle handle):
  one far left of the section and one right of it. Both notched covers lie with the notch toward the robot

The instruction gives only the goal (cover every pan to keep the food hot). During service the ladles stay in their
pans, so a cover that fits a pan with a ladle in it has to have its notch around the handle: a solid cover, or a
notched one with its notch the wrong way round, rests on the handle and sits proud of the rim.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

STEEL, STEEL_DARK = (0.76, 0.78, 0.81), (0.62, 0.64, 0.68)
RICE, SAUCE, VEG = (0.93, 0.89, 0.72), (0.55, 0.30, 0.13), (0.42, 0.58, 0.22)
LADLE_STEEL = (0.84, 0.85, 0.87)

# -- the steam-table section and its three pans (z values are heights above the table top) ---------------------------
CX = 0.30  # the pans' common centre, metres out from the arm bases
PAN_Y = (0.178, 0.0, -0.178)  # left, middle, right pan centres (y)
PAN_FOOD = (RICE, VEG, SAUCE)
PAN_L, PAN_W, PAN_FLANGE = 0.325, 0.176, 0.010  # 1/3-size hotel pan, flange to flange
WALL = 0.003
BASE_T = 0.002
RIM = BASE_T + 0.100  # the pans' rim top (the covers' seat) above the table
FOOD_TOP = BASE_T + WALL + 0.045  # 4.5 cm of food
SHELL_T = 0.004

# -- the ladles (pan frame: x out from the pan centre toward the far end) ------------------------------------------
LADLE_BOWL_X = 0.03  # the bowl's centre, 3 cm beyond the pan's centre
BOWL_R, BOWL_H = 0.035, 0.030
HANDLE_L, HANDLE_W, HANDLE_T = 0.30, 0.018, 0.008
LADLE_KG = 0.15
WALL_IN_X = PAN_L / 2 - PAN_FLANGE - WALL  # the far wall's inner top edge, 0.1495 from the pan centre
_Z0 = FOOD_TOP + BOWL_H - 0.006  # the handle's centre line where it leaves the bowl
_T = 0.27
for _ in range(60):  # slope of the handle so that its underside just touches the far wall's inner top edge
    _T = (RIM + HANDLE_T / 2 * math.sqrt(1 + _T * _T) - _Z0) / (WALL_IN_X - LADLE_BOWL_X)
HANDLE_ALPHA = math.atan(_T)

# -- the covers ---------------------------------------------------------------------------------------------------
PLATE_T = 0.003
NOTCH_D, NOTCH_W = 0.060, 0.050  # notch depth (along the cover) and width
POST_H, POST_X = 0.016, 0.040
STRAP_L, STRAP_W, STRAP_T = 0.100, 0.020, 0.010
# real 1/3-size covers: ~0.7 mm sheet, 325 x 176 mm -> 0.32 kg, plus a rolled rim and strap = about 0.35 kg. The sim plate is
# 3 mm (thicker, for stable contact) but carries the real mass.
COVER_KG = {"cover_solid": 0.35, "cover_a": 0.33, "cover_b": 0.33}  # notched: less sheet
COVER_Y = {"cover_solid": 0.370, "cover_a": 0.554, "cover_b": -0.370}
BAR_DZ = PLATE_T / 2 + POST_H + STRAP_T / 2  # the strap bar's centre above the plate's centre (the grip)


def _cover_parts(notch_sign):
    """(main box size, its offset from the plate's centre, [(size, offset from the main box's centre, rgb)])."""
    strap = [
        ((STRAP_W, 0.012, POST_H), (0.0, sy * POST_X, PLATE_T / 2 + POST_H / 2), STEEL_DARK) for sy in (-1, 1)
    ] + [((STRAP_W, STRAP_L, STRAP_T), (0.0, 0.0, BAR_DZ), STEEL_DARK)]  # strap bar along y (across the cover)
    if notch_sign == 0:
        return (PAN_L, PAN_W, PLATE_T), (0.0, 0.0), strap
    s = notch_sign
    main_len = PAN_L - NOTCH_D
    mc = -s * NOTCH_D / 2  # the main block's centre
    side_w = (PAN_W - NOTCH_W) / 2
    sides = [((NOTCH_D, side_w, PLATE_T), (s * (PAN_L / 2 - NOTCH_D / 2), sy * (NOTCH_W / 2 + side_w / 2), 0.0), STEEL)
             for sy in (-1, 1)]
    return (main_len, PAN_W, PLATE_T), (mc, 0.0), sides + strap


def _shifted(parts, main_off):
    return [(sz, (o[0] - main_off[0], o[1] - main_off[1], o[2]), *rest) for sz, o, *rest in parts]


def _plate_z():
    return TOP + PLATE_T / 2


OBJECTS = []
_PARTS = {}

# the fixed steam-table section: base plate + surround + three pans (as one compound) -----------------------------
_slab = (PAN_L + 2 * (SHELL_T + 0.002) + 0.0, 0.554, BASE_T)
OBJECTS.append(Box("steam_table", size=_slab, pos=(CX, 0.0, TOP + BASE_T / 2), color=STEEL_DARK))
_fixed = []


def _fx(size, x, y, z, rgb=STEEL):
    _fixed.append((size, (x - CX, y, z - BASE_T / 2), rgb))


_yo = PAN_Y[0] + PAN_W / 2 + 0.002 + SHELL_T / 2  # surround, outside the pans' flanges
_fx((PAN_L + 2 * (SHELL_T + 0.002), SHELL_T, RIM), CX, _yo, RIM / 2)
_fx((PAN_L + 2 * (SHELL_T + 0.002), SHELL_T, RIM), CX, -_yo, RIM / 2)
for _sx in (-1, 1):
    _fx((SHELL_T, 2 * _yo + SHELL_T, RIM), CX + _sx * (PAN_L / 2 + 0.002 + SHELL_T / 2), 0.0, RIM / 2)
for _cy, _food in zip(PAN_Y, PAN_FOOD):
    _fx((PAN_L - 2 * PAN_FLANGE, PAN_W - 2 * PAN_FLANGE, WALL), CX, _cy, BASE_T + WALL / 2)  # floor
    _hw = (PAN_W - 2 * PAN_FLANGE) / 2 - WALL / 2
    _hl = (PAN_L - 2 * PAN_FLANGE) / 2 - WALL / 2
    for _sy in (-1, 1):  # long walls (along x)
        _fx((PAN_L - 2 * PAN_FLANGE, WALL, RIM - BASE_T), CX, _cy + _sy * _hw, (RIM + BASE_T) / 2)
        _fx((PAN_L, PAN_FLANGE, WALL), CX, _cy + _sy * (PAN_W / 2 - PAN_FLANGE / 2), RIM - WALL / 2)  # flanges
    for _sx in (-1, 1):  # end walls
        _fx((WALL, PAN_W - 2 * PAN_FLANGE - 2 * WALL, RIM - BASE_T), CX + _sx * _hl, _cy, (RIM + BASE_T) / 2)
        _fx((PAN_FLANGE, PAN_W - 2 * PAN_FLANGE, WALL), CX + _sx * (PAN_L / 2 - PAN_FLANGE / 2), _cy, RIM - WALL / 2)
    _fx((PAN_L - 2 * PAN_FLANGE - 2 * WALL - 0.002, PAN_W - 2 * PAN_FLANGE - 2 * WALL - 0.002, FOOD_TOP - BASE_T - WALL),
        CX, _cy, (FOOD_TOP + BASE_T + WALL) / 2, _food)
mech.compound("steam_table", _fixed)

# the ladles ------------------------------------------------------------------------------------------------------
LADLE_PANS = {"ladle_l": PAN_Y[0], "ladle_r": PAN_Y[2]}
_BOWL_Z = FOOD_TOP + BOWL_H / 2 + 0.0005
_hc = HANDLE_L / 2
for _name, _cy in LADLE_PANS.items():
    OBJECTS.append(Box(_name, size=(0.02, 0.02, 0.02), pos=(CX + LADLE_BOWL_X, _cy, TOP + _BOWL_Z), color=LADLE_STEEL,
                       mass=LADLE_KG))
    mech.compound(_name, [
        mech.cylinder_part(BOWL_R, BOWL_H, (0.0, 0.0, 0.0), LADLE_STEEL),
        ((HANDLE_L, HANDLE_W, HANDLE_T),
         (_hc * math.cos(HANDLE_ALPHA), 0.0, _Z0 + _hc * math.sin(HANDLE_ALPHA) - _BOWL_Z),
         LADLE_STEEL, (0.0, -math.degrees(HANDLE_ALPHA), 0.0)),
    ])

# the covers --------------------------------------------------------------------------------------------------------
COVERS = {"cover_solid": 0, "cover_a": -1, "cover_b": -1}  # notch end: 0 none, -1 toward the robot
COVER_START = {}
for _name, _ns in COVERS.items():
    _msize, _moff, _parts = _cover_parts(_ns)
    _pc = (CX, COVER_Y[_name])
    COVER_START[_name] = _pc
    OBJECTS.append(Box(_name, size=_msize, pos=(_pc[0] + _moff[0], _pc[1] + _moff[1], _plate_z()), color=STEEL,
                       mass=COVER_KG[_name]))
    mech.compound(_name, _shifted(_parts, _moff) if _ns else _parts)
    # a small marker welded at the plate's centre: the body's own origin is offset on a notched cover
    OBJECTS.append(Box(_name + "_mark", size=(0.01, 0.01, PLATE_T), pos=(_pc[0], _pc[1], _plate_z()), color=STEEL,
                       mass=0.002))
    mech.weld(_name + "_mark", parent=_name)

# -- scoring ---------------------------------------------------------------------------------------------------
SEAT_Z_M = 0.003  # the plate's underside within 3 mm of the rim top
SEAT_XY_M = 0.015  # the cover's centre within 1.5 cm of the pan's centre
SEAT_TILT = math.cos(math.radians(3.0))
SEAT_YAW = math.sin(math.radians(5.0))  # long side within 5 degrees of the pan's
LADLE_MOVED_M, LADLE_TURNED = 0.03, math.cos(math.radians(10.0) / 2)
STILL_S, STILL_M = 1.0, 0.003
LET_GO_M = 0.03  # every finger and TCP this far from every cover's plate and strap (nothing touching it anywhere)
NOTCHED_ON_EMPTY = 0.5  # a notched cover on the pan with no ladle counts half
PAN_SHARE = 1.0 / 3.0
_WINDOW = int(STILL_S * CONTROL_HZ)


def _rot(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def plate_centre(objects, name):
    """A cover's true plate centre: the welded marker's position (the body's origin is 3 cm off on a notched cover and
    its reported position is a bounding-box centre)."""
    return np.asarray(objects[name + "_mark"].pos, float)


def cover_boxes(notch_sign):
    """The cover's solid boxes in the plate-centre frame: (centre, half-size) for the plate pieces, posts and strap."""
    s = notch_sign
    if s == 0:
        plate = [((0.0, 0.0, 0.0), (PAN_L / 2, PAN_W / 2, PLATE_T / 2))]
    else:
        side_w = (PAN_W - NOTCH_W) / 2
        plate = [((-s * NOTCH_D / 2, 0.0, 0.0), ((PAN_L - NOTCH_D) / 2, PAN_W / 2, PLATE_T / 2))]
        plate += [((s * (PAN_L / 2 - NOTCH_D / 2), sy * (NOTCH_W / 2 + side_w / 2), 0.0),
                   (NOTCH_D / 2, side_w / 2, PLATE_T / 2)) for sy in (-1, 1)]
    posts = [((0.0, sy * POST_X, PLATE_T / 2 + POST_H / 2), (STRAP_W / 2, 0.006, POST_H / 2)) for sy in (-1, 1)]
    bar = [((0.0, 0.0, BAR_DZ), (STRAP_W / 2, STRAP_L / 2, STRAP_T / 2))]
    return plate + posts + bar


def gap_to_cover(objects, name, point):
    """Distance (m) from a world point to the cover's plate outline, posts and strap (0 if inside)."""
    c = plate_centre(objects, name)
    q = _rot(objects[name].quat_xyzw).T @ (np.asarray(point, float) - c)
    best = 1e9
    for ctr, half in cover_boxes(COVERS[name]):
        d = np.maximum(np.abs(q - np.asarray(ctr)) - np.asarray(half), 0.0)
        best = min(best, float(np.linalg.norm(d)))
    return best


def _fresh():
    return dict(start={}, bad={}, hist=[], n=0)


_state = _fresh()
_TRACKED = tuple(COVERS) + tuple(n + "_mark" for n in COVERS) + tuple(LADLE_PANS)


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


def pan_of(pos):
    """The index of the pan a cover's centre is over (within its footprint), else None."""
    for i, cy in enumerate(PAN_Y):
        if abs(pos[0] - CX) <= PAN_L / 2 and abs(pos[1] - cy) <= PAN_W / 2:
            return i
    return None


def seated_on(st_obj, mark):
    """The pan index a cover rests flat on the pan's rim, else None: underside at the rim (3 mm), centred (1.5 cm),
    flat (3 degrees), square (5 degrees)."""
    r = _rot(st_obj.quat_xyzw)
    p = np.asarray(mark.pos, float)  # the plate's true centre
    i = pan_of(p)
    if i is None:
        return None
    if abs(p[2] - PLATE_T / 2 - (TOP + RIM)) > SEAT_Z_M:
        return None
    if abs(p[0] - CX) > SEAT_XY_M or abs(p[1] - PAN_Y[i]) > SEAT_XY_M:
        return None
    if float(r[2, 2]) < SEAT_TILT or abs(float(r[1, 0])) > SEAT_YAW:
        return None
    return i


def ladle_displaced(st_obj, start) -> bool:
    """Out of place: its centre moved 3 cm or more, or turned 10 degrees or more, since the start."""
    p = np.asarray(st_obj.pos, float)
    if float(np.linalg.norm(p - start["pos"])) >= LADLE_MOVED_M:
        return True
    q = np.asarray(st_obj.quat_xyzw, float)
    return abs(float(np.dot(q, start["quat"]))) < LADLE_TURNED


def still(st) -> bool:
    h = st["hist"]
    if len(h) < _WINDOW:
        return False
    a = np.asarray(h[-_WINDOW:])
    return bool(np.all(np.abs(a - a[-1]) < STILL_M))


def let_go(objects) -> bool:
    return all(gap_to_cover(objects, c, objects[f"{a}{k}"].pos) >= LET_GO_M
               for a in ("left", "right") for k in ("_tcp", "_finger_l", "_finger_r") for c in COVERS)


def pan_scores(objects):
    """Credit per pan (left, middle, right) as the scene is now, before stillness and let-go."""
    out = [0.0, 0.0, 0.0]
    for name, notch in COVERS.items():
        i = seated_on(objects[name], objects[name + "_mark"])
        if i is None:
            continue
        ladle = [n for n, cy in LADLE_PANS.items() if cy == PAN_Y[i]]
        if ladle:
            if _state["bad"].get(ladle[0]):
                continue
            out[i] = PAN_SHARE
        else:
            out[i] = PAN_SHARE * (NOTCHED_ON_EMPTY if notch else 1.0)
    return out


def score(objects):
    """Per pan, a third of the score: a cover resting flat on the pan's rim (underside within 3 mm of the rim, centre
    within 1.5 cm of the pan's, within 3 degrees of level and 5 degrees of square), on the empty pan a solid cover
    (1/3) or a notched one (1/6), on a pan with a ladle one that is flat (which needs the notch round the handle),
    while that ladle has stayed in place (never moved 3 cm or turned 10 degrees since the start). The plate centre is the
    welded marker's, not the body origin. Counted only at the end, with everything still for 1 s and no finger or TCP of
    either gripper within 3 cm of any cover's plate, posts or strap."""
    st = _state
    st["n"] += 1
    for n in _TRACKED:
        o = objects[n]
        if n not in st["start"]:
            st["start"][n] = dict(pos=np.asarray(o.pos, float).copy(), quat=np.asarray(o.quat_xyzw, float).copy())
    for n in LADLE_PANS:
        if ladle_displaced(objects[n], st["start"][n]):
            st["bad"][n] = True
    st["hist"] = (st["hist"] + [np.concatenate([np.asarray(objects[n].pos, float) for n in _TRACKED])])[-(_WINDOW + 5):]
    sc = pan_scores(objects)
    if DEBUG and st["n"] % 20 == 0:
        info = " ".join(f"{n}=({objects[n].pos[0]:.3f},{objects[n].pos[1]:.3f},low {objects[n].low[2] - TOP:.4f},"
                        f"seat {seated_on(objects[n], objects[n + '_mark'])} mark ({objects[n + '_mark'].pos[0]:.3f},{objects[n + '_mark'].pos[1]:.3f}))" for n in COVERS)
        lad = " ".join(f"{n}=({objects[n].pos[0]:.3f},{objects[n].pos[1]:.3f},{objects[n].pos[2] - TOP:.3f}) "
                       f"bad={st['bad'].get(n, False)}" for n in LADLE_PANS)
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} {info} | {lad} still={still(st)} let_go={let_go(objects)} "
              f"pans={[round(v, 3) for v in sc]}", flush=True)
    if not (still(st) and let_go(objects)):
        return 0.0
    return round(float(sum(sc)), 4)


# -- scripted demo -----------------------------------------------------------------------------------------------
GRASP_Z = TOP + PLATE_T / 2 + BAR_DZ + 0.001  # the strap bar's centre when a cover lies on the table
HOVER_Z = TOP + 0.09
TRAVEL_Z = TOP + RIM + 0.030 + PLATE_T / 2 + BAR_DZ  # the cover's underside 3 cm above the rims (it sags in the grip)
SEAT_Z = TOP + RIM + PLATE_T / 2 + BAR_DZ + 0.002  # released 2 mm above the rim
APPROACH_DX = 0.05  # comes in from the near side, then slides out to the pan centre (the handle is far out)
PARK = {"left": (0.18, 0.27, TOP + 0.20), "right": (0.18, -0.27, TOP + 0.20)}


YAW_PICK = {"left": -math.pi / 2, "right": -math.pi / 2}  # fingers close along x: across the strap bar (along y)
YAW_TURN = {"left": math.pi / 2, "right": math.pi / 2}  # the other way up: the cover has turned half a turn
LIFT_UP = 0.02


def _carry(arm, name, pan_y, turn):
    x0, y0 = COVER_START[name]
    yaw0 = YAW_PICK[arm]
    yaw1 = YAW_TURN[arm] if turn else yaw0
    w = [
        Waypoint(2.0, (x0, y0, HOVER_Z), FINGER_OPEN, f"over {name}'s strap", yaw0),
        Waypoint(1.5, (x0, y0, GRASP_Z), FINGER_OPEN, "down around the strap", yaw0),
        Waypoint(0.6, (x0, y0, GRASP_Z), FINGER_CLOSED, "grip it", yaw0),
        Waypoint(2.0, (x0, y0, TRAVEL_Z), FINGER_CLOSED, "lift it clear", yaw0),
    ]
    if turn:
        w.append(Waypoint(2.5, (x0, y0, TRAVEL_Z), FINGER_CLOSED, "turn the notch round", yaw1))
    w += [
        Waypoint(3.0, (CX - (APPROACH_DX if turn else 0.0), pan_y, TRAVEL_Z), FINGER_CLOSED, "over the pan", yaw1),
        Waypoint(1.2, (CX, pan_y, TRAVEL_Z), FINGER_CLOSED, "centre it", yaw1),
        Waypoint(1.5, (CX, pan_y, SEAT_Z), FINGER_CLOSED, "lower it onto the rim", yaw1),
        Waypoint(0.6, (CX, pan_y, SEAT_Z), FINGER_OPEN, "let go", yaw1),
        Waypoint(1.2, (CX, pan_y, SEAT_Z + LIFT_UP), FINGER_OPEN, "up, clear of the strap", yaw1),
    ]
    return w


def _park(arm, secs=2.0):
    return [Waypoint(secs, PARK[arm], FINGER_OPEN, "park")]


def demo():
    right = _carry("right", "cover_b", PAN_Y[2], True) + _park("right")
    t_right = sum(w.duration for w in right)
    left = [Waypoint(t_right - 1.0, PARK["left"], FINGER_OPEN, "wait, parked")]
    left += _carry("left", "cover_solid", PAN_Y[1], False) + _park("left", 1.5)
    left += _carry("left", "cover_a", PAN_Y[0], True) + _park("left", 2.0)
    left.append(Waypoint(2.0, PARK["left"], FINGER_OPEN, "still"))
    right.append(Waypoint(sum(w.duration for w in left) - t_right, PARK["right"], FINGER_OPEN, "stay parked"))
    return {"left": left, "right": right}
