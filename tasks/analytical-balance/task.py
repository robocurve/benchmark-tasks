"""On the station table: an analytical balance with a glass draft shield, an empty plastic weighing boat on the bench
to its left, and a small square glass vial of plastic pellets, with a pouring spout, beside the boat.

The balance: a white base with a key strip along its front edge (a live display reading in grams to 0.01 g, a green
TARE key and a dark ON key) and, behind the strip, a low glass chamber round the round steel pan. The chamber's top is
a glass door that slides back, towards the far end of the balance, on a dark handle near its front edge; its front and
side walls are fixed. The door starts closed, and the display reads 0.00 with the pan empty.

The task, in the order an analyst weighs out a powder:
1. Open the top door, put the empty boat in the middle of the pan, close the door.
2. Press TARE: the display goes back to 0.00 with the boat on the pan.
3. Open the door and pour granules from the vial into the boat, a little at a time, until the display reads 1.00 g.
4. Close the door and leave the balance; put the vial down anywhere.

The display shows the mass resting on the pan (the boat and anything in it or on the pan) minus the mass on the pan at
the last TARE press. Two hands: one runs the door and the TARE key, the other carries the boat and pours.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CASE = (0.93, 0.93, 0.91)
DARK = (0.16, 0.17, 0.19)
SCREEN = (0.04, 0.05, 0.05)
LIT = (0.55, 1.00, 0.70)  # the display's lit segments
STEEL = (0.70, 0.71, 0.73)
GREEN = (0.10, 0.55, 0.30)
GLASS = (0.78, 0.88, 0.96)  # light, slightly blue
BOAT_C = (0.95, 0.95, 0.97)
VIAL_C = (0.80, 0.90, 0.92)
GRAIN_C = (0.95, 0.62, 0.15)
labkit.GLASS_COLORS.update({GLASS, VIAL_C})

# -- the balance (fixed). Frame: origin on the table under the pan's centre, x away from the robots, y to their left. --
PX, PY = 0.30, 0.0  # pan centre, world
WALL_T = 0.004
CH_X0, CH_X1, CH_HY = -0.075, 0.085, 0.085  # chamber inside: front and back faces, half width
FLOOR_Z = 0.035  # chamber floor (top of the base)
WALL_TOP = 0.100  # top of the glass walls; the door slides on them
DOOR_T = 0.004
DOOR_X0, DOOR_X1 = CH_X0 - WALL_T, 0.095  # the closed door's front and back edges
DOOR_TRAVEL = 0.13  # fully open: slid 13 cm back over the rear housing
REAR = (CH_X1, 0.225)  # rear housing, x span; its top sits just under the door
PAN_R, PAN_T, PAN_TOP = 0.040, 0.003, 0.050  # round pan 8 cm across, its top 5 cm above the table
STRIP_X0, STRIP_X1, STRIP_TOP = -0.170, CH_X0 - WALL_T, 0.040  # key strip in front of the chamber
HALF_W = CH_HY + WALL_T + 0.002  # half width of the base, strip and rear housing
HANDLE = (0.010, 0.040, 0.012)  # dark bar on the door, along y
HANDLE_X, HANDLE_Y = DOOR_X0 + 0.013, -0.065
KEY = (0.018, 0.018, 0.004)
KEY_X, TARE_Y, ON_Y = -0.128, -0.062, -0.034
KEY_TRAVEL = 0.003

DOOR = Joint(
    "door", kind="slide", axis="X", anchor=((DOOR_X0 + DOOR_X1) / 2, 0.0, WALL_TOP + DOOR_T / 2), lower=0.0,
    upper=DOOR_TRAVEL, initial=0.0, stiffness=0.0, damping=4.0, target=0.0, mass=0.15,
    boxes=[
        (((DOOR_X0 + DOOR_X1) / 2, 0.0, WALL_TOP + DOOR_T / 2), (DOOR_X1 - DOOR_X0, 2 * (CH_HY + WALL_T), DOOR_T),
         GLASS),
        ((HANDLE_X, HANDLE_Y, WALL_TOP + DOOR_T + HANDLE[2] / 2), HANDLE, DARK),
    ],
)
TARE = Joint(
    "tare", kind="slide", axis="Z", anchor=(KEY_X, TARE_Y, STRIP_TOP), lower=-KEY_TRAVEL, upper=0.0, initial=0.0,
    stiffness=150.0, damping=2.0, target=0.0, mass=0.002,  # a sprung key: about 0.45 N at the bottom of its travel
    boxes=[((KEY_X, TARE_Y, STRIP_TOP + KEY[2] / 2), KEY, GREEN)],
)

# Live display: four 7-segment digits 2 cm tall (their top towards the far end of the balance, so they read upright from
# the robots' side), the decimal point after the second. Each segment is a box on its own sprung slide, driven up flush
# with the screen when lit and down out of sight into the strip when not.
DISP_X, DISP_Y = -0.118, 0.030  # screen centre
SCREEN_SIZE = (0.030, 0.068, 0.001)
FACE_Z = STRIP_TOP + SCREEN_SIZE[2]
DIG_H, DIG_W, SEG_T, DIG_PITCH = 0.020, 0.011, 0.0022, 0.0155
SEG_SUNK = -0.003
SEG_Z = FACE_Z + 0.0004 - 0.0005  # centre of a lit segment: its top 0.4 mm proud of the screen
SEGS = "abcdefg"
GLYPHS = {"0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg", "5": "acdfg", "6": "acdefg",
          "7": "abc", "8": "abcdefg", "9": "abcdfg", "-": "g", " ": ""}


def _digit_y(i: int) -> float:
    return DISP_Y + (1.5 - i) * DIG_PITCH  # digit 0 (tens or minus sign) on the left


def _segment(i: int, s: str):
    """(centre xy, size xy) of segment s of digit i, in the balance frame."""
    xc, yc = DISP_X, _digit_y(i)
    hl, vl = DIG_W - SEG_T - 0.0006, DIG_H / 2 - SEG_T / 2 - 0.0006  # horizontal and vertical segment lengths
    side = DIG_W / 2 - SEG_T / 2
    return {
        "a": ((xc + DIG_H / 2 - SEG_T / 2, yc), (SEG_T, hl)),
        "b": ((xc + DIG_H / 4, yc - side), (vl, SEG_T)),
        "c": ((xc - DIG_H / 4, yc - side), (vl, SEG_T)),
        "d": ((xc - DIG_H / 2 + SEG_T / 2, yc), (SEG_T, hl)),
        "e": ((xc - DIG_H / 4, yc + side), (vl, SEG_T)),
        "f": ((xc + DIG_H / 4, yc + side), (vl, SEG_T)),
        "g": ((xc, yc), (SEG_T, hl)),
    }[s]


def display_text(grams: float) -> str:
    """The four characters the display shows for a reading (grams): sign or tens, units, tenths, hundredths."""
    v = min(max(round(float(grams), 2), -9.99), 99.99)
    s = f"{abs(v):5.2f}"
    return ("-" if v < 0 else s[0]) + s[1] + s[3] + s[4]


def _lit(text: str) -> list[float]:
    return [0.0 if s in GLYPHS[ch] else SEG_SUNK for ch in text for s in SEGS]


_START = _lit(display_text(0.0))
SEGMENT_JOINTS = []
for _i in range(4):
    for _k, _s in enumerate(SEGS):
        (_cx, _cy), (_sx, _sy) = _segment(_i, _s)
        _q = _START[7 * _i + _k]
        SEGMENT_JOINTS.append(Joint(
            f"seg{_i}{_s}", kind="slide", axis="Z", anchor=(_cx, _cy, SEG_Z), lower=SEG_SUNK, upper=0.0, initial=_q,
            target=_q, stiffness=200.0, damping=2.0, mass=0.002,
            boxes=[((_cx, _cy, SEG_Z), (_sx, _sy, 0.001), LIT)],
        ))
_dot = (DISP_X - DIG_H / 2 + SEG_T / 2, (_digit_y(1) + _digit_y(2)) / 2)

_glass_wall_h = WALL_TOP - FLOOR_Z
BALANCE = part(
    "balance", pos=(PX, PY, TOP), fixed=True, friction=0.6,
    boxes=[
        (((CH_X0 - WALL_T + CH_X1) / 2, 0.0, FLOOR_Z / 2), (CH_X1 - CH_X0 + WALL_T, 2 * HALF_W, FLOOR_Z), CASE),  # base
        (((STRIP_X0 + STRIP_X1) / 2, 0.0, STRIP_TOP / 2), (STRIP_X1 - STRIP_X0, 2 * HALF_W, STRIP_TOP), CASE),  # strip
        (((REAR[0] + REAR[1]) / 2, 0.0, (WALL_TOP - 0.0005) / 2), (REAR[1] - REAR[0], 2 * HALF_W, WALL_TOP - 0.0005),
         CASE),  # rear housing
        ((CH_X0 - WALL_T / 2, 0.0, FLOOR_Z + _glass_wall_h / 2), (WALL_T, 2 * (CH_HY + WALL_T), _glass_wall_h),
         GLASS),  # front glass
        *[(((CH_X0 + CH_X1) / 2, s * (CH_HY + WALL_T / 2), FLOOR_Z + _glass_wall_h / 2),
           (CH_X1 - CH_X0, WALL_T, _glass_wall_h), GLASS) for s in (1, -1)],  # side glass
        ((DISP_X, DISP_Y, STRIP_TOP + SCREEN_SIZE[2] / 2), SCREEN_SIZE, SCREEN),
        ((_dot[0], _dot[1], SEG_Z), (SEG_T, SEG_T, 0.001), LIT),  # decimal point (always lit)
        ((KEY_X, ON_Y, STRIP_TOP + KEY[2] / 2), KEY, DARK),  # ON key (not used)
    ],
    cylinders=[
        ((0.0, 0.0, (FLOOR_Z + PAN_TOP - PAN_T) / 2), 0.008, PAN_TOP - PAN_T - FLOOR_Z, STEEL),  # pan post
        ((0.0, 0.0, PAN_TOP - PAN_T / 2), PAN_R, PAN_T, STEEL),  # pan
    ],
    joints=[DOOR, TARE, *SEGMENT_JOINTS],
)

# -- the weighing boat (loose, on the bench) ----------------------------------------------------------------------------
BOAT, BOAT_WALL, BOAT_FLOOR, BOAT_H = 0.050, 0.0015, 0.004, 0.012  # floor 4 mm: falling pellets do not pass through it
BOAT_G = 1.20  # grams
BOAT_XY = (0.18, 0.20)
BOAT_SIM_KG = 0.012  # simulated mass (kg): heavier than a real 1.2 g boat so falling pellets don't shove it; the
#   reading counts the boat as BOAT_G
_w = BOAT / 2 - BOAT_WALL / 2
BOAT_PART = part(
    "boat", pos=(*BOAT_XY, TOP), mass=BOAT_SIM_KG, friction=0.9,
    boxes=[
        ((0.0, 0.0, BOAT_FLOOR / 2), (BOAT, BOAT, BOAT_FLOOR), BOAT_C),
        ((_w, 0.0, BOAT_H / 2), (BOAT_WALL, BOAT, BOAT_H), BOAT_C),
        ((-_w, 0.0, BOAT_H / 2), (BOAT_WALL, BOAT, BOAT_H), BOAT_C),
        ((0.0, _w, BOAT_H / 2), (BOAT - 2 * BOAT_WALL, BOAT_WALL, BOAT_H), BOAT_C),
        ((0.0, -_w, BOAT_H / 2), (BOAT - 2 * BOAT_WALL, BOAT_WALL, BOAT_H), BOAT_C),
    ],
)

# -- the vial (loose): a square glass bottle with a pouring spout, open at the top, and its granules ----------------------
VIAL_RIN, VIAL_WALL, VIAL_H, VIAL_FLOOR = 0.011, 0.0015, 0.026, 0.002  # VIAL_RIN: half the inside width
SPOUT_DEG = 30.0  # the wall facing the robots leans out by this much: a pouring spout
VIAL_G = 8.0
VIAL_XY = (0.26, 0.21)
_vo = VIAL_RIN + VIAL_WALL  # half the outside width
_vz = VIAL_FLOOR + (VIAL_H - VIAL_FLOOR) / 2
_sf = math.radians(SPOUT_DEG)
_sl = (VIAL_H - VIAL_FLOOR) / math.cos(_sf)  # the spout wall's length
SPOUT = _sl * math.sin(_sf)  # how far the spout's lip reaches out past the bottle's side
_spout_c = (-VIAL_RIN - math.sin(_sf) * _sl / 2 - math.cos(_sf) * VIAL_WALL / 2, 0.0,
            VIAL_FLOOR + math.cos(_sf) * _sl / 2 - math.sin(_sf) * VIAL_WALL / 2)
_side_x0, _side_x1 = -(VIAL_RIN + SPOUT + VIAL_WALL), _vo
VIAL = part(
    "vial", pos=(*VIAL_XY, TOP), mass=VIAL_G / 1000, friction=0.3,
    boxes=[
        ((0.0, 0.0, VIAL_FLOOR / 2), (2 * _vo, 2 * _vo, VIAL_FLOOR), VIAL_C),
        ((VIAL_RIN + VIAL_WALL / 2, 0.0, _vz), (VIAL_WALL, 2 * _vo, VIAL_H - VIAL_FLOOR), VIAL_C),  # back wall
        (_spout_c, (VIAL_WALL, 2 * _vo, _sl), VIAL_C, (0.0, -_sf)),  # spout wall, leaning out towards the robots
        *[(((_side_x0 + _side_x1) / 2, s * (VIAL_RIN + VIAL_WALL / 2), _vz),
           (_side_x1 - _side_x0, VIAL_WALL, VIAL_H - VIAL_FLOOR), VIAL_C) for s in (1, -1)],  # side walls
    ],
)

# Granules: 70 polypropylene pellets, stacked in the bottle in three layers (25, 25, 20).
GRAIN_R, GRAIN_H, GRAIN_G = 0.002, 0.003, 0.03  # cylindrical pellets 4 mm across, 3 mm long; 0.03 g each
GRAIN_SIM_KG = 0.0002  # simulated mass (kg), heavier than a real pellet for a stable contact solve; the reading
#   counts each pellet as GRAIN_G
LAYER_DZ = GRAIN_H + 0.0004
GRAIN_PITCH = 0.0043


def _layer(k: int) -> list[tuple[float, float]]:
    pts = [((i - 2) * GRAIN_PITCH, (j - 2) * GRAIN_PITCH) for i in range(5) for j in range(5)]
    return pts if k < 2 else [p for p in pts if p != (0.0, 0.0) and abs(p[0]) + abs(p[1]) > 1.5 * GRAIN_PITCH][:20]


GRAIN_START = [(VIAL_XY[0] + x, VIAL_XY[1] + y, TOP + VIAL_FLOOR + 0.0003 + LAYER_DZ * k)
               for k in range(3) for x, y in _layer(k)]
GRAINS = [part(f"grain{i:02d}", pos=p, mass=GRAIN_SIM_KG, friction=0.3,
               cylinders=[((0.0, 0.0, GRAIN_H / 2), GRAIN_R, GRAIN_H, GRAIN_C)])
          for i, p in enumerate(GRAIN_START)]
GRAIN_NAMES = [g.name for g in GRAINS]

OBJECTS = [BALANCE, BOAT_PART, VIAL, *GRAINS]

# -- the reading: worked out from what rests on the pan ----------------------------------------------------------------
TARGET_G = 1.00
PAN_XY = np.array([PX, PY])
PAN_Z = TOP + PAN_TOP
PRESSED = -0.0015  # TARE counts as pressed past half its travel
DOOR_SHUT = 0.005  # the door counts as closed within 5 mm of its front stop
_RIN_BOAT = BOAT / 2 - BOAT_WALL


def _local(state, pts) -> np.ndarray:
    return (np.atleast_2d(pts) - state.pos) @ state.rotation()


def boat_on_pan(boat) -> bool:
    """The boat rests flat on the pan: its underside within 3 mm of the pan's top, its centre over the pan."""
    return (abs(boat.pos[2] - PAN_Z) <= 0.003 and float(np.hypot(*(boat.pos[:2] - PAN_XY))) <= PAN_R
            and boat.tilt_deg() <= 10.0)


def vial_on_pan(vial) -> bool:
    """The vial rests on the pan or in the boat on it."""
    return (PAN_Z - 0.002 <= vial.low[2] <= PAN_Z + 0.006
            and float(np.hypot(*((vial.low[:2] + vial.high[:2]) / 2 - PAN_XY))) <= PAN_R + 0.01)


def in_boat(boat, pts) -> np.ndarray:
    q = _local(boat, pts)
    return (np.abs(q[:, 0]) <= _RIN_BOAT + 0.0005) & (np.abs(q[:, 1]) <= _RIN_BOAT + 0.0005) & (q[:, 2] >= 0.0) \
        & (q[:, 2] <= BOAT_H + 0.004)


def in_vial(vial, pts) -> np.ndarray:
    q = _local(vial, pts)
    x_min = -(VIAL_RIN + np.clip(q[:, 2] - VIAL_FLOOR, 0.0, None) * math.tan(_sf)) - 0.0005  # the spout leans out
    return (q[:, 0] >= x_min) & (q[:, 0] <= VIAL_RIN + 0.0005) & (np.abs(q[:, 1]) <= VIAL_RIN + 0.0005) \
        & (q[:, 2] >= VIAL_FLOOR - 0.001) \
        & (q[:, 2] <= VIAL_H + 0.003)


def on_pan(pts) -> np.ndarray:
    """Granules lying on the pan itself (spilled beside the boat)."""
    p = np.atleast_2d(pts)
    return (np.hypot(p[:, 0] - PAN_XY[0], p[:, 1] - PAN_XY[1]) <= PAN_R + 0.0015) & (p[:, 2] >= PAN_Z) \
        & (p[:, 2] <= PAN_Z + 0.010)


def weigh(boat, vial, grains) -> tuple[float, int, int]:
    """(grams on the pan, granules counted on the pan, granules spilled: in neither the boat on the pan nor the vial)."""
    g = np.asarray(grains, float)
    vial_in = in_vial(vial, g)
    on_boat = boat_on_pan(boat)
    counted = ~vial_in & ((on_boat & in_boat(boat, g)) | on_pan(g))
    grams = GRAIN_G * int(counted.sum())
    if on_boat:
        grams += BOAT_G
    if vial_on_pan(vial):
        grams += VIAL_G + GRAIN_G * int(vial_in.sum())
    spilled = int((~vial_in & ~(on_boat & in_boat(boat, g))).sum())
    return grams, int(counted.sum()), spilled


_trial = {"tare_g": 0.0, "tared": False, "tare_ok": False, "tare_door_shut": False, "pressed": False, "reading": None,
          "text": display_text(0.0)}
_ids: dict = {}


@on_reset
def _clear(env):
    _trial.update(tare_g=0.0, tared=False, tare_ok=False, tare_door_shut=False, pressed=False, reading=None,
                  text=display_text(0.0))
    _pour_reset()


def _grain_positions(env) -> np.ndarray:
    base = env.unwrapped
    origin = labkit._np(base.scene.env_origins[0])
    return np.array([labkit._np(base.scene[n].data.root_pos_w)[0] - origin for n in GRAIN_NAMES])


@on_step
def _watch(env):
    import torch

    s = {n: labkit._part_state(env, labkit._SPECS[n]) for n in ("balance", "boat", "vial")}
    grains = _grain_positions(env)
    grams, counted, _ = weigh(s["boat"], s["vial"], grains)
    down = s["balance"].joints["tare"] < PRESSED
    if down and not _trial["pressed"]:  # a new press: the balance zeroes on what is on the pan now
        boat = s["boat"]
        _trial.update(
            tare_g=grams, tared=True, tare_door_shut=s["balance"].joints["door"] <= DOOR_SHUT,
            tare_ok=bool(boat_on_pan(boat) and np.linalg.norm(boat.lin_vel) < 0.02 and counted == 0
                         and not vial_on_pan(s["vial"])),
        )
    _trial["pressed"] = down
    _trial["reading"] = grams - _trial["tare_g"]
    text = display_text(_trial["reading"])
    asset = env.unwrapped.scene["balance"]
    if "segments" not in _ids:
        names = list(asset.joint_names)
        _ids["segments"] = [names.index(j.joint_name) for j in SEGMENT_JOINTS]
    if text != _trial["text"] or "set" not in _ids:
        targets = torch.tensor([_lit(text)], device=asset.device)
        asset.set_joint_position_target(targets, joint_ids=_ids["segments"])
        _trial["text"], _ids["set"] = text, True


# -- scoring -----------------------------------------------------------------------------------------------------------
TOL = (0.05, 0.15, 0.50)  # grams


def reading(objects) -> float:
    """What the display shows (grams, to 0.01 g)."""
    grams, _, _ = weigh(objects["boat"], objects["vial"], [objects[n].pos for n in GRAIN_NAMES])
    return round(grams - _trial["tare_g"], 2)


def score(objects):
    """0 unless the last TARE press was made with the empty boat resting on the pan (nothing else on it) and the boat
    is on the pan now. Then by the reading's distance from 1.00 g: 1.0 within 0.05 g, 0.6 within 0.15 g, 0.3 within
    0.5 g, otherwise 0.1. Halved if the top door is not closed now or was not closed at that TARE press; halved again if
    any granule lies anywhere but in the boat or in the vial."""
    if not (_trial["tared"] and _trial["tare_ok"]) or not boat_on_pan(objects["boat"]):
        return 0.0
    _, _, spilled = weigh(objects["boat"], objects["vial"], [objects[n].pos for n in GRAIN_NAMES])
    e = abs(reading(objects) - TARGET_G)
    s = 1.0 if e <= TOL[0] + 1e-6 else 0.6 if e <= TOL[1] + 1e-6 else 0.3 if e <= TOL[2] + 1e-6 else 0.1
    if objects["balance"].joints["door"] > DOOR_SHUT or not _trial["tare_door_shut"]:
        s *= 0.5
    if spilled:
        s *= 0.5
    return s


SUCCESS_HOLD_S = 6.0  # end a trial early only once it has held for 6 s (time for the arms to park)


def success(objects):
    return score(objects) == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Right arm: slides the top door open by its handle; once the left arm has put the boat on the pan, closes it, presses
# TARE, opens it again and parks; at the end, closes it. Left arm: carries the boat onto the pan; picks up the vial;
# tips it over the boat and pours, closed loop on the display (see below), levels it, lifts it out and sets it down.
# The task file lets a Waypoint's ``yaw`` carry a full gripper rotation (``Orient``), and the pour's waypoints read
# their pose from the trial each control step.


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj._rot = np.asarray(rot, float)
        return obj

    @property
    def rot(self):
        return self._rot


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot_duck", False):
    def _grip_rotation(yaw=0.0, _prev=_prev_grip):
        return yaw.rot if hasattr(yaw, "rot") else _prev(yaw)

    _grip_rotation._rot_duck = True
    _demo.grip_rotation = _grip_rotation

_prev_action7 = _demo.ArmPlan.action7
if not getattr(_prev_action7, "_clock", False):
    def _action7(self, t, _prev=_prev_action7):
        _demo._plan_clock = float(t)  # the plan's time, for waypoints that move with the trial
        return _prev(self, t)

    _action7._clock = True
    _demo.ArmPlan.action7 = _action7

_DOWN = _prev_grip(0.0)  # straight down, fingers closing along world y


def pour_rot(theta: float) -> np.ndarray:
    """The vial gripped from above, its mouth tipped towards the robots by theta (rad) from upright."""
    return axis_angle_matrix(np.array([0.0, -1.0, 0.0]), theta) @ _DOWN


# The pour. The TCP sits at the vial's mouth (pad tips 13 mm below the rim). Tipped by theta, the rim's lowest point is
# R_LIP * (-cos theta, 0, -sin theta) from the TCP; it is kept POUR_DX beyond the boat's centre (the granules leave the
# rim moving towards the robots). The TCP height follows theta so the gripper clears the front glass and the vial
# clears the boat (POUR_Z: (theta in degrees, TCP height above the table), interpolated).
R_LIP = VIAL_RIN + SPOUT + VIAL_WALL  # the spout's lip, from the bottle's axis
POUR_DX = 0.010
POUR_Z = [(0.0, 0.108), (60.0, 0.108), (70.0, 0.124), (80.0, 0.140), (90.0, 0.152), (95.0, 0.160)]  # spout clear of the boat
POUR_TH0 = math.radians(65.0)  # tipped to here before the closed loop starts (pellets not yet at the lip)
POUR_TH_MAX = math.radians(95.0)  # the steepest the left arm reaches; the spout pours from about 70 to 80 degrees
POUR_RISE = math.radians(1.5)  # rad/s, tipping further while nothing is falling
POUR_FAST = math.radians(6.0)  # rad/s, tipping back up to just short of where pellets last started to flow
POUR_FALL = math.radians(25.0)  # rad/s, tipping back to stop the flow, and to upright at the end
POUR_BACK = math.radians(6.0)  # how far to tip back after each burst
POUR_SETTLE = 1.0  # s to wait after each burst, for the reading to settle
POUR_BURST_G = 0.03  # a burst: the reading has gone up by a pellet since the last pause
POUR_STOP_G = TARGET_G - 0.03  # level the bottle at this reading
POUR_WINDOW = 45.0  # s


def pour_tcp(theta: float) -> np.ndarray:
    deg = math.degrees(theta)
    z = np.interp(deg, [a for a, _ in POUR_Z], [b for _, b in POUR_Z])
    return np.array([PX + POUR_DX + R_LIP * math.cos(theta), PY, TOP + z])


_pour = {"t0": None, "tl": 0.0, "theta": POUR_TH0, "mode": "rise", "mark": 0.0, "flow": POUR_TH_MAX, "since": 0.0}


def _pour_reset():
    _pour.update(t0=None, tl=0.0, theta=POUR_TH0, mode="rise", mark=0.0, flow=POUR_TH_MAX, since=0.0, low=None)


def _pour_theta() -> float:
    """The pour angle now, poured in short bursts as a person taps out powder: tip slowly until the reading goes up by
    a pellet, tip back a little, wait for the reading to settle, tip quickly back to just short of where it flowed and
    on slowly again; once the reading reaches POUR_STOP_G, back to upright. Frozen after POUR_WINDOW. Offline (no
    simulator, no reading) it rises to POUR_TH_MAX: the plan check sees the steepest pose."""
    t = float(getattr(_demo, "_plan_clock", 0.0))
    p = _pour
    if p["t0"] is None or t < p["tl"] - 0.5:
        p.update(t0=t, tl=t, theta=POUR_TH0, mode="rise", mark=_trial["reading"] or 0.0, flow=POUR_TH_MAX, since=t)
    t_eff = min(t, p["t0"] + POUR_WINDOW)
    dt = max(0.0, t_eff - p["tl"])
    p["tl"] = max(p["tl"], t_eff)
    r = _trial["reading"]
    th = p["theta"]
    if r is None:  # offline
        rate = POUR_RISE
    else:
        if p["mode"] != "level" and r >= POUR_STOP_G:
            p["mode"] = "level"
        if r < p["mark"] - 0.1:  # the reading fell; if it stays down for a second, something is wrong: stop pouring
            if p.get("low") is None:
                p["low"] = t_eff
            elif t_eff - p["low"] >= 1.0 and p["mode"] != "level":
                p["mode"] = "level"
        else:
            p["low"] = None
        if p["mode"] == "rise" and r - p["mark"] >= POUR_BURST_G:
            p.update(mode="back", flow=th)
        if p["mode"] == "back" and th <= p["flow"] - POUR_BACK:
            p.update(mode="settle", since=t_eff)
        if p["mode"] == "settle" and t_eff - p["since"] >= POUR_SETTLE:
            p.update(mode="rise", mark=r)
        if p["mode"] == "rise":
            rate = POUR_FAST if th < p["flow"] - math.radians(2.0) else POUR_RISE
        elif p["mode"] == "settle":
            rate = 0.0
        else:
            rate = -POUR_FALL
    p["theta"] = float(np.clip(th + rate * dt, 0.0, POUR_TH_MAX))
    return p["theta"]


class _PourOrient(float):
    """The pour's rotation, read every control step."""

    def __new__(cls):
        return float.__new__(cls, 0.0)

    @property
    def rot(self):
        return pour_rot(_pour_theta())


class _PourPos:
    """The pour's TCP position, read every control step."""

    def __array__(self, dtype=None, copy=None):
        return np.asarray(pour_tcp(_pour_theta()), dtype=dtype or float)


TIP_ABOVE_TCP = 0.013
LEFT_PARK = (0.15, 0.25, TOP + 0.14)
RIGHT_PARK = (0.15, -0.25, TOP + 0.14)
RIGHT_ASIDE = (0.22, -0.20, TOP + 0.14)


def W(x, y, z):
    """Balance coordinates to world."""
    return (PX + x, PY + y, TOP + z)


HANDLE_GRIP_Z = TOP + WALL_TOP + DOOR_T + 0.004 + TIP_ABOVE_TCP  # pad tips 4 mm above the door glass
HANDLE_HOVER = HANDLE_GRIP_Z + 0.02
HANDLE_SHUT = W(HANDLE_X, HANDLE_Y, 0.0)[:2]
HANDLE_OPEN = W(HANDLE_X + DOOR_TRAVEL - 0.002, HANDLE_Y, 0.0)[:2]  # 2 mm short of the back stop
HANDLE_WIDE = -0.012  # fingers part-open round the 10 mm handle
HANDLE_HOLD = -0.003
KEY_TOP = TOP + STRIP_TOP + KEY[2]
PRESS_Z = KEY_TOP - KEY_TRAVEL - 0.0005 + TIP_ABOVE_TCP  # pad tips aimed 0.5 mm past the bottom of the key's travel
BOAT_GRIP_Z = TOP + 0.004 + TIP_ABOVE_TCP  # pad tips 4 mm above the boat's underside
BOAT_WIDE = -0.031  # 62 mm between the pads round the 50 mm boat
BOAT_HOLD = -0.022  # squeezes it lightly
BOAT_CARRY_Z = TOP + WALL_TOP + 0.016 + 0.004 + TIP_ABOVE_TCP  # the boat's underside 16 mm over the glass walls
BOAT_PLACE_Z = PAN_Z + 0.001 + 0.004 + TIP_ABOVE_TCP
VIAL_GRIP_Z = TOP + VIAL_H  # TCP at the rim: pad tips 13 mm down the vial
VIAL_WIDE = -0.020  # 40 mm between the pads round the 25 mm bottle
VIAL_HOLD = -0.010
VIAL_CARRY_Z = TOP + WALL_TOP + VIAL_H + 0.014  # the vial's bottom 14 mm over the glass walls

PLANCHECK_BOXES = [
    ("table", (0.35, 0.0, TOP - 0.02), (0.8, 1.2, 0.04)),
    ("base", W((CH_X0 - WALL_T + CH_X1) / 2, 0.0, FLOOR_Z / 2), (CH_X1 - CH_X0 + WALL_T, 2 * HALF_W, FLOOR_Z)),
    ("key strip", W((STRIP_X0 + STRIP_X1) / 2, 0.0, STRIP_TOP / 2), (STRIP_X1 - STRIP_X0, 2 * HALF_W, STRIP_TOP)),
    ("rear housing", W((REAR[0] + REAR[1]) / 2, 0.0, WALL_TOP / 2), (REAR[1] - REAR[0], 2 * HALF_W, WALL_TOP)),
    ("front glass", W(CH_X0 - WALL_T / 2, 0.0, (FLOOR_Z + WALL_TOP) / 2), (WALL_T, 2 * HALF_W, WALL_TOP - FLOOR_Z)),
    ("left glass", W((CH_X0 + CH_X1) / 2, CH_HY + WALL_T / 2, (FLOOR_Z + WALL_TOP) / 2),
     (CH_X1 - CH_X0, WALL_T, WALL_TOP - FLOOR_Z)),
    ("right glass", W((CH_X0 + CH_X1) / 2, -CH_HY - WALL_T / 2, (FLOOR_Z + WALL_TOP) / 2),
     (CH_X1 - CH_X0, WALL_T, WALL_TOP - FLOOR_Z)),
    ("open door", W((DOOR_X0 + DOOR_X1) / 2 + DOOR_TRAVEL, 0.0, WALL_TOP + DOOR_T / 2),
     (DOOR_X1 - DOOR_X0, 2 * (CH_HY + WALL_T), DOOR_T)),
    ("pan", W(0.0, 0.0, PAN_TOP - PAN_T / 2), (2 * PAN_R, 2 * PAN_R, PAN_T)),
]


def _dur(wps):
    return sum(w.duration for w in wps)


def _door(wps, frm, to, label):
    """Grip the door handle at ``frm`` (world xy), slide it to ``to``, let go and lift."""
    q = math.pi / 2  # fingers closing along x, either side of the bar
    wps += [
        Waypoint(1.8, (*frm, HANDLE_HOVER), HANDLE_WIDE, "above the door handle", q),
        Waypoint(1.0, (*frm, HANDLE_GRIP_Z), HANDLE_WIDE, "round the handle", q),
        Waypoint(0.6, (*frm, HANDLE_GRIP_Z), HANDLE_HOLD, "grip the handle", q),
        Waypoint(3.0, (*to, HANDLE_GRIP_Z), HANDLE_HOLD, label, q),
        Waypoint(0.5, (*to, HANDLE_GRIP_Z), HANDLE_WIDE, "let go of the handle", q),
        Waypoint(0.8, (*to, HANDLE_HOVER), HANDLE_WIDE, "up", q),
    ]


def _wait_until(wps, t, pos, finger, label, yaw=0.0):
    wps.append(Waypoint(max(0.3, t - _dur(wps)), pos, finger, label, yaw))


def demo():
    _pour_reset()
    bx, by = BOAT_XY
    vx, vy = VIAL_XY
    right: list = []
    _door(right, HANDLE_SHUT, HANDLE_OPEN, "slide the top door open")
    right.append(Waypoint(1.5, RIGHT_ASIDE, FINGER_OPEN, "aside"))
    t_open = _dur(right)  # the door is open and the right arm is aside

    left = [
        Waypoint(2.5, (bx, by, BOAT_GRIP_Z + 0.05), BOAT_WIDE, "above the boat"),
        Waypoint(1.2, (bx, by, BOAT_GRIP_Z), BOAT_WIDE, "fingers either side of the boat"),
        Waypoint(0.8, (bx, by, BOAT_GRIP_Z), BOAT_HOLD, "grip the boat's walls"),
        Waypoint(1.2, (bx, by, BOAT_CARRY_Z), BOAT_HOLD, "lift"),
    ]
    _wait_until(left, t_open, (bx, by, BOAT_CARRY_Z), BOAT_HOLD, "wait for the door")
    left += [
        Waypoint(2.0, (PX, PY, BOAT_CARRY_Z), BOAT_HOLD, "over the pan"),
        Waypoint(1.5, (PX, PY, BOAT_PLACE_Z), BOAT_HOLD, "lower the empty boat onto the pan"),
        Waypoint(0.6, (PX, PY, BOAT_PLACE_Z), BOAT_WIDE, "let go"),
        Waypoint(1.2, (PX, PY, BOAT_CARRY_Z), BOAT_WIDE, "up out of the chamber"),
    ]
    t_boat_out = _dur(left)
    left += [
        Waypoint(2.0, (vx, vy, BOAT_CARRY_Z), VIAL_WIDE, "over to the vial"),
        Waypoint(1.0, (vx, vy, VIAL_GRIP_Z + 0.05), VIAL_WIDE, "above the vial"),
        Waypoint(1.2, (vx, vy, VIAL_GRIP_Z), VIAL_WIDE, "fingers round the vial's neck"),
        Waypoint(0.8, (vx, vy, VIAL_GRIP_Z), VIAL_HOLD, "grip the vial"),
        Waypoint(1.5, (vx, vy, VIAL_CARRY_Z), VIAL_HOLD, "lift it, upright"),
    ]

    _wait_until(right, t_boat_out, RIGHT_ASIDE, FINGER_OPEN, "wait for the boat", math.pi / 2)
    _door(right, HANDLE_OPEN, HANDLE_SHUT, "slide the door shut")
    kx, ky = W(KEY_X, TARE_Y, 0.0)[:2]
    right += [
        Waypoint(1.5, (kx, ky, KEY_TOP + 0.04), FINGER_CLOSED, "above TARE"),
        Waypoint(1.0, (kx, ky, KEY_TOP + 0.003 + TIP_ABOVE_TCP), FINGER_CLOSED, "onto the key"),
        Waypoint(0.8, (kx, ky, PRESS_Z), FINGER_CLOSED, "press TARE (door shut, empty boat on the pan)"),
        Waypoint(0.6, (kx, ky, PRESS_Z), FINGER_CLOSED, "hold"),
        Waypoint(0.8, (kx, ky, KEY_TOP + 0.04), FINGER_CLOSED, "release"),
    ]
    _door(right, HANDLE_SHUT, HANDLE_OPEN, "slide the door open again")
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"))
    t_tared = _dur(right) - 1.0

    xs = pour_tcp(0.0)[0]
    _wait_until(left, t_tared, (vx, vy, VIAL_CARRY_Z), VIAL_HOLD, "hold the vial while the balance is tared")
    left += [
        Waypoint(2.5, (xs, PY, VIAL_CARRY_Z), VIAL_HOLD, "carry the vial over the boat"),
        Waypoint(2.0, tuple(pour_tcp(POUR_TH0)), VIAL_HOLD, "down into the opening, tipping it",
                 Orient(pour_rot(POUR_TH0))),
        Waypoint(0.1, _PourPos(), VIAL_HOLD, "pour", _PourOrient()),
        Waypoint(POUR_WINDOW, _PourPos(), VIAL_HOLD, "pour slowly, watching the display", _PourOrient()),
        Waypoint(2.0, (xs, PY, VIAL_CARRY_Z), VIAL_HOLD, "level the vial and lift it out"),
        Waypoint(2.5, (vx, vy, VIAL_CARRY_Z), VIAL_HOLD, "carry it back"),
        Waypoint(1.5, (vx, vy, VIAL_GRIP_Z + 0.002), VIAL_HOLD, "set it down"),
        Waypoint(0.6, (vx, vy, VIAL_GRIP_Z + 0.002), VIAL_WIDE, "let go"),
        Waypoint(1.0, (vx, vy, VIAL_GRIP_Z + 0.05), VIAL_WIDE, "up"),
        Waypoint(2.0, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    t_vial_out = _dur(left) - 7.6  # the vial is out of the chamber and on its way back

    _wait_until(right, t_vial_out, RIGHT_PARK, FINGER_OPEN, "wait while the left arm pours")
    _door(right, HANDLE_OPEN, HANDLE_SHUT, "slide the door shut")
    right.append(Waypoint(2.0, RIGHT_PARK, FINGER_OPEN, "park"))
    t_end = max(_dur(left), _dur(right)) + 2.0
    left.append(Waypoint(t_end - _dur(left), LEFT_PARK, FINGER_OPEN, "rest"))
    right.append(Waypoint(t_end - _dur(right), RIGHT_PARK, FINGER_OPEN, "rest"))
    return {"left": left, "right": right}
