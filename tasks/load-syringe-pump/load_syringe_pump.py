"""On the station table, 33 cm out and 2 cm left of the midline: a white syringe pump, its cradle running left-right.
At the cradle's left end is the narrow slot that takes the syringe's barrel flange, and past it, on its own track, the
grey plunger driver, pulled back. To the right, nearer the arms, a filled syringe (a clear barrel 3 cm across with a
flange at one end and a white plunger in it) lies on two black stands, flange to the left. Load it: barrel in the
cradle with the flange down in its slot, then the driver up against the end of the plunger without pushing it in.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from parts import Part, Piece  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE, DARK, GREY = (0.92, 0.92, 0.92), (0.15, 0.15, 0.17), (0.55, 0.55, 0.58)
CLEAR, PLUNGER_WHITE, STAND_BLACK = (0.80, 0.85, 0.90), (0.95, 0.95, 0.95), (0.12, 0.12, 0.13)

# -- the pump, mm, in its frame: origin on the table under the cradle's middle; the cradle runs along y ---------
PUMP_XY = (0.33, 0.02)
FLOOR_Z = 25.0  # the cradle floor
BARREL_R, BARREL_L = 14.8, 100.0  # stand-in for a 60 mL syringe [UNVERIFIED]
AXIS_Z = FLOOR_Z + BARREL_R  # the barrel's axis, seated
SLIT_Y, SLIT_W, SLIT_DEPTH = 31.25, 3.5, 14.0  # the flange slot: centre, width (flange 2.5 mm), depth below the floor
FLANGE_T, FLANGE_W, FLANGE_H = 2.5, 40.0, 50.0
# WHY a rounded mouth (5 mm): with square edges the flange landed on the body 3 to 5 mm short of the slot, the arm
# wandering 2 to 5 mm on the way down, and the syringe sat cocked 8 degrees (official sim, 2026-11-05, five runs).
# Real flange clips have a lead-in for the same reason.
FUNNEL_R = 5.0
DRIVER_HOME_GAP = 30.0  # the driver's face starts this far from the plunger's thumb disc
STOPPER_AT = 35.0  # the stopper's centre from the barrel's centre, toward the flange: a full syringe
# WHY a 20 mm stopper with 0.1 mm play: a 10 mm stopper with 0.3 mm play let the plunger sag 3.6 degrees under its thumb
# disc, whose rim then met the driver 0.94 mm before its centre did (traced, 2026-11-06). Real stoppers fit tightly.
STOPPER_L, STOPPER_SQ = 20.0, 18.67  # two squares, one turned 45 degrees: corners 13.2 mm out, the bore 13.3
THUMB_FROM_STOPPER = 53.5  # the thumb disc's outer face from the stopper's centre
_fy = BARREL_L / 2 + FLANGE_T / 2  # the flange's centre from the barrel's centre
BARREL_SEAT_Y = SLIT_Y - _fy  # the barrel's centre, seated, in the pump frame (-20 mm)
CRADLE_CLEAR = 1.0  # each side of the barrel

near, far = SLIT_Y - SLIT_W / 2, SLIT_Y + SLIT_W / 2  # 29.5, 33.0
PUMP_BASE = (
    ("body", (0, (-75 + near - FUNNEL_R) / 2, FLOOR_Z / 2), (50, near - FUNNEL_R + 75, FLOOR_Z)),
    ("funnel_near", (0, near - FUNNEL_R / 2, (FLOOR_Z - FUNNEL_R) / 2), (50, FUNNEL_R, FLOOR_Z - FUNNEL_R)),
    ("funnel_far", (0, far + FUNNEL_R / 2, (FLOOR_Z - FUNNEL_R) / 2), (50, FUNNEL_R, FLOOR_Z - FUNNEL_R)),
    ("body_past", (0, (far + FUNNEL_R + 45) / 2, FLOOR_Z / 2), (50, 45 - far - FUNNEL_R, FLOOR_Z)),
    ("slit_floor", (0, SLIT_Y, (FLOOR_Z - SLIT_DEPTH) / 2), (50, SLIT_W, FLOOR_Z - SLIT_DEPTH)),
    ("wall_l", (BARREL_R + CRADLE_CLEAR + 2, -23.0, FLOOR_Z + 2), (4, 94, 4)),
    ("wall_r", (-BARREL_R - CRADLE_CLEAR - 2, -23.0, FLOOR_Z + 2), (4, 94, 4)),
    ("track", (0, 82.5, 10), (50, 75, 20)),
)


def _m(v):
    return tuple(x / 1000 for x in v)


def _pump_shapes():
    shapes = [Shape("box", _m(c), _m(s), DARK if name == "slit_floor" else WHITE, contact_offset=0.001)
              for name, c, s in PUMP_BASE]
    for y in (near - FUNNEL_R, far + FUNNEL_R):  # the slot's rounded mouth
        shapes.append(Shape("cylinder", _m((0, y, FLOOR_Z - FUNNEL_R)), (FUNNEL_R / 1000, 0.050), WHITE, axis="X",
                            contact_offset=0.001))
    return tuple(shapes)


THUMB_FACE_Y = BARREL_SEAT_Y + STOPPER_AT + THUMB_FROM_STOPPER  # the thumb disc's outer face, seated (68.5 mm)
DRIVER_FACE_Y = THUMB_FACE_Y + DRIVER_HOME_GAP
DRIVER = ((0, DRIVER_FACE_Y + 10, 35.5), (40, 20, 30))
TAB = ((0, DRIVER_FACE_Y + 10, 55.5), (12, 6, 10))  # the grip on the driver's top, where a nurse squeezes it
DRIVER_TRAVEL = 0.040
PUMP = mechanism(
    "pump", (*PUMP_XY, TOP), base=Link("pump_body", 1.0, _pump_shapes()),
    links=(Link("driver", 0.05, (Shape("box", _m(DRIVER[0]), _m(DRIVER[1]), GREY, contact_offset=0.001),
                                  Shape("box", _m(TAB[0]), _m(TAB[1]), DARK, contact_offset=0.001))),),
    joints=(Joint("driver_slide", "prismatic", "driver", _m(DRIVER[0]), "Y", -DRIVER_TRAVEL, 0.0, damping=2.0),),
    materials={"default": (1.2, 1.0)})

# -- the syringe: barrel (frame: barrel centre, axis +y toward the flange), plunger (frame: stopper centre) -------
_rm, _wall = BARREL_R - 0.75, 1.5
BARREL_PIECES = tuple(
    Piece(_m((_rm * math.sin(a), 0.0, _rm * math.cos(a))), _m((5.8, BARREL_L, _wall)), rot=(0.0, a, 0.0))
    for a in (2 * math.pi * k / 16 for k in range(16)))
_bore = BARREL_R - _wall  # 13.3
FLANGE_PIECES = (Piece(_m((0, _fy, _bore + (FLANGE_H / 2 - _bore) / 2)), _m((FLANGE_W, FLANGE_T, FLANGE_H / 2 - _bore))),
                 Piece(_m((0, _fy, -_bore - (FLANGE_H / 2 - _bore) / 2)), _m((FLANGE_W, FLANGE_T, FLANGE_H / 2 - _bore))),
                 Piece(_m((_bore + (FLANGE_W / 2 - _bore) / 2, _fy, 0)), _m((FLANGE_W / 2 - _bore, FLANGE_T, 2 * _bore))),
                 Piece(_m((-_bore - (FLANGE_W / 2 - _bore) / 2, _fy, 0)), _m((FLANGE_W / 2 - _bore, FLANGE_T, 2 * _bore))))
NOSE_PIECES = (Piece(_m((0, -BARREL_L / 2 - 1, 0)), _m((20, 2, 20))), Piece(_m((0, -BARREL_L / 2 - 7, 0)), _m((6, 10, 6))))
SYRINGE_PIECES = BARREL_PIECES + FLANGE_PIECES + NOSE_PIECES
PLUNGER_PIECES = (Piece(_m((0, 0, 0)), _m((STOPPER_SQ, STOPPER_L, STOPPER_SQ))),
                  Piece(_m((0, 0, 0)), _m((STOPPER_SQ, STOPPER_L, STOPPER_SQ)), rot=(0, math.pi / 4, 0)),
                  Piece(_m((0, (STOPPER_L / 2 + 51.5) / 2, 0)), _m((8, 51.5 - STOPPER_L / 2, 8))),
                  Piece(_m((0, 52.5, 0)), _m((30, 2, 30)), PLUNGER_WHITE))
THUMB_HALF = 0.015  # the thumb disc is 30 mm square


def _bbox_centre(pieces):
    """The pieces' bounding-box centre in the part frame (the turned staves counted as boxes at their extents)."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for p in pieces:
        half = np.asarray(p.size) / 2
        r = np.array([[math.cos(p.rot[1]), 0, math.sin(p.rot[1])], [0, 1, 0], [-math.sin(p.rot[1]), 0, math.cos(p.rot[1])]])
        corners = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * half @ r.T + p.center
        lo, hi = np.minimum(lo, corners.min(axis=0)), np.maximum(hi, corners.max(axis=0))
    return (lo + hi) / 2, lo, hi


STAND_XY = (0.25, -0.20)
STAND_H = 0.026
START_Z = TOP + STAND_H + BARREL_R / 1000  # barrel resting on the stands
SYRINGE = Part("syringe", pos=(*STAND_XY, START_Z), size=_m((FLANGE_W, BARREL_L + 14, FLANGE_H)), mass=0.075,
               color=CLEAR, pieces=SYRINGE_PIECES)
PLUNGER = Part("plunger", pos=(STAND_XY[0], STAND_XY[1] + STOPPER_AT / 1000, START_Z), size=_m((30, 53.5 + STOPPER_L / 2, 30)),
               mass=0.012, color=PLUNGER_WHITE, pieces=PLUNGER_PIECES)
STANDS = [Box(n, pos=(STAND_XY[0], STAND_XY[1] + dy, TOP + STAND_H / 2), size=(0.020, 0.010, STAND_H), color=STAND_BLACK)
          for n, dy in (("stand_a", -0.030), ("stand_b", 0.010))]
OBJECTS = [PUMP, SYRINGE, PLUNGER, *STANDS]

# -- scoring --------------------------------------------------------------------------------------------------------
SUCCESS_HOLD_S = 1.0
SEAT_AXIS = 0.001  # seated: the barrel's axis within this of its seat (height and sideways)
SEAT_TILT_DEG = 1.5
FLANGE_IN = 0.0008  # the flange's centre within this of the slot's (it has 0.5 mm a side)
IN_CRADLE = (0.003, 0.015, 15.0)  # in the cradle: axis within 3 mm sideways, 15 mm up, 15 degrees of the cradle's line
PUSHED = 0.001  # the plunger pushed in more than this: a bolus
NEAR, FLUSH = 0.005, 0.001  # the driver's face within these of the thumb disc
STILL = 0.01
_SC, _, _ = _bbox_centre(SYRINGE_PIECES)
_PC, _, _ = _bbox_centre(PLUNGER_PIECES)
SEAT = np.array([PUMP_XY[0], PUMP_XY[1] + BARREL_SEAT_Y / 1000, TOP + AXIS_Z / 1000])  # the barrel's centre, seated
SLIT_W_Y = PUMP_XY[1] + SLIT_Y / 1000


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _frame(state, centre):
    """A part's frame origin and rotation from its state (pos is its bounding box's centre)."""
    r = _rot(state.quat_xyzw)
    return np.asarray(state.pos, float) - r @ centre, r


def measures(objects):
    """The quantities the score uses, in metres and degrees."""
    s_org, s_r = _frame(objects["syringe"], _SC)
    p_org, p_r = _frame(objects["plunger"], _PC)
    axis = s_r @ np.array([0.0, 1.0, 0.0])
    flange = s_org + s_r @ np.array([0.0, _fy / 1000, 0.0])
    thumb = p_org + p_r @ np.array([0.0, THUMB_FROM_STOPPER / 1000, 0.0])
    # the disc's outermost point along the cradle: a tilted disc meets the driver at its rim
    thumb_out = thumb[1] + THUMB_HALF * (abs(p_r[1, 0]) + abs(p_r[1, 2]))
    driver_face = PUMP_XY[1] + DRIVER_FACE_Y / 1000 + objects["pump"].joints["driver_slide"]
    return {
        "side": float(s_org[0] - SEAT[0]), "up": float(s_org[2] - SEAT[2]),
        "tilt": math.degrees(math.asin(min(1.0, abs(float(axis[2]))))),
        "off_line": math.degrees(math.acos(min(1.0, abs(float(axis[1]))))),
        "flange": float(flange[1] - SLIT_W_Y),
        "pushed": float(STOPPER_AT / 1000 - np.dot(p_org - s_org, axis)),
        "gap": float(driver_face - thumb_out),
        "still": max(float(np.linalg.norm(objects[n].lin_vel)) for n in ("syringe", "plunger")) < STILL,
    }


def score(objects) -> float:
    """0 if the plunger was pushed in (a bolus) or the syringe isn't in the cradle; 0.2 in the cradle but not seated
    (the flange out of its slot, the syringe cocked); 0.5 seated; 0.75 seated with the driver within 5 mm of the
    plunger; 1.0 seated, the driver within 1 mm, everything still."""
    m = measures(objects)
    if m["pushed"] > PUSHED:
        return 0.0
    if abs(m["side"]) > IN_CRADLE[0] or m["up"] > IN_CRADLE[1] or m["off_line"] > IN_CRADLE[2] or m["up"] < -0.003:
        return 0.0
    seated = abs(m["side"]) <= SEAT_AXIS and abs(m["up"]) <= SEAT_AXIS and m["tilt"] <= SEAT_TILT_DEG \
        and abs(m["flange"]) <= FLANGE_IN
    if not seated:
        return 0.2
    if m["gap"] > NEAR:
        return 0.5
    if m["gap"] > FLUSH or not m["still"]:
        return 0.75
    return 1.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- the scripted demo: the right arm loads the syringe, the left arm brings the driver up --------------------------
GRIP_Z = 0.009  # TCP above the barrel's axis: fingertips 4 mm below it
SQUEEZE, RELEASE = -0.012, -0.020  # across the 29.6 mm barrel
# WHY 0.5 mm past the cradle's centre line along x: the arm sets the syringe down about 0.5 mm short (toward the bases):
# the barrel's axis ended 0.47 mm short when aimed at the line, and 1.55 mm short (not seated) when aimed 1 mm short
# (traced, official sim, 2026-11-06). The cradle floor is flat, so the barrel stays where it is put.
TARGET = (PUMP_XY[0] + 0.0005, PUMP_XY[1] + BARREL_SEAT_Y / 1000)  # barrel centre over the cradle, flange over its slot
SEAT_TCP_Z = TOP + AXIS_Z / 1000 + GRIP_Z
TAB_TCP = (PUMP_XY[0], PUMP_XY[1] + TAB[0][1] / 1000, TOP + (TAB[0][2] - TAB[1][2] / 2) / 1000 + 0.013 + 0.001)
PINCH_TAB = -0.004  # across the tab's 12 mm
# WHY 0.8 mm past touching: the pinched driver lags its target by about 1.0 mm (traced, 2026-11-06). Aimed 0.3 mm
# past, it ended 0.99 mm short of the thumb disc's rim, at the edge of the 1 mm window; 0.5 mm more should leave about
# 0.5 mm. On a runner without the lag it would push the plunger about 0.8 mm, inside the 1 mm allowance.
SLIDE = -(DRIVER_HOME_GAP + 0.8) / 1000
ACROSS_X = math.pi / 2


def demo():
    right = [
        Waypoint(2.0, (*STAND_XY, START_Z + 0.06), FINGER_OPEN, "over the syringe", ACROSS_X),
        Waypoint(1.5, (*STAND_XY, START_Z + GRIP_Z), FINGER_OPEN, "down to the barrel", ACROSS_X),
        Waypoint(0.8, (*STAND_XY, START_Z + GRIP_Z), SQUEEZE, "grip the barrel", ACROSS_X),
        Waypoint(1.5, (*STAND_XY, TOP + 0.12), SQUEEZE, "lift", ACROSS_X),
        Waypoint(2.0, (*TARGET, TOP + 0.12), SQUEEZE, "carry", ACROSS_X),
        Waypoint(1.5, (*TARGET, SEAT_TCP_Z + 0.030), SQUEEZE, "over the cradle", ACROSS_X),
        Waypoint(3.5, (*TARGET, SEAT_TCP_Z - 0.001), SQUEEZE, "slowly down: the flange into its slot", ACROSS_X),
        Waypoint(0.8, (*TARGET, SEAT_TCP_Z - 0.001), RELEASE, "let go", ACROSS_X),
        Waypoint(1.5, (*TARGET, TOP + 0.12), RELEASE, "clear", ACROSS_X),
        Waypoint(2.0, (0.20, -0.25, TOP + 0.15), FINGER_OPEN, "out of the way", ACROSS_X),
    ]
    # WHY the left arm waits high and away and pinches along x: with its fingers open along y over the pump, one
    # finger knocked the plunger's thumb disc (official sim, 2026-11-05)
    away = (TAB_TCP[0], TAB_TCP[1] + 0.07, TAB_TCP[2] + 0.10)
    left_across = -math.pi / 2  # the left arm turns this way for fingers along x (at +90 its wrist falls short here)
    left = [
        Waypoint(19.0, away, FINGER_OPEN, "wait high and away", left_across),
        Waypoint(2.0, (TAB_TCP[0], TAB_TCP[1], TAB_TCP[2] + 0.03), FINGER_OPEN, "over the driver's grip", left_across),
        Waypoint(1.5, TAB_TCP, FINGER_OPEN, "down to the grip", left_across),
        Waypoint(0.8, TAB_TCP, PINCH_TAB, "pinch the grip", left_across),
        Waypoint(3.0, (TAB_TCP[0], TAB_TCP[1] + SLIDE * 0.8, TAB_TCP[2]), PINCH_TAB, "slide", left_across),
        Waypoint(2.0, (TAB_TCP[0], TAB_TCP[1] + SLIDE, TAB_TCP[2]), PINCH_TAB, "creep up to the plunger", left_across),
        Waypoint(0.8, (TAB_TCP[0], TAB_TCP[1] + SLIDE, TAB_TCP[2]), FINGER_OPEN, "let go", left_across),
        Waypoint(1.5, (TAB_TCP[0], TAB_TCP[1] + SLIDE, TAB_TCP[2] + 0.05), FINGER_OPEN, "straight up", left_across),
    ]
    return {"left": left, "right": right}
