"""On the table stands an empty long-arm stapler, its nose toward the middle of the table: a long grey base with a paper
rest and a paper guide beside the nose, a black housing (the long arm) over the base, and a sprung top arm hinged at
the housing's far end. The staple magazine is a front-loading drawer: a steel tray with a rail along its floor, closed
at the nose by a cap with a red pull tab. It slides out of the nose against its return spring and is pulled shut again
when let go. Inside the housing a staple pusher (the follower), on a long spring (2.0 N at the nose, 2.5 N behind a full
strip), presses toward the nose. On a tray lies a fresh strip of staples; on the paper rest lie two sheets of paper.

Task: reload the stapler and staple the two sheets together, leaving both arms at rest and clear of the stapler and the
paper. The strip has to be laid legs down over the rail of the drawer while the drawer is held out; let go, the drawer
is pulled shut and the strip, carried in, meets the follower, which then holds its front staple against the nose under
the driver. Pressing the top arm past its firing travel drives that front staple through whatever paper lies flat under
the nose.

Run it:  inspect-robots run --embodiment yam_arena -E task=reload_a_long_arm_stapler_then_staple_two_sheets.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_glue_on, slider)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the stapler (fixed on the table): base and housing along y, its nose toward -y ------------------------------------
SX = 0.30  # the stapler's middle line (x)
Y_NOSE = 0.055  # the nose: the housing's front face, where the front staple's front face sits (under the driver)
BASE_Y = (0.025, 0.335)
BASE_W, BASE_H = 0.060, 0.015
REST_X, REST_Y = (0.12, SX - BASE_W / 2), (0.030, 0.205)  # the paper rest, flush with the base, on the arms' side
FENCE_X = 0.3125  # the paper guide's face (a fence along y on the base, under the housing)
FENCE_H = 0.006
THROAT = 0.014  # the gap between the base's top and the housing's underside (the sheets go in there)
HB = BASE_H + THROAT  # the housing's underside (above the table)
HOUSE_L, HOUSE_W = 0.250, 0.045
HOUSE_Y = (Y_NOSE, Y_NOSE + HOUSE_L)
H_FLOOR = 0.003
MAG_Z = HB + H_FLOOR  # the drawer's underside (above the table)
CH_FLOOR = 0.002
Z0 = MAG_Z + CH_FLOOR  # the drawer channel's floor (above the table)
H_CEIL = Z0 + 0.014  # the housing's inside top
HOUSE_TOP = H_CEIL + 0.003
H_IN = 0.0175  # the housing's inside half width
STAPLE_AT = np.array([SX, Y_NOSE + 0.00025])  # where a staple is driven (x, y): under the driver, over the anvil

# -- the staples: standard 26/6 (12.7 mm crown, 6 mm legs, 0.5 mm wire), a full strip of 210 ---------------------------
WIRE, CROWN, LEG = 0.0005, 0.0127, 0.0060
PITCH = 0.0005  # each staple's thickness along the strip
N_STAPLES = 210
STRIP_L = N_STAPLES * PITCH  # 105 mm
N_LOOSE = 2  # the two front staples are bodies of their own, held to the strip by its glue until the driver shears them
STRIP_MASS = 0.0063  # 0.03 g a staple
LOOSE_MASS = 0.0015  # each loose staple's body (the strip's body has the rest of its mass: 3.3 g)

# -- the drawer (magazine): a channel with a rail, sprung shut, its cap and pull tab at the nose -----------------------
CH_L = 0.220
CH_IN, CH_WALL, CH_WALL_H = 0.030, 0.0015, 0.003  # a tray 3 cm wide inside with 3 mm lips: room for fingertips
RAIL_W, RAIL_TOP, RAIL_FLAT = 0.010, 0.0058, 0.002  # rail width, its top above the floor, its flat top's width
# (the legs of a seated strip end 0.3 mm above the floor, so a strip set down off the rail cannot tip over beside it)
STROKE = 0.200  # the drawer pulls out this far
SPRING, SPRING_PRE, DRAG = 20.0, 0.25, 8.0  # N/m, preload (m: 5 N shut, 9 N fully out), N s/m
TAB = (0.012, 0.016, 0.032)
CAP_T = 0.006
GATE_Z = Z0 + RAIL_TOP + WIRE + 0.001  # the strip guide over the seated strip: its underside (1 mm over the crowns)
GATE_T, GATE_HW = 0.002, 0.0145

# -- the follower (staple pusher): on its own slide in the housing, a long spring toward the nose ----------------------
FOLLOW_K = 5.0  # N/m
FOLLOW_PRE = 0.40  # m: 2.0 N at the nose, 2.5 N behind a full strip
FOLLOW_DAMP = 4.0  # N s/m
FOLLOW_TRAVEL = 0.200
FOLLOW_L = 0.016
# body damping (1/s): the pressed stack (drawer cap, staples, strip, follower) otherwise rings at the solver's substep
DAMP_F, DAMP_S = 40.0, 10.0  # the follower; the strip and its loose staples
VEL_ITERS = 8  # solver velocity iterations for the moving parts

# -- the top arm: hinged at the housing's far end, sprung up; the staple fires once it is pressed down FIRE_DEG --------
PIVOT = np.array([SX, HOUSE_Y[1] - 0.012, TOP + HOUSE_TOP + 0.004])
LEVER_L, LEVER_W, LEVER_T = 0.236, 0.040, 0.010
LEVER_TILT = 4.0  # degrees, nose up, at rest
LEVER_STOP = 4.5  # its stop (degrees pressed): just past flat
LEVER_MASS = 0.25
PAD_AT = 0.205  # the press pad's middle, along the arm from the pivot
LEVER_K = 0.84  # N m per degree: about 12 N on the pad at the firing point
FIRE_DEG = 3.0  # pressed this far (11 mm at the pad), the driver drives the front staple
REARM_DEG = 1.0  # and it must come back up to within this of rest before the next staple

# -- the strip's tray, and the two sheets on the paper rest ------------------------------------------------------------
TRAY_AT = np.array([0.20, 0.27])
TRAY_T = 0.008
SHEET = (0.105, 0.148, 0.0005)  # A6: 0.5 mm stiff boards stand in for 0.1 mm paper
SHEET_MASS = 0.00124  # 80 g/m2
TOP_SHEET_AT = np.array([0.2125, 0.107])  # its middle (x, y), out from under the head
UNDER = np.array([-0.002, -0.003])  # the lower sheet lies this far offset, its edges showing

STEEL, DARK, BLACK, BASE_C, TRAY_C, RED, CHROME, PAPER, PAPER2, INK, REST_C = \
    (0.75, 0.76, 0.78), (0.20, 0.20, 0.22), (0.06, 0.06, 0.07), (0.34, 0.35, 0.37), (0.85, 0.80, 0.70), \
    (0.72, 0.10, 0.09), (0.86, 0.87, 0.89), (0.96, 0.96, 0.94), (0.97, 0.92, 0.70), (0.35, 0.40, 0.55), \
    (0.55, 0.56, 0.58)

# -- scoring -----------------------------------------------------------------------------------------------------------
SHUT = 0.001  # the drawer within this of its stop
SEAT_SIDE = 0.0015  # the strip's middle within this of the rail's middle line (its legs either side of the rail)
SEAT_Z = 0.001  # its crowns within this of resting on the rail
SEAT_UP, SEAT_ALONG = 8.0, 5.0  # degrees: legs down (upright) and along the channel
AT_NOSE = 0.001  # its front face within this of the nose
PRESSED_N = 0.5  # contact forces read as "pressed together" (N)
MARGIN = 0.002  # a staple goes through a sheet whose edges are at least this far from the staple's ends
FLAT = 10.0  # degrees: a sheet lying flat
ON_BASE = 0.0015
CLEAR_N = 0.2  # a hand touching anything with more than this counts as not clear
STILL_MM, STILL_DEG = 0.5, 0.5  # still: moved less than this over the last second
STILL_ARMS = 0.05  # rad/s: arms at rest
SUCCESS_HOLD_S = 1.0


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _quat_xyzw(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return (float((m[2, 1] - m[1, 2]) / (4 * w)), float((m[0, 2] - m[2, 0]) / (4 * w)),
            float((m[1, 0] - m[0, 1]) / (4 * w)), float(w))


def _box(x0, x1, y0, y1, z0, z1, color, collide=True) -> BoxShape:
    return BoxShape(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color, collide=collide)


def _stapler() -> Body:
    """Origin on the table under the middle line, at y = 0; x relative to the middle line."""
    yb0, yb1 = BASE_Y
    y0, y1 = HOUSE_Y
    rx0, rx1 = REST_X[0] - SX, REST_X[1] - SX
    hw = HOUSE_W / 2
    slot = (Z0 + 0.0040, Z0 + 0.0070)  # the follower's window along the housing's near side
    shapes = [
        # the base: the front part and the paper rest as one plate (no seam for a sheet's edge to catch), the back part
        _box(rx0, BASE_W / 2, yb0, REST_Y[1], 0.0, BASE_H, BASE_C),
        _box(-BASE_W / 2, BASE_W / 2, REST_Y[1], yb1, 0.0, BASE_H, BASE_C),
        _box(rx0 + 0.004, rx1 - 0.004, REST_Y[0] + 0.004, REST_Y[1] - 0.004, BASE_H, BASE_H + 0.0004, REST_C, False),
        _box(FENCE_X - SX, FENCE_X - SX + 0.006, REST_Y[0], REST_Y[1], BASE_H, BASE_H + FENCE_H, DARK),
        # the post under the housing's far end
        _box(-hw + 0.003, hw - 0.003, y1 - 0.035, y1, BASE_H, HB, BLACK),
        # the housing: floor, far side wall, near side wall (with the follower's window), top, closed far end
        _box(-hw, hw, y0, y1, HB, MAG_Z, BLACK),
        _box(H_IN, hw, y0, y1, MAG_Z, H_CEIL, BLACK),
        _box(-hw, -H_IN, y0, y1, MAG_Z, slot[0], BLACK),
        _box(-hw, -H_IN, y0, y1, slot[1], H_CEIL, BLACK),
        _box(-hw, -H_IN, y0, y0 + 0.003, slot[0], slot[1], BLACK),
        _box(-hw, -H_IN, y0 + 0.205, y1, slot[0], slot[1], BLACK),
        _box(-hw, hw, y0, y1, H_CEIL, HOUSE_TOP, BLACK),
        _box(-H_IN, H_IN, y1 - 0.006, y1, MAG_Z, H_CEIL, BLACK),
        # the strip guide inside the mouth: it clears a strip seated on the rail by 1 mm (and stops one riding higher)
        _box(-GATE_HW, GATE_HW, y0, y0 + 0.215, GATE_Z, GATE_Z + GATE_T, DARK),
        # the hinge's cheeks at the far end of the housing's top
        *[_box(s * hw - 0.002, s * hw + 0.002, PIVOT[1] - 0.008, PIVOT[1] + 0.008, HOUSE_TOP, HOUSE_TOP + 0.010,
               CHROME) for s in (-1.0, 1.0)],
        # markings: the anvil under the driver, a ruler along the paper guide, the follower's scale, a label
        _box(-0.008, 0.008, STAPLE_AT[1] - 0.003, STAPLE_AT[1] + 0.003, BASE_H, BASE_H + 0.0008, CHROME, False),
        *[_box(FENCE_X - SX - (0.007 if k % 5 == 4 else 0.004), FENCE_X - SX - 0.001, REST_Y[0] + 0.01 * (k + 1)
               - 0.0004, REST_Y[0] + 0.01 * (k + 1) + 0.0004, BASE_H, BASE_H + 0.0006, BLACK, False)
          for k in range(15)],
        *[_box(-hw - 0.0004, -hw, y0 + 0.012 + 0.0175 * k - 0.0005, y0 + 0.012 + 0.0175 * k + 0.0005, slot[1],
               slot[1] + 0.003, CHROME, False) for k in range(12)],
        _box(-0.015, 0.015, y0 + 0.13, y0 + 0.19, HOUSE_TOP, HOUSE_TOP + 0.0008, DARK, False)]
    return Body.build("stapler", pos=(SX, 0.0, TOP), mass=1.4, shapes=shapes, kinematic=True, color=BLACK,
                      friction=(0.12, 0.10), slick=True)


def _magazine() -> Body:
    """The drawer: origin at the middle of its floor's underside at the nose (the cap's inside face)."""
    hw = CH_IN / 2 + CH_WALL
    q45 = (0.0, math.sin(math.pi / 8), 0.0, math.cos(math.pi / 8))  # a square bar turned 45 degrees about y: a ridge
    a = RAIL_W / math.sqrt(2.0)
    ridge = CH_FLOOR + RAIL_TOP - RAIL_W / 2  # the ridge bar's middle (its top corner at the rail's top)
    shapes = [_box(-hw, hw, 0.0, CH_L, 0.0, CH_FLOOR, STEEL),
              *[_box(s * CH_IN / 2 - (CH_WALL if s < 0 else 0.0), s * CH_IN / 2 + (CH_WALL if s > 0 else 0.0), 0.0,
                     CH_L, CH_FLOOR, CH_FLOOR + CH_WALL_H, STEEL) for s in (-1.0, 1.0)],
              # the rail: full width up to its shoulders, a 45-degree roof above them with a 3 mm flat top
              _box(-RAIL_W / 2, RAIL_W / 2, 0.0, CH_L, CH_FLOOR, ridge, STEEL),
              BoxShape((0.0, CH_L / 2, ridge), (a, CH_L, a), STEEL, quat_xyzw=q45),
              _box(-RAIL_FLAT / 2, RAIL_FLAT / 2, 0.0, CH_L, CH_FLOOR + RAIL_TOP - RAIL_FLAT / 2,
                   CH_FLOOR + RAIL_TOP, STEEL),
              # the cap at the nose, closing the housing's mouth when shut, and its pull tab
              _box(-HOUSE_W / 2, HOUSE_W / 2, -CAP_T, 0.0, -H_FLOOR, HOUSE_TOP - MAG_Z, BLACK),
              _box(-TAB[0] / 2, TAB[0] / 2, -CAP_T - TAB[1], -CAP_T, 0.0, TAB[2], RED),
              _box(-0.004, 0.004, -CAP_T - 0.0002, -CAP_T, HOUSE_TOP - MAG_Z - 0.006, HOUSE_TOP - MAG_Z - 0.002,
                   CHROME, False)]
    return Body.build("magazine", pos=(SX, Y_NOSE, TOP + MAG_Z), mass=0.06, shapes=shapes, color=STEEL,
                      friction=(0.2, 0.15), com=(0.0, CH_L / 2 - 0.01, 0.004), solver_iterations=32)


def _follower() -> Body:
    """The staple pusher: origin at the middle of its front face, level with the tray's floor; it straddles the rail."""
    leg_x = (RAIL_W / 2 + 0.0003, CROWN / 2 + 0.00015)
    top_z = (RAIL_TOP + 0.0004, RAIL_TOP + 0.0012)
    shapes = [_box(leg_x[0], leg_x[1], 0.0, FOLLOW_L, 0.0005, top_z[0], STEEL),
              _box(-leg_x[1], -leg_x[0], 0.0, FOLLOW_L, 0.0005, top_z[0], STEEL),
              _box(-GATE_HW, GATE_HW, 0.0, FOLLOW_L, top_z[0], top_z[1], STEEL),
              _box(-GATE_HW, GATE_HW, FOLLOW_L, FOLLOW_L + 0.010, 0.001, top_z[1], DARK, False),  # its spring's seat
              _box(-HOUSE_W / 2 - 0.004, -GATE_HW, 0.004, 0.012, 0.0045, 0.0065, RED, False)]  # its indicator
    return Body.build("follower", pos=(SX, Y_NOSE, TOP + Z0), mass=0.010, shapes=shapes, color=STEEL,
                      friction=(0.2, 0.15), solver_iterations=32, damping=(DAMP_F, DAMP_F))


def _lever() -> Body:
    """The top arm, origin on the hinge line; it runs toward the nose (-y), rising 4 degrees at rest."""
    r = _rx(-LEVER_TILT)
    q = _quat_xyzw(r)

    def at(y, z):
        return tuple(float(v) for v in r @ np.array([0.0, y, z]))

    shapes = [BoxShape(at(-LEVER_L / 2, LEVER_T / 2), (LEVER_W, LEVER_L, LEVER_T), BLACK, quat_xyzw=q),
              BoxShape(at(-PAD_AT, LEVER_T + 0.003), (LEVER_W - 0.006, 0.050, 0.006), RED, quat_xyzw=q),  # the pad
              BoxShape(at(-0.10, LEVER_T + 0.0005), (0.006, 0.09, 0.001), CHROME, quat_xyzw=q, collide=False),
              BoxShape(at(-LEVER_L + 0.006, LEVER_T + 0.0005), (LEVER_W, 0.012, 0.001), CHROME, quat_xyzw=q,
                       collide=False),
              CylShape((0.0, 0.0, 0.0), 0.005, LEVER_W + 0.010, CHROME, axis="X")]
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=LEVER_MASS, shapes=shapes, color=BLACK,
                      friction=(0.8, 0.7))


def _tray() -> Body:
    x, y = TRAY_AT
    return Body.build("tray", pos=(float(x), float(y), TOP), mass=0.1, kinematic=True, color=TRAY_C,
                      friction=(0.6, 0.5), shapes=[BoxShape((0.0, 0.0, TRAY_T / 2), (0.06, 0.12, TRAY_T), TRAY_C)])


def _staples(length: float, marks: bool) -> list:
    """A row of staples ``length`` long (along y), origin at the middle of the crowns' underside: crowns on top, legs
    down at both sides."""
    shapes = [BoxShape((0.0, 0.0, WIRE / 2), (CROWN, length, WIRE), STEEL),
              *[BoxShape((s * (CROWN / 2 - WIRE / 2), 0.0, -(LEG - WIRE) / 2), (WIRE, length, LEG - WIRE), STEEL)
                for s in (-1.0, 1.0)]]
    if marks:  # the staples' lines, shown
        for k in range(int(length / 0.005)):
            shapes.append(BoxShape((0.0, -length / 2 + 0.005 * (k + 0.5), WIRE + 0.0001), (CROWN, 0.0004, 0.0002),
                                   DARK, collide=False))
    return shapes


ON_TRAY_Z = TOP + TRAY_T + LEG - WIRE  # the crowns' underside with the legs standing on the tray
# test only (STAPLER_FLIP=1): the strip starts upside down on the tray (legs up), to check that it cannot seat or feed
FLIP = bool(os.environ.get("STAPLER_FLIP"))
if FLIP:
    ON_TRAY_Z = TOP + TRAY_T + WIRE
FLIP_Q = (0.0, 1.0, 0.0, 0.0) if FLIP else None  # half a turn about y
STRIP_FRONT = float(TRAY_AT[1]) - STRIP_L / 2  # the strip's front end (-y) on the tray
LOOSE_AT = [np.array([float(TRAY_AT[0]), STRIP_FRONT + PITCH * (k + 0.5), ON_TRAY_Z]) for k in range(N_LOOSE)]
BODY_L = STRIP_L - N_LOOSE * PITCH
STRIP_AT = np.array([float(TRAY_AT[0]), STRIP_FRONT + N_LOOSE * PITCH + BODY_L / 2, ON_TRAY_Z])


def _strip() -> Body:
    return Body.build("strip", pos=tuple(STRIP_AT.tolist()), mass=STRIP_MASS - N_LOOSE * LOOSE_MASS,
                      shapes=_staples(BODY_L, True), friction=(0.8, 0.7), color=STEEL, contact_offset=0.0005,
                      solver_iterations=64, damping=(DAMP_S, DAMP_S), start_xyzw=FLIP_Q)


def _loose(k: int) -> Body:
    return Body.build(f"staple{k + 1}", pos=tuple(LOOSE_AT[k].tolist()), mass=LOOSE_MASS, shapes=_staples(PITCH, False),
                      friction=(0.8, 0.7), color=STEEL, contact_offset=0.0005, solver_iterations=64,
                      damping=(DAMP_S, DAMP_S), start_xyzw=FLIP_Q,
                      ignore=("stapler",) + tuple(f"staple{j + 1}" for j in range(k)))


def _sheet(name: str, xy, color, lines: bool) -> Body:
    """Origin at the sheet's middle."""
    shapes = [BoxShape((0.0, 0.0, 0.0), SHEET, color)]
    if lines:  # printed lines, shown
        for k in range(8):
            shapes.append(BoxShape((-0.040 + 0.011 * k, 0.010, SHEET[2] / 2 + 0.0001), (0.0012, 0.100, 0.0002), INK,
                                   collide=False))
    z = TOP + BASE_H + SHEET[2] / 2 + (SHEET[2] if name == "sheet_top" else 0.0) + 0.0003
    return Body.build(name, pos=(float(xy[0]), float(xy[1]), z), mass=SHEET_MASS, shapes=shapes, color=color,
                      friction=(0.9, 0.8), contact_offset=0.0006,
                      ignore=tuple(f"staple{k + 1}" for k in range(N_LOOSE)))


LOW_AT = TOP_SHEET_AT + UNDER
OBJECTS = [_stapler(), _magazine(), _follower(), _lever(), _tray(), _strip(), *[_loose(k) for k in range(N_LOOSE)],
           _sheet("sheet_low", LOW_AT, PAPER2, False), _sheet("sheet_top", TOP_SHEET_AT, PAPER, True)]
_SPEC = {o.name: o for o in OBJECTS}
LOOSE = [f"staple{k + 1}" for k in range(N_LOOSE)]
SHEETS = ("sheet_low", "sheet_top")
STILL_PARTS = ("magazine", "follower", "lever", "strip", *SHEETS)  # these must be still for success

# the drawer's slide: shut at 0, out to -STROKE; its spring pulls it shut (5 N shut, 9 N fully out)
slider("drawer", "stapler", "magazine", anchor=(SX, Y_NOSE, TOP + MAG_Z), axis="Y", limits=(-STROKE, 0.0),
       stiffness=SPRING, damping=DRAG, target=SPRING_PRE, max_force=50.0)
# the follower's slide in the housing: at the nose (0) when the stapler is empty, back up to 20 cm; a long preloaded
# spring pushes it toward the nose with 2.0 N there and 2.5 N behind a full strip
slider("feed", "stapler", "follower", anchor=(SX, Y_NOSE, TOP + Z0), axis="Y", limits=(0.0, FOLLOW_TRAVEL),
       stiffness=FOLLOW_K, damping=FOLLOW_DAMP, target=-FOLLOW_PRE, max_force=20.0)
# the top arm: no preload; its spring holds it up against its own weight and gives about 12 N at the pad when pressed
# to the firing point
hinge("press", "stapler", "lever", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(-0.5, LEVER_STOP),
      spring=LEVER_K, damping=0.02, target_deg=-LEVER_MASS * 9.81 * (LEVER_L / 2) / LEVER_K)
# the strip's glue: each loose staple held to the strip's body until the driver shears it off
for _k in range(N_LOOSE):
    glue(f"hold{_k + 1}", "strip", LOOSE[_k], anchor=tuple(LOOSE_AT[_k].tolist()))
# a driven staple's legs through the sheets: switched on when it is driven through them (left out for the upside-down
# test, whose staples can never be driven and whose flipped start would not match these joints' frames)
for _k in range(N_LOOSE if not FLIP else 0):
    for _s in SHEETS:
        glue(f"{LOOSE[_k]}_{_s}", LOOSE[_k], _s, anchor=tuple(LOOSE_AT[_k].tolist()))

# -- contact sensing: every moving part reports its contacts -----------------------------------------------------------
_PATHS: dict = {}
_author_plain = bodykit._author_sliders


def _author_sliders(stage, prim_path: str, spec) -> None:
    _author_plain(stage, prim_path, spec)
    if spec.name in ("magazine", "follower", "lever", "strip", *LOOSE, *SHEETS):
        from pxr import PhysxSchema

        prim = stage.GetPrimAtPath(f"{prim_path}/body")
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        if VEL_ITERS:
            PhysxSchema.PhysxRigidBodyAPI(prim).CreateSolverVelocityIterationCountAttr(VEL_ITERS)
        _PATHS[spec.name] = f"{prim_path}/body"


bodykit._author_sliders = _author_sliders

# -- followed after every control step ---------------------------------------------------------------------------------


def _fresh() -> dict:
    return {"tcps": None, "s": {}, "fired": 0, "armed": True, "through": [set() for _ in range(N_LOOSE)],
            "views": None, "view_err": "", "dry": 0, "hist": {n: [] for n in STILL_PARTS}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("STAPLER_DEBUG")), "steps": 0}


def _quat_wxyz(m: np.ndarray):
    x, y, z, w = _quat_xyzw(m)
    return w, x, y, z


def _set_frames(name: str, pos0, rot0, pos1, rot1) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[name]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos0]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(rot0)))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in pos1]))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(*_quat_wxyz(rot1)))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for k, name in enumerate(LOOSE):
        for s in SHEETS if not FLIP else ():  # back to the frames they were authored with (as the reset poses), off
            g = f"{name}_{s}"
            _set_frames(g, LOOSE_AT[k] - LOOSE_AT[k], np.eye(3), LOOSE_AT[k] - np.array(_SPEC[s].pos), np.eye(3))
            set_glue_on(g, False)
        set_glue_on(f"hold{k + 1}", True)


def _views():
    """PhysX rigid-contact views, made once on the first control step: (1) the follower against the strip and each
    staple body against the drawer, (2) every moving part against every link of both arms."""
    if _EP["views"] is None and not _EP["view_err"] and len(_PATHS) >= 4:
        try:
            from isaaclab_physx.physics import PhysxManager
            from yam_arena.robot import asset_link_paths

            sim = PhysxManager.get_physics_sim_view()
            env = _PATHS["magazine"].rsplit("/", 2)[0]
            feed = sim.create_rigid_contact_view(
                [_PATHS["follower"], _PATHS["strip"], *[_PATHS[n] for n in LOOSE]],
                filter_patterns=[[_PATHS["strip"]], [_PATHS["magazine"]], *[[_PATHS["magazine"]] for _ in LOOSE]],
                max_contact_data_count=0)
            links = [f"{env}/{arm}{p}" for arm in ("LeftArm", "RightArm") for p in asset_link_paths().values()]
            parts = ["magazine", "lever", "strip", *LOOSE, *SHEETS]
            hands = sim.create_rigid_contact_view([_PATHS[n] for n in parts], filter_patterns=[links] * len(parts),
                                                  max_contact_data_count=0)
            _EP["views"] = (feed, hands, parts)
        except Exception as e:  # noqa: BLE001
            _EP["view_err"] = repr(e)[:300]
    return _EP["views"]


def _np(a) -> np.ndarray:
    if hasattr(a, "numpy") and not hasattr(a, "detach"):
        return np.asarray(a.numpy(), float)
    return as_numpy(a)


def _contacts():
    """(force on the follower from the strip, forces on the strip and each loose staple from the drawer (world, N),
    the largest force between any arm link and any moving part, which part)."""
    v = _views()
    if v is None:
        return None
    feed, hands, parts = v
    m = _np(feed.get_contact_force_matrix(dt=1.0 / 240.0)).reshape(2 + N_LOOSE, 3)
    h = _np(hands.get_contact_force_matrix(dt=1.0 / 240.0)).reshape(len(parts), -1, 3)
    per_part = np.linalg.norm(h, axis=2).max(axis=1)
    i = int(np.argmax(per_part))
    if _DEBUG["on"]:  # the left arm's links (the first half of the filters) on the strip and on the drawer
        half = h.shape[1] // 2
        _DEBUG["left_on"] = {n: round(float(np.linalg.norm(h[parts.index(n), :half].sum(axis=0))), 2)
                             for n in ("strip", "magazine")}
    return m[0], m[1:], float(per_part[i]), parts[i]


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _arm_speed(base) -> float:
    top = 0.0
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        qd = as_numpy(robot.data.joint_vel)[0]
        idx = [j for j, n in enumerate(robot.joint_names) if n.endswith(("joint1", "joint2", "joint3", "joint4",
                                                                           "joint5", "joint6"))]
        top = max(top, float(np.abs(qd[idx]).max()) if idx else 0.0)
    return top


def _pose(base, origin, name):
    t = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return t[:3] - origin, rotation(t[3:7])


def _place(base, origin, name, at, rot) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *_quat_xyzw(rot)]], dtype=torch.float32,
                                              device=asset.device))
    asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=asset.device))


def _tilt(rot) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot[2, 2])))))


def _takes_staple(pos, rot, level: float) -> bool:
    """Does this sheet lie flat on the base (its underside at ``level`` above the table) under the driver, with the
    staple's crown at least MARGIN inside its edges?"""
    if _tilt(rot) > FLAT or abs(float(pos[2]) - SHEET[2] / 2 - (TOP + level)) > ON_BASE:
        return False
    for dx in (-CROWN / 2, CROWN / 2):
        v = rot.T @ (np.array([STAPLE_AT[0] + dx, STAPLE_AT[1], pos[2]]) - pos)
        if abs(float(v[0])) > SHEET[0] / 2 - MARGIN or abs(float(v[1])) > SHEET[1] / 2 - MARGIN:
            return False
    return True


def _drive(base, origin, k: int, sheets: dict) -> None:
    """The driver shears the front staple off the strip and drives it down through the sheets that lie flat under the
    nose: its crown ends on the top one, its legs through them (clinched under the anvil), and it holds them together.
    With no paper under the nose it drops out onto the table beside the anvil."""
    name = LOOSE[k]
    set_glue_on(f"hold{k + 1}", False)
    low_ok = _takes_staple(*sheets["sheet_low"], BASE_H)
    top_ok = _takes_staple(*sheets["sheet_top"], BASE_H + (SHEET[2] if low_ok else 0.0))
    through = [s for s, ok in (("sheet_low", low_ok), ("sheet_top", top_ok)) if ok]
    if not through:
        _place(base, origin, name, (STAPLE_AT[0] + 0.045, STAPLE_AT[1] - 0.01, TOP + LEG - WIRE + 0.0005), np.eye(3))
        _EP["dry"] += 1
        return
    p_top, r_top = sheets[through[-1]]
    surface = float((p_top + r_top @ np.array([0.0, 0.0, SHEET[2] / 2]))[2])
    at = np.array([STAPLE_AT[0], STAPLE_AT[1], surface + 0.0001])
    _place(base, origin, name, at, np.eye(3))
    for s in through:
        ps, rs = sheets[s]
        _set_frames(f"{name}_{s}", np.zeros(3), np.eye(3), rs.T @ (at - ps), rs.T)
        set_glue_on(f"{name}_{s}", True)
    _EP["through"][k] = set(through)


def _fingers(base):
    robot = base.scene["left_robot"]
    q = as_numpy(robot.data.joint_pos)[0]
    return [round(float(q[j]) * 1000, 1) for j, n in enumerate(robot.joint_names) if n.endswith(("joint7", "joint8"))]


def _vel(base, name) -> np.ndarray:
    d = base.scene[name].data
    return (as_numpy(d.root_com_vel_w) if hasattr(d, "root_com_vel_w") else as_numpy(d.root_vel_w))[0][:3]


def _gaps(base, origin, s_pos, s_rot):
    """Debug: each loose staple's distance (mm) from where its glue holds it on the strip."""
    out = []
    for k, n in enumerate(LOOSE):
        p, _r = _pose(base, origin, n)
        out.append(round(float((s_rot.T @ (p - s_pos))[1] - (LOOSE_AT[k][1] - STRIP_AT[1])) * 1000, 2))
    return out


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    m_pos, m_rot = _pose(base, origin, "magazine")
    f_pos, _f_rot = _pose(base, origin, "follower")
    s_pos, s_rot = _pose(base, origin, "strip")
    l_pos, l_rot = _pose(base, origin, "lever")
    sheets = {s: _pose(base, origin, s) for s in SHEETS}
    out = Y_NOSE - float(m_pos[1])  # how far the drawer is pulled out
    fed = float(f_pos[1]) - Y_NOSE  # how far the follower is pushed back from the nose
    # the strip in the drawer's frame: on the rail (crowns resting on it), legs down, along the channel
    v = m_rot.T @ (s_pos - m_pos)
    rel = m_rot.T @ s_rot
    up = math.degrees(math.acos(max(-1.0, min(1.0, float(rel[2, 2])))))
    along = math.degrees(math.acos(min(1.0, abs(float(rel[1, 1])))))
    seated = (abs(float(v[0])) <= SEAT_SIDE and abs(float(v[2]) - CH_FLOOR - RAIL_TOP) <= SEAT_Z and up <= SEAT_UP
              and along <= SEAT_ALONG)
    # the front staple: the first loose one not yet driven, else the strip's own front
    k = _EP["fired"]
    if k < N_LOOSE:
        q_pos, _q = _pose(base, origin, LOOSE[k])
        front = float((m_rot.T @ (q_pos - m_pos))[1]) - PITCH / 2
    else:
        front = float(v[1]) - BODY_L / 2
    c = _contacts()
    if c is None:
        f_follow, f_nose, touch, touched = float("nan"), float("nan"), float("nan"), ""
    else:
        f_follow = float(np.linalg.norm(c[0]))
        f_nose = float((m_rot.T @ c[1][1 + k if k < N_LOOSE else 0])[1])  # pushed back (+y) by the nose
        touch, touched = c[2], c[3]
        _EP["fy"] = [round(float((m_rot.T @ f)[1]), 2) for f in c[1]]
    shut = out <= SHUT
    at_nose = abs(front) <= AT_NOSE and f_nose >= PRESSED_N
    loaded = shut and seated and at_nose and f_follow >= PRESSED_N
    pressed = math.degrees(math.atan2(float(l_rot[2, 1]), float(l_rot[1, 1])))  # pressed down from rest (degrees)
    if pressed <= REARM_DEG:
        _EP["armed"] = True
    if _EP["armed"] and pressed >= FIRE_DEG and loaded and k < N_LOOSE:
        _EP["armed"] = False
        _EP["fired"] += 1
        _drive(base, origin, k, sheets)
    stapled = any(t == set(SHEETS) for t in _EP["through"])
    for n in STILL_PARTS:  # the last second of each part's pose, for the stillness check
        h = _EP["hist"][n]
        h.append(_pose(base, origin, n))
        del h[:-int(20 * SUCCESS_HOLD_S) - 1]
    _EP["tcps"] = _tcps(base, origin)
    speed = _arm_speed(base)
    clear = c is not None and touch <= CLEAR_N
    _EP["s"] = {"loaded": loaded, "stapled": stapled, "clear": clear, "resting": speed <= STILL_ARMS}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0 or _DEBUG["steps"] < 3:
            tl, tr = _EP["tcps"]
            lo, hi = sheets["sheet_low"], sheets["sheet_top"]
            print(f"[stapler] t={_DEBUG['steps'] / 20:5.1f}s out={out * 1000:6.1f}mm fed={fed * 1000:6.1f}mm "
                  f"strip=({v[0] * 1000:.1f},{v[1] * 1000:.1f},{(v[2] - CH_FLOOR - RAIL_TOP) * 1000:.1f})mm "
                  f"up={up:.1f} "
                  f"along={along:.1f} seated={seated} front={front * 1000:.2f}mm F_follow={f_follow:.2f} "
                  f"F_nose={f_nose:.2f} loaded={loaded} pressed={pressed:4.2f} fired={_EP['fired']} "
                  f"through={[sorted(t) for t in _EP['through']]} dry={_EP['dry']} "
                  f"low={np.round(lo[0] * 1000).astype(int).tolist()} "
                  f"top={np.round(hi[0] * 1000).astype(int).tolist()} "
                  f"tilt={_tilt(hi[1]):.1f} touch={touch:.2f}({touched}) speed={speed:.3f} "
                  f"L={np.round(tl * 1000).astype(int).tolist()} R={np.round(tr * 1000).astype(int).tolist()} "
                  f"strip_w={np.round(s_pos * 1000).astype(int).tolist()} score={_points(_status())} "
                  f"v_f={np.round(_vel(base, 'follower'), 3).tolist()} v_s={np.round(_vel(base, 'strip'), 3).tolist()} "
                  f"left_on={_DEBUG.get('left_on')} fingers={_fingers(base)} "
                  f"fy={_EP.get('fy')} gaps={_gaps(base, origin, s_pos, s_rot)} {_EP['view_err']}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------


def _status() -> dict:
    s = _EP["s"]
    return {key: bool(s.get(key)) for key in ("loaded", "stapled", "clear", "resting")}


def _points(s) -> float:
    if s["loaded"] and s["stapled"] and s["clear"] and s["resting"]:
        return 1.0
    if s["stapled"]:
        return 0.5
    return 0.25 if s["loaded"] else 0.0


def score(objects) -> float:
    """Read live at the end. 1.0: the stapler loaded, the two sheets stapled together, no arm touching the drawer, the
    top arm, the strip, its staples or the sheets (contact force above 0.2 N), and both arms at rest (every joint under
    0.05 rad/s); 0.5: the two sheets stapled together; 0.25: not stapled, but the stapler loaded; else 0.0. Loaded:
    the drawer slid shut (within 1 mm), the strip seated on the rail legs down (its middle within 1.5 mm of the
    rail's middle line, its crowns within 1 mm of resting on the rail, upright within 8 degrees, along the channel
    within 5 degrees), its front staple against the nose (within 1 mm, pressed on it with at least 0.5 N) and the
    follower pressing on its back (at least 0.5 N). Stapled: a staple driven through both sheets (it goes in only
    when the arm is pressed 3 degrees with the stapler loaded, through each sheet lying flat on the base under the
    nose with the staple at least 2 mm inside its edges)."""
    return _points(_status())


def _still(name: str) -> bool:
    """Still: over the last second the part moved less than 0.5 mm and turned less than 0.5 degrees. (Measured on its
    pose, not on the solver's velocity, which reads about 1 cm/s for a part held pressed against a joint's stop.)"""
    h = _EP["hist"].get(name, [])
    if len(h) <= int(20 * SUCCESS_HOLD_S):
        return False
    p1, r1 = h[-1]
    for p0, r0 in h:
        turn = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r0.T @ r1)) - 1.0) / 2.0))))
        if float(np.linalg.norm(p1 - p0)) > STILL_MM / 1000 or turn > STILL_DEG:
            return False
    return True


def success(objects) -> bool:
    """Full score with the drawer, the follower, the top arm, the strip and both sheets still (each moved less than
    0.5 mm and turned less than 0.5 degrees over the last second)."""
    full = score(objects) >= 1.0
    if full and _DEBUG["on"] and any(not _still(n) for n in STILL_PARTS):
        print("[stapler] moving:", [n for n in STILL_PARTS if not _still(n)], file=sys.stderr, flush=True)
    return full and all(_still(n) for n in STILL_PARTS)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes straight down on the drawer's pull tab, fingers closing across it, pulls the drawer out 20 cm and
# holds it there. The left hand takes the strip of staples off its tray, fingers closing across it, carries it over the
# open channel, lowers it legs down over the rail and lets go. The right lets the drawer slide back in slowly: the strip
# rides in, meets the follower and is pressed to the nose. It lets go and goes back to where it started. The left
# presses its closed fingertips on the top sheet and slides both sheets in under the nose against the paper guide,
# presses the top arm's pad down past the firing point (the front staple goes through both sheets), lets it spring back,
# presses on the stapled sheets again, slides them back out onto the paper rest and goes back to where it started.

ACROSS = grip_rotation(math.pi / 2)  # right: straight down, fingers closing along x
ACROSS_L = grip_rotation(-math.pi / 2)  # left: the same
ALONG_L = grip_rotation(0.0)  # left: straight down, fingers closing along y (narrow along x)
TAB_OPEN = -0.016
TAB_IN = -0.004
STRIP_OPEN = -0.014
RELEASE_OPEN = -0.0085  # let go of the laid strip: pads about 3 mm clear of it
WIDE_OPEN = -0.022
STRIP_UP = 0.006  # the TCP this far above the crowns while holding
GRIP_OFF = 0.010  # the strip held this far behind (+y) its middle
CARRY_UP, CARRY_OVER = 0.10, 0.08  # lift it this high, and come down onto the rail from this high (the strip reaches
# 9.5 cm ahead of the grip, over the top arm's raised front end on the way)
DROP_GAP = -0.001  # press it onto the rail (it may slip up in the grip), then let go
DROP_DX = 0.005  # aim against the measured offset of the held strip
TIP = 0.0105  # the closed fingertips reach this far below the TCP
PRESS_IN = 0.003  # pressing on the sheets: the fingertips this far into them
PULL = STROKE - 0.004
LEAD_S, LEAD_STEPS = 3.0, 15
SLIDE_IN = 0.065  # push this far: the sheets stop at the paper guide after about 5 cm (the arm lags 1-2 cm under load)
SLIDE_OUT = 0.068
PAD_PUSH = 0.030  # aim this far below the pad at the arm's stop


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger, secs: float = LEAD_S) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(secs / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _tab_tcp(out: float) -> np.ndarray:
    return np.array([SX, Y_NOSE - CAP_T - TAB[1] / 2 - out, TOP + MAG_Z + TAB[2] - TAB_IN])


def _right(dropped_at: float) -> tuple[list[Waypoint], float]:
    tab = _tab_tcp(0.0)
    out = _lead_in("right", tab + [0.0, 0.0, 0.05], ACROSS, "over the drawer's pull tab", TAB_OPEN)
    out += [_w(1.0, tab, TAB_OPEN, "down round it", ACROSS),
            _w(0.6, tab, FINGER_CLOSED, "take it", ACROSS),
            _w(2.5, _tab_tcp(PULL), FINGER_CLOSED, "pull the drawer out of the nose", ACROSS)]
    pulled = sum(w.duration for w in out)
    out.append(_w(max(dropped_at + 0.8 - pulled, 0.5), _tab_tcp(PULL), FINGER_CLOSED,
                  "hold it out while the strip goes in", ACROSS))
    out += [_w(3.5, _tab_tcp(0.004), FINGER_CLOSED, "let it slide back in: the strip meets the follower", ACROSS),
            _w(0.6, _tab_tcp(0.0), TAB_OPEN, "let go", ACROSS),
            _w(0.8, _tab_tcp(0.0) + [0.0, 0.0, 0.05], TAB_OPEN, "up", ACROSS)]
    out += _go_home("right", _tab_tcp(0.0) + [0.0, 0.0, 0.05], ACROSS, TAB_OPEN)
    return out, pulled


def _press_point(x: float) -> np.ndarray:
    """The left TCP pressing its closed fingertips into the top sheet at x (on the sheet's middle line along y)."""
    top = TOP + BASE_H + 2 * SHEET[2]
    return np.array([x, TOP_SHEET_AT[1], top + TIP - PRESS_IN])


def _pad_tcp(pressed_deg: float) -> np.ndarray:
    """The left TCP on the arm's press pad with the arm pressed ``pressed_deg`` down from rest."""
    p = PIVOT + _rx(pressed_deg - LEVER_TILT) @ np.array([0.0, -PAD_AT, LEVER_T + 0.006])
    return p + np.array([0.0, 0.0, TIP])


def _left(pulled_at: float, back_at: float, total: float) -> tuple[list[Waypoint], float]:
    x, y = float(STRIP_AT[0]), STRIP_FRONT + STRIP_L / 2 + GRIP_OFF
    pick = np.array([x, y, TOP + TRAY_T + LEG + STRIP_UP])  # the TCP over the strip's top
    out = _lead_in("left", pick + [0.0, 0.0, 0.05], ACROSS_L, "over the strip of staples", STRIP_OPEN)
    out += [_w(1.0, pick, STRIP_OPEN, "down round it", ACROSS_L),
            _w(0.6, pick, FINGER_CLOSED, "take it", ACROSS_L),
            _w(1.2, pick + [0.0, 0.0, CARRY_UP], FINGER_CLOSED, "lift it", ACROSS_L)]
    rear = Y_NOSE - 0.008  # its back end 8 mm in front of the follower
    drop = np.array([SX + DROP_DX, rear - STRIP_L / 2 + GRIP_OFF,
                     TOP + Z0 + RAIL_TOP + WIRE + DROP_GAP + STRIP_UP])
    t = sum(w.duration for w in out)
    out.append(_w(max(pulled_at + 0.3 - t, 2.2), drop + [0.0, 0.0, CARRY_OVER], FINGER_CLOSED,
                  "over the open channel, high enough to clear the top arm", ACROSS_L))
    out += [_w(1.6, drop + [0.0, 0.0, 0.004], FINGER_CLOSED, "lower it legs down over the rail", ACROSS_L),
            _w(0.6, drop, FINGER_CLOSED, "onto the rail", ACROSS_L),
            _w(0.6, drop, RELEASE_OPEN, "let go", ACROSS_L)]
    dropped = sum(w.duration for w in out)
    out += [_w(0.6, drop + [0.0, 0.0, 0.004], RELEASE_OPEN, "up a little, clear of the tray's lips", ACROSS_L),
            _w(0.5, drop + [0.0, 0.0, 0.004], WIDE_OPEN, "open wide", ACROSS_L),
            _w(1.0, drop + [0.0, 0.0, 0.06], WIDE_OPEN, "straight up", ACROSS_L)]
    # wait clear of the drawer until it is back in, then the sheets
    x0 = TOP_SHEET_AT[0] - SHEET[0] / 2 + 0.020
    p_in = _press_point(x0)
    out.append(_w(2.0, p_in + [0.0, 0.0, 0.06], FINGER_CLOSED, "over the top sheet's back edge", ALONG_L))
    t = sum(w.duration for w in out)
    out.append(_w(max(back_at - t, 0.5), p_in + [0.0, 0.0, 0.06], FINGER_CLOSED, "wait for the drawer", ALONG_L))
    out += [_w(1.0, p_in, FINGER_CLOSED, "press on the sheets", ALONG_L),
            _w(2.5, p_in + [SLIDE_IN, 0.0, 0.0], FINGER_CLOSED, "slide them in under the nose, to the paper guide",
               ALONG_L),
            _w(0.8, p_in + [SLIDE_IN, 0.0, 0.05], FINGER_CLOSED, "up", ALONG_L)]
    pad0 = _pad_tcp(0.0)
    deep = _pad_tcp(LEVER_STOP) - [0.0, 0.0, PAD_PUSH]
    out += [_w(1.5, pad0 + [0.0, 0.0, 0.04], FINGER_CLOSED, "over the arm's press pad", ALONG_L),
            _w(0.8, pad0, FINGER_CLOSED, "onto the pad", ALONG_L),
            _w(1.5, deep, FINGER_CLOSED, "press the arm down past the firing point: the staple goes in", ALONG_L),
            _w(0.5, deep, FINGER_CLOSED, "hold it down", ALONG_L),
            _w(1.2, pad0 + [0.0, 0.0, 0.04], FINGER_CLOSED, "let it spring back up", ALONG_L)]
    p_out = _press_point(x0 + SLIDE_IN - 0.006)
    out += [_w(1.5, p_out + [0.0, 0.0, 0.05], FINGER_CLOSED, "over the stapled sheets", ALONG_L),
            _w(1.0, p_out, FINGER_CLOSED, "press on them", ALONG_L),
            _w(2.5, p_out - [SLIDE_OUT, 0.0, 0.0], FINGER_CLOSED, "slide them back out onto the paper rest", ALONG_L),
            _w(0.8, p_out - [SLIDE_OUT, 0.0, -0.06], FINGER_CLOSED, "up", ALONG_L)]
    out += _go_home("left", p_out - [SLIDE_OUT, 0.0, -0.06], ALONG_L, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(Waypoint(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                        "hold still: the trial ends once the full score has held 1 s", out[-1].yaw))
    return out, dropped


def demo():
    _, pulled = _right(0.0)
    _, dropped = _left(pulled, 0.0, 0.0)
    right, _ = _right(dropped)
    back = sum(w.duration for w in right[:-LEAD_STEPS - 1])  # the drawer back in and the tab let go
    left, _ = _left(pulled, back, 0.0)
    total = max(sum(w.duration for w in left), sum(w.duration for w in right)) + 3.0
    right.append(Waypoint(max(total - sum(w.duration for w in right), 0.5), right[-1].pos, TAB_OPEN,
                          "wait at the start", right[-1].yaw))
    left, _ = _left(pulled, back, total)
    if os.environ.get("STAPLER_ONE_HAND"):  # the one-hand check: the left stays where it started
        from yam_arena import ik, station
        from yam_arena.constants import READY

        p, r = ik.tcp_pose(np.array(READY, float), station.BASES["left"])
        left = [Waypoint(total, tuple(float(v) for v in p), FINGER_CLOSED, "idle at the start", r)]
    return {"left": left, "right": right}
