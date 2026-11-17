"""Mount an ASTM D638 Type I dogbone tensile specimen coaxially in the two opposed wedge grips of a horizontal force
test stand.

Bench layout in station coordinates (TOP = station.TABLE_TOP, +x forward, +y left; arm bases at y = +-0.30 m):
- Horizontal motorized force test stand along y at (0.300, 0): loading axis 76 mm above
  the bench, force gauge at the +y end, crosshead at the -y end.
- Two wedge grips (19.1 mm jaws, 0-6.4 mm opening, housing 49.5 x 62.2 x 38.1 mm, open thru slot). Grip A (+y) is
  fixed; grip B (-y) rides the crosshead (joint ``crosshead``, 0 = backed off 35 mm), whose drive target moves at
  500 mm/min only while the control panel's LEFT key (``key_left``) is held down (RIGHT, ``key_right``, moves it
  back). Each upper jaw (part
  ``upper_jaw_l`` / ``upper_jaw_r``, joint ``gap_l`` / ``gap_r`` = jaw gap) is spring-closed; its red lever
  (``lever_l`` / ``lever_r``, 0 = closed, 0.022 = open) springs back unless held, and its wedge lifts the jaw to a
  6.2 mm gap. The +y tab is slid in sideways (+x) through the open front of the thru slot; grip B is jogged onto the
  -y tab along the axis.
- An ASTM D638 Type I LDPE dogbone (165 x 19 x 3.2 mm, 13 x 57 mm neck) lies flat on a specimen tray at
  (0.180, 0), 12 mm above the bench.

Run it:
  inspect-robots run --embodiment yam_arena -E task=qc_met10.py -E object_jitter=0 -E control_hz=20 \\
    -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" --scorer task_score ...
"""

from __future__ import annotations

import json
import math
import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
from labkit import Joint, PartState, on_reset, on_step, part  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

# -- geometry (metres; the test-stand part origin is on the bench top under the loading axis) -------------------
STAND_XY = (0.300, 0.000)
MAT_H = 0.003
AXIS_Z = 0.076  # loading axis above the bench (horizontal motorized force test stand)
SPEC_T = 0.0032  # ASTM D638 Type I thickness
SPEC_HALF = 0.0825  # 165 mm overall
FLOOR_Z = AXIS_Z - SPEC_T / 2.0  # serrated lower-jaw face: a seated dogbone's mid-plane is on the loading axis
BED_TOP = 0.040
GAUGE_Y = 0.205  # force gauge body 0.170..0.240: clear of a gripper working the +y lever
CROSSHEAD_Y = 0.160  # crosshead centre with grip B at the 115 mm gauge length (tie rod 17 mm)
GRIP_FACE = 0.0575  # jaw faces at +-57.5 mm: 115.0 mm grip separation
STOP_Y = 0.0905  # slot end behind each jaw: 33 mm jaw length (scaled from the maker drawing, unverified)
JAW_W = 0.0191  # wedge-grip jaw width, maker drawing C (0.75 in); the tab is 19 mm wide
HOUSING_D = 0.0381  # housing depth along the thru slot, drawing D (1.50 in); the slot is open front and back (x)
HOUSING_A = 0.0495  # housing width across the jaws, drawing A (1.95 in); vertical here (jaw faces horizontal)
HOUSING_B = 0.0622  # housing length along the pull axis, drawing B (2.45 in)
HOUSING_Y0 = GRIP_FACE + 0.003  # serrated jaw tips stand 3 mm out of the housing mouth (as drawn)
HOUSING_TOP = AXIS_Z + HOUSING_A / 2.0
LEAD_IN = 0.003  # 45 deg chamfer on the front edge of the slot floor (unverified: edge finish not on the drawing)
JAW_EDGE = 0.0015  # 45 deg chamfer on the lower jaw's entry edge at the jaw face (unverified)
JAW_OPEN = 0.0064  # jaw gap 0-6.4 mm, drawing F (0-0.25 in): the real opening, nothing enlarged
JAW_H = 0.008  # upper jaw thickness
JAW_TOP = FLOOR_Z + JAW_H  # upper jaw top with the gap at 0 (closed on nothing)
SLOT_CEIL = FLOOR_Z + JAW_OPEN + JAW_H + 0.0006  # housing above the fully opened upper jaw
STEM = (0.006, 0.010)  # jaw stem up through the housing top (x, y)
FOLLOWER_Z = HOUSING_TOP + 0.005  # underside of the jaw's cross-bar follower with the gap at 0
FOLLOWER = (0.006, 0.026, 0.004)
GAP_LIFTED = 0.0062  # the lever's wedge lifts the jaw to a 6.2 mm gap (inside the real 0-6.4 mm range)
RAMP_LOW = FOLLOWER_Z - 0.001  # wedge surface under the follower with the lever closed: the jaw rests on the tab
RAMP_RUN = 0.016  # wedge length along x; then a flat land holds the jaw open with no back-drive
RAMP_Y = 0.009  # the two wedge rails run either side of the jaw stem
LEVER_OPEN = 0.022  # lever travel back (+x) until the follower sits on the flat land (unverified vs the real lever)
PADDLE_X = 0.026  # lever handle behind the housing
PADDLE_SIZE = (0.006, 0.026, 0.030)
PADDLE_BOT = HOUSING_TOP + 0.001
PADDLE_TOP = PADDLE_BOT + PADDLE_SIZE[2]
LEVER_K = 300.0  # N/m: the lever springs back to closed (6.6 N held fully open, a light finger push; declared)
LEVER_FRICTION = 0.3  # N
JAW_K = 600.0  # N/m: spring-closed jaw: 1.9 N initial bite on a 3.2 mm tab, 3.7 N held open (declared, see README)
JAW_MASS = 0.040  # steel upper jaw + stem
# -- motorised crosshead carrying grip B (horizontal motorized force test stand, user's guide in README provenance) --
XH_BACK = 0.035  # start: crosshead backed off 35 mm along -y, as a previous test leaves it (grip B face at -92.5 mm)
XH_MAX = XH_BACK + 0.015  # modelled end of travel for this scene (historical: 457 mm was the layout-source stand's travel; the build list's stand is a bench motorised screw stage)
JOG_SPEED = 0.500 / 60.0  # m/s: SPEED set to 500 mm/min (guide: 0.5-1,100 mm/min; default 250)
XH_K, XH_D, XH_MASS = 20000.0, 1000.0, 0.8  # lead-screw drive: stiff position servo on the crosshead target
PANEL_XY = (0.300, 0.275)  # control panel on its bracket at the +y end of the frame (guide: left or right end)
PANEL_TOP = 0.085  # key face above the bench (unverified)
PANEL_SIZE = (0.110, 0.060, 0.016)
KEY_SIZE = (0.024, 0.024, 0.004)  # 24 mm square simulation surrogate for the round Maple Systems PBS111 push-button head (head diameter not published in the datasheet; bezel 29 mm is the datasheet value)
KEY_DX = 0.045  # key pitch along x: two PBS111 buttons with their 30 mm clip blocks side by side (30 mm datasheet block width) and 15 mm between blocks (estimate)
KEY_ROW = 0.010  # key row on the panel, away from the gauge (unverified)
KEY_TRAVEL = 0.002  # key stroke (unverified)
KEY_TRIP = 0.0012  # pressed past this, the key runs the crosshead (momentary KEYS setting)
KEY_K = 2000.0  # N/m: key return spring (4 N bottomed out; unverified)
TRAY_XY = (0.180, 0.000)
REST_H = 0.012  # dogbone underside above the bench on the tray rests
REST_Y = 0.074  # rests under the tab ends
STOP_X = 0.0205  # tray back stops (front face, from the tray centre): squaring the dogbone against them registers it
STOP_TOP = REST_H - MAT_H + 0.005  # stop tops 5 mm above the rests (1.8 mm above the dogbone)

# -- scoring constants (single source of truth for score(), success(), distance(), demo, tests) --------------------
# One additive table of end-state checks (ruler / protractor / photo, README "Scoring"). Each grip's rows count on
# their own; the mount rows count once both tabs are clamped (seated with the lever forward).
DEPTH_MIN = 0.017  # tab seated: past the 17 mm ruler mark at its jaw face (25 mm nominal bite) ...
SEAT_Z_TOL = 0.003  # ... inside the slot between the jaws (not on top of the 8 mm upper jaw) ...
SEAT_X_TOL = 0.0040  # ... and its edge overhangs the 19.1 mm jaw by at most 4 mm
STOP_GAP = 0.002  # visible gap of at least 2 mm between a tab end and its slot end
JAW_ON_TAB_MAX = SPEC_T + 0.0015  # jaw closed on the tab: gap at most 1.5 mm over the 3.2 mm tab (feeler check); a lever forward with the jaw held open or off the tab does not count
LEVER_FWD_MAX = 0.002  # lever fully forward: within 2 mm of its forward stop (22 mm travel)
SKEW_MAX_DEG = 2.0  # specimen within 2 deg of the pull axis in yaw and pitch (protractor)
GAUGE_L = 0.115  # ASTM D638 Type I distance between grips D = 115 +- 5 mm (Table 1), ruler between the jaw faces
GAUGE_TOL = 0.005
HAND_CLEAR_M = 0.050
SEAT_CREDIT = 0.10  # per grip: tab seated
LEVER_CREDIT = 0.10  # per grip: lever fully forward on the seated tab (the spring-closed jaw clamps it)
GAP_CREDIT = 0.05  # per grip: the seated tab's end >= STOP_GAP from its slot end
STRAIGHT_CREDIT = 0.15  # both tabs clamped: skew <= SKEW_MAX_DEG
GAUGE_CREDIT = 0.20  # both tabs clamped: jaw faces 115 +- 5 mm apart
CLEAR_CREDIT = 0.15  # both tabs clamped: both grippers >= 5 cm from the specimen, both levers and both jog keys
SCORE_OVER = "end"
SUCCESS_HOLD_S = 3.0  # success(): the full end state (score 1.0) held 3 s; the runtime counts the hold, score() not

# -- look -----------------------------------------------------------------------------------------------------------
STAND_GREY = (0.62, 0.64, 0.66)
ALU = (0.80, 0.81, 0.83)
HOUSING_BLACK = (0.10, 0.10, 0.11)
JAW_STEEL = (0.70, 0.72, 0.74)
LEVER_RED = (0.78, 0.12, 0.10)
GAUGE_GREY = (0.22, 0.23, 0.25)
DISPLAY = (0.05, 0.12, 0.22)
PC_NATURAL = (0.93, 0.90, 0.80)
TRAY_BLUE = (0.30, 0.40, 0.52)
MAT_GREY = (0.16, 0.17, 0.18)
VIS = "assets/qc_met10/"  # authored PBR meshes (tools/author_visuals.py), drawn over the invisible collider shapes


def _shift(boxes, dy: float):
    """Boxes moved dy along y (grip B's shapes at the crosshead's start, backed off by XH_BACK)."""
    return [((c[0], c[1] + dy, c[2]), *rest) for c, *rest in boxes]


def _lever_joint(side: int, name: str, dy: float = 0.0, parent: str = "") -> Joint:
    """Release lever of one grip: a handle behind the housing on a slide along +x (0 = closed, LEVER_OPEN = open),
    carrying two wedge rails that run under the upper jaw's follower bar and lift the jaw to GAP_LIFTED. A spring
    returns it to closed, so it has to be held open."""
    yc = side * (GRIP_FACE + STOP_Y) / 2.0 + dy
    rise = GAP_LIFTED + 0.001
    th = math.atan2(rise, RAMP_RUN)
    t = 0.003
    xa = -0.001  # low end of the wedge (just in front of the follower with the lever closed)
    xm, zm = xa - RAMP_RUN / 2.0, RAMP_LOW + rise / 2.0
    boxes = [((PADDLE_X, yc, PADDLE_BOT + PADDLE_SIZE[2] / 2.0), PADDLE_SIZE, LEVER_RED)]  # lever handle
    for dy in (-RAMP_Y, RAMP_Y):
        y = yc + dy
        boxes += [
            ((xm - math.sin(th) * t / 2.0, y, zm - math.cos(th) * t / 2.0), (math.hypot(RAMP_RUN, rise), 0.005, t),
             HOUSING_BLACK, (0.0, th)),  # wedge
            ((xa - RAMP_RUN - 0.005, y, RAMP_LOW + rise - t / 2.0), (0.010, 0.005, t), HOUSING_BLACK),  # flat land
            (((xa + PADDLE_X) / 2.0, y, RAMP_LOW - 0.003), (PADDLE_X - xa, 0.005, 0.002), HOUSING_BLACK),  # rail
        ]
    return Joint(name, kind="slide", axis="X", boxes=tuple(boxes), lower=0.0, upper=LEVER_OPEN, stiffness=LEVER_K,
                 damping=4.0, friction=LEVER_FRICTION, target=0.0, initial=0.0, mass=0.030, parent=parent,
                 visual=f"{VIS}{name}.usdc")


def _jaw_part(side: int, name: str):
    """Upper jaw of one grip on a z slide (joint value = jaw gap): a spring closes it onto the lower jaw or onto a
    seated tab; the lever's wedge lifts it by its cross-bar follower. Grip B's jaw (-y) rides a y carriage driven to
    the crosshead's own target (the jaw and the lever wedge must be separate articulations to touch)."""
    dy = 0.0 if side > 0 else -XH_BACK
    yc = side * (GRIP_FACE + STOP_Y) / 2.0 + dy
    bite = STOP_Y - GRIP_FACE - 0.001
    boxes = (
        ((0.0, yc, FLOOR_Z + JAW_H / 2.0), (JAW_W, bite, JAW_H), JAW_STEEL),  # upper serrated jaw
        ((0.0, yc, (JAW_TOP + FOLLOWER_Z) / 2.0), (*STEM, FOLLOWER_Z - JAW_TOP), JAW_STEEL),  # stem
        ((0.0, yc, FOLLOWER_Z + FOLLOWER[2] / 2.0), FOLLOWER, JAW_STEEL),  # follower bar on the wedge rails
    )
    base = (((0.0, side * (STOP_Y + 0.020), BED_TOP + 0.006), (0.010, 0.010, 0.010), HOUSING_BLACK),)  # inside the housing
    if side < 0:  # inside the stand base, below the crosshead's path
        base = (((0.0, -(STOP_Y + 0.020), BED_TOP - 0.010), (0.010, 0.010, 0.010), HOUSING_BLACK),)
    if side > 0:
        gap = Joint("gap_l", kind="slide", axis="Z", boxes=boxes, lower=0.0, upper=JAW_OPEN, stiffness=JAW_K,
                    damping=2.0, friction=0.0, target=0.0, initial=0.0, mass=JAW_MASS, visual=f"{VIS}{name}.usdc")
        return part(name, (*STAND_XY, TOP + MAT_H), fixed=True, friction=1.0, boxes=base, joints=(gap,))
    pin = (((0.0, yc, JAW_TOP + 0.0105), (0.004, 0.004, 0.004), JAW_STEEL),)  # guide pin inside the jaw stem
    carriage = Joint("carriage_r", kind="slide", axis="Y", boxes=pin, lower=0.0, upper=XH_MAX, stiffness=XH_K,
                     damping=XH_D, target=0.0, initial=0.0, mass=0.050)
    gap = Joint("gap_r", kind="slide", axis="Z", boxes=boxes, lower=0.0, upper=JAW_OPEN, stiffness=JAW_K,
                damping=2.0, friction=0.0, target=0.0, initial=0.0, mass=JAW_MASS, parent="carriage_r",
                visual=f"{VIS}{name}.usdc")
    return part(name, (*STAND_XY, TOP + MAT_H), fixed=True, friction=1.0, boxes=base, joints=(carriage, gap))


def _grip_shapes(side: int):
    s = float(side)
    yh = s * (HOUSING_Y0 + HOUSING_B / 2.0)
    y_end = HOUSING_Y0 + HOUSING_B
    cheek = HOUSING_D / 2.0 - STEM[0] / 2.0 - 0.001
    upper = HOUSING_TOP - SLOT_CEIL
    rod = 0.170 if side > 0 else CROSSHEAD_Y - 0.020
    e = JAW_EDGE
    return [
        ((0.0, s * (GRIP_FACE + e + STOP_Y) / 2.0, (BED_TOP + FLOOR_Z) / 2.0),
         (JAW_W, STOP_Y - GRIP_FACE - e, FLOOR_Z - BED_TOP), JAW_STEEL),  # lower jaw
        ((0.0, s * (GRIP_FACE + e / 2.0 + 0.0015 * math.sqrt(0.5)), FLOOR_Z - e / 2.0 - 0.0015 * math.sqrt(0.5)),
         (e * math.sqrt(2.0), JAW_W, 0.003), JAW_STEEL, (math.pi / 2.0, -s * math.pi / 4.0)),  # entry-edge chamfer
        ((LEAD_IN / 2.0, yh, (BED_TOP + FLOOR_Z) / 2.0), (HOUSING_D - LEAD_IN, HOUSING_B, FLOOR_Z - BED_TOP),
         HOUSING_BLACK),  # housing below the thru slot (the slot floor, flush with the lower jaw), on its mount
        ((-HOUSING_D / 2.0 + LEAD_IN / 2.0 + 0.004 * math.sqrt(0.5), yh, FLOOR_Z - LEAD_IN / 2.0 - 0.004 * math.sqrt(0.5)),
         (LEAD_IN * 1.4142, HOUSING_B, 0.008), HOUSING_BLACK, (0.0, -math.pi / 4.0)),  # 3 mm x 45 deg lead-in
        ((0.0, s * (STOP_Y + 0.004), (FLOOR_Z + SLOT_CEIL) / 2.0), (HOUSING_D, 0.008, SLOT_CEIL - FLOOR_Z),
         HOUSING_BLACK),  # slot end behind the jaws (axial stop)
        ((-(HOUSING_D / 2.0 - cheek / 2.0), yh, SLOT_CEIL + upper / 2.0), (cheek, HOUSING_B, upper), HOUSING_BLACK),
        ((HOUSING_D / 2.0 - cheek / 2.0, yh, SLOT_CEIL + upper / 2.0), (cheek, HOUSING_B, upper), HOUSING_BLACK),
        ((0.0, s * (y_end + rod) / 2.0, (BED_TOP + AXIS_Z) / 2.0), (0.024, rod - y_end, AXIS_Z - BED_TOP),
         ALU),  # tie rod from the grip to the gauge (+y) / crosshead (-y)
    ]


def _stand_shapes():
    boxes = [
        ((0.0, 0.0, BED_TOP / 2.0), (0.130, 0.460, BED_TOP), STAND_GREY),  # horizontal stand base
        ((-0.050, 0.0, BED_TOP + 0.002), (0.012, 0.300, 0.004), ALU),  # scale rail
        ((0.0, GAUGE_Y, BED_TOP + 0.050), (0.090, 0.070, 0.100), GAUGE_GREY),  # force gauge
        ((-0.046, GAUGE_Y, BED_TOP + 0.065), (0.003, 0.050, 0.040), DISPLAY),  # gauge display (faces the operator)
    ]
    boxes += _grip_shapes(+1)
    return tuple(boxes)


def _crosshead_joint() -> Joint:
    """Crosshead carrying grip B along the loading axis (slide y, 0 = backed off by XH_BACK). Its drive holds the
    target that the jog key advances at JOG_SPEED while the key is held down."""
    boxes = [((0.0, -CROSSHEAD_Y, BED_TOP + 0.050), (0.100, 0.040, 0.100), STAND_GREY)] + _grip_shapes(-1)  # crosshead
    return Joint("crosshead", kind="slide", axis="Y", anchor=(0.0, -CROSSHEAD_Y - XH_BACK, BED_TOP + 0.050),
                 boxes=tuple(_shift(boxes, -XH_BACK)), lower=0.0, upper=XH_MAX, stiffness=XH_K, damping=XH_D,
                 target=0.0, initial=0.0, mass=XH_MASS, visual=f"{VIS}crosshead.usdc")


def _panel_part():
    """Stand control panel on its bracket at the +y end of the frame, keys facing up: LEFT runs the crosshead
    toward +y (operator's left), RIGHT toward -y, while held (momentary keys); STOP is fixed."""
    px, py = PANEL_XY
    face = PANEL_TOP
    body = [
        ((0.0, 0.0, face - PANEL_SIZE[2] / 2.0), PANEL_SIZE, GAUGE_GREY),  # panel housing
        ((0.0, -0.034, 0.0225), (0.030, 0.022, 0.025), STAND_GREY),  # bracket on the frame end, under the gauge
        ((0.0, -0.026, (0.035 + face - PANEL_SIZE[2]) / 2.0), (0.030, 0.008, face - PANEL_SIZE[2] - 0.035),
         STAND_GREY),  # bracket riser
        ((0.0, -0.016, face + 0.0004), (0.050, 0.016, 0.0008), DISPLAY),  # display window
        ((0.0, KEY_ROW, face + 0.0005), (KEY_SIZE[0], KEY_SIZE[1], 0.001), LEVER_RED),  # STOP key (fixed, flush)
    ]

    def key(name, dx):
        return Joint(name, kind="slide", axis="Z", boxes=(((dx, KEY_ROW, face + KEY_SIZE[2] / 2.0), KEY_SIZE, ALU),),
                     lower=-KEY_TRAVEL, upper=0.0, stiffness=KEY_K, damping=2.0, target=0.0, initial=0.0, mass=0.005)

    return part("control_panel", (px, py, TOP + MAT_H), fixed=True, friction=1.0, boxes=body,
                joints=(key("key_left", -KEY_DX), key("key_right", KEY_DX)), visual=f"{VIS}control_panel.usdc")


def _dogbone_shapes():
    t = SPEC_T
    steps = ((0.0285, 0.0370, 0.0145), (0.0370, 0.0455, 0.0165), (0.0455, 0.0540, 0.0182))
    boxes = [((0.0, 0.0, 0.0), (0.013, 0.057, t), PC_NATURAL)]
    for s in (1.0, -1.0):
        for y0, y1, w in steps:
            boxes.append(((0.0, s * (y0 + y1) / 2.0, 0.0), (w, y1 - y0, t), PC_NATURAL))
        boxes.append(((0.0, s * (0.0540 + SPEC_HALF) / 2.0, 0.0), (0.019, SPEC_HALF - 0.0540, t), PC_NATURAL))
    return tuple(boxes)


MAT = part("bench_mat", (0.270, 0.000, TOP + MAT_H / 2.0), fixed=True, friction=1.0,
           boxes=(((0.0, 0.0, 0.0), (0.420, 0.600, MAT_H), MAT_GREY),), visual=f"{VIS}bench_mat.usdc")
STAND = part("test_stand", (*STAND_XY, TOP + MAT_H), fixed=True, friction=1.0, boxes=_stand_shapes(),
             visual=f"{VIS}test_stand.usdc",
             joints=(_lever_joint(+1, "lever_l"), _crosshead_joint(),
                     _lever_joint(-1, "lever_r", -XH_BACK, "crosshead")))
PANEL = _panel_part()
JAW_L = _jaw_part(+1, "upper_jaw_l")
JAW_R = _jaw_part(-1, "upper_jaw_r")
TRAY = part("specimen_tray", (*TRAY_XY, TOP + MAT_H), fixed=True, friction=1.0,
            boxes=(((0.00225, 0.0, 0.0015), (0.0445, 0.200, 0.003), TRAY_BLUE),
                   ((STOP_X + 0.002, REST_Y - 0.005, (0.003 + STOP_TOP) / 2.0), (0.004, 0.010, STOP_TOP - 0.003),
                    TRAY_BLUE),
                   ((STOP_X + 0.002, -REST_Y + 0.005, (0.003 + STOP_TOP) / 2.0), (0.004, 0.010, STOP_TOP - 0.003),
                    TRAY_BLUE),
                   ((0.0, REST_Y, (0.003 + REST_H - MAT_H) / 2.0), (0.030, 0.022, REST_H - MAT_H - 0.003), TRAY_BLUE),
                   ((0.0, -REST_Y, (0.003 + REST_H - MAT_H) / 2.0), (0.030, 0.022, REST_H - MAT_H - 0.003), TRAY_BLUE)),
            visual=f"{VIS}specimen_tray.usdc")
DOGBONE = part("dogbone", (*TRAY_XY, TOP + REST_H + SPEC_T / 2.0), mass=0.0078, friction=1.0,
               boxes=_dogbone_shapes(), visual=f"{VIS}dogbone.usdc")

# Bench dressing behind the stand, visual only (NVIDIA SimReady models, tools/make_props.py): PC keyboard, mouse on its
# pad, vernier caliper, safety glasses. Origin on the bench under the station origin; its one collider is a 1 mm tile
# under the keyboard (x >= 0.48 m, where the demo's arms never come below 0.30 m above the bench).
PROPS = part("bench_props", (0.0, 0.0, TOP), fixed=True, friction=1.0,
             boxes=(((0.640, 0.080, 0.0005), (0.020, 0.020, 0.001), MAT_GREY),), visual=f"{VIS}bench_props.usda")

OBJECTS = (MAT, STAND, JAW_L, JAW_R, PANEL, TRAY, DOGBONE, PROPS)

# world positions the demo and the clear check use (fixed geometry, no state)
_BASE_Z = TOP + MAT_H
AXIS_WORLD = (STAND_XY[0], STAND_XY[1], _BASE_Z + AXIS_Z)
PADDLE_WORLD = {s: (STAND_XY[0] + PADDLE_X, s * (GRIP_FACE + STOP_Y) / 2.0, _BASE_Z + PADDLE_TOP) for s in (1, -1)}
KEY_WORLD = {k: (PANEL_XY[0] + dx, PANEL_XY[1] + KEY_ROW, _BASE_Z + PANEL_TOP + KEY_SIZE[2])
             for k, dx in (("left", -KEY_DX), ("right", KEY_DX))}  # key tops at rest

# -- trial state ----------------------------------------------------------------------------------------------------
_trial: dict = {"tcps": None, "step": -1, "hold_s": 0.0, "xh": 0.0}
_ids: dict = {}


def _set_test_trial(tcps=None) -> None:
    """Test hook for offline scoring tests (never called by demo()): gripper positions."""
    _trial["tcps"] = None if tcps is None else tuple(np.asarray(p, float) for p in tcps)


@on_reset
def _reset(env) -> None:
    _trial.update(tcps=None, step=-1, hold_s=0.0, xh=0.0)
    _ids.clear()
    if env is not None:
        _drive_crosshead(env, 0.0)


def _arm_tcps(env):
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            jp = labkit._np(asset.data.joint_pos)[0]
            tcp_p, _ = ik.tcp_pose(jp[_ids[key]], station.BASES[side])
            out.append(tcp_p)
        return (out[0], out[1])
    except Exception:
        return None


def _drive_crosshead(env, q: float) -> None:
    """The stand's drive: the crosshead and the carriage under grip B's jaw both servo to the jog target q."""
    import torch

    base = env.unwrapped
    for name, joint in (("test_stand", "crosshead"), ("upper_jaw_r", "carriage_r")):
        asset = base.scene[name]
        if name not in _ids:
            _ids[name] = [list(asset.joint_names).index(f"{joint}_joint")]
        asset.set_joint_position_target(torch.tensor([[q]], device=asset.device, dtype=torch.float32),
                                        joint_ids=_ids[name])


# (name, joint_or_body) of each scored sub-action, printed to the run log with its end value
SUBACTIONS = (
    ("seat +y tab in the +y grip (lever held open)", "dogbone.tab_+y"),
    ("clamp +y grip: lever let go, spring closes the jaw on the tab", "test_stand.lever_l+upper_jaw_l.gap_l"),
    ("jog the crosshead: grip B over the -y tab (lever held open)", "test_stand.crosshead+dogbone.tab_-y"),
    ("clamp -y grip: lever let go, spring closes the jaw on the tab", "test_stand.lever_r+upper_jaw_r.gap_r"),
    ("specimen straight at the 115 mm gauge length, ends off the slot ends", "dogbone+gauge"),
)


def _log_subaction_end(objects: dict, step: int, run_end: bool) -> None:
    """Print one SUBACTION_END JSON line per sub-action (metres, degrees), the 0-based step and run_end (the success
    hold is complete). Called on every step of the full-score hold, so a successful run's last lines are its end
    state. Observe-only."""
    dog, stand = objects["dogbone"], objects["test_stand"]
    end_l, end_r = tab_ends(dog)
    yaw, pitch = skew_deg(dog)
    face_l, face_r = grip_faces(objects)
    r6 = lambda v: round(float(v), 6)  # noqa: E731
    ends = (
        {"end": [r6(v) for v in end_l], "depth": r6(end_l[1] - face_l)},
        {"lever_l": r6(stand.joints.get("lever_l", -1.0)), "gap_l": r6(jaw_gap(objects, "l"))},
        {"crosshead": r6(stand.joints.get("crosshead", -1.0)), "end": [r6(v) for v in end_r],
         "depth": r6(face_r - end_r[1])},
        {"lever_r": r6(stand.joints.get("lever_r", -1.0)), "gap_r": r6(jaw_gap(objects, "r"))},
        {"pos": [r6(v) for v in dog.pos], "yaw_deg": round(yaw, 3), "pitch_deg": round(pitch, 3),
         "gauge_m": r6(face_l - face_r), "stop_gap_m": [r6(STOP_Y - GRIP_FACE - (end_l[1] - face_l)),
                                                         r6(STOP_Y - GRIP_FACE - (face_r - end_r[1]))]},
    )
    for (name, key), end in zip(SUBACTIONS, ends):
        rec = {"name": name, "joint_or_body": key, "end_value": end, "step": step, "run_end": run_end}
        print("SUBACTION_END " + json.dumps(rec), flush=True)


@on_step
def _watch(env) -> None:
    _trial["tcps"] = _arm_tcps(env)
    _trial["step"] += 1
    try:
        objects = {n: labkit._part_state(env, labkit._SPECS[n]) for n in ("dogbone", "test_stand", "upper_jaw_l",
                                                                            "upper_jaw_r", "control_panel")}
    except Exception:
        return
    dt = float(getattr(env.unwrapped, "step_dt", 0.05))
    keys = objects["control_panel"].joints
    run = (keys.get("key_left", 0.0) <= -KEY_TRIP) - (keys.get("key_right", 0.0) <= -KEY_TRIP)
    _trial["xh"] = min(max(_trial["xh"] + run * JOG_SPEED * dt, 0.0), XH_MAX)  # momentary jog: LEFT +y, RIGHT -y
    _drive_crosshead(env, _trial["xh"])
    _trial["hold_s"] = _trial["hold_s"] + dt if success(objects) else 0.0
    if _trial["hold_s"] > 0.0:
        _log_subaction_end(objects, _trial["step"], _trial["hold_s"] >= SUCCESS_HOLD_S - 0.5 * dt)


# -- scoring --------------------------------------------------------------------------------------------------------
def tab_ends(dog: PartState):
    """World positions of the tab ends nearer +y and nearer -y (on the dogbone's mid-plane). Ordered by world y, so a
    specimen mounted turned end-for-end (the part is symmetric) is scored exactly like one in its tray orientation."""
    a, b = dog.to_world((0.0, SPEC_HALF, 0.0)), dog.to_world((0.0, -SPEC_HALF, 0.0))
    return (a, b) if float(a[1]) >= float(b[1]) else (b, a)


def skew_deg(dog: PartState):
    """(yaw, pitch) of the dogbone's long axis against the loading axis (world y), degrees."""
    a = dog.rotation()[:, 1]
    yaw = math.degrees(math.atan2(abs(float(a[0])), abs(float(a[1]))))
    pitch = math.degrees(math.asin(min(1.0, abs(float(a[2])))))
    return yaw, pitch


def grip_faces(objects: dict):
    """World y of the +y (fixed) and -y (crosshead) jaw faces; a missing crosshead reads as backed off."""
    q = float(objects["test_stand"].joints.get("crosshead", 0.0))
    return STAND_XY[1] + GRIP_FACE, STAND_XY[1] - GRIP_FACE - XH_BACK + q


def _depth(end, side: int, face: float) -> float:
    """How far a tab end reaches past its jaw face into the grip, metres."""
    return side * (float(end[1]) - face)


def _seated(end, side: int, face: float) -> bool:
    """Tab inside the slot between the jaws, past the 17 mm ruler mark, not beyond the slot end, overhanging its jaw
    by at most SEAT_X_TOL."""
    x, _, z = (float(v) for v in end)
    return (DEPTH_MIN <= _depth(end, side, face) <= STOP_Y - GRIP_FACE and abs(z - AXIS_WORLD[2]) <= SEAT_Z_TOL
            and abs(x - STAND_XY[0]) <= SEAT_X_TOL)


def _off_slot_end(end, side: int, face: float) -> bool:
    """A visible gap of at least STOP_GAP between the tab end and its slot end."""
    return (STOP_Y - GRIP_FACE) - _depth(end, side, face) >= STOP_GAP


def _keep_clear(objects: dict):
    """Points both grippers must stay HAND_CLEAR_M from: tab ends, specimen centre, both lever handles, jog keys."""
    dog = objects["dogbone"]
    q = float(objects["test_stand"].joints.get("crosshead", 0.0))
    px, py, pz = PADDLE_WORLD[-1]
    return [*tab_ends(dog), dog.pos, PADDLE_WORLD[1], (px, py - XH_BACK + q, pz), *KEY_WORLD.values()]


def _hands_clear(tcps, objects: dict) -> bool:
    if tcps is None or len(tcps) < 2:
        return False
    keep = [np.asarray(p, float) for p in _keep_clear(objects)]
    return all(float(np.linalg.norm(np.asarray(t, float) - k)) >= HAND_CLEAR_M for t in tcps for k in keep)


def jaw_gap(objects: dict, s: str) -> float:
    """Upper-jaw gap of grip s ("l" = +y, "r" = -y), metres (scored through jaw_on_tab(): the clamped condition needs gap <= JAW_ON_TAB_MAX; also logged); a missing part reads as open."""
    jaw = objects.get(f"upper_jaw_{s}")
    return JAW_OPEN if jaw is None else float(jaw.joints.get(f"gap_{s}", JAW_OPEN))


def lever_forward(objects: dict, s: str) -> bool:
    """Lever of grip s let go and fully forward (its spring returned it); a missing joint reads as held open."""
    return float(objects["test_stand"].joints.get(f"lever_{s}", LEVER_OPEN)) <= LEVER_FWD_MAX


def jaw_on_tab(objects: dict, s: str) -> bool:
    """The upper jaw of grip s is closed on the tab (gap <= JAW_ON_TAB_MAX); a missing jaw reads as open."""
    return jaw_gap(objects, s) <= JAW_ON_TAB_MAX


def components(objects: dict) -> dict:
    """One additive table (README "Scoring"): rows 1-3 (+y grip) and 4-6 (-y grip) count independently of each other;
    rows 7-9 count once both tabs are clamped (seated, lever forward AND the jaw closed on the tab: gap <= JAW_ON_TAB_MAX)."""
    dog = objects["dogbone"]
    end_l, end_r = tab_ends(dog)
    face_l, face_r = grip_faces(objects)
    skew = max(skew_deg(dog))
    gauge = face_l - face_r
    clear = _hands_clear(_trial["tcps"], objects)
    seat_l, seat_r = _seated(end_l, +1, face_l), _seated(end_r, -1, face_r)
    clamp_l = seat_l and lever_forward(objects, "l") and jaw_on_tab(objects, "l")
    clamp_r = seat_r and lever_forward(objects, "r") and jaw_on_tab(objects, "r")
    both = clamp_l and clamp_r
    rows = (
        ("seat_a", SEAT_CREDIT, seat_l),
        ("lever_a", LEVER_CREDIT, clamp_l),
        ("gap_a", GAP_CREDIT, seat_l and _off_slot_end(end_l, +1, face_l)),
        ("seat_b", SEAT_CREDIT, seat_r),
        ("lever_b", LEVER_CREDIT, clamp_r),
        ("gap_b", GAP_CREDIT, seat_r and _off_slot_end(end_r, -1, face_r)),
        ("straight", STRAIGHT_CREDIT, both and skew <= SKEW_MAX_DEG),
        ("gauge", GAUGE_CREDIT, both and abs(gauge - GAUGE_L) <= GAUGE_TOL),
        ("clear", CLEAR_CREDIT, both and clear),
    )
    passed = [name for name, _, ok in rows if ok]
    sc = sum(credit for _, credit, ok in rows if ok)
    return {"score": round(sc, 6), "passed": passed, "skew_deg": skew, "gauge_m": gauge, "clear": clear}


def score(objects: dict) -> float:
    """End-state sum of the additive table: per grip +0.10 tab seated, +0.10 lever forward on it with the jaw closed on the tab (gap <= JAW_ON_TAB_MAX), +0.05 tab end >= 2 mm
    off its slot end; once both tabs are clamped +0.15 skew <= 2 deg, +0.20 jaw faces 115 +- 5 mm, +0.15 both
    grippers clear. The 3 s hold is not scored (success() only)."""
    return components(objects)["score"]


def success(objects: dict) -> bool:
    """Full end state (score 1.0); the runtime ends the episode as a success once it has held SUCCESS_HOLD_S."""
    return components(objects)["score"] >= 1.0 - 1e-9


def distance(objects: dict) -> float:
    dog = objects["dogbone"]
    face_l, face_r = grip_faces(objects)
    d = float(np.linalg.norm(np.asarray(dog.pos, float) - np.asarray(AXIS_WORLD)))
    d += sum(max(0.0, float(objects["test_stand"].joints.get(f"lever_{s}", LEVER_OPEN)) - LEVER_FWD_MAX)
             for s in ("l", "r"))
    d += abs(face_l - face_r - GAUGE_L)
    return d


# -- scripted demo (fixed station geometry only) --------------------------------------------------------------------
HOLD_FINGER = -0.020  # 40 mm opening around the 6 mm lever handle and the 13 mm neck
TIP_BELOW_PADDLE = 0.012  # pad tips this far below the lever handle top
TIP_BELOW_DOG = 0.0035  # pad tips this far below the dogbone underside
PUSH_PAST = {+1: 0.040, -1: 0.030}  # lever commanded this far past the open stop (arm compliance vs the spring)
CLOSE_PAST = 0.004  # and this far past the closed stop when easing it back
LIFT_BACK = 0.012  # lift off a lever slightly toward the robot
INSERT_HOVER = -0.0004  # planned tab height vs the slot floor; the neck grasp lifts the dogbone 1-2.5 mm (MuJoCo gate)
INSERT_RUN = HOUSING_D / 2.0 + 0.0095 + 0.004  # start with the 19 mm tab fully in front of the housing
LEVER_HOVER = 0.025  # pad tips this far above their lever grip height on approach/lift-off
KEY_PRESS = KEY_TRAVEL + 0.004  # pad tips commanded this far below the key top (the key bottoms out at KEY_TRAVEL)
KEY_HOVER = 0.030
JOG_HOLD = XH_BACK / JOG_SPEED  # s the jog key is held: 35 mm at 500 mm/min
JOG_SEGS = 7  # the lever arm follows the crosshead in 5 mm steps
GRIP_DY = -0.024  # right arm grips the neck on its -y half: pads on the 14.5 mm step, off the 16.5 mm step edge
DOG_Y_TRIM = -0.0035  # aim the dogbone centre 3.5 mm toward -y: +y start jitter and the jog's drag push it +y
Y_REG = 0.007  # register the dogbone: push its -y end until its centre sits this far +y of the tray centre
Y_REG_FACE = 0.0045  # closed finger pads: TCP to the pad face that pushes along +y (measured in MuJoCo)
INSERT_YAW_TRIM = -math.radians(1.2)  # right wrist yaw while carrying/inserting: the +y tab leads, so the slot's
# friction on it (about 0.1 m from the off-centre pinch) turns the dogbone back onto the axis instead of past it
CARRY_TEMPO = 0.7  # time scale on lifting, carrying and lowering the gripped dogbone (the pinch creeps over time)
TEMPO = 0.75  # time scale on the nominal segment durations below
PARK = {+1: (0.180, 0.270, TOP + 0.200), -1: (0.180, -0.270, TOP + 0.200)}
WAIT_R = (0.200, -0.190, TOP + 0.150)  # right arm stands by here while the left arm opens the +y grip


def demo() -> dict:
    """Right arm registers the dogbone on its tray (pushed along it to a mark by its -y end, then squared against the
    back stops) and picks it by the neck. Left arm holds the +y lever open while the right arm slides the +y tab
    sideways into the +y grip, then eases the lever back (the spring clamps the tab). Right arm holds the crosshead grip's lever open while the left arm
    holds the stand's LEFT jog key: the crosshead drives grip B over the -y tab to the gauge length; the right arm
    follows the moving lever, then eases it back (the spring clamps the tab). Both arms park."""
    def W(duration, pos, finger, label, yaw_):  # transit moves run at TEMPO; grip/release keep >= 0.6 s (rate limit)
        return _demo.Waypoint(max(0.6, round(duration * TEMPO, 2)), pos, finger, label, yaw_)

    yaw = math.pi / 2.0  # fingers close along x: across the lever handle, the dogbone neck and the key row
    yl, yr = -yaw, yaw  # the left arm turns the other way (same finger axis): +90 deg is its wrist limit
    yt = yr + INSERT_YAW_TRIM

    def grip_z(s):
        return PADDLE_WORLD[s][2] - TIP_BELOW_PADDLE + PAD_TIP_PAST_TCP

    def lever_open(s, label, y, yw, approach=1.2):
        x, z = PADDLE_WORLD[s][0], grip_z(s)
        return [
            W(approach, (x, y, z + LEVER_HOVER), HOLD_FINGER, f"over the {label} lever", yw),
            W(0.6, (x, y, z), HOLD_FINGER, f"straddle the {label} lever", yw),
            W(0.8, (x, y, z), FINGER_CLOSED, f"grip the {label} lever", yw),
            W(1.6, (x + LEVER_OPEN + PUSH_PAST[s], y, z), FINGER_CLOSED,
              f"push the {label} lever back against its spring: the wedge lifts the jaw", yw),
        ]

    def lever_back(s, label, y, yw):
        x, z = PADDLE_WORLD[s][0], grip_z(s)
        return [
            W(1.2, (x - CLOSE_PAST, y, z), FINGER_CLOSED, f"ease the {label} lever forward: its spring clamps the tab",
              yw),
            W(0.8, (x - CLOSE_PAST, y, z), HOLD_FINGER, f"let go of the {label} lever", yw),
            W(0.6, (x - LIFT_BACK, y, z + LEVER_HOVER), HOLD_FINGER, f"lift off the {label} lever", yw),
        ]

    ty = TRAY_XY[1]
    tx = TRAY_XY[0] + STOP_X - 0.0095  # dogbone centre once squared against the tray stops
    tx0 = TRAY_XY[0] - 0.0065 - 0.005 - 0.018  # pusher start: in front of the neck, clear of +-5 mm placement
    zp = TOP + REST_H + 0.0016 + PAD_TIP_PAST_TCP  # closed finger tips at the dogbone's mid-thickness
    pick = TOP + REST_H - TIP_BELOW_DOG + PAD_TIP_PAST_TCP
    slot = _BASE_Z + FLOOR_Z + INSERT_HOVER - TIP_BELOW_DOG + PAD_TIP_PAST_TCP
    ax, ay, _ = AXIS_WORLD
    x_in, x_out = ax, ax - INSERT_RUN
    gy = ay + GRIP_DY + DOG_Y_TRIM  # gripper y over the neck with the dogbone on the axis
    carry = slot + 0.060
    yb0 = PADDLE_WORLD[-1][1] - XH_BACK  # grip B's lever at the crosshead's start
    kx, ky, kz = KEY_WORLD["left"]
    press = kz - KEY_PRESS + PAD_TIP_PAST_TCP

    # step 1: left arm holds the +y lever open; right arm brings the dogbone and slides the +y tab in
    open_a = lever_open(+1, "+y grip", PADDLE_WORLD[1][1] + 0.008, yl, approach=1.6)
    t_a = sum(w.duration for w in open_a)
    ye = ty - SPEC_HALF + Y_REG - Y_REG_FACE  # pusher TCP with the -y end at its registered mark
    specimen = [
        W(1.0, (TRAY_XY[0], ye - 0.020, zp + 0.030), FINGER_CLOSED, "beside the dogbone's -y end", yr),
        W(0.7, (TRAY_XY[0], ye - 0.020, zp), FINGER_CLOSED, "fingers down beside the -y end", yr),
        W(1.2, (TRAY_XY[0], ye, zp), FINGER_CLOSED, "push it along the tray to its mark: registered along its length",
          yr),
        W(0.6, (TRAY_XY[0], ye - 0.010, zp + 0.030), FINGER_CLOSED, "back off the end", yr),
        W(1.0, (tx0, ty, zp + 0.030), FINGER_CLOSED, "in front of the dogbone neck", yr),
        W(0.7, (tx0, ty, zp), FINGER_CLOSED, "fingers down in front of the neck", yr),
        W(1.4, (tx - 0.0065 - 0.002, ty, zp), FINGER_CLOSED,
          "push it back against the tray stops: squared and registered", yr),
        W(0.6, (tx - 0.030, ty, zp + 0.030), HOLD_FINGER, "back off", yr),
        W(0.9, (tx, ty + Y_REG + GRIP_DY, pick + 0.050), HOLD_FINGER, "over the dogbone's -y fillet", yr),
        W(0.7, (tx, ty + Y_REG + GRIP_DY, pick), HOLD_FINGER, "straddle the fillet", yr),
        W(0.8, (tx, ty + Y_REG + GRIP_DY, pick), FINGER_CLOSED, "grip the fillet", yr),
        W(1.2 * CARRY_TEMPO, (tx, ty + Y_REG + GRIP_DY, carry), FINGER_CLOSED, "lift the dogbone", yr),
        W(1.5 * CARRY_TEMPO, (x_out, gy, carry), FINGER_CLOSED,
          "carry it in front of the stand, long axis on the loading axis", yt),
        W(1.0 * CARRY_TEMPO, (x_out, gy, slot + 0.010), FINGER_CLOSED, "lower toward the open slot height", yt),
    ]
    t_spec = sum(w.duration for w in specimen)
    right = [_demo.Waypoint(max(0.6, t_a - t_spec + 0.2), WAIT_R, HOLD_FINGER,
                            "stand by", yr)] + specimen + [
        W(1.0, (x_out, gy, slot), FINGER_CLOSED, "settle the +y tab at the open slot height", yt),
        W(1.6, (x_in, gy, slot), FINGER_CLOSED, "slide the +y tab sideways into the open +y grip", yt),
    ]
    t_in = sum(w.duration for w in right)
    xa, ya, za = open_a[-1].pos
    left = open_a + [_demo.Waypoint(t_in - t_a + 1.0, (xa - 0.003, ya, za), FINGER_CLOSED,
                                    "keep holding the +y lever open", yl)]
    left += [
        W(0.8, (xa - 0.003, ya, za), FINGER_OPEN,
          "open the fingers: the spring snaps the +y lever forward, the jaw clamps the tab", yl),
        W(0.6, (xa - LEVER_OPEN - 0.004, ya, za + LEVER_HOVER), FINGER_OPEN, "lift off the +y grip lever", yl),
    ]
    t_clamp_a = sum(w.duration for w in left[:-1])  # jaw A on the tab once the lever is back
    right += [
        _demo.Waypoint(t_clamp_a - t_in, (x_in, gy, slot - 0.001), FINGER_CLOSED,
                       "hold the dogbone down on the slot floor while the +y grip clamps", yt),
        W(1.6, (x_in, gy, slot), HOLD_FINGER, "let go: the dogbone hangs clamped in the +y grip", yt),
        W(0.6, (x_in, gy, carry), HOLD_FINGER, "lift clear", yr),
    ]
    # step 2: right arm holds grip B's lever open; left arm holds the LEFT jog key; the right arm follows the lever
    right += lever_open(-1, "crosshead grip", yb0, yr, approach=1.4)
    t_b = sum(w.duration for w in right)
    left += [W(1.0, (kx, ky - 0.040, TOP + 0.200), FINGER_CLOSED, "up over the force gauge", yl)]
    t_k = sum(w.duration for w in left)
    left += [
        _demo.Waypoint(round(max(0.75, t_b + 0.2 - t_k), 2), (kx, ky, press + KEY_HOVER), FINGER_CLOSED,
                       "over the stand's LEFT jog key", yl),
        _demo.Waypoint(0.6, (kx, ky, press), FINGER_CLOSED, "press and hold LEFT: the crosshead moves", yl),
    ]
    t_press = sum(w.duration for w in left)
    left += [
        _demo.Waypoint(round(JOG_HOLD - 0.3, 2), (kx, ky, press - 0.0005), FINGER_CLOSED, "keep LEFT pressed", yl),
        _demo.Waypoint(0.6, (kx, ky, press + KEY_HOVER), FINGER_CLOSED, "let go of LEFT: the crosshead stops", yl),
    ]
    xo = PADDLE_WORLD[-1][0] + LEVER_OPEN + PUSH_PAST[-1]
    zb = grip_z(-1)
    right.append(_demo.Waypoint(round(t_press - 0.3 - t_b, 2), (xo - 0.002, yb0, zb), FINGER_CLOSED,
                                "keep holding the crosshead grip's lever open", yr))
    for k in range(1, JOG_SEGS + 1):
        right.append(_demo.Waypoint(round(JOG_HOLD / JOG_SEGS, 2), (xo - 0.002, yb0 + XH_BACK * k / JOG_SEGS, zb),
                                    FINGER_CLOSED, "follow the lever as grip B moves over the -y tab", yr))
    right += lever_back(-1, "crosshead grip", yb0 + XH_BACK, yr)
    right += [W(1.4, PARK[-1], HOLD_FINGER, "park right clear", yr)]
    left += [W(1.4, PARK[+1], HOLD_FINGER, "park left clear", yl)]
    return {"left": left, "right": right}
