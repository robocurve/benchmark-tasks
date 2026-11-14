"""On the bench is an aluminium air manifold with three push-in ports on top. Two are already blanked with stem
plugs; the middle one has an 8 mm polyurethane air line pushed into its push-to-connect fitting. The line rises out
of the fitting, bends over and runs along the bench to a small pneumatic cylinder on the right, whose own fitting it
is pushed into. A parts tray on the left holds one spare 8 mm stem plug standing head up. The manifold is fed
through a lockable 3-way exhaust ball valve (open, 6 bar on the line); an open lockout padlock stands on the bench
behind the tray, and the manifold carries a pressure gauge and a dump (bleed) button.

The job is to take the line off the manifold (the cylinder is being swapped) and blank the open port with the
plug. A push-to-connect fitting grips the tube with a toothed collet: the tube only comes out while the collet ring
is held pressed down against the fitting body, so one hand holds the collet down while the other draws the tube out.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/air-line-push-fitting/air_line_push_fitting.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1800 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
ALU = (0.70, 0.72, 0.75)
BRASS = (0.78, 0.62, 0.30)
BLACK = (0.10, 0.10, 0.11)
COLLET_BLUE = (0.15, 0.35, 0.80)
TUBE_BLUE = (0.20, 0.50, 0.90)
PLUG_GREY = (0.30, 0.31, 0.33)

# Manifold block, ports along y at its top
MANI = (0.040, 0.120, 0.020)
MANI_XY = (0.250, 0.000)
MT = TOP + MANI[2]  # manifold top
PORT_Y = (0.040, 0.000, -0.040)
PORT = (MANI_XY[0], 0.000)  # the live port

# Push-to-connect fitting (real 8 mm fitting proportions)
TUBE_D = 0.008
BORE = 0.009  # bore and collet inner opening (real collet ID equals the tube OD; 0.5 mm play each side here)
BODY_W, BODY_H = 0.018, 0.014
FLOOR = MT + 0.002  # tube stop inside the body
COLLET_W, COLLET_H = 0.018, 0.004  # release ring with its flange
COLLET_TOP = MT + BODY_H + COLLET_H
COLLET_TRAVEL = 0.002
RELEASE_AT = 0.0015  # collet pressed this far: teeth off the tube
INSERT = 0.012  # tube depth in the fitting (body top to floor)


def _square_ring(outer, inner, h, z0, color=None):
    w = (outer - inner) / 2
    c = (outer + inner) / 4
    return [
        mk.box((outer, w, h), at=(0, c, z0 + h / 2), color=color),
        mk.box((outer, w, h), at=(0, -c, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(c, 0, z0 + h / 2), color=color),
        mk.box((w, inner, h), at=(-c, 0, z0 + h / 2), color=color),
    ]


kit = mk.Kit()
# fitting body (fixed): square ring around the bore plus the floor/stop at the bottom
kit.part("fitting", (PORT[0], PORT[1], MT),
         [*_square_ring(BODY_W, BORE, BODY_H - 0.002, 0.002), mk.box((BODY_W, BODY_W, 0.002), at=(0, 0, 0.001))],
         mass=None, color=BRASS)
# collet ring: slides down 2 mm against a spring
kit.part("collet", (PORT[0], PORT[1], MT + BODY_H), _square_ring(COLLET_W, BORE, COLLET_H, 0.0), mass=0.030,
         color=COLLET_BLUE)
kit.slider("collet_travel", "fitting", "collet", axis=(0, 0, -1), lower=0.0, upper=COLLET_TRAVEL, stiffness=600.0,
           rest=-0.001, damping=5.0)

# Air line: a chain of rigid 8 mm segments on spring hinges (the PU tube's bending stiffness)
RISE = 0.050  # tube end segment length (12 mm in the fitting, 38 mm above it)
SEG = 0.040
N_RUN = 4  # horizontal run away from the robot (+x) to the cylinder
ELBOW_Z = FLOOR + RISE  # the bend
TUBE_KG = 0.004  # per segment (8x5.5 mm PU, about 25 g/m)
kit.part("tube_end", (PORT[0], PORT[1], FLOOR), [mk.box((TUBE_D, TUBE_D, RISE), at=(0, 0, RISE / 2))],
         mass=TUBE_KG * 1.5, color=TUBE_BLUE)
kit.bond("collet_grip", "fitting", "tube_end", anchor=(PORT[0], PORT[1], FLOOR + INSERT / 2), slip_mm=1.5)
_prev = "tube_end"
for _i in range(N_RUN):
    _name = f"tube_{_i}"
    x0 = PORT[0] + _i * SEG
    kit.part(_name, (x0, PORT[1], ELBOW_Z), [mk.box((SEG, TUBE_D, TUBE_D), at=(SEG / 2, 0, -TUBE_D / 2))],
             mass=TUBE_KG, color=TUBE_BLUE)
    _axis = (0, 1, 0) if _i % 2 == 0 else (0, 0, 1)  # alternate: bend up/down, bend sideways
    kit.hinge(f"bend_{_i}", _prev, _name, anchor=(x0, PORT[1], ELBOW_Z - TUBE_D / 2), axis=_axis,
              lower_deg=-90, upper_deg=90, stiffness=0.15, damping=0.002)
    _prev = _name
TUBE_PARTS = ["tube_end", *[f"tube_{i}" for i in range(N_RUN)]]
CYL_PORT_X = PORT[0] + N_RUN * SEG  # where the run ends (in the cylinder's fitting)

# Lockable 3-way exhaust ball valve on the manifold's supply: lever in line with the pipe = open (6 bar on the
# line), turned a quarter turn across it = closed, supply side shut and its exhaust port open. Lockout: the lever's
# end has an 8 mm hole that, with the valve closed, lines up over the hole of a lock plate fixed to the valve body;
# a padlock's shackle goes down through both holes and is pushed shut into the padlock's body.
VALVE_XY = (0.250, 0.095)  # on the supply line, at the manifold's left end
VALVE = 0.025
VALVE_TOP = TOP + VALVE
LEVER_L = 0.057
LOCK_R = 0.050  # lever hole centre from the valve stem
CLOSED_DEG = 80.0
LOCK_DEG = 85.0
LOCK_HOLE = (VALVE_XY[0] + LOCK_R, VALVE_XY[1])  # where the lever's hole sits with the valve closed
PLATE_BOT = VALVE_TOP  # lock plate: 3 mm brass tab level with the valve body's top
PLATE_TOP = VALVE_TOP + 0.003
LEVER_TOP = VALVE_TOP + 0.008  # the lever's 4 mm thick end ring sits 1 mm over the plate
RED = (0.85, 0.12, 0.10)


def _ring_at(cx, cy, outer, inner, h, z0, color=None):
    return [mk.box(b.size, at=(b.at[0] + cx, b.at[1] + cy, b.at[2]), color=color)
            for b in _square_ring(outer, inner, h, z0)]


kit.part("valve_body", (VALVE_XY[0], VALVE_XY[1], TOP), [mk.box((VALVE, VALVE, VALVE), at=(0, 0, VALVE / 2))],
         mass=None, color=BRASS)
kit.part("valve_lever", (VALVE_XY[0], VALVE_XY[1], VALVE_TOP),
         [mk.box((0.006, 0.006, 0.006), at=(0, 0, 0.003)),
          mk.box((0.012, LOCK_R - 0.007, 0.004), at=(0, (LOCK_R - 0.007) / 2, 0.006), color=RED),
          *_ring_at(0, LOCK_R, 0.014, 0.008, 0.004, 0.004, color=RED)],
         mass=0.030, color=(0.6, 0.6, 0.62))
kit.hinge("valve_turn", "valve_body", "valve_lever", anchor=(VALVE_XY[0], VALVE_XY[1], VALVE_TOP), axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=90.0, friction=0.08)

# Lockout padlock: red body 26 x 16 x 16 mm (x, y, z) and a 5.5 mm square U shackle, its legs 16 mm apart along x.
# The shackle slides up and down in the body on a slider (padlock to shackle, 24 mm travel, starts fully open, held
# where it is left by 0.5 N of friction). Open, the free leg's end stands 18 mm above the body; pushed down to the
# closed stop, both legs are 6 mm deep in the body and the latch inside snaps shut: a weld between shackle and body,
# authored disarmed with the scene and enabled at that moment (no joint is created during a run). It stands on the
# bench, shackle open. Body frame: bottom centre. Shackle frame: the free leg's end.
LOCKBODY = (0.026, 0.016, 0.016)
LEG_W = 0.0055
LEG_DX = 0.016  # long leg this far +x of the free leg
LEG_IN = 0.006  # leg depth in the body when closed
SHACKLE_IN = 0.020  # clear height under the bridge when closed
LEG_L = LEG_IN + SHACKLE_IN
SHACKLE_OPEN = 0.024
STAND_XY = (0.200, 0.205)  # where the padlock stands (body centre)
_FREE_DX = -LEG_DX / 2  # free leg from the body centre
kit.part("padlock", (STAND_XY[0], STAND_XY[1], TOP), [mk.box(LOCKBODY, at=(0, 0, LOCKBODY[2] / 2))],
         mass=0.040, color=RED)
_STEEL = (0.75, 0.75, 0.78)
kit.part("shackle", (STAND_XY[0] + _FREE_DX, STAND_XY[1], TOP + LOCKBODY[2] - LEG_IN),
         [mk.box((0.004, 0.004, 0.002), at=(0, 0, 0.001)),  # chamfered end of the free leg
          mk.box((LEG_W, LEG_W, LEG_L - 0.002), at=(0, 0, 0.002 + (LEG_L - 0.002) / 2)),
          mk.box((LEG_W, LEG_W, LEG_L), at=(LEG_DX, 0, LEG_L / 2)),
          mk.box((LEG_DX + LEG_W, LEG_W, LEG_W), at=(LEG_DX / 2, 0, LEG_L + LEG_W / 2))],
         mass=0.015, color=_STEEL)
kit.slider("shackle_travel", "padlock", "shackle", axis=(0, 0, 1), lower=0.0, upper=SHACKLE_OPEN,
           start=SHACKLE_OPEN, friction=0.5)
SHUT_AT = 0.001  # shackle within 1 mm of its closed stop: the latch catches

# The latch inside the padlock (shackle welded to the body at the closed stop) and, once that shackle is shut through
# both holes, the lever held at 90 deg (the shackle's leg in the lever's and plate's holes).
_REF = kit._fk(q={"shackle_travel": 0.0})  # the shackle on its closed stop
kit.catch("shackle_latch", "shackle", _REF["shackle"], "padlock", _REF["padlock"])
_LEVER_LOCKED = kit._fk(q={"valve_turn": math.radians(90.0)})["valve_lever"]
kit.catch("lever_lock", "valve_lever", _LEVER_LOCKED)

# Dump (bleed) button on the manifold's line side: pressed 3 mm, it vents the line to atmosphere.
BLEED_XY = (0.200, -0.085)
BLEED_HOUSE = (0.030, 0.021, 0.020)  # outer, inner, height
BLEED_AT = 0.003
kit.part("bleed_housing", (BLEED_XY[0], BLEED_XY[1], TOP),
         [*_square_ring(BLEED_HOUSE[0], BLEED_HOUSE[1], BLEED_HOUSE[2] - 0.002, 0.002),
          mk.box((BLEED_HOUSE[0], BLEED_HOUSE[0], 0.002), at=(0, 0, 0.001))], mass=None, color=BRASS)
BLEED_CAP_Z = TOP + 0.014
kit.part("bleed_button", (BLEED_XY[0], BLEED_XY[1], BLEED_CAP_Z), [mk.box((0.020, 0.020, 0.012), at=(0, 0, 0.006))],
         mass=0.020, color=RED)
kit.slider("bleed_travel", "bleed_housing", "bleed_button", axis=(0, 0, -1), lower=0.0, upper=0.005,
           stiffness=300.0, rest=-0.001, damping=2.0)

# Line pressure gauge (0-10 bar over 270 deg); its needle is a sprung hinge whose rest point follows the pressure.
GAUGE_XY = (0.170, -0.045)
GAUGE_Z = TOP + 0.032
P_LINE = 6.0  # bar
DEG_PER_BAR = 27.0
kit.part("gauge_dial", (GAUGE_XY[0], GAUGE_XY[1], TOP),
         [mk.box((0.010, 0.010, GAUGE_Z - TOP - 0.004), at=(0, 0, (GAUGE_Z - TOP - 0.004) / 2), color=BRASS),
          mk.box((0.036, 0.036, 0.004), at=(0, 0, GAUGE_Z - TOP - 0.002), color=(0.95, 0.95, 0.93)),
          mk.box((0.004, 0.003, 0.0012), at=(-0.016, 0, GAUGE_Z - TOP + 0.0006), color=BLACK)],  # the 0 bar mark
         mass=None, color=BRASS)
kit.part("gauge_needle", (GAUGE_XY[0], GAUGE_XY[1], GAUGE_Z),
         [mk.box((0.015, 0.003, 0.0015), at=(-0.0065, 0, 0.0015)), mk.box((0.004, 0.004, 0.002), at=(0, 0, 0.001))],
         mass=0.020, color=BLACK)
kit.hinge("gauge", "gauge_dial", "gauge_needle", anchor=(GAUGE_XY[0], GAUGE_XY[1], GAUGE_Z), axis=(0, 0, -1),
          lower_deg=0.0, upper_deg=270.0, start_deg=P_LINE * DEG_PER_BAR, stiffness=0.02,
          rest_deg=P_LINE * DEG_PER_BAR, damping=0.0005)

# Spare stem plug: 8 mm stem, 14 mm head
STEM_L, HEAD_W, HEAD_H = 0.016, 0.014, 0.008
TRAY_XY = (0.200, 0.130)
TRAY = (0.060, 0.060, 0.012)
TRAY_TOP = TOP + TRAY[2]
kit.part("plug", (TRAY_XY[0], TRAY_XY[1], TRAY_TOP - 0.010),
         [mk.box((0.0065, 0.0065, 0.002), at=(0, 0, 0.001)),  # lead-in chamfer at the stem's end
          mk.box((TUBE_D, TUBE_D, STEM_L - 0.002), at=(0, 0, 0.002 + (STEM_L - 0.002) / 2)),
          mk.box((HEAD_W, HEAD_W, HEAD_H), at=(0, 0, STEM_L + HEAD_H / 2))],
         mass=0.004, color=PLUG_GREY)


def _blanked_port(y):
    """A port already blanked: fitting body, collet and a plug head (fixed, scenery)."""
    x = PORT[0]
    return [
        Box(f"fit_{'L' if y > 0 else 'R'}", pos=(x, y, MT + BODY_H / 2), size=(BODY_W, BODY_W, BODY_H), color=BRASS),
        Box(f"col_{'L' if y > 0 else 'R'}", pos=(x, y, MT + BODY_H + COLLET_H / 2),
            size=(COLLET_W, COLLET_W, COLLET_H), color=COLLET_BLUE),
        Box(f"cap_{'L' if y > 0 else 'R'}", pos=(x, y, COLLET_TOP + HEAD_H / 2), size=(HEAD_W, HEAD_W, HEAD_H),
            color=PLUG_GREY),
    ]


def _tray():
    """Parts tray with a 9 mm hole holding the plug's stem (four blocks around the hole)."""
    (x, y), (lx, ly, h) = TRAY_XY, TRAY
    g = BORE / 2
    side = (lx / 2 - g)
    return [
        Box("tray_a", pos=(x + g + side / 2, y, TOP + h / 2), size=(side, ly, h), color=BLACK),
        Box("tray_b", pos=(x - g - side / 2, y, TOP + h / 2), size=(side, ly, h), color=BLACK),
        Box("tray_c", pos=(x, y + g + (ly / 2 - g) / 2, TOP + h / 2), size=(2 * g, ly / 2 - g, h), color=BLACK),
        Box("tray_d", pos=(x, y - g - (ly / 2 - g) / 2, TOP + h / 2), size=(2 * g, ly / 2 - g, h), color=BLACK),
        Box("tray_floor", pos=(x, y, TOP + 0.001), size=(2 * g, 2 * g, 0.002), color=BLACK),
    ]


def _lock_plate():
    """Lock plate fixed to the valve body: an arm out to a 9 mm hole under the closed lever's hole."""
    x, y = LOCK_HOLE
    o, g, t = 0.016, BORE / 2, PLATE_TOP - PLATE_BOT
    z = PLATE_TOP - t / 2
    side = o / 2 - g
    arm0 = VALVE_XY[0] + VALVE / 2 - 0.004  # the tab's root lies on the body's top edge
    return [
        Box("plate_arm", pos=((arm0 + x - o / 2) / 2, y, z), size=(x - o / 2 - arm0, 0.010, t), color=BRASS),
        Box("plate_a", pos=(x + g + side / 2, y, z), size=(side, o, t), color=BRASS),
        Box("plate_b", pos=(x - g - side / 2, y, z), size=(side, o, t), color=BRASS),
        Box("plate_c", pos=(x, y + g + side / 2, z), size=(2 * g, side, t), color=BRASS),
        Box("plate_d", pos=(x, y - g - side / 2, z), size=(2 * g, side, t), color=BRASS),
    ]


FIXTURES = [
    Box("manifold", pos=(MANI_XY[0], MANI_XY[1], TOP + MANI[2] / 2), size=MANI, color=ALU),
    Box("supply_nipple", pos=(VALVE_XY[0], (MANI[1] / 2 + VALVE_XY[1] - VALVE / 2) / 2, TOP + 0.0125),
        size=(0.014, VALVE_XY[1] - VALVE / 2 - MANI[1] / 2, 0.014), color=BRASS),
    Box("supply_pipe", pos=(VALVE_XY[0], VALVE_XY[1] + VALVE / 2 + 0.040, TOP + 0.007), size=(0.014, 0.080, 0.014),
        color=(0.25, 0.25, 0.27)),
    *_blanked_port(PORT_Y[0]),
    *_blanked_port(PORT_Y[2]),
    # cylinder on its mounting foot, its port facing the manifold (-x) at the tube run's height
    Box("cyl_barrel", pos=(CYL_PORT_X + 0.016 + 0.050, PORT[1], ELBOW_Z - TUBE_D / 2), size=(0.100, 0.026, 0.026), color=ALU),
    Box("cyl_foot", pos=(CYL_PORT_X + 0.060, PORT[1], (TOP + ELBOW_Z - TUBE_D / 2 - 0.013) / 2),
        size=(0.030, 0.040, ELBOW_Z - TUBE_D / 2 - 0.013 - TOP), color=(0.35, 0.36, 0.38)),
    Box("cyl_fit_top", pos=(CYL_PORT_X + 0.008, PORT[1], ELBOW_Z - TUBE_D / 2 + 0.006), size=(0.016, 0.014, 0.003), color=BRASS),
    Box("cyl_fit_bot", pos=(CYL_PORT_X + 0.008, PORT[1], ELBOW_Z - TUBE_D / 2 - 0.006), size=(0.016, 0.014, 0.003), color=BRASS),
    Box("cyl_fit_l", pos=(CYL_PORT_X + 0.008, PORT[1] + 0.006, ELBOW_Z - TUBE_D / 2), size=(0.016, 0.003, 0.009), color=BRASS),
    Box("cyl_fit_r", pos=(CYL_PORT_X + 0.008, PORT[1] - 0.006, ELBOW_Z - TUBE_D / 2), size=(0.016, 0.003, 0.009), color=BRASS),
    *_tray(),
    *_lock_plate(),
]
OBJECTS = [*kit.objects(), *FIXTURES]


DT = 0.05  # one control step at 20 Hz
BLEED_TAU = 0.4  # s: the dump valve empties the line
P_ZERO = 0.1  # bar: gauge reads zero


def _air_rule(objects):
    """Line pressure. The valve open: supply pressure on the line. Closed: the supply is shut and exhausted, but the
    manifold's inlet check valve keeps the line's air in until the dump button is held down. The gauge needle
    follows the line pressure."""
    p = mk.MEMORY.get("pressure", P_LINE)
    if kit.q_deg(objects, "valve_turn") < CLOSED_DEG:
        p = P_LINE
    elif kit.q(objects, "bleed_travel") >= BLEED_AT:
        p = p * math.exp(-DT / BLEED_TAU)
        p = 0.0 if p < 0.01 else p
    mk.MEMORY["pressure"] = p
    kit.drive("gauge", p * DEG_PER_BAR)


def shackle_through(objects) -> bool:
    """The padlock's free leg runs down through the lever's hole and the lock plate's hole: on the hole axis within
    3 mm at the lever ring and at the plate, its end below the plate."""
    for z_leg in (0.0, LEG_L):
        p = kit.point(objects, "shackle", (0, 0, z_leg))
        if math.dist((float(p[0]), float(p[1])), LOCK_HOLE) > 0.003:
            return False
    lo = float(kit.point(objects, "shackle", (0, 0, 0))[2])
    hi = float(kit.point(objects, "shackle", (0, 0, LEG_L))[2])
    return lo < PLATE_BOT - 0.002 and hi > LEVER_TOP


def shackle_shut(objects) -> bool:
    return kit.q(objects, "shackle_travel") <= SHUT_AT


def _lock_rule(objects):
    """Pushed onto its closed stop, the shackle latches shut (wherever it is). It counts as the lockout only if it
    was shut through both holes with the valve closed; then the lever cannot be turned back."""
    if mk.MEMORY.get("shut") or not shackle_shut(objects):
        return
    kit.engage("shackle_latch")
    mk.MEMORY["shut"] = True
    if kit.q_deg(objects, "valve_turn") >= LOCK_DEG and shackle_through(objects):
        kit.engage("lever_lock")
        mk.MEMORY["locked"] = True


def _release_rule(objects):
    """The collet teeth let go of the tube only while the collet is pressed down. Released with air on the line, the
    tube whips out (a live disconnect). Opening the supply again once the line is off is also latched."""
    if kit.q(objects, "collet_travel") >= RELEASE_AT and "collet_grip" not in kit._released:
        kit.release("collet_grip")
        mk.MEMORY["released_by_collet"] = True
        mk.MEMORY["locked_at_release"] = bool(mk.MEMORY.get("locked"))
        if not mk.MEMORY.get("locked"):
            mk.MEMORY["unlocked_release"] = True
        if mk.MEMORY.get("pressure", P_LINE) > P_ZERO:
            mk.MEMORY["live_disconnect"] = True
    if mk.MEMORY.get("released_by_collet") and kit.q_deg(objects, "valve_turn") < CLOSED_DEG:
        mk.MEMORY["reopened"] = True


kit.on_step(_air_rule)
kit.on_step(_lock_rule)
kit.on_step(_release_rule)


SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _tilt(st) -> float:
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def tube_out(objects) -> bool:
    """The tube end is out of the fitting (its tip 3 mm or more above the collet top, or off the port's axis) and
    nothing holds the line."""
    tip = kit.point(objects, "tube_end", (0, 0, 0))
    off_axis = math.dist((float(tip[0]), float(tip[1])), PORT) > BORE
    clear = float(tip[2]) >= COLLET_TOP + 0.003 or off_axis
    return clear and all(_released(objects[p]) for p in TUBE_PARTS)


def plug_seated(objects) -> bool:
    """Plug stem down in the bore (stem end within 3 mm of the floor), upright within 10 deg, let go and still."""
    st = objects["plug"]
    tip = kit.point(objects, "plug", (0, 0, 0))
    on_axis = math.dist((float(tip[0]), float(tip[1])), PORT) <= 0.002
    return (on_axis and float(tip[2]) <= FLOOR + 0.003 and _tilt(st) <= 10.0 and st.resting() and _released(st))


def valve_closed(objects) -> bool:
    return kit.q_deg(objects, "valve_turn") >= CLOSED_DEG


def failed(objects) -> bool:
    """Latched for the episode: the line was released before the padlock was shut through both holes, or with air on
    it, or the supply was opened after the release."""
    return bool(mk.MEMORY.get("live_disconnect") or mk.MEMORY.get("reopened") or mk.MEMORY.get("unlocked_release"))


def score(objects) -> float:
    """0 if the line was released (collet pressed 1.5 mm) before the padlock was shut through the lever's and the
    plate's holes (a shackle hung through the holes but not pushed shut does not count), or with pressure on it
    (gauge above 0.1 bar), or the valve was opened at any time after the release, or the valve is not closed at the
    end. Otherwise: line off and port blanked 1.0; line off, port open 0.5; line still on, valve padlocked 0.2;
    valve only closed 0.1."""
    if failed(objects) or not valve_closed(objects):
        return 0.0
    if not tube_out(objects):
        return 0.2 if mk.MEMORY.get("locked") else 0.1
    return 1.0 if plug_seated(objects) else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    tip = kit.point(objects, "tube_end", (0, 0, 0))
    plug = kit.point(objects, "plug", (0, 0, 0))
    d = max(0.0, COLLET_TOP + 0.010 - float(tip[2]))
    return float(d + math.dist([float(v) for v in plug], [PORT[0], PORT[1], FLOOR]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm: sweeps the valve lever closed, carries the open padlock in by its shackle and pushes the shackle shut
# through the lever's and the lock plate's holes, then comes down with its fingers 6.5 mm either side of the tube (the fingers
# close along y; each pad is 15 mm wide in x) onto the collet ring and holds it pressed. Right arm: once the valve is
# shut, holds the dump button down until the gauge reads zero; then grips the tube's run 100 mm from the bend, lifts
# it (the tube end slides up out of the fitting), swings it aside and lays it on the bench. Left arm lets the collet
# up, takes the plug from the tray, starts its stem in the bore, lets go and pushes it home by the head.

TIP = 0.013
TRAVEL = TOP + 0.130
STRADDLE = -0.0065  # each finger 6.5 mm off the axis: inside faces clear the 8 mm tube, pads on the collet ring
PRESS_Z = COLLET_TOP - 0.006 + TIP  # pushes the collet onto its 2 mm stop
GRIP_RUN_X = PORT[0] + 0.100  # on the run, 100 mm from the bend
PRESS_X = PORT[0] - 0.008  # pads (15 mm wide in x) over the ring's near half, the hand kept back from the other arm
RUN_Z = ELBOW_Z - TUBE_D / 2
PARK_R = (0.20, -0.20, TRAVEL)


LEVER_Z = VALVE_TOP + 0.0055 + TIP  # finger tips 2.5 mm below the lever's top face, 2.5 mm over the lock plate


def _valve_sweep(start=-5, stop=115, step=15, side=1.0):
    """Closed fingers come down beside the lever, 45 mm out, and sweep it: side +1 pushes from the near (-x) side,
    from along the pipe (+y, open) to across it (+x, closed); side -1 pushes it back open."""
    (vx, vy), r, off = VALVE_XY, 0.045, 0.014 * side

    def at(deg):
        a = math.radians(deg)
        return (vx + r * math.sin(a) - off * math.cos(a), vy + r * math.cos(a) + off * math.sin(a))

    plan = [Waypoint(2.0, (*at(start), TRAVEL), FINGER_CLOSED, "over the valve lever", 0.0),
            Waypoint(1.5, (*at(start), LEVER_Z), FINGER_CLOSED, "down beside the lever", 0.0)]
    plan += [Waypoint(1.2, (*at(d), LEVER_Z), FINGER_CLOSED, f"sweep the lever to {d} deg", 0.0)
             for d in range(start + step, stop + (1 if step > 0 else -1), step)]
    plan.append(Waypoint(1.0, (*at(stop), LEVER_Z), FINGER_CLOSED, "hold it", 0.0))
    return plan + [Waypoint(1.0, (*at(stop - 10 * side), TRAVEL), FINGER_CLOSED, "up off the lever", 0.0)]


def _padlock(shut=True):
    """Pick the open padlock up by its shackle's bridge (the body hangs from it), carry it in from the +y side with
    the stacked lever and plate between the free leg's end and the body, so the free leg comes over the holes, lower
    the body onto the bench and push the shackle down through both holes until it latches. shut=False stops with the
    leg through the holes, 8 mm short of the stop (a hung, unlatched padlock)."""
    (sx, sy), (hx, hy) = STAND_XY, LOCK_HOLE
    bx = hx - _FREE_DX  # body centre with the free leg on the hole
    lift = 0.003  # body bottom over the bench while it is carried in under the plate
    grip = LOCKBODY[2] + SHACKLE_IN + 0.001 + TIP  # finger tips 1 mm above the bridge's underside (closed)
    o = FINGER_OPEN * 0.5
    end = 0.0 if shut else 0.008
    top = TOP + LOCKBODY[2] + SHACKLE_IN + LEG_W  # the bridge's top face, shut
    return [
        Waypoint(2.0, (sx, sy, TRAVEL), o, "over the padlock", 0.0),
        Waypoint(1.5, (sx, sy, TOP + grip + SHACKLE_OPEN), o, "down round the shackle's bridge", 0.0),
        Waypoint(0.5, (sx, sy, TOP + grip + SHACKLE_OPEN), o, "settle", 0.0),
        Waypoint(1.0, (sx, sy, TOP + grip + SHACKLE_OPEN), FINGER_CLOSED, "grip the bridge", 0.0),
        Waypoint(1.5, (sx, sy, TRAVEL), FINGER_CLOSED, "lift it", 0.0),
        Waypoint(1.5, (bx, hy + 0.040, TRAVEL), FINGER_CLOSED, "beside the lock plate", 0.0),
        Waypoint(1.2, (bx, hy + 0.040, TOP + lift + grip + SHACKLE_OPEN), FINGER_CLOSED, "down, body under the plate",
                 0.0),
        Waypoint(2.0, (bx, hy, TOP + lift + grip + SHACKLE_OPEN), FINGER_CLOSED, "free leg in over the holes", 0.0),
        Waypoint(0.4, (bx, hy, TOP + lift + grip + SHACKLE_OPEN), FINGER_CLOSED, "settle", 0.0),
        Waypoint(1.0, (bx, hy, TOP + grip + SHACKLE_OPEN), FINGER_CLOSED, "body down on the bench", 0.0),
        Waypoint(2.0, (bx, hy, TOP + grip + end - (0.001 if shut else 0.0)), FINGER_CLOSED,
                 "push the shackle down through lever and plate" + (" until it latches" if shut else ""), 0.0),
        Waypoint(0.5, (bx, hy, TOP + grip + end - (0.001 if shut else 0.0)), FINGER_CLOSED, "hold", 0.0),
        Waypoint(0.8, (bx, hy, TOP + grip + end), o, "let go", 0.0),
    ] + ([
        Waypoint(0.6, (bx, hy, top + TIP + 0.012), o, "up over the bridge", 0.0),
        Waypoint(0.6, (bx, hy, top + TIP + 0.012), FINGER_CLOSED, "close the fingers", 0.0),
        Waypoint(1.0, (bx, hy, top + TIP - 0.004), FINGER_CLOSED, "press the bridge down onto the stop", 0.0),
        Waypoint(0.4, (bx, hy, top + TIP - 0.004), FINGER_CLOSED, "hold", 0.0),
    ] if shut else []) + [
        Waypoint(1.0, (bx, hy, TRAVEL), o, "up", 0.0),
    ]


def _collet_hold():
    px, py = PORT
    return [
        Waypoint(2.0, (px, py, TRAVEL), STRADDLE, "over the fitting", 0.0),
        Waypoint(1.0, (PRESS_X, py, TRAVEL), STRADDLE, "fingers either side of the tube", 0.0),
        Waypoint(1.5, (PRESS_X, py, COLLET_TOP + TIP + 0.010), STRADDLE, "down beside the tube", 0.0),
        Waypoint(1.0, (PRESS_X, py, PRESS_Z), STRADDLE, "press the collet ring down", 0.0),
        Waypoint(12.0, (PRESS_X, py, PRESS_Z), STRADDLE, "hold it down while the tube comes out", 0.0),
        Waypoint(1.0, (px, py, TRAVEL), STRADDLE, "let the collet up", 0.0),
    ]


def _bleed():
    bx, by = BLEED_XY
    top = BLEED_CAP_Z + 0.012
    return [
        Waypoint(2.0, (bx, by, TRAVEL), FINGER_CLOSED, "over the dump button", 0.0),
        Waypoint(1.5, (bx, by, top + TIP + 0.005), FINGER_CLOSED, "down to it", 0.0),
        Waypoint(1.0, (bx, by, top - 0.0065 + TIP), FINGER_CLOSED, "press it", 0.0),
        Waypoint(2.5, (bx, by, top - 0.0065 + TIP), FINGER_CLOSED, "hold it until the gauge reads zero", 0.0),
        Waypoint(1.0, (bx, by, TRAVEL), FINGER_CLOSED, "let it up", 0.0),
        Waypoint(1.5, PARK_R, FINGER_OPEN * 0.5, "clear", 0.0),
    ]


def _left(lock=True):
    px, py = PORT
    tx, ty = TRAY_XY
    grip_z = TRAY_TOP - 0.010 + STEM_L - 0.003 + TIP  # pad tips 3 mm below the head: mostly on the head
    start_z = COLLET_TOP - 0.009 + STEM_L - 0.003 + TIP  # stem 9 mm into the collet
    head_top = FLOOR + STEM_L + HEAD_H  # the head's top face with the plug home
    ix, iy = px, py
    return _valve_sweep() + (_padlock() if lock else []) + _collet_hold() + [
        Waypoint(2.0, (tx, ty, TRAVEL), FINGER_OPEN * 0.4, "over the plug", 0.0),
        Waypoint(1.5, (tx, ty, grip_z), FINGER_OPEN * 0.4, "down round the plug", 0.0),
        Waypoint(1.0, (tx, ty, grip_z), FINGER_OPEN * 0.4, "settle", 0.0),
        Waypoint(1.0, (tx, ty, grip_z), FINGER_CLOSED, "grip it", 0.0),
        Waypoint(1.0, (tx, ty, TRAVEL), FINGER_CLOSED, "lift", 0.0),
        Waypoint(2.0, (px, py, TRAVEL), FINGER_CLOSED, "over the port", 0.0),
        Waypoint(1.5, (ix, iy, start_z + 0.010), FINGER_CLOSED, "stem over the bore", 0.0),
        Waypoint(1.0, (ix, iy, start_z + 0.010), FINGER_CLOSED, "settle", 0.0),
        Waypoint(1.0, (ix, iy, start_z + 0.007), FINGER_CLOSED, "chamfer into the collet", 0.0),
        *[Waypoint(0.3, (ix + r * math.cos(k * math.pi / 4), iy + r * math.sin(k * math.pi / 4), start_z + 0.005),
                   FINGER_CLOSED, "feel round for the bore", 0.0)
          for r in (0.0015, 0.003) for k in range(8)],
        Waypoint(0.6, (ix, iy, start_z), FINGER_CLOSED, "stem into the collet", 0.0),
        Waypoint(0.8, (ix, iy, start_z), FINGER_OPEN * 0.4, "let go", 0.0),
        Waypoint(0.8, (px, py, head_top + TIP + 0.015), FINGER_OPEN * 0.4, "up", 0.0),
        Waypoint(0.8, (px, py, head_top + TIP + 0.015), FINGER_CLOSED, "close the fingers", 0.0),
        Waypoint(1.2, (px, py, head_top + TIP - 0.003), FINGER_CLOSED, "push the plug home", 0.0),
        Waypoint(0.8, (px, py, head_top + TIP - 0.003), FINGER_CLOSED, "hold", 0.0),
        Waypoint(1.0, (px, py, TRAVEL), FINGER_CLOSED, "straight up off the plug", 0.0),
        Waypoint(1.5, (px, py + 0.10, TRAVEL), FINGER_CLOSED, "clear", 0.0),
    ]


def _dur(plan):
    return sum(w.duration for w in plan)


def _right(lock=True):
    shut = _dur(_valve_sweep())
    held = shut + (_dur(_padlock()) if lock else 0.0) + _dur(_collet_hold()[:4]) + 0.5  # the collet held down
    return [
        Waypoint(shut, PARK_R, FINGER_OPEN * 0.5, "wait for the valve to be shut", 0.0),
        *_bleed(),
        Waypoint(max(0.5, held - shut - _dur(_bleed())), PARK_R, FINGER_OPEN * 0.5, "wait for the collet to be held",
                 0.0),
        Waypoint(2.0, (GRIP_RUN_X, -0.120, RUN_Z + TIP + 0.035), FINGER_OPEN * 0.5, "out to the far side of the run", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, 0.0, RUN_Z + TIP + 0.035), FINGER_OPEN * 0.5, "over the tube run", 0.0),
        Waypoint(1.5, (GRIP_RUN_X, 0.0, RUN_Z + TIP - 0.010), FINGER_OPEN * 0.5, "down round the run", 0.0),
        Waypoint(1.0, (GRIP_RUN_X, 0.0, RUN_Z + TIP - 0.010), FINGER_CLOSED, "grip the tube", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, 0.0, RUN_Z + TIP + 0.035), FINGER_CLOSED, "draw it up out of the fitting", 0.0),
        Waypoint(2.0, (GRIP_RUN_X, -0.090, RUN_Z + TIP + 0.035), FINGER_CLOSED, "swing it aside", 0.0),
        Waypoint(1.5, (GRIP_RUN_X, -0.090, TOP + TUBE_D / 2 + TIP), FINGER_CLOSED, "lay it down", 0.0),
        Waypoint(1.0, (GRIP_RUN_X, -0.090, TOP + TUBE_D / 2 + TIP), FINGER_OPEN * 0.5, "let go", 0.0),
        Waypoint(1.5, (GRIP_RUN_X, -0.200, RUN_Z + TIP + 0.035), FINGER_OPEN * 0.5, "up and away", 0.0),
        Waypoint(1.5, PARK_R, FINGER_OPEN * 0.5, "clear", 0.0),
    ]


def demo():
    return {"left": _left(), "right": _right()}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = np.array([0, 0, 0.040])
    V = {"valve_turn": 90.0}
    start = kit.fake_states()
    end_pos, end_q = kit.pose(start, "tube_end")
    out = {"tube_end": (end_pos + up, end_q)}
    seat = (np.array([PORT[0], PORT[1], FLOOR]), np.array([0, 0, 0, 1.0]))

    def st(q=V, **moved):
        return kit.fake_states(q=q, deg=True, moved=moved)

    plugged = st(**out, plug=seat)
    L = {"locked": True, "locked_at_release": True}
    late = {"locked": True, "locked_at_release": False}
    cases = {  # name: (state, tcps, MEMORY latches, score)
        "start": (start, [], {}, 0.0),
        "valve closed only": (st(), [], {}, 0.1),
        "valve closed and padlocked only": (st(), [], {"locked": True}, 0.2),
        "valve half turned, nothing else": (st(q={"valve_turn": 45.0}), [], {}, 0.0),
        "valve closed, tube pulled up only 4 mm": (st(tube_end=(end_pos + np.array([0, 0, 0.004]), end_q)), [], L,
                                                   0.2),
        "locked, tube out, port open": (st(**out), [], L, 0.5),
        "not locked, tube out, port open": (st(**out), [], {"unlocked_release": True}, 0.0),
        "full: locked first, bled, tube out, plug seated": (plugged, [], L, 1.0),
        "no padlock, otherwise all done": (plugged, [], {"unlocked_release": True}, 0.0),
        "padlocked only after the release": (plugged, [], {**late, "unlocked_release": True}, 0.0),
        "plug half in": (st(**out, plug=(seat[0] + np.array([0, 0, 0.008]), seat[1])), [], L, 0.5),
        "plug tilted 23 deg": (st(**out, plug=(seat[0], mk.mat_to_quat(mk.axis_angle((1, 0, 0), 0.4)))), [], L,
                               0.5),
        "tube still held by a gripper": (plugged, [kit.point(plugged, "tube_0", (0, -0.02, 0))], L, 0.2),
        "plug still held": (plugged, [np.array([PORT[0], PORT[1], FLOOR + 0.019])], L, 0.5),
        "released under pressure, all else done": (plugged, [], {**L, "live_disconnect": True}, 0.0),
        "valve reopened after the release, closed again": (plugged, [], {**late, "reopened": True}, 0.0),
        "valve open at the end, tube out, plug seated": (kit.fake_states(moved={**out, "plug": seat}), [], L, 0.0),
    }
    # the lock rule on padlock poses: shut through both holes, hung through them but open, shut away from the valve
    on_holes = {"padlock": (np.array([LOCK_HOLE[0] - _FREE_DX, LOCK_HOLE[1], TOP]), np.array([0, 0, 0, 1.0]))}
    lock_cases = {  # name: (shackle travel m, padlock moved, valve deg) -> (shut, locked)
        "shut through both holes": ((0.0, on_holes, 90.0), (True, True)),
        "hung through both holes, 8 mm open": ((0.008, on_holes, 90.0), (False, False)),
        "open, standing in place": ((SHACKLE_OPEN, {}, 90.0), (False, False)),
        "shut on the bench away from the valve": ((0.0, {}, 90.0), (True, False)),
        "shut through the holes, lever at 80 deg": ((0.0, on_holes, 80.0), (True, False)),
    }
    for name, ((d, mv, deg), want) in lock_cases.items():
        mk.new_episode()
        s_ = kit.fake_states(q={"shackle_travel": d, "valve_turn": deg}, deg=True, moved=mv)
        _lock_rule(s_)
        got = (bool(mk.MEMORY.get("shut")), bool(mk.MEMORY.get("locked")))
        assert got == want, (name, got, want)
    assert not shackle_through(kit.fake_states(q={"shackle_travel": SHACKLE_OPEN}, moved=on_holes))
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + len(lock_cases) + 1


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
