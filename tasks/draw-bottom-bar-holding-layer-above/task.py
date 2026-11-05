"""On the station table, between the arms: a small pile of pine bars. Three bars lie side by side on the table,
running left to right; the middle one is red, 4 mm thicker than the two beside it and 60 mm longer, its extra
length sticking out toward the robot's left. Two short bars lie across the red bar, side by side, carried by the
red bar alone. A yellow pad is taped to the table nearer the left arm. Draw the red bar out endwise from under the
two crossing bars and lay it on the pad, without moving any of the other four bars more than 1 cm or turning them
more than 5 degrees.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
PILE_X = 0.30
GAP = 0.002  # between neighbouring bottom bars

SIDE = (0.030, 0.260, 0.030)  # two plain bars beside the red one, long side along y
RED_SIZE = (0.030, 0.320, 0.034)  # thicker and longer: it alone carries the crossing bars
CROSS = (0.120, 0.030, 0.030)  # the crossing bars, long side along x
SIDE_MASS, RED_MASS, CROSS_MASS = 0.12, 0.16, 0.055  # pine, about 500 kg/m3

SIDE_Y = 0.0
RED_Y = SIDE_Y + (RED_SIZE[1] - SIDE[1]) / 2  # flush with the side bars at -y, 60 mm proud at +y
SIDE_XS = {"bar_near": PILE_X - SIDE[0] - GAP, "bar_far": PILE_X + SIDE[0] + GAP}
CROSS_YS = {"cross_right": -(CROSS[1] + 0.002) / 2, "cross_left": (CROSS[1] + 0.002) / 2}  # 2 mm apart

SIDE_TOP = TOP + SIDE[2]
RED_TOP = TOP + RED_SIZE[2]

PAD_XY = (0.17, 0.32)
PAD_SIZE = (0.070, 0.360, 0.002)
PAD_TOP = TOP + PAD_SIZE[2]

PINE, RED, YELLOW = (0.80, 0.66, 0.45), (0.80, 0.12, 0.10), (0.95, 0.80, 0.15)

SIDE_BARS = [Box(n, pos=(x, SIDE_Y, TOP + SIDE[2] / 2), size=SIDE, mass=SIDE_MASS, color=PINE)
             for n, x in SIDE_XS.items()]
RED_BAR = Box("red_bar", pos=(PILE_X, RED_Y, TOP + RED_SIZE[2] / 2), size=RED_SIZE, mass=RED_MASS, color=RED)
CROSS_BARS = [Box(n, pos=(PILE_X, y, RED_TOP + CROSS[2] / 2), size=CROSS, mass=CROSS_MASS, color=PINE)
              for n, y in CROSS_YS.items()]
PAD = Box("yellow_pad", pos=(*PAD_XY, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=YELLOW)

OBJECTS = [*SIDE_BARS, RED_BAR, *CROSS_BARS, PAD]
PROTECTED = {o.name: o for o in SIDE_BARS + CROSS_BARS}

# The pile's footprint on the table (the three bottom bars), and the crossing bars' extent along y.
PILE_X0 = SIDE_XS["bar_near"] - SIDE[0] / 2
PILE_X1 = SIDE_XS["bar_far"] + SIDE[0] / 2
PILE_Y0, PILE_Y1 = SIDE_Y - SIDE[1] / 2, SIDE_Y + SIDE[1] / 2
CROSS_Y1 = max(CROSS_YS.values()) + CROSS[1] / 2

MOVE_TOL = 0.010  # a protected bar's centre may move at most 10 mm (any direction) at any time
TURN_TOL_DEG = 5.0  # and turn at most 5 degrees from its starting orientation at any time
ON_TOL = 0.001  # resting on a surface: the bar's bottom within 1 mm of it (the demo lets go 4 to 7 mm higher)
LYING_H = 0.040  # lying flat: the red bar's top at most 40 mm above its underside (34 mm lying as it started)
FOOT_SLACK = 0.001
EPS = 1e-6  # so that a value exactly at a stated limit is judged as the text says, not by rounding
START_TOL = 0.0005  # back at the start: every bar within 0.5 mm of its starting pose and at rest
RESET_JUMP = 0.020  # ... reached by a jump of more than 20 mm in one control step (a reset, not the robot)


def _turn_deg(st, start_quat=(0.0, 0.0, 0.0, 1.0)) -> float:
    """Angle between the bar's orientation and its starting one (every start is unrotated)."""
    w = abs(float(np.dot(np.asarray(st.quat_xyzw, float), np.asarray(start_quat, float))))
    return float(np.degrees(2.0 * np.arccos(min(1.0, w))))


def _moved(st, spec) -> float:
    return float(np.linalg.norm(np.asarray(st.pos) - np.asarray(spec.pos)))


BARS = [*SIDE_BARS, RED_BAR, *CROSS_BARS]

# Trial history: whether any protected bar has moved more than 10 mm or turned more than 5 degrees since the trial
# started. The scorer sees one step at a time, so it is kept here. It is cleared only at a new trial: every bar
# within 0.5 mm of its starting pose and at rest, reached by a jump of more than 20 mm in one control step (the
# simulator's reset), or on the first step the scorer sees. Putting the bars back by hand does not clear it.
_HISTORY = {"violated": False, "prev": None}


def _at_start(objects) -> bool:
    for o in BARS:
        st = objects[o.name]
        if _moved(st, o) > START_TOL or _turn_deg(st) > 0.1 or not st.resting():
            return False
    return True


def _jump(objects) -> float:
    prev = _HISTORY["prev"]
    if prev is None:
        return float("inf")
    return max(float(np.linalg.norm(np.asarray(objects[o.name].pos) - prev[o.name])) for o in BARS)


def _update(objects) -> None:
    if _at_start(objects) and _jump(objects) > RESET_JUMP:
        _HISTORY["violated"] = False
    _HISTORY["prev"] = {o.name: np.asarray(objects[o.name].pos, float).copy() for o in BARS}
    for name, spec in PROTECTED.items():
        st = objects[name]
        if _moved(st, spec) > MOVE_TOL + EPS or _turn_deg(st) > TURN_TOL_DEG + EPS:
            _HISTORY["violated"] = True


def _cross_settled(objects) -> bool:
    """Both crossing bars resting on the two side bars (bottom within 1 mm of their tops) and at rest."""
    return all(abs(objects[n].low[2] - SIDE_TOP) <= ON_TOL + EPS and objects[n].resting() for n in CROSS_YS)


def _sides_still(objects) -> bool:
    return all(objects[n].resting() for n in SIDE_XS)


def _red_on(st, surface_z) -> bool:
    """Lying flat on the surface: its underside within 1 mm of it and its top at most 40 mm above its underside."""
    return bool(abs(st.low[2] - surface_z) <= ON_TOL + EPS and st.high[2] - st.low[2] <= LYING_H + EPS)


def _red_on_pad(st) -> bool:
    px, py = PAD_XY
    hx, hy = PAD_SIZE[0] / 2 + FOOT_SLACK, PAD_SIZE[1] / 2 + FOOT_SLACK
    return bool(
        st.low[0] >= px - hx - EPS and st.high[0] <= px + hx + EPS
        and st.low[1] >= py - hy - EPS and st.high[1] <= py + hy + EPS
        and _red_on(st, PAD_TOP)
    )


def _red_clear_of_pile(st) -> bool:
    """No part of the red bar's footprint over the pile's footprint."""
    return bool(st.low[1] > PILE_Y1 or st.high[1] < PILE_Y0 or st.low[0] > PILE_X1 or st.high[0] < PILE_X0)


def _tier(objects) -> float:
    _update(objects)
    if _HISTORY["violated"] or not (_cross_settled(objects) and _sides_still(objects)):
        return 0.0
    red = objects["red_bar"]
    if not red.resting():
        return 0.0
    if _red_on_pad(red):
        return 1.0
    if not (_red_on(red, TOP) or _red_on(red, PAD_TOP)):
        return 0.0
    if _red_clear_of_pile(red):
        return 0.5
    return 0.25 if red.low[1] > CROSS_Y1 else 0.0


def success(objects):
    """The red bar lying wholly on the pad and at rest; the crossing bars resting on the side bars; no other bar
    moved more than 10 mm or turned more than 5 degrees at any time during the trial."""
    return _tier(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(objects["red_bar"].pos[:2] - np.asarray(PAD_XY)))


def score(objects):
    """1.0 as success; 0.5 with the red bar clear of the pile, lying on the table or partly on the pad; 0.25 with it out from under the
    crossing bars but still between the side bars; 0 otherwise, and 0 for the whole trial once any other bar has
    moved more than 10 mm or turned more than 5 degrees."""
    return _tier(objects)


# ---- scripted demo ------------------------------------------------------------------------------------------
# Right arm: grips the two crossing bars together (fingers closing along y) over the red bar and holds them.
# Left arm: grips the red bar's proud end from above (fingers closing along x) and draws it 290 mm toward its own
# base, along the bar. Once the red bar is out, the crossing bars hang in the right gripper about 1.1 mm above the side
# bars (the arm sags under them); it raises them 4 mm more, so they are clearly not yet resting, and lets go slowly,
# dropping them onto the side bars. The left arm then moves the red bar to the pad, gripping it at its middle, and
# lets go with its underside 7 mm above the pad.
TIP = 0.013  # finger tips below the TCP
TRAVEL_Z = TOP + 0.13
YAW_X = np.pi / 2  # fingers closing along x
DRAW = 0.290  # red bar's far end ends 30 mm past the side bars' ends
HOLD_TIPS = RED_TOP + 0.008  # right finger tips 8 mm above the red bar while it slides under them
HOLD_Z = HOLD_TIPS + TIP
END_GRIP_Y = RED_Y + RED_SIZE[1] / 2 - 0.020
END_TIPS = TOP + 0.006
END_Z = END_TIPS + TIP
MID_Z = TOP + 0.010 + TIP
SET_DOWN = 0.007  # release the red bar with its bottom 7 mm above the pad (beyond the 1 mm tolerance)
LIFT_OFF = 0.004  # before letting go, raise the crossing bars 4 mm clear of the side bars
# Measured on the L40S with the trace wrapper: the right gripper closes turned about 4 degrees counterclockwise
# (seen from above) from its target, and the left gripper, closing on the red bar's end, about 4.5 degrees the same
# way; uncorrected, they turn the crossing bars and the red bar, which then shoves the far side bar. The demo turns
# both grips back by that much. The right arm also takes a second or two to settle over the bars, so it waits there
# before it closes.
HOLD_YAW = -0.068
# The crossing bars are 62 mm across together. The right fingers squeeze them with their pads commanded 4 mm inside
# each face: fully closed, both fingers push at their force limit and the pair can slide between them, and easing
# off from there kicks the bars about 17 mm sideways. They let go slowly, to 4 mm clear of each face.
PAIR_W = 2 * max(CROSS_YS.values()) + CROSS[1]
SQUEEZE = -(PAIR_W / 2 - 0.004)
LET_GO = -(PAIR_W / 2 + 0.004)
END_YAW = np.pi / 2 - 0.078  # +pi/2: -pi/2 minus the offset needs joint6 past its limit


def _wp(t, x, y, z, f, label, yaw=0.0):
    return Waypoint(t, (float(x), float(y), float(z)), f, label, yaw)


def demo():
    x = PILE_X
    hold_y = 0.0
    right = [
        _wp(2.0, x, hold_y, TRAVEL_Z, FINGER_OPEN, "over the crossing bars", HOLD_YAW),
        _wp(1.5, x, hold_y, HOLD_Z, FINGER_OPEN, "descend beside them", HOLD_YAW),
        _wp(1.5, x, hold_y, HOLD_Z, FINGER_OPEN, "settle", HOLD_YAW),
        _wp(0.8, x, hold_y, HOLD_Z, SQUEEZE, "grip both", HOLD_YAW),
        _wp(7.2, x, hold_y, HOLD_Z, SQUEEZE, "hold while the red bar is drawn", HOLD_YAW),
        _wp(1.0, x, hold_y, HOLD_Z + LIFT_OFF, SQUEEZE, "raise them clear of the side bars", HOLD_YAW),
        _wp(1.5, x, hold_y, HOLD_Z + LIFT_OFF, LET_GO, "release slowly: they drop onto the side bars", HOLD_YAW),
        _wp(1.2, x, hold_y, TRAVEL_Z, LET_GO, "up", HOLD_YAW),
    ]
    y_end = END_GRIP_Y
    red_mid_after = RED_Y + DRAW
    pad_z = PAD_TOP + 0.010 + TIP + SET_DOWN
    left = [
        _wp(2.0, x, y_end, TRAVEL_Z, FINGER_OPEN, "over the red bar's end", END_YAW),
        _wp(1.5, x, y_end, END_Z, FINGER_OPEN, "descend", END_YAW),
        _wp(0.8, x, y_end, END_Z, FINGER_CLOSED, "grip the end", END_YAW),
        _wp(2.0, x, y_end, END_Z, FINGER_CLOSED, "wait for the hold", END_YAW),
        _wp(5.0, x, y_end + DRAW, END_Z, FINGER_CLOSED, "draw it out along its length", END_YAW),
        _wp(0.6, x, y_end + DRAW, END_Z, FINGER_OPEN, "let go of the end", END_YAW),
        _wp(1.0, x, y_end + DRAW, TRAVEL_Z, FINGER_OPEN, "up", YAW_X),
        _wp(1.5, x, red_mid_after, TRAVEL_Z, FINGER_OPEN, "over its middle", YAW_X),
        _wp(1.2, x, red_mid_after, MID_Z, FINGER_OPEN, "descend", YAW_X),
        _wp(0.8, x, red_mid_after, MID_Z, FINGER_CLOSED, "grip the middle", YAW_X),
        _wp(1.2, x, red_mid_after, TRAVEL_Z, FINGER_CLOSED, "lift", YAW_X),
        _wp(1.8, PAD_XY[0], PAD_XY[1], TRAVEL_Z, FINGER_CLOSED, "over the pad", YAW_X),
        _wp(1.2, PAD_XY[0], PAD_XY[1], pad_z, FINGER_CLOSED, "lower", YAW_X),
        _wp(0.3, PAD_XY[0], PAD_XY[1], pad_z, FINGER_OPEN, "release: fully open before the bar lands", YAW_X),
        _wp(0.3, PAD_XY[0], PAD_XY[1], pad_z, FINGER_OPEN, "wait", YAW_X),
        _wp(1.0, PAD_XY[0], PAD_XY[1], TRAVEL_Z, FINGER_OPEN, "up", YAW_X),
    ]
    return {"right": right, "left": left}
