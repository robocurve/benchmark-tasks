"""On the station table, 26 cm out on the midline: a closed 12 oz drink can (6.6 cm across, 12.3 cm tall), standing
upright in a holder that keeps it from moving. Its lid is recessed 6 mm inside the rim. On the lid lies the
stay-on tab, flat, its rivet in the middle of the lid; the tab's rolled tail points away from the arms and its nose
lies over the scored opening, which is on the side nearer the arms. Open the can.
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, mechanism  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SILVER, LID_GREY, TAB_GREY, RED, DARK = (0.80, 0.81, 0.83), (0.70, 0.71, 0.73), (0.62, 0.63, 0.66), (0.75, 0.10, 0.12), \
    (0.15, 0.15, 0.16)
MATERIALS = {"default": (1.2, 1.0), "metal": (0.3, 0.2)}

# -- sizes, mm, in the can's frame: origin at the can's bottom centre on the table, x away from the arms -----------
# A US 12 fl oz (355 ml) can: body 66.0 mm across, 122.7 mm tall; a 202 end (lid) 54 mm across over its rim
# (Wikipedia "Beverage can"). Lid recess (countersink) about 6.3 mm [estimate].
BODY_R, BODY_H = 33.0, 100.0
NECK_R = 28.6  # the neck, as a ring of posts so the can is hollow under the lid
RIM_IN, RIM_OUT, RIM_TOP = 25.5, 27.0, 122.7
LID_Z = RIM_TOP - 6.3  # the lid's top surface: 116.4
LID_T = 1.0
# The opening (the scored panel), on the -x side (towards the arms), hinged near the rivet.
# WHY the hinge 4 mm from the rivet: with it at 1.5 mm, the tab's underside pressed on the panel right by its hinge,
# where the panel can't go down, and the tab jammed at 4 degrees (official sim, 2026-11-13).
PANEL_X0, PANEL_X1, PANEL_HALF_W = -21.0, -4.0, 7.0  # panel from x -21 to -4, 14 mm wide
OPEN_GAP = 0.4  # the opening is this much bigger than the panel all round
# The tab: rivet at the lid's centre; nose over the panel (-x), tail (+x) with its rolled edge.
# WHY the body 1.2 mm up with a nub under its nose: a real tab's nose bends down and presses the panel at one
# point; a flat body lying on the panel touched it everywhere, including by the hinge (see above).
TAB_LIFT, TAB_T = 1.2, 1.0  # the body's underside above the lid, and its thickness
RIVET = (0.0, 0.0, LID_Z + TAB_LIFT + TAB_T / 2)
TAB_NOSE_X, TAB_TAIL_X, TAB_HALF_W = -7.0, 8.0, 6.0  # its body runs on under the tail
BODY_X0 = PANEL_X1  # the flat body stops where the panel starts
# WHY the nose raised 1.8 mm over the panel, pressing only through its nub: flat over the panel, the body met the
# panel by its hinge as both swung down and stopped the tab at 18.6 degrees (official sim, 2026-11-13).
NOSE_LIFT = 3.0  # the nose's underside above the lid
NUB_X0, NUB_X1 = -7.0, -5.8  # the nose's nub, down to 0.1 mm over the panel
# WHY a 3.6 mm round tail: a real tab's tail is a rolled edge about 1 mm thick. The fingertips are 26 mm wide and
# 6.3 mm thick (the gripper's collision hulls), so they pinch the tail across x; round about the rivet's axis, it
# turns in the pinch as the tab swings up (the same as the accepted processor task's lever knob).
# WHY the tail 8 mm from the rivet and 2.4 mm proud: fingers 10 mm apart round it reach 19.3 mm out with their 26 mm
# width across, 2 mm inside the rim; and the near finger's tips, 0.3 mm above the tail's bottom, pass over the tab's
# body (2.2 mm up). A real tail is lifted a little over its finger well; this is higher, for the 6 mm fingertips.
KNOB_X, KNOB_R, KNOB_LEN = 8.0, 1.8, 8.0
KNOB_Z = LID_Z + TAB_LIFT + TAB_T + 0.2 + KNOB_R  # its bottom 0.2 mm over the body's top
# The seal: a sprung catch under the panel's +y edge, beside where the tab's nose presses, overlapping the edge by
# 0.4 mm. Pushed down, the panel cams it aside and swings in; once past, it is free. The catch's spring sets the force
# the seal takes to break.
# WHY beside the nose, 0.4 mm over: under the panel's far end, 0.8 mm over, the nose (2-3 mm from the panel's hinge)
# had too little leverage and the round catch wedged: the arm could not lift the tail at all (official sim, 2026-11-13).
CATCH_X, CATCH_R, CATCH_LEN = -7.0, 1.5, 4.0
CATCH_OVERLAP = 0.4
CATCH_Y = PANEL_HALF_W + CATCH_R - CATCH_OVERLAP
CATCH_Z = LID_Z - LID_T - CATCH_R
CATCH_K, CATCH_PRELOAD, CATCH_TRAVEL = 100.0, 0.002, 0.004  # N/m, m, m [tuned in sim]

CAN_XY = (0.26, 0.0)
TAB_KG, PANEL_KG, CATCH_KG = 0.003, 0.003, 0.002  # real tab and panel are about 0.5 g; heavier here for stability


def _m(v):
    return tuple(x / 1000 for x in v)


def _box(c, s, color, material="metal"):
    return Shape("box", _m(c), _m(s), color, material=material, contact_offset=0.001)


def _cyl(c, r, length, axis, color, material="metal"):
    return Shape("cylinder", _m(c), (r / 1000, length / 1000), color, axis=axis, material=material,
                 contact_offset=0.001)


def _lid_strips():
    """The lid as 2 mm strips across x, each as wide as the lid's circle there, with the opening left out."""
    shapes, step, z = [], 2.0, LID_Z - LID_T / 2
    x = -RIM_IN
    while x < RIM_IN - 1e-9:
        cx = x + step / 2
        half = math.sqrt(max(RIM_IN ** 2 - max(abs(x), abs(x + step)) ** 2, 0.0))
        lo, hi = PANEL_X0 - OPEN_GAP, PANEL_X1
        if half > 0.5:
            if lo <= cx <= hi:  # beside the opening: two pieces
                w = PANEL_HALF_W + OPEN_GAP
                if half > w:
                    for sy in (1, -1):
                        shapes.append(_box((cx, sy * (w + half) / 2, z), (step, half - w, LID_T), LID_GREY))
            else:
                shapes.append(_box((cx, 0.0, z), (step, 2 * half, LID_T), LID_GREY))
        x += step
    return shapes


def _ring(r, post_r, z0, z1, n, color):
    """A wall of upright posts round a circle (boxes can't be turned within a link, so a round wall is posts)."""
    return [_cyl((r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n), (z0 + z1) / 2), post_r, z1 - z0,
                 "Z", color) for i in range(n)]


def _can_base():
    lid_bottom = LID_Z - LID_T
    return tuple([
        _cyl((0, 0, BODY_H / 2), BODY_R, BODY_H, "Z", RED),
        _cyl((0, 0, BODY_H + 0.25), BODY_R - 1.0, 0.5, "Z", DARK),  # seen through the opening, inside the can
        # posts touching one another, so the neck and rim read as solid walls
        *_ring(NECK_R, 0.95, BODY_H, lid_bottom, 96, SILVER),
        *_ring((RIM_IN + RIM_OUT) / 2, 0.75, lid_bottom, RIM_TOP, 128, SILVER),
        *_lid_strips(),
    ])


def _panel():
    length = PANEL_X1 - PANEL_X0
    return (_box(((PANEL_X0 + PANEL_X1) / 2, 0, LID_Z - LID_T / 2), (length, 2 * PANEL_HALF_W, LID_T), LID_GREY),)


def _tab():
    body = TAB_TAIL_X - BODY_X0
    return (
        _box(((BODY_X0 + TAB_TAIL_X) / 2, 0, LID_Z + TAB_LIFT + TAB_T / 2), (body, 2 * TAB_HALF_W, TAB_T), TAB_GREY),
        # the nose, bent up over the panel, and its nub pressing down onto it
        _box(((TAB_NOSE_X + BODY_X0) / 2, 0, LID_Z + NOSE_LIFT + TAB_T / 2), (BODY_X0 - TAB_NOSE_X, 8.0, TAB_T), TAB_GREY),
        _box((BODY_X0 - 0.25, 0, LID_Z + (TAB_LIFT + NOSE_LIFT + TAB_T) / 2), (0.5, 8.0, NOSE_LIFT + TAB_T - TAB_LIFT),
             TAB_GREY),
        _box(((NUB_X0 + NUB_X1) / 2, 0, LID_Z + (0.1 + NOSE_LIFT) / 2), (NUB_X1 - NUB_X0, 6.0, NOSE_LIFT - 0.1), TAB_GREY),
        Shape("capsule", _m((KNOB_X, 0, KNOB_Z)), (KNOB_R / 1000, KNOB_LEN / 1000), TAB_GREY, axis="Y",
              material="default", contact_offset=0.001),
    )


def _catch():
    return (_cyl((CATCH_X, CATCH_Y, CATCH_Z), CATCH_R, CATCH_LEN, "X", LID_GREY),)


TAB_MAX = math.radians(120)
PANEL_MAX = math.radians(100)
CAN = mechanism(
    "can", (*CAN_XY, TOP), base=Link("can_body", 0.40, _can_base()),
    links=(Link("tab", TAB_KG, _tab()), Link("panel", PANEL_KG, _panel()), Link("catch", CATCH_KG, _catch())),
    joints=(
        # - lifts the tab's tail (+x) about the rivet; its nose (-x) then goes down onto the panel. No spring: a
        # real tab stays where it is bent to; friction holds it there.
        Joint("tab_hinge", "revolute", "tab", _m(RIVET), "Y", -TAB_MAX, 0.0, damping=0.0005, friction=0.002),
        # - swings the panel down into the can about its hinge by the rivet. Friction keeps it where it is pushed,
        # as the real panel stays folded in.
        Joint("panel_hinge", "revolute", "panel", _m((PANEL_X1, 0, LID_Z - LID_T / 2)), "Y", -PANEL_MAX, 0.0,
              damping=0.0005, friction=0.002),
        # + moves the catch out from under the panel; preloaded against its stop at 0
        Joint("catch_slide", "prismatic", "catch", _m((CATCH_X, CATCH_Y, CATCH_Z)), "Y", 0.0, CATCH_TRAVEL,
              stiffness=CATCH_K, damping=0.5, target=-CATCH_PRELOAD),
    ),
    materials=MATERIALS)
OBJECTS = [CAN]

# -- scoring ----------------------------------------------------------------------------------------------------------
OPEN_DEG = 45.0  # the panel pushed in at least this far: the can is open
# WHY the tab only 10 degrees: it shows the tab opened the can (a panel poked in with a finger leaves the tab flat at
# 0). Lifting the tail turns the panel about 4 degrees per degree of tab, so the panel is fully in with the tab at
# about 20 degrees, and its nose then meets the panel by the hinge (official sim, 2026-11-13: tab 22, panel 82).
TAB_UP_DEG = 10.0
SEAL_BROKEN_DEG = 10.0  # the panel in this far: the seal has gone
TAB_STARTED_DEG = 3.0
TAB_LIFTED_DEG = 10.0


def angles(objects) -> tuple[float, float]:
    """How far the tab's tail is raised and the panel pushed in (degrees, both positive)."""
    joints = objects["can"].joints
    return -math.degrees(joints["tab_hinge"]), -math.degrees(joints["panel_hinge"])


COMPLETE_SCORE = 0.9  # score() for the open can: 1.0 comes only from success() holding for 1 s, which ends the trial


def _grade(objects) -> float:
    """1.0 for the panel pushed in at least 45 degrees with the tab raised at least 10 degrees (it was the tab that
    opened it); 0.5 if the seal is broken (the panel in at least 10 degrees) with the tab raised at least 3 degrees;
    0.25 if the tab is raised at least 10 degrees but the seal holds; 0 otherwise, including a panel pushed in
    with the tab still flat."""
    tab, panel = angles(objects)
    if panel >= OPEN_DEG and tab >= TAB_UP_DEG:
        return 1.0
    if panel >= SEAL_BROKEN_DEG and tab >= TAB_STARTED_DEG:
        return 0.5
    if tab >= TAB_LIFTED_DEG and panel < SEAL_BROKEN_DEG:
        return 0.25
    return 0.0


def score(objects) -> float:
    """0.9 for the can opened with its tab (1.0 once success() has held 1 s); 0.5 seal broken with the tab started;
    0.25 tab raised but the seal holding; 0 otherwise."""
    return min(_grade(objects), COMPLETE_SCORE)


def success(objects) -> bool:
    """The panel pushed in at least 45 degrees with the tab raised at least 10 degrees. Must hold for 1 s."""
    return _grade(objects) == 1.0


# -- the scripted demo (left arm) -------------------------------------------------------------------------------------
CX, CY = CAN_XY
RIVET_W = np.array([CX + RIVET[0] / 1000, CY + RIVET[1] / 1000, TOP + RIVET[2] / 1000])
KNOB_FROM_RIVET = np.array([(KNOB_X - RIVET[0]) / 1000, 0.0, (KNOB_Z - RIVET[2]) / 1000])
KNOB_GRIP_Z = PAD_TIP_PAST_TCP - KNOB_R / 1000 + 0.0003  # TCP above the tail's axis: the fingertips 0.3 mm above its bottom
ACROSS_X = -math.pi / 2  # fingers closing along x (turned the left arm's easy way)
PINCH_KNOB = FINGER_OPEN * 0.0010 / 0.094  # aimed 1 mm apart across the 3.6 mm tail: a firm pinch
APPROACH = FINGER_OPEN * 0.010 / 0.094  # 10 mm open coming down round the tail, inside the lid's 51 mm recess
DEMO_AIM_SHIFT_MM = 0.0  # the pinch aimed this far off the tail (+x and +y), for margin runs
# WHY commanded to 80 degrees and held: the tab and seal hold the arm back, so the tail lags the commanded arc; the
# tab stops near 22 degrees against the open panel anyway. Lifting only to 40 left the tab at 10.7 and the panel at
# 37 degrees when the fingers opened (official sim, 2026-11-13).
TAB_STEPS = (-8, -16, -24, -32, -40, -48, -56, -64, -72, -80)
HOLD_S = 2.0


def _turn_y(v, deg):
    t = math.radians(deg)
    return np.array([v[0] * math.cos(t) + v[2] * math.sin(t), v[1], -v[0] * math.sin(t) + v[2] * math.cos(t)])


def tail_tcp(deg):
    """TCP pinching the tab's tail with the tab at ``deg`` (negative lifts the tail)."""
    shift = DEMO_AIM_SHIFT_MM / 1000
    return tuple(RIVET_W + _turn_y(KNOB_FROM_RIVET, deg) + np.array([shift, shift, KNOB_GRIP_Z]))


def _above(p, dz):
    return (p[0], p[1], p[2] + dz)


def demo():
    left = [
        Waypoint(2.0, _above(tail_tcp(0), 0.025), APPROACH, "over the tab's tail", ACROSS_X),
        Waypoint(1.5, tail_tcp(0), APPROACH, "down into the lid's recess", ACROSS_X),
        Waypoint(0.8, tail_tcp(0), PINCH_KNOB, "pinch the tail", ACROSS_X),
        *[Waypoint(0.7, tail_tcp(d), PINCH_KNOB, f"tail up {-d}", ACROSS_X) for d in TAB_STEPS],
        Waypoint(HOLD_S, tail_tcp(TAB_STEPS[-1]), PINCH_KNOB, "keep pulling", ACROSS_X),
        Waypoint(0.8, tail_tcp(TAB_STEPS[-1]), APPROACH, "let go", ACROSS_X),
        Waypoint(1.2, _above(tail_tcp(TAB_STEPS[-1]), 0.02), APPROACH, "up", ACROSS_X),
        Waypoint(1.5, (0.22, 0.25, TOP + 0.15), FINGER_OPEN, "out of the way"),
    ]
    right = [Waypoint(1.0, (0.22, -0.30, TOP + 0.15), FINGER_OPEN, "stay clear")]
    return {"left": left, "right": right}
