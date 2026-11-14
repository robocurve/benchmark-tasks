"""On the station table: the front corner of a built-in gas hob (brushed steel deck, two black knobs on its front
edge) with the pan grate lifted off. The front burner is taken apart for cleaning: only its grey cast base is on the
deck, a flange with a round locating spigot and a white porcelain igniter standing beside the spigot. On the robot's
left a pale blue ribbed drainer holds the two washed parts: the silver cast aluminium burner head (a ring with one
slot in its rim) and the matt black cast-iron cap. The head sits flat on the base only when its slot is turned round
the igniter; turned wrong, it rides on the igniter, tilted. The cap then goes on the head, over its raised lip. Head
and cap are loose; everything else is fixed.

Requires -E object_jitter=0: the scripted demo starts from the fixed layout below.

Run it:  inspect-robots run --embodiment yam_arena -E task=refit_a_gas_burner_head_and_cap.py -E object_jitter=0 \\
           --instruction "Put the burner head back on its base so it sits flat round the igniter, then put the cap on
           it, centred." --scorer task_score ...
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dataclasses  # noqa: E402

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import mechkit  # noqa: E402
from belimits import bottom, is_airborne, tilt_deg  # noqa: E402
from closedloop import placeholder_demo  # noqa: E402
from mechkit import Mech, box, cyl, lathe, on_reset, released, ring_profile, tube  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to open, rise and park after the first 1.0

STEEL, CAST, SILVER, WHITE = (0.70, 0.71, 0.72), (0.40, 0.41, 0.42), (0.86, 0.86, 0.84), (0.97, 0.97, 0.95)
BLACK, DARK, BLUE, TIP = (0.05, 0.05, 0.05), (0.12, 0.12, 0.12), (0.62, 0.80, 0.93), (0.55, 0.56, 0.58)

# ---- the hob (fixed): the front corner of a full-size hob, running out of the work area to the right and far side -------
DECK_Z = 0.045  # deck top above the table
HOB_X, HOB_Y = (0.20, 0.50), (-0.52, 0.055)
BURNER = (0.29, -0.015)  # front burner axis
BURNER2 = (0.29, -0.255)  # the next burner along the front, 24 cm away, assembled
KNOB_X, KNOB_Y, KNOB_R, KNOB_H = 0.225, (-0.10, -0.17), 0.017, 0.020  # a row of knobs on the front edge
_hc = ((HOB_X[0] + HOB_X[1]) / 2, (HOB_Y[0] + HOB_Y[1]) / 2)


def _h(x, y, z):
    """A point given in the station frame (z above the table), in the hob's own frame."""
    return (x - _hc[0], y - _hc[1], z)


HOB = Mech(
    "hob", pos=(_hc[0], _hc[1], TOP), mass=None, friction=0.4,
    shapes=[
        box(_h(*_hc, DECK_Z / 2), (HOB_X[1] - HOB_X[0], HOB_Y[1] - HOB_Y[0], DECK_Z), STEEL),
        cyl(_h(KNOB_X, KNOB_Y[0], DECK_Z + KNOB_H / 2), KNOB_R, KNOB_H, BLACK),
        cyl(_h(KNOB_X, KNOB_Y[1], DECK_Z + KNOB_H / 2), KNOB_R, KNOB_H, BLACK),
        # the second burner, assembled: flange, head and a 64 mm cap
        cyl(_h(*BURNER2, DECK_Z + 0.002), 0.036, 0.004, CAST),
        cyl(_h(*BURNER2, DECK_Z + 0.004 + 0.008), 0.031, 0.016, SILVER),
        lathe(_h(*BURNER2, DECK_Z + 0.020), [(0.032, 0.0), (0.032, 0.006), (0.027, 0.008)], BLACK),
    ],
)

# ---- the front burner's base (fixed): flange, locating spigot and igniter ------------------------------------------------
FLANGE_R, FLANGE_T = 0.042, 0.004
SEAT = DECK_Z + FLANGE_T  # 0.049: the flange top, where the head's rim sits
SPIGOT_R, SPIGOT_H = 0.026, 0.010
IGN_R, IGN_H, IGN_OFF = 0.006, 0.016, 0.036  # porcelain igniter: 12 mm across, 16 mm above the seat, 36 mm toward +y
IGNITER = (BURNER[0], BURNER[1] + IGN_OFF)
IGN_BEARING = math.pi / 2  # the igniter lies toward +y from the burner axis

BASE = Mech(
    "burner_base", pos=(BURNER[0], BURNER[1], TOP + DECK_Z), mass=None, friction=0.4,
    shapes=[
        cyl((0, 0, FLANGE_T / 2), FLANGE_R, FLANGE_T, CAST),
        cyl((0, 0, FLANGE_T + SPIGOT_H / 2), SPIGOT_R, SPIGOT_H, CAST),
        cyl((0, IGN_OFF, FLANGE_T + IGN_H / 2), IGN_R, IGN_H, WHITE),
        dataclasses.replace(cyl((0, IGN_OFF, FLANGE_T + IGN_H + 0.001), 0.002, 0.002, TIP), collide=False),
    ],
)
FLANGE, SPIGOT, IGN = BASE.shapes[0], BASE.shapes[1], BASE.shapes[2]

# ---- the drainer (fixed): a ribbed plastic tray --------------------------------------------------------------------------
DRAIN_XY, DRAIN_SIZE = (0.26, 0.175), (0.110, 0.180)
RIB_TOP, RIB_W, RIB_L, RIB_DX = 0.035, 0.006, 0.170, (-0.044, -0.022, 0.0, 0.022, 0.044)
DRAINER = Mech(
    "drainer", pos=(DRAIN_XY[0], DRAIN_XY[1], TOP), mass=None, friction=0.4,
    shapes=[box((0, 0, 0.0025), (DRAIN_SIZE[0], DRAIN_SIZE[1], 0.005), BLUE)]
    + [box((dx, 0, 0.005 + (RIB_TOP - 0.005) / 2), (RIB_W, RIB_L, RIB_TOP - 0.005), BLUE) for dx in RIB_DX],
)

# ---- the burner head (free): cast aluminium, 160 g -----------------------------------------------------------------------
# Head frame: origin on its axis at the underside of its rim; the slot's centre line points along +x.
RIM_RIN, RIM_ROUT, RIM_H = 0.029, 0.0375, 0.019
CAVITY = 0.013  # the cavity under the crown: 3 mm above the 10 mm spigot
CROWN_R = 0.0255  # the crown's edge is cut back at the slot, so the igniter passes with the head anywhere on the spigot
SLOT_HALF = math.radians(19.0)  # a 38 degree slot: 5.7 mm each side of the 12 mm igniter
LIP_R, LIP_H = 0.028, 0.006
HEAD_H = RIM_H + LIP_H
HEAD_XY, HEAD_SLOT0 = (0.26, 0.13), math.radians(150.0)


def _ports():
    """18 flame ports: dark notches on the rim's upper outer edge (look-only)."""
    out = []
    for k in range(18):
        a = SLOT_HALF + (2 * math.pi - 2 * SLOT_HALF) * (k + 0.5) / 18
        q = (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))
        r = RIM_ROUT - 0.0012
        notch = box((r * math.cos(a), r * math.sin(a), RIM_H - 0.0028), (0.003, 0.004, 0.006), DARK, tilt=q)
        out.append(dataclasses.replace(notch, collide=False))
    return out


HEAD = Mech(
    "burner_head", pos=(HEAD_XY[0], HEAD_XY[1], TOP + RIB_TOP), mass=0.160, yaw=HEAD_SLOT0, friction=0.4,
    shapes=[
        tube((0, 0, CAVITY / 2), RIM_RIN, RIM_ROUT, CAVITY, SILVER, a0=SLOT_HALF, a1=2 * math.pi - SLOT_HALF),
        tube((0, 0, (CAVITY + RIM_H) / 2), CROWN_R - 0.0005, RIM_ROUT, RIM_H - CAVITY, SILVER, a0=SLOT_HALF, a1=2 * math.pi - SLOT_HALF),
        cyl((0, 0, CAVITY + (RIM_H - CAVITY) / 2), CROWN_R, RIM_H - CAVITY, SILVER),
        lathe((0, 0, RIM_H), [(LIP_R, 0.0), (LIP_R, 0.0045), (LIP_R - 0.0015, LIP_H)], SILVER),
        *_ports(),
    ],
)
RIM, RIM_TOP, CROWN, LIP = HEAD.shapes[0], HEAD.shapes[1], HEAD.shapes[2], HEAD.shapes[3]

# ---- the burner cap (free): matt black enamelled cast iron, 0.35 kg -------------------------------------------------------
# Cap frame: origin on its axis at the underside of its skirt.
CAP_R, SKIRT_RIN, SKIRT_H, CAP_TOP_H = 0.038, 0.031, 0.008, 0.009
CAP_H = SKIRT_H + CAP_TOP_H
CAP_XY = (0.26, 0.22)
CAP = Mech(
    "burner_cap", pos=(CAP_XY[0], CAP_XY[1], TOP + RIB_TOP), mass=0.35, friction=0.3,
    shapes=[
        ring_profile((0, 0, 0), [(SKIRT_RIN, CAP_R, 0.0), (SKIRT_RIN, CAP_R, SKIRT_H)], BLACK),
        lathe((0, 0, SKIRT_H), [(CAP_R, 0.0), (CAP_R, 0.006), (0.034, 0.0082), (0.020, CAP_TOP_H)], BLACK),
    ],
)
SKIRT, CAP_DOME = CAP.shapes

OBJECTS = [HOB, BASE, DRAINER, HEAD, CAP]

# ---- scoring ----------------------------------------------------------------------------------------------------------
HEAD_TILT, HEAD_LOW, HEAD_HIGH, HEAD_XY_TOL = 2.0, -0.0010, 0.0015, 0.0035  # deg, m, m, m
SLOT_TOL = math.radians(12.0)  # the igniter's bearing from the head's axis within this of the slot's centre line
CAP_TILT, CAP_XY_TOL = 3.0, 0.004  # deg, m
CAP_LOW, CAP_HIGH = 0.018, 0.0215  # the cap's lowest point above the seat: on the head's rim (19 mm)
DROP_SPEED = 0.8  # m/s: a free fall of about 3 cm, or a throw
REST_SPEED, REST_STEPS, STEP_HZ = 0.01, 20, 20.0  # under 1 cm/s for 1 s at 20 Hz: 0.5 mm or less per control step
RELEASE_CLEAR = 0.03
# Heights above the table a part can rest on: table, drainer tray, ribs, deck, flange, spigot, igniter and knob tops,
# the seated head's rim and lip.
SUPPORTS = (0.0, 0.005, RIB_TOP, DECK_Z, SEAT, SEAT + SPIGOT_H, SEAT + IGN_H, SEAT + RIM_H, SEAT + HEAD_H)
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it)

_L = {"dropped": False, "still": 0, "seen": None, "last": None}


@on_reset
def _clear(_env=None):
    _L.update(dropped=False, still=0, seen=None, last=None)


def _fingers_clear(objects) -> bool:
    """Both grippers at least 3 cm from the head and from the cap. Only the simulator can tell; offline replays have no
    fingers to measure, so they use OFFLINE_FINGERS_CLEAR."""
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return released(objects["burner_head"], RELEASE_CLEAR) and released(objects["burner_cap"], RELEASE_CLEAR)


def _axis(s) -> np.ndarray:
    return np.asarray(s.to_world((0.0, 0.0, 0.0)), dtype=float)


def slot_bearing(head) -> float:
    """World bearing (rad from +x) of the slot's centre line, seen from above."""
    r = head.rotation()
    return math.atan2(r[1, 0], r[0, 0])


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def slot_error(head) -> float:
    """Angle between the slot's centre line and the line from the head's own axis to the igniter."""
    c = _axis(head)
    return abs(_wrap(math.atan2(IGNITER[1] - c[1], IGNITER[0] - c[0]) - slot_bearing(head)))


def head_seated(objects) -> bool:
    """Flat on the flange round the spigot, with the igniter standing in the slot."""
    h = objects["burner_head"]
    c = _axis(h)
    low = bottom(h, HEAD) - (TOP + SEAT)
    return bool(
        tilt_deg(h) < HEAD_TILT
        and HEAD_LOW <= low <= HEAD_HIGH
        and math.hypot(c[0] - BURNER[0], c[1] - BURNER[1]) <= HEAD_XY_TOL
        and slot_error(h) <= SLOT_TOL
    )


def cap_seated(objects) -> bool:
    """Right way up, flat on the head's rim over the lip, centred on the burner."""
    s = objects["burner_cap"]
    c = _axis(s)
    low = bottom(s, CAP) - (TOP + SEAT)
    return bool(
        tilt_deg(s) < CAP_TILT
        and CAP_LOW <= low <= CAP_HIGH
        and math.hypot(c[0] - BURNER[0], c[1] - BURNER[1]) <= CAP_XY_TOL
    )


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    parts = [objects["burner_head"], objects["burner_cap"]]
    if any(float(np.linalg.norm(p.lin_vel)) > DROP_SPEED for p in parts):
        _L["dropped"] = True
    # Rest is judged from how far each part's axis point moved since the last control step (a ruler on two frames).
    here, last = [_axis(p) for p in parts], _L["last"]
    _L["last"] = here
    still = last is not None and all(float(np.linalg.norm(a - b)) * STEP_HZ < REST_SPEED for a, b in zip(here, last))
    _L["still"] = _L["still"] + 1 if still else 0


def score(objects):
    """0 unless both grippers are 3 cm clear of both parts and both parts are at rest on something. Then 1.0 with the
    head and the cap seated and nothing dropped, 0.5 with the head seated and the cap not, 0.25 with the head seated
    after a drop, else 0."""
    _update(objects)
    held = any(is_airborne(objects[p.name], p, [TOP + z for z in SUPPORTS]) for p in (HEAD, CAP))
    if _L["still"] < REST_STEPS or held or not _fingers_clear(objects):
        return 0.0
    if not head_seated(objects):
        return 0.0
    if _L["dropped"]:
        return 0.25
    return 1.0 if cap_seated(objects) else 0.5


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps its hands off the igniter ---------------------------------------------------------------------------
# Additional documentation, not a scored rule: a gripper pressing on the porcelain electrode cracks it. Near the front
# burner, below 9 cm above the table, no part of an arm enters a 30 mm square round the igniter.
IGN_KEEP_X, IGN_KEEP_Y, IGN_KEEP_Z = (0.275, 0.305), (0.006, 0.036), 0.09
ARM_KEEP_OUT = [{"name": "igniter", "x": IGN_KEEP_X, "y": IGN_KEEP_Y, "z_top": TOP + IGN_KEEP_Z}]

# ---- the scripted demo: left arm only ------------------------------------------------------------------------------------
OPEN_GAP, CLOSE_GAP = 0.094, 0.060  # pad gaps (m): fully open, and the close command on a 75 or 76 mm part
HEAD_FELT, CAP_FELT = 0.070, 0.071  # a closed grip that stops wider than this holds the part
HEAD_GRIP, CAP_GRIP = 0.018, 0.019  # tool point above the part's underside: pad tips 5 and 6 mm above the ribs
CARRY_Z, CAP_CARRY_Z, HOVER_Z = 0.120, 0.130, 0.120
RELEASE_GAP = 0.002  # both parts are let go 2 mm above where they sit
CARRY_SPEED, LOWER_SPEED = 0.05, 0.01


def _finger(gap: float) -> float:
    """Finger joint target for a pad gap in metres (94 mm is fully open)."""
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def _half_turn(a: float) -> float:
    """The same jaw line (a jaw line repeats every 180 degrees) as the yaw nearest zero."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def _pt(xy, z):
    return np.array([xy[0], xy[1], TOP + z], dtype=float)


def _take(ctx, arm, xy, grip_z, yaw, felt):
    """Hover, descend, close and confirm from the finger gap; one retry 3 mm lower. True when the part is in hand."""
    hover, grasp = _pt(xy, HOVER_Z), _pt(xy, grip_z)
    yield from arm.move(hover, 2.5, _finger(OPEN_GAP), yaw=yaw)
    yield from arm.trim_to_measured(hover)
    yield from arm.move(grasp, 3.0, yaw=yaw)
    yield from arm.trim_to_measured(grasp, tol=0.0015)
    for attempt in range(2):
        yield from arm.grip(_finger(CLOSE_GAP), 0.8)
        yield from arm.hold(0.3)
        if arm.holding(felt):
            return True
        if attempt == 0:
            yield from arm.grip(_finger(OPEN_GAP), 0.5)
            yield from arm.move(arm.pos - np.array([0.0, 0.0, 0.003]), 0.5, yaw=yaw)
    yield from arm.grip(_finger(OPEN_GAP), 0.5)
    yield from arm.move(hover, 2.0, yaw=yaw)
    return False


def _centre(ctx, arm, name, target_xy, yaw, turn_fn=None):
    """With the part in hand over the burner, read its pose and shift the hand (and turn the wrist) until its axis is
    over ``target_xy()`` and, for the head, its slot points at the igniter. Up to four rounds. Offline nothing moves,
    so the replay keeps the nominal pose. Returns the wrist yaw it ended with."""
    for _ in range(4):
        if ctx.offline:
            break
        s = ctx.obj(name)
        err = np.asarray(target_xy(), dtype=float) - _axis(s)[:2]
        turn = float(np.clip(turn_fn(s), -0.35, 0.35)) if turn_fn else 0.0
        if float(np.linalg.norm(err)) < 0.0006 and abs(turn) < math.radians(0.7):
            break
        err = err * min(1.0, 0.012 / max(float(np.linalg.norm(err)), 1e-9))
        yaw += turn
        yield from arm.move(arm.pos + np.array([err[0], err[1], 0.0]), 0.6, yaw=yaw)
        yield from arm.hold(0.5)
    return yaw


def _descend(ctx, arm, name, spec, to_z, nominal_drop, yaw):
    """Lower at 1 cm/s until the part's underside is ``to_z`` above the table (read from its pose)."""
    drop = nominal_drop if ctx.offline else bottom(ctx.obj(name), spec) - (TOP + to_z)
    drop = float(np.clip(drop, 0.0, 0.09))
    if drop > 0.0002:
        yield from arm.move(arm.pos - np.array([0.0, 0.0, drop]), max(0.2, drop / LOWER_SPEED), yaw=yaw)
    yield from arm.hold(0.4)
    if not ctx.offline:  # the wrist sags a little under the part: take up what is left, at most 4 mm
        left = float(np.clip(bottom(ctx.obj(name), spec) - (TOP + to_z), 0.0, 0.004))
        if left > 0.0005:
            yield from arm.move(arm.pos - np.array([0.0, 0.0, left]), 0.5, yaw=yaw)
            yield from arm.hold(0.3)


def _set_down(ctx, arm, name, spec, rest_z, clear_z, from_z, yaw, target_xy, turn_fn=None):
    """From ``from_z`` (underside height) over the burner: centre, lower to 3 mm above ``clear_z`` (the top of what the
    part has to go over), centre again, lower to 2 mm above ``rest_z``, open and rise 6 cm. Returns the wrist yaw."""
    yaw = yield from _centre(ctx, arm, name, target_xy, yaw, turn_fn)
    yield from _descend(ctx, arm, name, spec, clear_z + 0.003, from_z - clear_z - 0.003, yaw)
    yaw = yield from _centre(ctx, arm, name, target_xy, yaw, turn_fn)
    yield from _descend(ctx, arm, name, spec, rest_z + RELEASE_GAP, clear_z + 0.003 - rest_z - RELEASE_GAP, yaw)
    yield from arm.grip(_finger(OPEN_GAP), 0.8)
    yield from arm.hold(0.3)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.06]), 1.5, yaw=yaw)
    return yaw


def _slot_turn(head) -> float:
    return _wrap(IGN_BEARING - slot_bearing(head))


def _cap_target(ctx):
    """The burner axis, pulled toward the seated head's own axis until it is within 1.5 mm of it: centred on the
    burner and well inside the lip's 3 mm fit."""
    if ctx.offline:
        return np.array(BURNER)
    b, h = np.array(BURNER), _axis(ctx.obj("burner_head"))[:2]
    d = float(np.linalg.norm(h - b))
    return b if d <= 0.0015 else b + (h - b) * (1.0 - 0.0015 / d)


demo = placeholder_demo


def demo_script(ctx):
    """The left arm takes the head off the drainer with its jaws clear of the slot, turns it over the burner until the
    slot points at the igniter, lowers it onto the base and checks that it sat flat; then it takes the cap, lowers it
    over the head's lip, checks it and parks. The right arm does not move."""
    A = ctx.left

    # 1) take the head, jaw line 90 degrees from its slot
    head = ctx.obj("burner_head")
    yaw = _half_turn(slot_bearing(head) - math.pi)
    got = yield from _take(ctx, A, _axis(head)[:2], RIB_TOP + HEAD_GRIP, yaw, HEAD_FELT)
    if not got:
        yield from A.home()
        return

    # 2) carry it over the burner, turning the wrist so the slot points at the igniter, and set it down
    yield from A.move(_pt(A.pos[:2], CARRY_Z), 2.0, yaw=yaw)
    place_yaw = yaw + _slot_turn(head)
    over = _pt(BURNER, CARRY_Z)
    yield from A.move(over, float(np.linalg.norm(over - A.pos)) / CARRY_SPEED, yaw=place_yaw)
    yield from A.hold(0.5)
    yield from A.trim_to_measured(over)
    place_yaw = yield from _set_down(ctx, A, "burner_head", HEAD, SEAT, SEAT + IGN_H, CARRY_Z - HEAD_GRIP, place_yaw,
                                     lambda: BURNER, _slot_turn)

    # 3) check the head: flat and down, or lift it, turn by the slot error and set it down again (once)
    yield from A.hold(0.5)
    bad = A.pessimistic if ctx.offline else not head_seated({"burner_head": ctx.obj("burner_head")})
    if bad:
        h = None if ctx.offline else ctx.obj("burner_head")
        xy = BURNER if h is None else _axis(h)[:2]
        low = SEAT if h is None else float(_axis(h)[2] - TOP)
        grasp = _pt(xy, low + HEAD_GRIP)
        yield from A.move(grasp, 1.8, yaw=place_yaw)
        yield from A.grip(_finger(CLOSE_GAP), 0.8)
        yield from A.move(_pt(xy, SEAT + IGN_H + 0.006 + HEAD_GRIP), 1.0, yaw=place_yaw)
        yield from A.hold(0.3)
        place_yaw = yield from _set_down(ctx, A, "burner_head", HEAD, SEAT, SEAT + IGN_H, SEAT + IGN_H + 0.006, place_yaw,
                                         lambda: BURNER, _slot_turn)

    # 4) take the cap, jaw line along x
    cap = ctx.obj("burner_cap")
    cap_yaw = -math.pi / 2
    got = yield from _take(ctx, A, _axis(cap)[:2], RIB_TOP + CAP_GRIP, cap_yaw, CAP_FELT)
    if not got:
        yield from A.home()
        return

    # 5) carry it over the burner and lower it over the head's lip
    yield from A.move(_pt(A.pos[:2], CAP_CARRY_Z), 2.0, yaw=cap_yaw)
    over = _pt(BURNER, CAP_CARRY_Z)
    yield from A.move(over, float(np.linalg.norm(over - A.pos)) / CARRY_SPEED, yaw=cap_yaw)
    yield from A.hold(0.5)
    yield from A.trim_to_measured(over)
    yield from _set_down(ctx, A, "burner_cap", CAP, SEAT + RIM_H, SEAT + HEAD_H, CAP_CARRY_Z - CAP_GRIP, cap_yaw,
                         lambda: _cap_target(ctx))

    # 6) check the cap: centred and flat, or lift it clear of the lip, recentre and set it down again (once)
    yield from A.hold(0.5)
    bad = A.pessimistic if ctx.offline else not cap_seated({"burner_cap": ctx.obj("burner_cap")})
    if bad:
        c = None if ctx.offline else ctx.obj("burner_cap")
        xy = BURNER if c is None else _axis(c)[:2]
        low = SEAT + RIM_H if c is None else float(_axis(c)[2] - TOP)
        yield from A.move(_pt(xy, low + CAP_GRIP), 1.8, yaw=cap_yaw)
        yield from A.grip(_finger(CLOSE_GAP), 0.8)
        yield from A.move(_pt(xy, SEAT + HEAD_H + 0.006 + CAP_GRIP), 1.0, yaw=cap_yaw)
        yield from A.hold(0.3)
        yield from _set_down(ctx, A, "burner_cap", CAP, SEAT + RIM_H, SEAT + HEAD_H, SEAT + HEAD_H + 0.006, cap_yaw,
                             lambda: _cap_target(ctx))

    # 7) park, gripper open
    yield from A.home()
