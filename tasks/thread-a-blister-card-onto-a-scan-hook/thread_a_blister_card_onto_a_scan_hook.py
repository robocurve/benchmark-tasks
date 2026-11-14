"""On the station table: a pegboard panel stands across the far side with one chrome double-wire scan hook pointing at
the robot. The hook's lower wire carries two carded fastener packs (yellow cards with an orange header and a blue
blister); its upper wire ends in a white price label plate with a red stripe that overhangs the lower wire's upturned
tip. To the right, a third identical card stands in a cut-down kraft shelf-ready case. The three cards are loose.

Requires -E object_jitter=0: the hung cards start resting on the wire.

Run it:  inspect-robots run --embodiment yam_arena -E task=thread_a_blister_card_onto_a_scan_hook.py \\
           -E object_jitter=0 --instruction "Hang the carded product from the case on the scan hook's lower wire,
           with the two cards already there." --scorer task_score ...
"""

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import mechkit  # noqa: E402
from belimits import tilt_deg  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Mech, box, cyl, hull, mass_props, on_reset, released, rod, sweep  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0  # the demo grips: leave time to open, withdraw and park after the first 1.0

CHROME, WHITE, RED = (0.86, 0.88, 0.90), (0.97, 0.97, 0.95), (0.85, 0.10, 0.10)
BOARD, HOLE = (0.80, 0.68, 0.50), (0.22, 0.16, 0.10)
YELLOW, ORANGE, BLUE = (0.98, 0.84, 0.10), (0.92, 0.38, 0.05), (0.12, 0.35, 0.90)
KRAFT = (0.62, 0.45, 0.27)

# ---- the scan hook (fixed): a stock 20 cm double-wire scan hook with a label holder, on pegboard ----------------------
P = TOP + 0.200  # height of the lower wire's axis
HOOK_Y = -0.030
WIRE_R = 0.004
ROOT_X, BEND_X, TIP_X, TIP_RISE = 0.400, 0.220, 0.200, 0.004  # lower wire: root, start of the upturn, tip centre
UPPER_Z = 0.044  # upper wire axis above the lower wire's
PLATE_X, PLATE_T, PLATE_W = 0.190, 0.004, 0.050  # label plate: front face x, thickness, width
PLATE_LOW, PLATE_HIGH = 0.021, 0.054  # label plate's lower and upper edges above the lower wire's axis
HOOK_POS = (ROOT_X, HOOK_Y, P)
PRONG = [(0.0, 0.0, 0.0), (BEND_X - ROOT_X, 0.0, 0.0), (TIP_X - ROOT_X, 0.0, TIP_RISE)]
HOOK_SHAPES = [
    box((0.0025, 0.0, 0.020), (0.005, 0.030, 0.060), CHROME),  # mount plate on the pegboard
    sweep((0, 0, 0), PRONG, WIRE_R, CHROME, ends="round"),  # lower wire with its upturned tip
    rod((0.0, 0.0, UPPER_Z), (PLATE_X + PLATE_T - ROOT_X, 0.0, UPPER_Z), WIRE_R, CHROME),  # upper wire
    box((PLATE_X + PLATE_T / 2 - ROOT_X, 0.0, (PLATE_LOW + PLATE_HIGH) / 2), (PLATE_T, PLATE_W, PLATE_HIGH - PLATE_LOW), WHITE),
    dataclasses.replace(
        box((PLATE_X - 0.0005 - ROOT_X, 0.0, PLATE_LOW + 0.006), (0.001, PLATE_W, 0.006), RED), collide=False),
]
HOOK = Mech("scan_hook", pos=HOOK_POS, shapes=HOOK_SHAPES, friction=0.5)

# Pegboard: 12 mm hardboard, 340 x 320 mm, with 6 mm holes at 25 mm pitch (drawn, not cut).
BOARD_X, BOARD_T, BOARD_Y, BOARD_H = 0.405, 0.012, (-0.200, 0.140), 0.320
_holes = [
    dataclasses.replace(box((-BOARD_T / 2 - 0.0003, y, z), (0.0006, 0.006, 0.006), HOLE), collide=False)
    for y in np.arange(-0.150, 0.151, 0.025) for z in np.arange(0.035, 0.300, 0.025)
]
PEGBOARD = Mech("pegboard", pos=(BOARD_X + BOARD_T / 2, sum(BOARD_Y) / 2, TOP),
                shapes=[box((0, 0, BOARD_H / 2), (BOARD_T, BOARD_Y[1] - BOARD_Y[0], BOARD_H), BOARD), *_holes], friction=0.5)

# ---- the carded product (three, free, 0.12 kg): a fastener blister on a euro-slot card --------------------------------
# Card frame: origin at the hang slot's centre in the face's mid plane, x out of the back of the card, y across,
# z up the card. The blister is on the front (-x).
CARD_W, CARD_H, CARD_T = 0.110, 0.170, 0.003
SLOT_W, SLOT_H, BRIDGE = 0.032, 0.020, 0.007
TOP_EDGE = SLOT_H / 2 + BRIDGE  # card's top edge above the slot centre (17 mm)
BOTTOM_EDGE = CARD_H - TOP_EDGE  # card's bottom edge below the slot centre (153 mm)
BL_W, BL_H, BL_D, BL_CH = 0.060, 0.080, 0.040, 0.004
BL_TOP, BL_BOT = 0.045, 0.125  # blister's top and bottom below the slot centre
CARD_MASS = 0.120


def _ry(th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def card_shapes(th: float = 0.0):
    """The card's pieces, turned ``th`` about its y axis through the slot centre (the simulator places objects with a
    yaw only, so a card that starts hanging at its rest tilt is built already tilted)."""
    r, q = _ry(th), (math.cos(th / 2), 0.0, math.sin(th / 2), 0.0)
    side = (CARD_W - SLOT_W) / 2
    low_h = BOTTOM_EDGE - SLOT_H / 2
    pieces = [
        ((0, 0, SLOT_H / 2 + BRIDGE / 2), (CARD_T, CARD_W, BRIDGE), ORANGE),  # the bridge over the slot
        ((0, -(SLOT_W + side) / 2, 0), (CARD_T, side, SLOT_H), ORANGE),
        ((0, (SLOT_W + side) / 2, 0), (CARD_T, side, SLOT_H), ORANGE),
        ((0, 0, -SLOT_H / 2 - low_h / 2), (CARD_T, CARD_W, low_h), YELLOW),
    ]
    out = [box(tuple(r @ np.array(at)), size, col, tilt=q) for at, size, col in pieces]
    x0, x1, x2 = -CARD_T / 2, -CARD_T / 2 - BL_D + BL_CH, -CARD_T / 2 - BL_D
    zc = -(BL_TOP + BL_BOT) / 2
    pts = [(x, sy * (BL_W / 2 - i), zc + sz * (BL_H / 2 - i)) for x, i in ((x0, 0.0), (x1, 0.0), (x2, BL_CH))
           for sy in (-1, 1) for sz in (-1, 1)]
    out.append(hull((0, 0, 0), [tuple(r @ np.array(p)) for p in pts], BLUE))
    return out


_VOL, CARD_COM, _ = mass_props(card_shapes(), CARD_MASS)
REST_GAP = 0.0005  # the slot's top edge starts this far above the wire
# Hanging, the slot's top edge rests on the wire and the centre of mass hangs under it: bottom toward the pegboard.
HANG_TILT = -math.atan2(-CARD_COM[0], SLOT_H / 2 - CARD_COM[2])
HANG_DROP = WIRE_R + REST_GAP - SLOT_H / 2  # slot centre relative to the wire axis, along the card's own z


def hung_origin(face_x: float, th: float = HANG_TILT) -> tuple:
    """World position of the slot centre of a card resting on the wire where its face crosses x = face_x."""
    return tuple(float(v) for v in np.array([face_x, HOOK_Y, P]) + _ry(th) @ np.array([0.0, 0.0, HANG_DROP]))


A_X, B_X = 0.365, 0.300  # where the two hung cards' faces cross the wire
CASE_IN_X, CASE_IN_Y, CASE_Y, WALL_T, WALL_H = (0.2055, 0.2515), 0.130, -0.170, 0.003, 0.040
C_FACE_X = CASE_IN_X[1] - CARD_T / 2  # card C stands with its back against the case's rear wall
C_POS = (C_FACE_X, CASE_Y, TOP + WALL_T + BOTTOM_EDGE)
BUILT_TILT = {"card_a": HANG_TILT, "card_b": HANG_TILT, "card_c": 0.0}
CARD_A = Mech("card_a", pos=hung_origin(A_X), mass=CARD_MASS, friction=0.5, shapes=card_shapes(HANG_TILT))
CARD_B = Mech("card_b", pos=hung_origin(B_X), mass=CARD_MASS, friction=0.5, shapes=card_shapes(HANG_TILT))
CARD_C = Mech("card_c", pos=C_POS, mass=CARD_MASS, friction=0.5, shapes=card_shapes(0.0))

# The shelf-ready case: a cut-down corrugated tray, 3 mm board, taped to the table.
_cx, _dx = sum(CASE_IN_X) / 2, CASE_IN_X[1] - CASE_IN_X[0]
CASE = Mech("case", pos=(_cx, CASE_Y, TOP), friction=0.5, shapes=[
    box((0, 0, WALL_T / 2), (_dx + 2 * WALL_T, CASE_IN_Y + 2 * WALL_T, WALL_T), KRAFT),
    box((-(_dx + WALL_T) / 2, 0, WALL_T + WALL_H / 2), (WALL_T, CASE_IN_Y + 2 * WALL_T, WALL_H), KRAFT),
    box(((_dx + WALL_T) / 2, 0, WALL_T + WALL_H / 2), (WALL_T, CASE_IN_Y + 2 * WALL_T, WALL_H), KRAFT),
    box((0, -(CASE_IN_Y + WALL_T) / 2, WALL_T + WALL_H / 2), (_dx, WALL_T, WALL_H), KRAFT),
    box((0, (CASE_IN_Y + WALL_T) / 2, WALL_T + WALL_H / 2), (_dx, WALL_T, WALL_H), KRAFT),
])

OBJECTS = [PEGBOARD, HOOK, CASE, CARD_A, CARD_B, CARD_C]

# ---- the cards' swing damping -----------------------------------------------------------------------------------------
# A card hanging on a wire is a pendulum. A real card stops within a few swings because its slot edge rubs on the wire;
# a rigid contact in PhysX rolls without loss and the card swings for minutes. The three card bodies therefore carry
# PhysX damping, written to their USD when the scene is authored (before the simulation starts). It applies to every
# policy alike and nothing in the demo touches it.
CARD_NAMES = ("card_a", "card_b", "card_c")
CARD_ANGULAR_DAMPING, CARD_LINEAR_DAMPING = 4.0, 1.0  # 1/s


def _damped_author(plain):
    def author(prim_path, cfg, *args, **kwargs):
        prim = plain(prim_path, cfg, *args, **kwargs)
        name = getattr(cfg, "mech_name", "")
        if name in CARD_NAMES:
            try:
                from pxr import Sdf

                body = prim.GetStage().GetPrimAtPath(f"{prim_path}/base")
                if "PhysxRigidBodyAPI" not in body.GetAppliedSchemas():
                    body.AddAppliedSchema("PhysxRigidBodyAPI")
                for attr, value in (("angularDamping", CARD_ANGULAR_DAMPING), ("linearDamping", CARD_LINEAR_DAMPING)):
                    body.CreateAttribute(f"physxRigidBody:{attr}", Sdf.ValueTypeNames.Float).Set(float(value))
                print(f"SCENE {name}: swing damping {CARD_ANGULAR_DAMPING:g}/s angular, {CARD_LINEAR_DAMPING:g}/s linear",
                      flush=True)
            except Exception as exc:  # noqa: BLE001 - an undamped card still runs; the demo waits for it to settle
                print(f"SCENE {name}: swing damping not applied: {exc!r}", flush=True)
        return prim

    author.card_damping = True
    return author


if not getattr(mechkit._author, "card_damping", False):
    mechkit._author = _damped_author(mechkit._author)

# ---- scoring ----------------------------------------------------------------------------------------------------------
UPRIGHT_DEG = 20.0  # a card on the wire must be within this of upright
FULL_DEPTH = 0.030  # card C's face this far behind the wire's tip: past the upturn, beside the stock
THREAD_DEPTH = 0.010  # card C counts as threaded once it is on the wire this deep
DROP = 0.030  # a hung card that drops this far was knocked off
OFF_STEPS = 10  # a hung card not On for this many control steps in a row (0.5 s) was knocked off: a card cannot leave
# its closed slot and come back, so a shorter miss is a swing past 20 degrees, not a card off the wire
SETTLE_STEPS = 20  # the first second after reset, while the hung cards settle, does not count toward OFF_STEPS
STILL_SPEED, REST_STEPS = 0.01, 20  # under 1 cm/s for 1 s
RELEASE_CLEAR = 0.03
FLOOR_BAND = 0.015  # card C lying on the table or in the case: its lowest point within this of the table top
OFFLINE_FINGERS_CLEAR = True  # what the release test returns with no simulator (the checker flips it for a held card)
SCORE_LOG = True  # one line when K latches and one whenever a gate changes, so a run's log shows which gate failed
WIRE_PTS = [np.array([TIP_X, HOOK_Y, P + TIP_RISE]), np.array([BEND_X, HOOK_Y, P]), np.array([ROOT_X, HOOK_Y, P])]

_L = {"K": False, "T": False, "still": 0, "seen": None, "step": 0, "off": {}, "said": None, "lines": 0}


@on_reset
def _clear(_env=None):
    _L.update(K=False, T=False, still=0, seen=None, step=0, off={}, said=None, lines=0)


def _say(text: str) -> None:
    if SCORE_LOG and _L["lines"] < 80:
        _L["lines"] += 1
        print(f"SCORE step {_L['step']}: {text}", flush=True)


def card_rot(state, name: str) -> np.ndarray:
    """The card's own axes in the world (the hung cards are built tilted, see card_shapes)."""
    return state.rotation() @ _ry(BUILT_TILT[name])


def tilt_from_upright(state, name: str) -> float:
    """Degrees between the card's long axis and the vertical."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(card_rot(state, name)[2, 2])))))


def on_wire(state, name: str):
    """Depth (m) of the card on the wire, or None when it is not on it. On: the wire's axis crosses the card's face
    inside its 32 x 20 mm slot and the card is within 20 degrees of upright. Depth: how far along the wire from the
    tip's centre the crossing is."""
    r, o = card_rot(state, name), np.asarray(state.origin, dtype=float)
    if tilt_from_upright(state, name) > UPRIGHT_DEG:
        return None
    run = 0.0
    for a, b in zip(WIRE_PTS[:-1], WIRE_PTS[1:]):
        la, lb = r.T @ (a - o), r.T @ (b - o)
        if (la[0] > 0.0) != (lb[0] > 0.0):
            s = la[0] / (la[0] - lb[0])
            hit = la + s * (lb - la)
            if abs(hit[1]) <= SLOT_W / 2 and abs(hit[2]) <= SLOT_H / 2:
                return run + s * float(np.linalg.norm(b - a))
            return None
        run += float(np.linalg.norm(b - a))
    return None


def _fingers_clear(card) -> bool:
    if mechkit._ENV["env"] is None:
        return OFFLINE_FINGERS_CLEAR
    return released(card, RELEASE_CLEAR)


def _update(objects) -> None:
    """Latch what happened this control step. yam_arena calls success() and score() on the same dict each step, so a
    dict already seen changes nothing."""
    if objects is _L["seen"]:
        return
    _L["seen"] = objects
    _L["step"] += 1
    for name, spec in (("card_a", CARD_A), ("card_b", CARD_B)):
        s = objects[name]
        off = on_wire(s, name) is None and _L["step"] > SETTLE_STEPS
        _L["off"][name] = _L["off"].get(name, 0) + 1 if off else 0
        why = None
        if s.origin[2] < spec.pos[2] - DROP:
            why = f"dropped {1000 * (spec.pos[2] - s.origin[2]):.0f} mm"
        elif _L["off"][name] >= OFF_STEPS:
            why = f"not On for {OFF_STEPS} steps, {tilt_from_upright(s, name):.0f} deg from upright"
        if why and not _L["K"]:
            _L["K"] = True
            _say(f"K latched: {name} {why}")
    d = on_wire(objects["card_c"], "card_c")
    if d is not None and d >= THREAD_DEPTH and not _L["T"]:
        _L["T"] = True
        _say(f"T latched: card_c On at depth {1000 * d:.0f} mm")
    cards = [objects[n] for n in CARD_NAMES]
    _L["still"] = _L["still"] + 1 if all(float(np.linalg.norm(c.lin_vel)) < STILL_SPEED for c in cards) else 0


def score(objects):
    """0 if a hung card was off the wire for 0.5 s or dropped 3 cm (K), any card is off the table, card C is still
    held or not still for 1 s, or C was never threaded. Otherwise 1.0 C on the wire 30 mm or more behind the tip; 0.5 C
    on the wire short of that; 0.25 C was threaded 10 mm deep at some time (T) but ends off the wire, lying on the
    table or in the case."""
    _update(objects)
    c = objects["card_c"]
    d = on_wire(c, "card_c")
    off_table = any(objects[n].origin[2] < TOP - 0.05 for n in CARD_NAMES)
    still, clear = _L["still"] >= REST_STEPS, _fingers_clear(c)
    if _L["K"] or off_table or not still or not clear:
        value = 0.0
    elif d is not None:
        value = 1.0 if d >= FULL_DEPTH else 0.5
    elif _L["T"] and c.low[2] <= TOP + FLOOR_BAND:
        value = 0.25
    else:
        value = 0.0
    gates = (value, _L["K"], _L["T"], off_table, still, clear, d is not None and d >= FULL_DEPTH, d is not None)
    if gates != _L["said"]:
        _L["said"] = gates
        speeds = ", ".join(f"{1000 * float(np.linalg.norm(objects[n].lin_vel)):.0f}" for n in CARD_NAMES)
        _say(f"score {value:g}: K {_L['K']}, T {_L['T']}, off table {off_table}, still {still} (counter "
             f"{_L['still']}, speeds A B C {speeds} mm/s), fingers clear {clear}, C depth "
             f"{'off the wire' if d is None else f'{1000 * d:.0f} mm'}")
    return value


def success(objects):
    return score(objects) == 1.0


# ---- the robot keeps off the hung stock and the pegboard --------------------------------------------------------------
# No link origin and no point tracked along either arm may be in or over the hung cards and the pegboard behind them
# (4 cm margin) below the top of the label plate. The label plate and upper wire are a height band, which the envelope's
# keep-out prism carries (LABEL_BOX): the hand has to pass under it, so it cannot be a floor-to-ceiling region here.
ARM_KEEP_OUT = [
    {"name": "hung stock and pegboard", "x": (0.275, 0.46), "y": (-0.125, 0.065), "z_top": P + PLATE_HIGH + 0.02},
]
LABEL_BOX = ((0.185, -0.060, P + 0.015), (0.405, 0.000, P + 0.060))  # upper wire and label plate plus 5 mm

# ---- the scripted demo ------------------------------------------------------------------------------------------------
LEAN = math.radians(30.0)
ROT = grip_rot(pitch=math.pi / 2 - LEAN)  # pointing away from the robot, leaned 30 degrees down, pads closing along y
AIM = ROT[:, 2]  # the gripper's approach direction
GRIP_BELOW = 0.075  # pad centre below the slot centre: on the blister's flanks, just above the centre of mass
CLAMP_X = -(CARD_T + BL_D) / 2  # pad centre from the face's mid plane: half way along the blister's depth
OPEN_GAP, CLOSE_CMD, HOLD_GAP = 0.090, 0.045, 0.052  # the blister is 60 mm across
THREAD_Z = 0.001  # slot centre above the wire axis while the tip passes through the slot
FRONT_X, HANG_X = 0.175, 0.242  # card face x in front of the tip, and where it is let go
TRANSIT_UP = 0.030
REST_Z = WIRE_R - SLOT_H / 2  # slot centre relative to the wire axis, along the card, with the card resting on the wire
GRIP_AT = np.array([CLAMP_X, 0.0, -GRIP_BELOW])  # pad centre in the card's frame
SETTLED_SPEED, SETTLED_STEPS, SETTLE_WAIT_S = 0.005, 30, 8.0
demo = placeholder_demo


def _finger(gap: float) -> float:
    """Finger joint target for a pad gap in metres (94 mm is fully open)."""
    return FINGER_OPEN * gap / (2.0 * abs(FINGER_OPEN))


def _slot(face_x: float, y: float, lift: float, th: float = 0.0) -> np.ndarray:
    """Slot centre of a card tilted ``th`` whose face crosses the wire's line at face_x, its slot centre ``lift``
    above the wire's axis along the card."""
    return np.array([face_x, y, P]) + _ry(th) @ np.array([0.0, 0.0, lift])


def _tool(slot, th: float = 0.0) -> np.ndarray:
    """Tool point (pad centre) that holds a card tilted ``th`` with its slot centre at ``slot``."""
    return np.asarray(slot, dtype=float) + _ry(th) @ GRIP_AT


def demo_script(ctx):
    R = ctx.right
    fix, th_fix = np.zeros(3), 0.0
    at, th_at = np.zeros(3), 0.0  # where the card's slot centre and tilt were last commanded

    def carry(slot, secs, th=0.0, gain=0.25, level=False):
        """Move the held card so its slot centre goes to ``slot`` and its tilt to ``th``, correcting the hand each
        step from the card's pose as the simulator reports it (the arm sags under the load; the window at the tip is
        6 mm). ``level`` also corrects the hand's pitch from the card's measured tilt."""
        nonlocal fix, th_fix, at, th_at
        o0, t0, o1 = at.copy(), th_at, np.asarray(slot, dtype=float)
        n = max(1, round(secs * 20))
        for i in range(1, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)
            want, th_s = o0 + s * (o1 - o0), t0 + s * (th - t0)
            if not R.offline:
                c = ctx.obj("card_c")
                err = want - np.asarray(c.origin, dtype=float)
                fix = np.clip(fix + gain * np.clip(err, -0.004, 0.004), -0.035, 0.035)
                if level:
                    rc = c.rotation()
                    th_err = th_s - math.atan2(float(rc[0, 2]), float(rc[2, 2]))
                    th_fix = float(np.clip(th_fix + 0.15 * np.clip(th_err, -0.02, 0.02), -0.14, 0.14))
            yield from R.move(_tool(want, th_s + th_fix) + fix, 0.05, rot=_ry(th_s + th_fix) @ ROT)
        at, th_at = o1, th

    # 1) Grip: from the front, over the case's front wall, a pad on each flank of the blister.
    c0 = np.asarray(ctx.obj("card_c").origin, dtype=float)
    grip = _tool(c0)
    pre = grip - AIM * 0.09
    yield from R.move((0.13, -0.20, TOP + 0.20), 2.0, _finger(OPEN_GAP), rot=ROT)
    yield from R.move(pre, 1.5, rot=ROT)
    yield from R.trim_to_measured(pre)
    off = R.pos - pre
    yield from R.move(grip + off, 2.0, rot=ROT)
    yield from R.grip(_finger(CLOSE_CMD), 0.8)
    yield from R.hold(0.3)
    if not R.holding(HOLD_GAP):
        yield from R.grip(_finger(OPEN_GAP), 0.5)
        yield from R.move(grip + off + AIM * 0.004, 0.5, rot=ROT)
        yield from R.grip(_finger(CLOSE_CMD), 0.8)
        yield from R.hold(0.3)
    fix, at = off.copy(), c0.copy()

    # 2) Lift straight up out of the case, back toward the robot, then across in front of the label plate, then down.
    high = THREAD_Z + TRANSIT_UP
    yield from carry((c0[0], c0[1], P + high), 2.5)
    yield from carry((FRONT_X, c0[1], P + high), 2.0)
    yield from carry(_slot(FRONT_X, HOOK_Y, high), 3.5)
    yield from carry(_slot(FRONT_X, HOOK_Y, THREAD_Z), 2.0)
    yield from carry(_slot(FRONT_X, HOOK_Y, THREAD_Z), 1.0)

    # 3) Thread: the slot level with the tip, the card's top edge 3 mm under the label plate; then back past the upturn.
    yield from carry(_slot(HANG_X - 0.010, HOOK_Y, THREAD_Z), 5.0)
    yield from carry(_slot(HANG_X, HOOK_Y, REST_Z + 0.003), 1.5)

    # 4) Let it down as it will hang: tip the card to the tilt its weight gives it on the wire (bottom toward the
    # board), lower the slot's top edge to 0.7 mm above the wire, and only then open. Let go upright, the card would
    # swing 10 degrees each way and strike card B.
    yield from carry(_slot(HANG_X, HOOK_Y, REST_Z + 0.003, HANG_TILT), 2.5, th=HANG_TILT, level=True)
    yield from carry(_slot(HANG_X, HOOK_Y, REST_Z + 0.0007, HANG_TILT), 1.0, th=HANG_TILT, level=True)
    yield from carry(_slot(HANG_X, HOOK_Y, REST_Z + 0.0007, HANG_TILT), 0.7, th=HANG_TILT, level=True)
    rot = R.rot.copy()
    yield from R.grip(_finger(OPEN_GAP), 1.2)
    yield from R.hold(0.8)

    # 5) Draw the hand straight back toward the robot under the label, park, and wait until the three cards are still.
    out = R.pos + np.array([-0.11, 0.0, 0.0])
    yield from R.move(out, 1.8, rot=rot)
    yield from R.move(out + np.array([-0.02, -0.10, 0.04]), 1.2, rot=ROT)
    yield from R.home()
    calm = 0
    for _ in range(round(SETTLE_WAIT_S * 20)):
        speeds = [float(np.linalg.norm(ctx.obj(n).lin_vel)) for n in CARD_NAMES]
        calm = calm + 1 if max(speeds) < SETTLED_SPEED else 0
        if calm >= SETTLED_STEPS:
            break
        yield from ctx.wait(0.05)
    # Stay parked through the 8 s that success() has to hold (the episode ends itself when it has).
    yield from ctx.wait(SUCCESS_HOLD_S + 1.0)
