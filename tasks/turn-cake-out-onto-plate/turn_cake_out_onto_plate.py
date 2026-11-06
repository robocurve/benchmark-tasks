"""On the station table, in front of the arms: a cake stand (a pale blue pedestal 10 cm tall) with a round cake pan on it.
The pan holds a baked cake, top up, and a flat plate lies upside down on the pan, resting on its flat rim. The pan has
two small ears on its rim, one towards each arm.

Task: turn the cake out onto the plate. Hold the plate and the pan together, one hand on each side, turn them over
together, set them down on the stand plate side down, and lift the pan off: the cake ends upside down on the plate.
Plate, pan and cake are loose. Held on one side only the stack hangs off one grip and comes apart, and the plate and
the pan are too wide for one hand: the two hands clamp it from both sides while they turn it.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- stand: fixed pale blue cylinder, radius 6.5 cm, 10 cm tall, centred at (0.30, 0).
- pan: a round pan 14 cm across inside and 3.6 cm deep, 250 g, with a flat rim 2.5 cm wide (out to 9.8 cm from its
  centre) and two ears on the rim reaching 13 cm out along the left-right line.
- cake: 12.8 cm across and 2.8 cm tall, 160 g, a dark crust on its top.
- plate: a flat white disc 21 cm across and 6 mm thick, 220 g, lying on the pan's rim.

Full score: the plate on the stand, flat; the cake on the plate upside down (crust down, turned more than 160 degrees)
with its middle within 2.5 cm of the plate's; the pan lifted off and away from the cake (its middle 13 cm or more from
the cake's); everything still for 2 s.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, allow_matrix_grips, on_reset, plate, rotation  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "best"  # partial credit: the best reached during the run

# -- objects (each body's origin is the centre of its underside, axis up) --------------------------------------------
STAND_XY, STAND_R, STAND_H = (0.30, 0.0), 0.065, 0.10
PAN_R_IN, PAN_T, PAN_FLOOR, PAN_H = 0.070, 0.003, 0.004, 0.036  # inside radius, wall, floor, height to the rim's top
RIM_IN, RIM_OUT, RIM_T = PAN_R_IN + PAN_T, 0.098, 0.003  # the flat rim (flange) at the top of the wall
EAR_OUT, EAR_W = 0.130, 0.030  # two ears on the rim, along +-y
PAN_KG = 0.25
CAKE_R, CAKE_H, CAKE_KG = 0.064, 0.028, 0.16
PLATE_R, PLATE_T, PLATE_KG = 0.105, 0.006, 0.22

PAN_GREY, CRUST, CAKE_C, WHITE, STAND_C = (0.62, 0.63, 0.66), (0.36, 0.20, 0.10), (0.80, 0.62, 0.38), (0.94, 0.94, 0.92), \
    (0.52, 0.62, 0.72)

STAND_TOP = TOP + STAND_H


def _pan() -> Body:
    n = 24
    shapes = [CylShape((0.0, 0.0, PAN_FLOOR / 2), PAN_R_IN + PAN_T, PAN_FLOOR, PAN_GREY)]
    r_wall = PAN_R_IN + PAN_T / 2
    z_rim = PAN_H - RIM_T / 2
    for k in range(n):
        a = 2 * math.pi * k / n
        u = (math.cos(a), math.sin(a), 0.0)
        shapes.append(plate((r_wall * u[0], r_wall * u[1], PAN_FLOOR), (r_wall * u[0], r_wall * u[1], PAN_H - RIM_T),
                            2 * r_wall * math.tan(math.pi / n) + 0.001, PAN_T, u, PAN_GREY))
        shapes.append(plate((RIM_IN * u[0], RIM_IN * u[1], z_rim), (RIM_OUT * u[0], RIM_OUT * u[1], z_rim),
                            2 * RIM_OUT * math.tan(math.pi / n) + 0.001, RIM_T, (0.0, 0.0, 1.0), PAN_GREY))
    for s in (1, -1):
        shapes.append(BoxShape((0.0, s * (RIM_OUT + EAR_OUT) / 2, z_rim), (EAR_W, EAR_OUT - RIM_OUT + 0.004, RIM_T),
                               PAN_GREY))
    return Body.build("pan", pos=(*STAND_XY, STAND_TOP + 0.0005), mass=PAN_KG, shapes=shapes, friction=(0.6, 0.5),
                      color=PAN_GREY)


def _cake() -> Body:
    shapes = [CylShape((0.0, 0.0, CAKE_H / 2), CAKE_R, CAKE_H, CAKE_C),
              CylShape((0.0, 0.0, CAKE_H - 0.0015), CAKE_R - 0.001, 0.0035, CRUST, collide=False)]
    return Body.build("cake", pos=(*STAND_XY, STAND_TOP + PAN_FLOOR + 0.001), mass=CAKE_KG, shapes=shapes,
                      friction=(0.7, 0.6), color=CAKE_C)


def _plate() -> Body:
    shapes = [CylShape((0.0, 0.0, PLATE_T / 2), PLATE_R, PLATE_T, WHITE)]
    return Body.build("plate", pos=(*STAND_XY, STAND_TOP + PAN_H + 0.001), mass=PLATE_KG, shapes=shapes,
                      friction=(0.6, 0.5), color=WHITE)


PAN, CAKE, PLATE = _pan(), _cake(), _plate()
OBJECTS = [Cylinder("cake_stand", pos=(*STAND_XY, TOP + STAND_H / 2), radius=STAND_R, height=STAND_H, color=STAND_C),
           PAN, CAKE, PLATE]

# -- scoring --------------------------------------------------------------------------------------------------------
TURNED = math.radians(160)  # the cake's axis turned at least this far from up: crust down
FLAT = math.radians(10)
ON_PLATE_XY = 0.025
ON_STAND_XY = 0.03
PAN_AWAY = 0.13
SITS = 0.008  # the cake's lowest point at most this far above the plate's top

_DEBUG = os.environ.get("CAKE_DEBUG") == "1"
_EP = {"step": 0}


@on_reset
def _reset():
    _EP["step"] = 0


def _tilt(s) -> float:
    return float(math.acos(max(-1.0, min(1.0, rotation(s.quat_xyzw)[2, 2]))))


def _plate_on_stand(o) -> bool:
    p = o["plate"]
    flat = abs(rotation(p.quat_xyzw)[2, 2]) >= math.cos(FLAT)
    centred = math.hypot(p.pos[0] - STAND_XY[0], p.pos[1] - STAND_XY[1]) <= ON_STAND_XY
    on = STAND_TOP - 0.004 <= p.low[2] <= STAND_TOP + 0.006
    return bool(flat and centred and on)


def _cake_turned_out(o) -> bool:
    c, p = o["cake"], o["plate"]
    turned = _tilt(c) >= TURNED
    mid = c.pos + rotation(c.quat_xyzw) @ np.array([0.0, 0.0, CAKE_H / 2])
    centred = math.hypot(mid[0] - p.pos[0], mid[1] - p.pos[1]) <= ON_PLATE_XY
    sits = 0.0 <= c.low[2] - p.high[2] + 0.002 <= SITS
    return bool(turned and centred and sits)


def _pan_away(o) -> bool:
    c, pan = o["cake"], o["pan"]
    return bool(math.hypot(pan.pos[0] - c.pos[0], pan.pos[1] - c.pos[1]) >= PAN_AWAY)


def _still(o) -> bool:
    return all(o[n].resting() for n in ("pan", "cake", "plate"))


def _debug(o) -> None:
    _EP["step"] += 1
    if _EP["step"] % 10 != 1:
        return
    parts = []
    for n in ("pan", "cake", "plate"):
        s = o[n]
        parts.append(f"{n}=({s.pos[0]:.3f},{s.pos[1]:.3f},{s.pos[2] - TOP:.3f}) tilt={math.degrees(_tilt(s)):.0f} "
                     f"low={s.low[2] - TOP:.3f}")
    print(f"DBG step={_EP['step']} {' | '.join(parts)} on_stand={_plate_on_stand(o)} out={_cake_turned_out(o)} "
          f"away={_pan_away(o)} still={_still(o)}", flush=True)


def success(objects):
    if _DEBUG:
        _debug(objects)
    return _plate_on_stand(objects) and _cake_turned_out(objects) and _pan_away(objects) and _still(objects)


def score(objects):
    """Partial credit (success scores 1.0; the best over the run counts): 0.5 when the cake lies upside down on the
    plate on the stand but the pan is still over it; 0.25 when the cake has been turned over (more than 160 degrees)
    while off the table; 0 otherwise."""
    if _plate_on_stand(objects) and _cake_turned_out(objects) and _still(objects):
        return 0.5
    c = objects["cake"]
    if _tilt(c) >= TURNED and c.low[2] >= TOP + 0.05:
        return 0.25
    return 0.0


# -- scripted demo --------------------------------------------------------------------------------------------------
# Both hands come in level from their own sides with the fingers upright and clamp the plate and the pan's rim
# together, one hand on each side; they lift the stack, turn it over together about the line between them, lower it
# plate side down onto the stand and let go. Then each takes one of the pan's ears from above, they lift the pan off
# the cake together, carry it towards the arms and set it down upside down on the table.


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


SIDE = {"left": _rx(-math.pi / 2) @ grip_rotation(-math.pi / 2),  # pointing -y, fingers closing front and back
        "right": _rx(math.pi / 2) @ grip_rotation(math.pi / 2)}  # pointing +y
SIDE_ARM = {"left": 1, "right": -1}
TOPDOWN = {"left": grip_rotation(-math.pi / 2), "right": grip_rotation(math.pi / 2)}  # fingers closing along x


def _clamp_rot(arm: str, turn: float) -> np.ndarray:
    """Side grip with the fingers upright (closing along z), turned ``turn`` about world y: -90 deg at the start,
    +90 deg once the stack is upside down."""
    return _ry(turn) @ SIDE[arm]


GRIP_Y = float(os.environ.get("CAKE_GRIP_Y", "0.105"))  # TCPs this far either side of the centre line
OUT_Y = 0.12  # beyond the plate (radius 10.5 cm); a side grip reaches no further out than ~13 cm
BESIDE_X = 0.035  # wait this far in front of the ear (3 cm wide), so the fingers come down clear of it
STACK_MID = PAN_H + 0.0015  # the plate / rim interface, above the pan's underside
CLAMP = float(os.environ.get("CAKE_CLAMP", "-0.0025"))  # 5 mm between the fingers round the 9 mm stack
OPEN_SIDE = -0.02  # 4 cm apart: enough round the stack, small sweep
FLIP_Z = STAND_TOP + STACK_MID + float(os.environ.get("CAKE_LIFT", "0.08"))  # turn it over here: clear of the stand
# Turned over, the plate is below the stack's middle (6 mm thick, the middle 1.5 mm above its top): set down when its
# underside reaches the stand, pressing 1 mm.
SET_Z = STAND_TOP + 0.0045 - 0.001 + float(os.environ.get("CAKE_SET_DZ", "0.0"))
PLATE_TOP_DOWN = STAND_TOP + PLATE_T  # the plate's top once it lies on the stand: the pan's rim and ears rest on it
EAR_Y = 0.118  # grip the ears here: outside the plate (radius 10.5 cm)
EAR_GRIP = float(os.environ.get("CAKE_EAR_GRIP", "-0.011"))  # 2.2 cm between the fingers round the 3 cm ear
TIP = 0.013
# Set the pan down this far forward: its rim (9.8 cm) then passes 1.4 cm short of the plate's edge on the way down.
PARK_X = float(os.environ.get("CAKE_PARK_X", "0.08"))


def _good_ik(arm: str, pos, rot, seed=None):
    import itertools

    from yam_arena import ik, station

    seeds = [seed] if seed is not None else []
    seeds += list(itertools.product((0.0,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2), (-1.5, 0.0, 1.5)))
    best = None
    for sd in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if seed is not None and best is not None:
                break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _joint_move(arm: str, q0, q1, seconds: float, steps: int, finger: float, label: str) -> list[Waypoint]:
    """Waypoints along a straight line in joint space (big reorientations stay on one IK branch)."""
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _arm_plan(arm: str) -> list[Waypoint]:
    from yam_arena.constants import READY

    s = SIDE_ARM[arm]
    x0, _ = STAND_XY
    grip = np.array([x0, s * GRIP_Y, STAND_TOP + STACK_MID])
    beside = np.array([x0, s * OUT_Y, STAND_TOP + STACK_MID])
    wait = beside + [BESIDE_X, 0.0, 0.0]
    up = np.array([x0, s * GRIP_Y, FLIP_Z])
    down = np.array([x0, s * GRIP_Y, SET_Z])
    start = _clamp_rot(arm, -math.pi / 2)
    turned = _clamp_rot(arm, math.pi / 2)
    plan = _joint_move(arm, np.array(READY, float), _good_ik(arm, wait, start), 3.0, 15, OPEN_SIDE,
                       "level, beside the stack, fingers upright")
    plan += [
        _w(1.2, beside, OPEN_SIDE, "slide along, the ear between the fingers", start),
        _w(1.2, grip, OPEN_SIDE, "slide in round the plate and the pan's rim", start),
        _w(1.0, grip, CLAMP, "clamp them together", start),
        _w(0.4, grip, CLAMP, "steady", start),
        _w(1.5, up, CLAMP, "lift the stack off the stand", start),
    ]
    for k, deg in enumerate(range(-90 + 15, 91, 15)):
        plan.append(_w(4.0 / 12, up, CLAMP, "turn it over together", _clamp_rot(arm, math.radians(deg))))
    plan += [
        _w(0.6, up, CLAMP, "steady", turned),
        _w(2.0, down + [0.0, 0.0, 0.02], CLAMP, "lower it over the stand, plate side down", turned),
        _w(1.2, down, CLAMP, "set it down", turned),
        _w(0.6, down, CLAMP, "settle", turned),
        _w(0.8, down, OPEN_SIDE, "let go", turned),
        _w(1.0, down + [0.0, s * (OUT_Y - GRIP_Y), 0.0], OPEN_SIDE, "slide out", turned),
        _w(1.0, down + [BESIDE_X, s * (OUT_Y - GRIP_Y), 0.0], OPEN_SIDE, "slide along, clear of the ear", turned),
        _w(1.0, down + [BESIDE_X, s * (OUT_Y - GRIP_Y), 0.06], OPEN_SIDE, "rise", turned),
    ]
    # over the ear, from above: a joint-space move from the side grip to a downward grip
    ear_above = np.array([x0, s * EAR_Y, PLATE_TOP_DOWN + 0.07])
    q_out = _good_ik(arm, down + [BESIDE_X, s * (OUT_Y - GRIP_Y), 0.06], turned)
    q_ear = _good_ik(arm, ear_above, TOPDOWN[arm])
    plan += _joint_move(arm, q_out, q_ear, 2.4, 12, FINGER_OPEN, "turn the hand to come down on the ear")
    ear_z = PLATE_TOP_DOWN - 0.002 + TIP  # the pads' tips 2 mm below the ear (it overhangs the plate's edge)
    ear = np.array([x0, s * EAR_Y, ear_z])
    lifted = ear + [0.0, 0.0, 0.07]
    park = np.array([PARK_X, s * EAR_Y, TOP + 0.010 - 0.002 + TIP])  # the rim 1 cm above the table, then let go
    plan += [
        _w(1.2, ear, FINGER_OPEN, "down round the ear", TOPDOWN[arm]),
        _w(1.0, ear, EAR_GRIP, "grip the ear", TOPDOWN[arm]),
        _w(0.4, ear, EAR_GRIP, "steady", TOPDOWN[arm]),
        _w(1.5, lifted, EAR_GRIP, "lift the pan off the cake", TOPDOWN[arm]),
        _w(2.0, [PARK_X, s * EAR_Y, lifted[2]], EAR_GRIP, "carry it towards the arms, over the plate", TOPDOWN[arm]),
        _w(1.5, park, EAR_GRIP, "lower it past the plate's edge", TOPDOWN[arm]),
        _w(0.8, park, FINGER_OPEN, "let go", TOPDOWN[arm]),
        _w(1.0, park + [0.0, 0.0, 0.06], FINGER_OPEN, "rise", TOPDOWN[arm]),
    ]
    return plan


def demo():
    return {"left": _arm_plan("left"), "right": _arm_plan("right")}
