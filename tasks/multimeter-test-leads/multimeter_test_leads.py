"""On the station table, in front of the right arm: a dark grey bench power supply (24 x 20 x 10 cm) with a yellow
handheld multimeter (17.7 x 8.5 x 4.4 cm, 120 g) lying face up on it, its jack end toward the robot, flush with the
supply's front. The meter has three 4 mm jacks in a row across that end, 2 cm apart: from the robot's left, A (the
10 A current jack), COM and V ohm. In front of the supply, on a blue-grey antistatic mat, lie two test leads side by
side, pins toward the robot: a black one and, further right, a red one. Each is a plug (4 cm body, 1 cm across)
with a 4 mm pin 18 mm long and a 10 cm rigid stub of cable.

The task: plug the black lead into COM and the red lead into V ohm. The A jack is the trap: a lead in it is the
wrong set-up for measuring voltage, the classic way to blow a meter's fuse.

The meter is NVIDIA's SimReady ``testingTool_multimeter_c01`` (CC-BY 4.0, loaded by ``simready.py``). Its mesh
collider has no holes, so it collides as boxes instead, with a square 4.3 mm bore 20 mm deep at each jack and a
1 mm chamfer at the mouth (``METER_BOXES``). The leads are ``leads.py`` bodies: a plug box with box colliders for
the pin and the stub. The power supply's and the leads' looks are modelled in Blender (``looks/make_looks.py``).

Run it:  inspect-robots run --embodiment yam_arena -E task=multimeter_test_leads.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from leads import Lead, lead  # noqa: E402
from looks import dress  # noqa: E402
from simready import SimReady  # noqa: E402
from wrist import TiltWaypoint  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the multimeter, in its file's frame (metres): x from the display end to the jack end, z up from its back -------
# Measured on the mesh: its bounds, the face around the jacks, and the jacks' centres.
METER_LO = (-0.07977, -0.04241, 0.000425)
METER_HI = (0.09689, 0.04241, 0.043535)
METER_CENTROID = tuple((a + b) / 2 for a, b in zip(METER_LO, METER_HI))  # what the scorer's pos is
FACE = 0.0369  # the face around the jacks
JACK_X = 0.0724
JACK_Y = {"a": -0.0207, "com": -0.0005, "v_ohm": 0.0199}  # A on the -y side, V ohm on the +y side
BORE = 0.0043  # the bore's side, a real 4 mm jack's: a 4 mm pin with 0.15 mm of play each side
CHAMFER = 0.001  # at the bore's mouth, 45 degrees
BORE_DEPTH = 0.020
CELL = 0.0065  # half the square around each jack that holds its bore
BLOCK_END = 0.0935  # where the jack block ends, short of the rounded end


def _box(lo, hi, turn=None):
    centre = tuple((a + b) / 2 for a, b in zip(lo, hi))
    size = tuple(b - a for a, b in zip(lo, hi))
    return (centre, size) if turn is None else (centre, size, turn)


def meter_boxes(bore: float = BORE, chamfer: float = CHAMFER) -> list[tuple]:
    """The meter's colliders: a solid body, and around each jack a square bore ``bore`` wide and 20 mm deep whose
    mouth is chamfered ``chamfer`` deep at 45 degrees (four wedges, a pyramid funnel)."""
    floor, h, c, p = FACE - BORE_DEPTH, bore / 2, chamfer, CELL
    y_lo, y_hi, z0 = -0.042, 0.042, 0.0006
    ys = sorted(JACK_Y.values())
    boxes = [
        _box((-0.079, y_lo, z0), (JACK_X - p, y_hi, FACE)),  # the body, up to the jacks
        _box((JACK_X - p, y_lo, z0), (BLOCK_END, y_hi, floor)),  # under the bores
        _box((JACK_X + p, y_lo, floor), (BLOCK_END, y_hi, FACE)),  # between the bores and the end
    ]
    edges = [y_lo, *[e for y in ys for e in (y - p, y + p)], y_hi]
    for a, b in zip(edges[0::2], edges[1::2]):  # between the jacks' squares
        boxes.append(_box((JACK_X - p, a, floor), (JACK_X + p, b, FACE)))
    x = (JACK_X - p, JACK_X - h - c, JACK_X - h, JACK_X + h, JACK_X + h + c, JACK_X + p)
    for yj in ys:
        y = (yj - p, yj - h - c, yj - h, yj + h, yj + h + c, yj + p)
        boxes += [
            # the outer ring, up to the face
            _box((x[0], y[0], floor), (x[1], y[5], FACE)),
            _box((x[4], y[0], floor), (x[5], y[5], FACE)),
            _box((x[1], y[0], floor), (x[4], y[1], FACE)),
            _box((x[1], y[4], floor), (x[4], y[5], FACE)),
            # the inner ring, up to the chamfer's foot
            _box((x[1], y[1], floor), (x[2], y[4], FACE - c)),
            _box((x[3], y[1], floor), (x[4], y[4], FACE - c)),
            _box((x[2], y[1], floor), (x[3], y[2], FACE - c)),
            _box((x[2], y[3], floor), (x[3], y[4], FACE - c)),
        ]
        side = c * math.sqrt(2)  # a square bar turned 45 degrees: one face is the chamfer, the rest is in the solid
        for sx in (-1, 1):
            boxes.append(((JACK_X + sx * (h + c), yj, FACE - c), (side, bore + 2 * c, side), (0.0, 45.0, 0.0)))
        for sy in (-1, 1):
            boxes.append(((JACK_X, yj + sy * (h + c), FACE - c), (bore + 2 * c, side, side), (45.0, 0.0, 0.0)))
    return boxes


METER_BOXES = tuple(meter_boxes())

# -- the bench power supply: fixed, its front face toward the robot ---------------------------------------------------
PSU_SIZE = (0.24, 0.20, 0.10)
PSU_FRONT = 0.375
PSU_Y = -0.19
PSU_TOP = TOP + PSU_SIZE[2]
PSU = Box("power_supply", pos=(PSU_FRONT + PSU_SIZE[0] / 2, PSU_Y, TOP + PSU_SIZE[2] / 2), size=PSU_SIZE,
          color=(0.16, 0.17, 0.18))

# -- the multimeter: on the supply, turned so its jack end faces the robot, COM at (0.40, -0.19) ----------------------
METER_YAW = math.pi
COM_XY = (0.40, -0.19)
METER_XY = (COM_XY[0] + JACK_X, COM_XY[1] + JACK_Y["com"])  # the file's origin (turned 180 degrees)
METER_MASS = 0.12
METER = SimReady("multimeter", pos=(*METER_XY, PSU_TOP), size=(0.177, 0.085, 0.044), yaw=METER_YAW,
                 mass=METER_MASS, package="testingTool_multimeter_c01", usd="sm_testingTool_multimeter_c01_01.usd",
                 boxes=METER_BOXES)
FACE_Z = PSU_TOP + FACE  # the face around the jacks, as the meter lies at the start


def jack_xy(name: str) -> tuple[float, float]:
    """A jack's centre on the table, as the meter lies at the start."""
    return (METER_XY[0] - JACK_X, METER_XY[1] - JACK_Y[name])


# -- the test leads: on the mat, pins toward the robot ----------------------------------------------------------------
MAT_SIZE = (0.20, 0.17, 0.003)
MAT_XY = (0.26, -0.30)
MAT_TOP = TOP + MAT_SIZE[2]
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=(0.36, 0.45, 0.55))
PLUG = (0.040, 0.010, 0.010)  # the plug body: length, width, height
LEAD_MASS = 0.015
LEAD_X = 0.22  # the plug bodies' centres
BLACK_Y, RED_Y = -0.26, -0.33
BLACK = lead(Lead("black_lead", pos=(LEAD_X, BLACK_Y, MAT_TOP + PLUG[2] / 2), size=PLUG, mass=LEAD_MASS,
                  color=(0.04, 0.04, 0.045)))
RED = lead(Lead("red_lead", pos=(LEAD_X, RED_Y, MAT_TOP + PLUG[2] / 2), size=PLUG, mass=LEAD_MASS,
                color=(0.80, 0.06, 0.05)))
LEADS = {"black_lead": "com", "red_lead": "v_ohm"}  # each lead's correct jack

OBJECTS = [PSU, METER, MAT, BLACK, RED]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged.
LOOKS = {"power_supply": "power_supply.usdc", "black_lead": "black_lead.usdc", "red_lead": "red_lead.usdc"}
dress(LOOKS)

# -- scoring ---------------------------------------------------------------------------------------------------------
FULL_DEPTH = 0.012  # m of pin inside its jack for full credit (the pin is 18 mm)
PART_DEPTH = 0.005
TRAP_DEPTH = 0.003  # a pin this far into the A jack is in it
IN_BORE = BORE / 2 + 0.001  # m: the pin's tip this close to a jack's axis is in that jack
OFF_TABLE_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tip(objects, name: str) -> np.ndarray:
    """The lead's pin tip in the world."""
    s, spec = objects[name], {"black_lead": BLACK, "red_lead": RED}[name]
    return s.pos + _rot(s.quat_xyzw) @ (np.array([spec.tip, 0.0, 0.0]) - np.asarray(spec.centroid))


def depth_in(objects, name: str, jack: str) -> float:
    """How far the lead's pin tip is below the face, inside the jack (m), or 0 if the tip isn't in that jack."""
    m = objects["multimeter"]
    to_meter = _rot(m.quat_xyzw).T
    local = to_meter @ (tip(objects, name) - m.pos) + np.asarray(METER_CENTROID)
    lateral = math.hypot(local[0] - JACK_X, local[1] - JACK_Y[jack])
    if lateral > IN_BORE:
        return 0.0
    return float(max(0.0, FACE - local[2]))


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(objects[n].pos)) and np.all(np.isfinite(objects[n].quat_xyzw))
               for n in ("multimeter", *LEADS))


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return any(objects[n].pos[2] < TOP + OFF_TABLE_Z for n in ("multimeter", *LEADS))


def in_trap(objects) -> bool:
    """A lead's pin is at least 3 mm into the A jack."""
    return any(depth_in(objects, n, "a") >= TRAP_DEPTH for n in LEADS)


def lead_credit(objects, name: str) -> float:
    d = depth_in(objects, name, LEADS[name])
    return 0.5 if d >= FULL_DEPTH else 0.25 if d >= PART_DEPTH else 0.0


def success(objects) -> bool:
    """Both pins at least 12 mm into their jacks (black in COM, red in V ohm), the leads still, none in A."""
    if _violation(objects) or in_trap(objects):
        return False
    return all(lead_credit(objects, n) == 0.5 and objects[n].resting(max_speed=0.02) for n in LEADS)


def score(objects) -> float:
    """Per lead, 0.5 when its pin is at least 12 mm into its jack (black in COM, red in V ohm), 0.25 at 5 mm. 0 if
    any pin is 3 mm or more into the A jack, or the meter or a lead fell off the table."""
    if _violation(objects) or in_trap(objects):
        return 0.0
    return float(sum(lead_credit(objects, n) for n in LEADS))


def distance(objects) -> float:
    """The pins' tips' summed distance from where they sit fully in."""
    m = objects["multimeter"]
    rot = _rot(m.quat_xyzw)
    total = 0.0
    for name, jack in LEADS.items():
        seat = m.pos + rot @ (np.array([JACK_X, JACK_Y[jack], FACE - 0.018]) - np.asarray(METER_CENTROID))
        total += float(np.linalg.norm(tip(objects, name) - seat))
    return total


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm only: the meter lies still on the supply while a pin goes in from above. For each lead: a straight-down
# grip across the plug body 1 cm from its back, the pad tips 1 mm above the mat, squeezing 25 N. Lift, tilt the grip
# forward 90 degrees (the pin turns from pointing at the robot to pointing down), go over the jack, ease the grip to
# 5 N so the plug can line itself up with the bore, lower the pin in until the body sits on the face, open the
# fingers a little, back off. Black into COM first, then red into V ohm (its open fingers then clear the black plug
# beside it). Numbers are TCP positions.
FWD = math.pi / 2
GRIP_BACK = 0.010  # m: the TCP this far from the plug body's centre, toward its back
PICK_Z = MAT_TOP + 0.001 + PAD_TIP_PAST_TCP  # pad tips 1 mm above the mat
AHEAD = PICK_Z - (MAT_TOP + PLUG[2] / 2)  # the plug's axis is this far past the TCP along the approach (9 mm)
ABOVE_BODY = PLUG[0] / 2 + GRIP_BACK  # the TCP above the body's front (pin) end once the pin points down: 3 cm
SEAT_Z = FACE_Z + ABOVE_BODY  # the TCP with the body sitting on the face
# m past seated: the lightly held plug slips up in the fingers while the pin pauses at the mouth, so the grip goes on
# down past seated and the pin still goes all the way in
PRESS = 0.004
OVER = 0.035  # m: the pin's tip this far above the face before it goes in
GRIP = -(PLUG[1] / 2 - 0.005)  # finger target: 5 mm past the body's sides (closed), a 25 N squeeze to carry it
GRIP_LIGHT = -(PLUG[1] / 2 - 0.001)  # 1 mm past, 5 N: held lightly, the plug lines itself up with the bore
FINGER_WIDE = -0.025  # 5 cm open, for the pick
FINGER_LOOSE = -(PLUG[1] / 2 + 0.002)  # 2 mm clear of the body, short of the plug beside it
HOVER_Z = MAT_TOP + 0.08
TURN_Z = TOP + 0.20
BACK_OFF = 0.035  # m back along -x after letting go: the fingers clear the plug
AWAY = (0.28, -0.26, TOP + 0.20)
# The softly controlled arm droops a little with the pin held down: the plug arrives tilted 2.4 degrees to the right
# and 1.3 degrees toward the robot (measured in the simulator, the same at both jacks). Wedged at that tilt, a pin
# jams in a 4.3 mm bore, so the grip is sent turned that much the other way, and the pin goes in straight. AIM is
# what is left to correct in position.
TRIM_PITCH = math.radians(1.3)
TRIM_ROLL = math.radians(2.4)
AIM = (0.0, 0.0)
BLACK_JACK = "com"  # where the demo plugs each lead (the ablations change these)
RED_JACK = "v_ohm"


def _insert(lead_y: float, jack: str) -> list[TiltWaypoint]:
    def w(duration, pos, finger, label, pitch=0.0):
        trim = pitch > 0
        return TiltWaypoint(duration, pos, finger, label, pitch=pitch + TRIM_PITCH * trim, roll=TRIM_ROLL * trim)

    jx, jy = jack_xy(jack)
    pick = (LEAD_X + GRIP_BACK, lead_y)
    tx, jy = jx - AHEAD + AIM[0], jy + AIM[1]  # the TCP, so the plug's axis is over the jack
    over_z = SEAT_Z + RED.pin_length + OVER  # the pin's tip OVER above the face
    return [
        w(2.0, (*pick, HOVER_Z), FINGER_WIDE, "over the plug"),
        w(1.2, (*pick, PICK_Z), FINGER_WIDE, "down to the plug"),
        w(1.2, (*pick, PICK_Z), GRIP, "grip the plug"),
        w(1.0, (*pick, HOVER_Z), GRIP, "lift"),
        w(3.0, (0.30, jy, TURN_Z), GRIP, "turn the pin down", pitch=FWD),
        w(1.8, (tx, jy, over_z), GRIP, "over the jack", pitch=FWD),
        w(1.0, (tx, jy, over_z), GRIP, "line up", pitch=FWD),
        w(0.6, (tx, jy, over_z), GRIP_LIGHT, "ease the grip", pitch=FWD),
        w(2.5, (tx, jy, SEAT_Z - PRESS), GRIP_LIGHT, "push in", pitch=FWD),
        w(0.6, (tx, jy, SEAT_Z - PRESS), FINGER_LOOSE, "let go", pitch=FWD),
        w(1.0, (tx - BACK_OFF, jy, SEAT_Z), FINGER_LOOSE, "back off", pitch=FWD),
        w(1.0, (tx - BACK_OFF, jy, TURN_Z), FINGER_LOOSE, "up", pitch=FWD),
    ]


def demo():
    right = [*_insert(BLACK_Y, BLACK_JACK), *_insert(RED_Y, RED_JACK),
             TiltWaypoint(1.5, AWAY, FINGER_LOOSE, "away", pitch=FWD)]
    return {"right": right}
