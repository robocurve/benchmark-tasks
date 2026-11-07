"""On the station table: a cleanroom pass-through, a low-friction lane sloping down to the left through a chamber with a
mesh roof. A red gate closes each end of the chamber: gate A on the right, gate B on the left. Each slides up and down in
two channels and falls shut when let go. A wafer carrier rests on the lane against gate A. The goal is to pass the carrier
through to the receiving stop on the left, and leave both gates shut, without the two gates ever being open at once.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Pass the carrier through the airlock to the stop on the left and shut both gates. The two gates must never be open at the same time, and the carrier must stay on the lane." \\
           --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import rigid_part  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, MESH, RED, WHITE = (0.45, 0.47, 0.50), (0.28, 0.30, 0.33), (0.80, 0.15, 0.12), (0.88, 0.89, 0.90)

SLOPE = math.radians(10)  # the lane falls toward +y (left)
LANE_X, LANE_Z = 0.22, TOP + 0.045  # the lane's centre line, and its surface's height at y = 0
FRICTION = 0.05  # the lane's contact friction: a slick plastic strip
LANE_Y0, LANE_Y1 = -0.26, 0.22  # the lane's ends, along its own length
STOP_FACE = 0.21  # the receiving stop's face, along the lane
GATE_AT = {"gate_a": -0.095, "gate_b": 0.095}  # along the lane
GATE_SIZE = (0.11, 0.02, 0.11)
GATE_MASS = 0.25
CARRIER = (0.06, 0.10, 0.04)  # across the lane, along it, tall
CARRIER_MASS = 0.15
HALF_W = CARRIER[0] / 2 + 0.003  # 6 mm of play across the lane
SLOT = GATE_SIZE[1] / 2 + 0.006  # the rails stop this far either side of a gate, so it can reach the floor


def lane_point(along, up=0.0):
    """A point of the lane's frame in the world: ``along`` its length toward the stop, ``up`` from its surface."""
    return (LANE_X, along * math.cos(SLOPE) + up * math.sin(SLOPE), LANE_Z - along * math.sin(SLOPE) + up * math.cos(SLOPE))


def _rails():
    """Side rails 2 cm tall along the lane, with a gap at each gate."""
    edges = [LANE_Y0, *[e for a in sorted(GATE_AT.values()) for e in (a - SLOT, a + SLOT)], LANE_Y1]
    out = []
    for y0, y1 in zip(edges[::2], edges[1::2]):
        for s in (-1, 1):
            out.append(((s * (HALF_W + 0.005), (y0 + y1) / 2, 0.01), (0.01, y1 - y0, 0.02), STEEL))
    return out


LANE = rigid_part(
    "lane",
    pos=lane_point(0.0),
    boxes=[((0.0, (LANE_Y0 + LANE_Y1) / 2, -0.005), (2 * HALF_W + 0.02, LANE_Y1 - LANE_Y0, 0.01), STEEL),
           *_rails(),
           ((0.0, STOP_FACE + 0.005, 0.02), (2 * HALF_W, 0.01, 0.04), STEEL)],
    friction=FRICTION,
    slick=True,
    tilt=-SLOPE,
)
CARRIER_START = GATE_AT["gate_a"] - GATE_SIZE[1] / 2 - CARRIER[1] / 2 - 0.002
CARRIER_PART = rigid_part("carrier", pos=lane_point(CARRIER_START, CARRIER[2] / 2 + 0.0005),
                          boxes=[((0.0, 0.0, 0.0), CARRIER, WHITE)], mass=CARRIER_MASS, tilt=-SLOPE)


def closed_z(along) -> float:
    """A closed gate's centre height: it rests on the lane at its high side."""
    high_side = along - GATE_SIZE[1] / 2 / math.cos(SLOPE)
    return lane_point(high_side)[2] + GATE_SIZE[2] / 2 + 0.0005


GATES = {n: Box(n, pos=(LANE_X, lane_point(a)[1], closed_z(a)), size=GATE_SIZE, mass=GATE_MASS, color=RED) for n, a in GATE_AT.items()}


def _fixed():
    """Each gate's two channels, the chamber's walls and mesh roof, and the lane's legs."""
    out, ex, gap = [], GATE_SIZE[0] / 2, GATE_SIZE[1] / 2 + 0.002
    for n, a in GATE_AT.items():
        y = lane_point(a)[1]
        top = closed_z(a) - GATE_SIZE[2] / 2 + 0.085  # below the hand that holds the gate
        for i, s in enumerate((-1, 1)):
            for j, t in enumerate((-1, 1)):
                out.append(Box(f"{n}_post_{i}_{j}", pos=(LANE_X + s * (ex - 0.008), y + t * (gap + 0.005), (TOP + top) / 2),
                               size=(0.024, 0.01, top - TOP), color=STEEL))
            out.append(Box(f"{n}_end_{i}", pos=(LANE_X + s * (ex + 0.0065), y, (TOP + top) / 2),
                           size=(0.005, 2 * gap + 0.02, top - TOP), color=STEEL))
    roof_z = LANE_Z + 0.085
    wall_y = lane_point(GATE_AT["gate_b"])[1] - GATE_SIZE[1] / 2 - 0.01
    for i, s in enumerate((-1, 1)):
        out.append(Box(f"chamber_wall_{i}", pos=(LANE_X + s * (HALF_W + 0.015), 0.0, (TOP + roof_z) / 2),
                       size=(0.01, 2 * wall_y, roof_z - TOP), color=STEEL))
    y, i = -wall_y + 0.03, 0
    while y < wall_y - 0.02:
        out.append(Box(f"mesh_{i}", pos=(LANE_X, y, roof_z), size=(2 * HALF_W + 0.04, 0.01, 0.01), color=MESH))
        y += 0.025
        i += 1
    for i, a in enumerate((-0.20, 0.17)):
        z = lane_point(a)[2]
        out.append(Box(f"leg_{i}", pos=(LANE_X, lane_point(a)[1], (TOP + z - 0.012) / 2), size=(0.06, 0.02, z - 0.012 - TOP), color=STEEL))
    return out


OBJECTS = [LANE, CARRIER_PART, *GATES.values(), *_fixed()]

OPEN = 0.01  # m: a gate counts as open with its lower edge this far above where it closes
SHUT = 0.003  # m: a gate counts as shut this close to where it closes
AT_STOP = 0.01  # m: the carrier's nose within this of the receiving stop
MAX_LIFT = 0.01  # m the carrier may rise off the lane


def _along(state) -> float:
    """How far along the lane a part's centre is."""
    return float(state.pos[1]) * math.cos(SLOPE) + (LANE_Z - float(state.pos[2])) * math.sin(SLOPE)


def _rise(state) -> float:
    """How far above the lane's surface a part's centre is."""
    return float(state.pos[1]) * math.sin(SLOPE) + (float(state.pos[2]) - LANE_Z) * math.cos(SLOPE)


def raised(objects, name) -> float:
    """How far a gate's lower edge is above where it closes."""
    return float(objects[name].pos[2]) - closed_z(GATE_AT[name])


class Watch:
    """Whether both gates have been open at once, and whether the carrier has left the lane. The carrier jumping more than
    3 cm back up the lane in one control step means a reset; it never moves that way on its own."""

    def __init__(self):
        self.last, self.breach, self.lifted = None, False, False

    def update(self, objects):
        now = _along(objects["carrier"])
        if self.last is not None and now < self.last - 0.03:
            self.breach, self.lifted = False, False
        self.last = now
        self.breach = self.breach or all(raised(objects, n) > OPEN for n in GATE_AT)
        rest = CARRIER[2] / 2
        self.lifted = self.lifted or _rise(objects["carrier"]) > rest + MAX_LIFT or abs(objects["carrier"].pos[0] - LANE_X) > 0.02


WATCH = Watch()


def delivered(objects) -> bool:
    """The carrier's nose is within 1 cm of the receiving stop."""
    return bool(_along(objects["carrier"]) + CARRIER[1] / 2 > STOP_FACE - AT_STOP)


def in_chamber(objects) -> bool:
    """The carrier is wholly between the gates."""
    a = _along(objects["carrier"])
    return bool(a - CARRIER[1] / 2 > GATE_AT["gate_a"] and a + CARRIER[1] / 2 < GATE_AT["gate_b"])


def shut(objects) -> bool:
    """Both gates are down."""
    return all(raised(objects, n) < SHUT for n in GATE_AT)


def success(objects):
    """Delivered, both gates shut, the gates never open together, the carrier never off the lane, all still; must hold
    for 1 s. Runs first in every control step, so it also keeps the record."""
    WATCH.update(objects)
    return bool(delivered(objects) and shut(objects) and not WATCH.breach and not WATCH.lifted
                and all(objects[n].resting() for n in ("carrier", *GATE_AT)))


def score(objects):
    """Partial credit (success scores 1.0): 0 if the carrier ever left the lane; 0.5 if it was delivered and both gates are
    shut but they were once open together; 0.25 if it is shut in the chamber with the gates never open together."""
    if WATCH.lifted:
        return 0.0
    if delivered(objects) and shut(objects):
        return 0.5
    return 0.25 if in_chamber(objects) and shut(objects) and not WATCH.breach else 0.0


def demo():
    """Right arm: raise gate A, let the carrier slide into the chamber against gate B, lower gate A. Left arm, gripping gate
    B all the while: once gate A is down, raise gate B, let the carrier slide out to the stop, and lower gate B."""
    part = FINGER_OPEN * 0.5
    hover = TOP + 0.19
    lift = 0.06

    def grip_z(name):
        return closed_z(GATE_AT[name]) + GATE_SIZE[2] / 2 - 0.03

    a, b = lane_point(GATE_AT["gate_a"])[1], lane_point(GATE_AT["gate_b"])[1]
    za, zb = grip_z("gate_a"), grip_z("gate_b")
    right = [
        Waypoint(1.5, (LANE_X, a, hover), part, "above gate A"),
        Waypoint(1.5, (LANE_X, a, za), part, "down to its top edge"),
        Waypoint(0.8, (LANE_X, a, za), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (LANE_X, a, za + lift), FINGER_CLOSED, "raise gate A"),
        Waypoint(2.0, (LANE_X, a, za + lift), FINGER_CLOSED, "hold while the carrier slides in"),
        Waypoint(2.0, (LANE_X, a, za + 0.001), FINGER_CLOSED, "lower gate A"),
        Waypoint(0.6, (LANE_X, a, za + 0.001), part, "let go"),
        Waypoint(1.0, (LANE_X, a, hover), part, "up"),
    ]
    left = [
        Waypoint(1.5, (LANE_X, b, hover), part, "above gate B"),
        Waypoint(1.5, (LANE_X, b, zb), part, "down to its top edge"),
        Waypoint(0.8, (LANE_X, b, zb), FINGER_CLOSED, "grip"),
        Waypoint(6.0, (LANE_X, b, zb), FINGER_CLOSED, "wait for gate A to shut"),
        Waypoint(1.5, (LANE_X, b, zb + lift), FINGER_CLOSED, "raise gate B"),
        Waypoint(2.0, (LANE_X, b, zb + lift), FINGER_CLOSED, "hold while the carrier slides out"),
        Waypoint(2.0, (LANE_X, b, zb + 0.001), FINGER_CLOSED, "lower gate B"),
        Waypoint(0.6, (LANE_X, b, zb + 0.001), part, "let go"),
        Waypoint(1.0, (LANE_X, b, hover), part, "up"),
    ]
    return {"left": left, "right": right}
