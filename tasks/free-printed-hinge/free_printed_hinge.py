"""On the station table, right of the middle: a printed hinge standing on its edge on a card, the way a fresh print
comes off the printer with its hinge printed in place. Two 4 cm tall plates meet at a vertical hinge: a dark teal base
leaf (5 cm, fixed to the table) and an orange leaf (7.5 cm, 50 g, with the hinge's knuckle), both 1 cm thick. The card
under them is a printed protractor, 0 degrees along the leaves, the leaf swinging round to the left. The print is
fused: the orange leaf does not turn.

The task: free the hinge, so the orange leaf swings round through 120 degrees or more and stays there, without
snapping it off.

The hinge (``bonds.Hinged``): four stuck layers hold the leaf at 0, each a joint at the hinge whose twist is a
torsion spring (2 N m/rad, 8 N m/rad in all) capped at 0.2 N m, so the leaf gives about 1.8 degrees at 0.25 N m and
cannot be turned. Every rocking cycle of the orange leaf switches one layer off (a programmed rule, not wear
physics): the leaf pushed 0.9 degrees or more one way, then the other way, then back, and then once more each time it
swings back again. The leaf snaps off when the layers hold it with 0.49 N m (about 7.5 N at the grip): a hard push
reaches that long before it moves a hinge held at 0.8 N m, so it snaps. Pushes that are gentler than that, one way and
the other, loosen the layers one by one, until the hinge turns freely.

Run it:  inspect-robots run --embodiment yam_arena -E task=free_printed_hinge.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import Hinged, hinge_angle, on_reset, on_step, snap, unstick  # noqa: E402
from colliders import Part, shape, shapes  # noqa: E402
from looks import dress  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the hinge: its vertical axis, the plates' size, and how high they stand --------------------------------------------
HX, HY = 0.34, -0.12  # the hinge axis (x, y)
PLATE_T, PLATE_H = 0.010, 0.040  # the leaves' thickness and height
CARD_T = 0.002  # the card under the hinge
Z0 = TOP + CARD_T + 0.0015  # the plates' bottom: 1.5 mm over the card, so nothing drags on it
ZC = Z0 + PLATE_H / 2
KNUCKLE_SIZE = (0.014, 0.014, 0.024)  # the knuckle: a barrel 13 mm across, part of the orange leaf
LEAF_LEN = 0.075  # the plate, from 7 mm off the axis
LEAF_BOX = (LEAF_LEN + KNUCKLE_SIZE[0], KNUCKLE_SIZE[1], PLATE_H)  # plate and knuckle in one box, 8.9 cm long
LEAF_X = HX - KNUCKLE_SIZE[0] / 2 + LEAF_BOX[0] / 2  # its centre, 3.75 cm from the axis
BASE_LEN = 0.05  # the base leaf, from 1.2 cm short of the axis
BASE_GAP = 0.012

# Reproducible real specimen: PLA, 0.4 mm nozzle, 0.20 mm layers, three walls and 20% gyroid infill, printed flat with
# the hinge axis vertical. The nominal running clearance is 0.25 mm radial and 0.30 mm axial. Four sacrificial bridges,
# each 0.40 mm radial by 1.00 mm axial and one 0.20 mm layer high, join knuckle to pin at 90-degree intervals. They are
# the only intentional fuses and make a fresh, identical specimen for every trial.
NOZZLE_MM = 0.40
LAYER_MM = 0.20
WALLS = 3
INFILL_PERCENT = 20
RADIAL_CLEARANCE_MM = 0.25
AXIAL_CLEARANCE_MM = 0.30
FUSE_COUNT = 4
FUSE_RADIAL_MM = 0.40
FUSE_AXIAL_MM = 1.00
FUSE_HEIGHT_MM = 0.20

# The four printed bridges are represented by capped torsion springs. One switches off per rocking cycle.
LAYERS = tuple((2.0, 0.05, 0.2) for _ in range(4))  # (stiffness N m/rad, damping N m s/rad, cap N m)
LAYER_K = LAYERS[0][0]
STUCK_TORQUE = sum(cap for _, _, cap in LAYERS)
SNAP_TORQUE = 0.49  # N m the layers hold the leaf with, at which it snaps off
ON_DEG = 0.9  # a push counts when the leaf turns this far, either way
HINGE_STOPS = (-12.0, 130.0)  # degrees: the leaf's range: 12 back, and a stop at 130 round, where it nears the base leaf

# The base leaf is one box: the plate and the two outer barrels of the hinge (drawn, not colliding), 6.9 cm long, 1.3 cm
# thick. Only the plate collides.
BASE_BOX = (BASE_LEN + KNUCKLE_SIZE[0] / 2 + BASE_GAP, 0.013, PLATE_H)
BASE_CX = HX + KNUCKLE_SIZE[0] / 2 - BASE_BOX[0] / 2
BASE = Box("base", pos=(BASE_CX, HY, ZC), size=BASE_BOX, color=(0.07, 0.30, 0.34))
shape("base", [Part("plate", (-BASE_BOX[0] / 2 + BASE_LEN / 2, 0.0, 0.0), (BASE_LEN, PLATE_T, PLATE_H))])

# The orange leaf is one box too: the plate from 7 mm off the axis and the knuckle round the axis. It collides as those two.
LEAF = Hinged("leaf", pos=(LEAF_X, HY, ZC), size=LEAF_BOX, mass=0.05, color=(0.95, 0.45, 0.05), to="base", axis="Z",
              hinge=(HX - LEAF_X, 0.0, 0.0), lower=HINGE_STOPS[0], upper=HINGE_STOPS[1], damping=0.002, layers=LAYERS)
shape("leaf", [Part("plate", (LEAF_BOX[0] / 2 - LEAF_LEN / 2, 0.0, 0.0), (LEAF_LEN, PLATE_T, PLATE_H)),
               Part("knuckle", (HX - LEAF_X, 0.0, 0.0), KNUCKLE_SIZE)])
CARD_SIZE = (0.28, 0.17, CARD_T)
CARD_Y = HY + 0.04  # the card's centre: the protractor's origin is the hinge axis
CARD = Box("card", pos=(HX, CARD_Y, TOP + CARD_T / 2), size=CARD_SIZE, color=(0.92, 0.92, 0.9))

OBJECTS = [BASE, LEAF, CARD]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), each inside its box.
LOOKS = {"base": "base.usdc", "leaf": "leaf.usdc", "card": "card.usdc"}
dress(LOOKS)

# -- scoring ---------------------------------------------------------------------------------------------------------
FULL_DEG = 120.0  # the leaf round this far, at rest: freed and swinging
HALF_DEG = 60.0
SEAT_TOL = 0.008  # m: the leaf's centre this far off its place on the hinge: it has come off

# What the episode remembers: how many layers are still on (one switches off per rocking cycle) and whether the leaf
# snapped off. A snapped leaf scores 0 for the rest of the trial.
# After all bridges are broken, a calibrated spring gauge at the 75 mm leaf tip must move the released leaf from at
# most 30 degrees through at least 90 degrees while reading no more than 1.0 N (0.075 N m). The simulator records the
# same post-release angular traversal; because every remaining layer resists with a 0.2 N m cap, it cannot pass while
# a sacrificial bridge remains.
PROBE_START_DEG = 30.0
PROBE_END_DEG = 90.0
PROBE_FORCE_N = 1.0
PROBE_TORQUE_NM = PROBE_FORCE_N * LEAF_LEN

STATE = {"layers": len(LAYERS), "snapped": False, "probe_started": False, "probe_passed": False}


class Rocking:
    """Counts the rocking cycles from the leaf's angle ``angle`` (degrees, positive toward the swing). A push is a turn of
    ``on`` degrees or more to one side. The first push starts it. Each later push to the other side is half a cycle, and
    a cycle is two halves. :meth:`update` returns True when one completes."""

    def __init__(self, on: float = ON_DEG) -> None:
        self.on = on
        self.reset()

    def reset(self) -> None:
        self.side, self.half, self.cycles = 0, 0, 0

    def update(self, angle: float) -> bool:
        side = 1 if angle >= self.on else -1 if angle <= -self.on else 0
        if side == 0 or side == self.side:
            return False
        first = self.side == 0
        self.side = side
        if first:
            return False
        self.half += 1
        if self.half < 2:
            return False
        self.half, self.cycles = 0, self.cycles + 1
        return True


ROCK = Rocking()


@on_reset
def reset_memory(env=None) -> None:
    STATE.update(layers=len(LAYERS), snapped=False, probe_started=False, probe_passed=False)
    ROCK.reset()


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


NAMES = ("leaf",)


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(objects[n].pos)) and np.all(np.isfinite(objects[n].quat_xyzw)) for n in NAMES)


def leaf_angle(objects) -> float:
    """How far the leaf has swung round the hinge (degrees, counterclockwise from above), 0 where it starts."""
    return hinge_angle(LEAF, objects["leaf"].quat_xyzw)


def holding_torque(objects) -> float:
    """What the layers still on hold the leaf with (N m): their springs, turned by the leaf's angle. 0 when none is left."""
    return STATE["layers"] * LAYER_K * math.radians(abs(leaf_angle(objects)))


def seat_offset(objects) -> float:
    """How far the leaf's centre is from where the hinge puts it (m): its distance from the hinge axis, in plan, against
    where it starts (a leaf on its hinge keeps that distance)."""
    leaf = objects["leaf"]
    reach = math.hypot(*(np.asarray(leaf.pos, float)[:2] - np.array([HX, HY])))
    return abs(reach - abs(LEAF_X - HX))


def snapped_now(objects) -> bool:
    """The layers hold the leaf with the breaking torque or more, or the leaf sits off its hinge."""
    return bool(holding_torque(objects) >= SNAP_TORQUE or seat_offset(objects) >= SEAT_TOL)


def _remember(objects) -> None:
    if _finite(objects) and snapped_now(objects):
        STATE["snapped"] = True


def _update_probe(objects) -> None:
    """Record the low-force validation traversal only after every sacrificial bridge is gone."""
    if STATE["layers"] != 0 or STATE["snapped"]:
        return
    angle = leaf_angle(objects)
    if angle <= PROBE_START_DEG:
        STATE["probe_started"] = True
    elif STATE["probe_started"] and angle >= PROBE_END_DEG:
        STATE["probe_passed"] = True


@on_step
def watch(env) -> None:
    """Every control step: the rocking rule (switch a layer off per cycle) and the break (switch the hinge off). Between the
    scored steps too."""
    from yam_arena.scene import object_states

    objects = object_states(env, list(NAMES))
    if not _finite(objects):
        return
    _remember(objects)
    if STATE["snapped"]:
        snap("leaf")
    elif ROCK.update(leaf_angle(objects)) and STATE["layers"] > 0:
        STATE["layers"] = unstick("leaf")
    _update_probe(objects)


def _zero(objects) -> bool:
    if not _finite(objects):
        return True
    _remember(objects)
    return STATE["snapped"]


def success(objects) -> bool:
    """Freed, low-force traversal demonstrated, released at 120 degrees or more and still, never snapped."""
    if _zero(objects):
        return False
    return bool(STATE["layers"] == 0 and STATE["probe_passed"] and leaf_angle(objects) >= FULL_DEG
                and objects["leaf"].resting())


def distance(objects) -> float:
    """Degrees still to swing to 120."""
    return max(0.0, FULL_DEG - leaf_angle(objects)) if _finite(objects) else FULL_DEG


def score(objects) -> float:
    """1.0: validated success. 0.7: freed and opened 60 degrees. 0.4: all bridges broken. 0 otherwise; breakage is a
    latched zero."""
    if _zero(objects):
        return 0.0
    if success(objects):
        return 1.0
    if STATE["layers"] != 0:
        return 0.0
    return 0.7 if leaf_angle(objects) >= HALF_DEG else 0.4


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm only. Puts its open fingers either side of the leaf's outer end, from above (a 17 mm gap round the 10 mm
# leaf, 3.5 mm to spare each side), and rocks it: the hand goes 3 cm one way, then 3 cm the other, so each pad takes up
# the gap and then pushes the leaf. The soft arm pushes harder the further its hand is past the leaf, to a few newtons,
# not as hard as the layers hold (a grip closed on the leaf does not push it: the soft wrist tilts instead). After the
# layers have let go (five cycles, one more than the rule needs) it lifts its hand, comes round behind the leaf with its
# closed fingertips and sweeps it round along its arc to the stop at 130 degrees, then backs off. Numbers are TCP
# positions.
GRIP_R = 0.065  # m from the hinge axis to where the fingers grip
GRIP_FINGER = -0.0085  # each finger's target while rocking: 8.5 mm out from closed, a 17 mm gap round the 10 mm leaf
GRIP_H = 0.033  # the TCP's height over the table: the pad tips 2 cm up the 4 cm leaf
ROCKS = 5  # rocking cycles
SWING = 0.03  # m: how far the hand goes either side of the leaf, each way
HALF_S = 1.3  # s: for each push
FORCE_S = 0.0  # s: more than 0 swaps the rocking for one steady push of the hand round to 60 degrees, in this many seconds
SWEEP_TO = 131.0  # degrees: the hand goes 1 degree past the stop
SWEEP_R = 0.06  # m: the radius the closed fingertips push the leaf at
FINGER_HALF = 0.012  # m: half the closed fingertips' thickness
POKE_H = 0.030  # the TCP's height over the table while pushing, the tips 1.7 cm up the leaf
PARK = (0.25, -0.25, TOP + 0.15)


def _arc(deg: float, r: float, behind: float) -> tuple[float, float]:
    """A point ``r`` from the hinge at ``deg`` and ``behind`` back from the leaf's axis (the -y side at 0 degrees)."""
    a = math.radians(deg)
    return (HX + r * math.cos(a) + behind * math.sin(a), HY + r * math.sin(a) - behind * math.cos(a))


def demo():
    def w(duration, pos, finger, label, yaw=0.0):
        return Waypoint(duration, tuple(float(v) for v in pos), finger, label, math.radians(yaw))

    gx, gy = HX + GRIP_R, HY
    grip = [
        w(1.6, (gx, gy, TOP + 0.12), FINGER_OPEN, "over the leaf's end"),
        w(1.4, (gx, gy, TOP + GRIP_H), FINGER_OPEN, "fingers either side of the leaf"),
        w(0.8, (gx, gy, TOP + GRIP_H), GRIP_FINGER, "close on the leaf"),
    ]
    if FORCE_S > 0.0:
        a = math.radians(60.0)
        far = (HX + GRIP_R * math.cos(a), HY + GRIP_R * math.sin(a))
        rock = [w(FORCE_S, (far[0], far[1], TOP + GRIP_H), FINGER_CLOSED, "push it round")]
        end_y = far[1]
    else:
        rock = []
        for i in range(ROCKS):
            rock.append(w(HALF_S, (gx, gy + SWING, TOP + GRIP_H), GRIP_FINGER, f"rock {i + 1} one way"))
            rock.append(w(HALF_S, (gx, gy - SWING, TOP + GRIP_H), GRIP_FINGER, f"rock {i + 1} the other way"))
        rock.append(w(0.8, (gx, gy, TOP + GRIP_H), GRIP_FINGER, "back to the leaf's place"))
        end_y = gy
    behind = PLATE_T / 2 + FINGER_HALF + 0.003
    start = _arc(-12.0, SWEEP_R, behind)
    steps = [
        w(0.6, (gx, end_y, TOP + GRIP_H), FINGER_OPEN, "let go"),
        w(0.8, (gx, end_y, TOP + 0.09), FINGER_CLOSED, "up"),
        w(1.4, (start[0], start[1], TOP + 0.09), FINGER_CLOSED, "round behind the leaf", 0.0),
        w(1.0, (start[0], start[1], TOP + POKE_H), FINGER_CLOSED, "down behind the leaf", 0.0),
    ]
    deg = -12.0
    while deg < SWEEP_TO:
        deg = min(SWEEP_TO, deg + 12.0)
        x, y = _arc(deg, SWEEP_R, behind)
        steps.append(w(0.55, (x, y, TOP + POKE_H), FINGER_CLOSED, f"sweep to {deg:.0f} deg", min(max(deg, 0.0), 55.0)))
    x, y = _arc(SWEEP_TO, SWEEP_R, behind)
    ax, ay = _arc(SWEEP_TO, SWEEP_R, behind + 0.04)
    steps += [
        w(0.5, (x, y, TOP + POKE_H), FINGER_CLOSED, "hold", 55.0),
        w(0.8, (ax, ay, TOP + POKE_H), FINGER_CLOSED, "back off", 55.0),
        w(1.0, (ax, ay, TOP + 0.10), FINGER_OPEN, "up", 0.0),
        w(3.0, (ax, ay, TOP + 0.10), FINGER_OPEN, "wait for the leaf to settle", 0.0),
    ]
    return {"right": grip + rock + steps}
