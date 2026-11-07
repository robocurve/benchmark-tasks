"""On the station table, on the robot's right: a wiring board (21.5 x 12 cm plywood, 1 cm thick) with an inline lever
connector screwed to it. The connector (6.6 cm long, 2 cm wide, 1.4 cm tall, grey) has a port at each end, each
under its own orange lever, hinged at the port. Its far port, away from the robot, already holds a brown wire that
runs to the back of the board, its lever shut. Its near port faces the robot and is empty, its lever shut. A loose
brown wire lies on the table to the right of the board, parallel to the connector: 12 cm of insulation (7 mm thick)
and a bare copper end (4 mm thick, 22 mm long) pointing away from the robot. A blue multimeter lies on the robot's
left.

The task: lift the near lever, push the wire's bare end into the near port up to its insulation, and press the lever
back down. The lever is bistable: it springs shut below 30 degrees and open above, up to 60 degrees. Shut, its clamp
blocks the port and presses the copper onto the contact bar. The port opens at the board's surface, so a wire in it
lies level on the board. The port's chamber is 3 cm deep: pushed until it stops, the wire's insulation goes into
the clamp, the real wiring fault, and it scores 0.

Everything is drawn at twice the size of a real 2.5 mm2 lever connector and wire, so the parallel gripper and
PhysX can handle a 4 mm bare end in an 11 mm port (2 mm of play round the insulation). All the numbers below are the
scaled ones.

``bonds.py`` makes the lever's hinge and steers its spring, ``shell.py`` gives the lever its clamp tooth and the
wire its thinner bare end (each one rigid body), ``looks.py`` draws the parts (``looks/make_looks.py``), and
``simready.py`` loads the multimeter (NVIDIA SimReady, CC-BY 4.0).

Run it:  inspect-robots run --embodiment yam_arena -E task=lever_connector_wire.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402  (before bonds: see bonds.py)
from bonds import Hinged, on_step, set_target  # noqa: E402
from looks import dress  # noqa: E402
from wrist import TiltWaypoint  # noqa: E402
from shell import shell  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the connector's frame ---------------------------------------------------------------------------------------------
# The near port's axis runs along +x (into the connector, away from the robot). Connector coordinates (u, v, w): u from
# the near port's face into the connector, v across it (along y), w up from the wiring board's top.
FACE_X = 0.33  # m: x of the near port's face
CONN_Y = -0.22  # m: y of the ports' axis
BOARD_T = 0.010
BASE_TOP = TOP + BOARD_T  # the board's top, where the connector stands


def at(u: float, v: float, w: float) -> tuple[float, float, float]:
    """A point of the connector's frame on the table."""
    return (FACE_X + u, CONN_Y + v, BASE_TOP + w)


def _box(name, lo, hi, color, **kw) -> Box:
    """A fixed box between two corners given in the connector's frame."""
    centre = at(*((a + b) / 2 for a, b in zip(lo, hi)))
    return Box(name, pos=centre, size=tuple(b - a for a, b in zip(lo, hi)), color=color, **kw)


GREY = (0.78, 0.79, 0.80)
ORANGE = (0.98, 0.42, 0.02)
BROWN = (0.36, 0.20, 0.10)
COPPER = (0.80, 0.45, 0.25)

# -- the wiring board ------------------------------------------------------------------------------------------------
BOARD_SIZE = (0.215, 0.12, BOARD_T)  # it reaches 9 cm in front of the port, so a wire in the port lies level on it
BOARD = _box("board", (-0.09, -BOARD_SIZE[1] / 2, -BOARD_T), (-0.09 + BOARD_SIZE[0], BOARD_SIZE[1] / 2, 0.0),
             (0.72, 0.58, 0.40))

# -- the connector's housing: fixed boxes round the near port, and the far half as one block -------------------------
HALF_W = 0.010  # m: the housing's half width
BORE_HALF = 0.0055  # m: the port's half width: 11 mm square, 2 mm of play round the 7 mm insulation
FLOOR_T = 0.0  # the port's floor is the board's top: a wire in it lies level, its insulation on the board
BORE_TOP = 0.011  # its ceiling, 11 mm higher
HOUSING_TOP = 0.014
CEILING_FROM = 0.023  # m from the face: the clamp's window, under the lever, is open from the face to here
STOP_U = 0.030  # m: the chamber's back wall: a wire pushed until it stops is 3 cm in
NEAR_LEN = 0.033
FAR_LEN = 0.033
BAR = (0.011, 0.020)  # m from the face: the contact bar on the floor, under the clamp
BAR_TOP = 0.0015  # 1.5 mm proud of the floor: a level wire's copper lies on it
HOUSING = [
    _box("conn_side_left", (0.0, BORE_HALF, FLOOR_T), (NEAR_LEN, HALF_W, HOUSING_TOP), GREY),
    _box("conn_side_right", (0.0, -HALF_W, FLOOR_T), (NEAR_LEN, -BORE_HALF, HOUSING_TOP), GREY),
    _box("conn_ceiling", (CEILING_FROM, -BORE_HALF, BORE_TOP), (NEAR_LEN, BORE_HALF, HOUSING_TOP), GREY),
    _box("conn_stop", (STOP_U, -BORE_HALF, FLOOR_T), (NEAR_LEN, BORE_HALF, BORE_TOP), GREY),
    _box("conn_bar", (BAR[0], -BORE_HALF, FLOOR_T), (BAR[1], BORE_HALF, BAR_TOP), (0.75, 0.75, 0.72)),
    _box("conn_far", (NEAR_LEN, -HALF_W, 0.0), (NEAR_LEN + FAR_LEN, HALF_W, HOUSING_TOP), GREY),
]
# The far port's wire, from the connector's far end toward the back of the board (its collider: the rest is drawn).
WIRE_AXIS_W = FLOOR_T + 0.0035  # the axis of a 7 mm wire lying on the port's floor
FAR_WIRE = _box("far_wire", (NEAR_LEN + FAR_LEN, -0.0035, WIRE_AXIS_W - 0.0035),
                (NEAR_LEN + FAR_LEN + 0.045, 0.0035, WIRE_AXIS_W + 0.0035), BROWN)

# -- the near lever: a hinged handle on top, with the clamp's tooth hanging from it into the port -----------------------
# The hinge is across the housing at the port's face, the lever's free end toward the back: it opens by lifting its
# back end, so a hand above it can follow it round with the wrist tilting forward.
PIVOT = (0.0, 0.0, HOUSING_TOP + 0.0005)  # its hinge: at the handle's front bottom edge
HANDLE = ((0.0, -0.007, PIVOT[2]), (NEAR_LEN, 0.007, PIVOT[2] + 0.008))  # corners: 3.3 x 1.4 x 0.8 cm. Shut, it lies
# 0.5 mm above the housing.
TOOTH = ((0.012, -0.0045, 0.0052), (0.018, 0.0045, PIVOT[2]))  # the clamp: 6 x 9 mm, its foot 3.7 mm above the bar
LEVER_MASS = 0.02
SNAP_DEG = 30.0  # the lever springs toward open above this angle, shut below (angles are how far it is open)
OPEN_DEG = 60.0  # it opens this far
LEVER_STIFFNESS = 0.0004  # N m per degree: the spring aims 60 degrees past the stop it pulls toward, so it presses
SHUT_TARGET, OPEN_TARGET = 60.0, -OPEN_DEG - 60.0  # joint degrees: 0.024 N m at either stop (opening turns the lever
# about -y, so the joint's angle is minus the opening)
LEVER_DAMPING = 6.0e-4  # N m s per degree: it snaps over at about 1 rad/s, the arm hardly feels it
_hc = tuple((a + b) / 2 for a, b in zip(*HANDLE))
LEVER = Hinged(
    "lever",
    pos=at(*_hc),
    size=tuple(b - a for a, b in zip(*HANDLE)),
    color=ORANGE,
    mass=LEVER_MASS,
    axis="Y",
    pivot=at(*PIVOT),
    lower=-OPEN_DEG,
    upper=0.0,
    stiffness=LEVER_STIFFNESS,
    damping=LEVER_DAMPING,
    start_target=SHUT_TARGET,
)
_tc = tuple((a + b) / 2 - h for a, b, h in zip(*TOOTH, _hc))
shell("lever", [("handle", (0.0, 0.0, 0.0), LEVER.size), ("tooth", _tc, tuple(b - a for a, b in zip(*TOOTH)))])

# -- the loose wire: one rigid body, 12 cm of insulation and a 22 mm bare end ----------------------------------------
INS_LEN, BARE_LEN = 0.120, 0.022
INS_D, BARE_D = 0.007, 0.004
WIRE_LEN = INS_LEN + BARE_LEN
WIRE_XY = (0.25, -0.32)  # its centre, lying along x, bare end toward +x
WIRE_MASS = 0.025
WIRE = Box("wire", pos=(*WIRE_XY, TOP + INS_D / 2), size=(WIRE_LEN, INS_D, INS_D), mass=WIRE_MASS, color=BROWN)
shell("wire", [("insulation", (-WIRE_LEN / 2 + INS_LEN / 2, 0.0, 0.0), (INS_LEN, INS_D, INS_D)),
               ("bare", (WIRE_LEN / 2 - BARE_LEN / 2, 0.0, 0.0), (BARE_LEN, BARE_D, BARE_D))])

# -- a multimeter on the robot's left, out of the way (NVIDIA SimReady, fixed) ---------------------------------------
MULTIMETER = SimReady(
    "multimeter",
    pos=(0.30, 0.24, TOP + 0.0005),
    size=(0.19, 0.34, 0.04),
    package="testingTool_multimeter_digital_b01",
    usd="sm_testingTool_multimeter_digital_b01_01.usd",
    yaw=0.0,
)

OBJECTS = [BOARD, *HOUSING, FAR_WIRE, MULTIMETER, LEVER, WIRE]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The board's
# mesh draws the connector's housing, its far lever and the far wire.
LOOKS = {"board": "board.usdc", "lever": "lever.usdc", "wire": "wire.usdc"}
dress(LOOKS, hide=[b.name for b in (*HOUSING, FAR_WIRE)])

# -- the lever's two stops -------------------------------------------------------------------------------------------


def _finite(*states) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) and np.all(np.isfinite(s.lin_vel))
               for s in states)


def lever_angle(state) -> float:
    """How far the lever is open, in degrees (0 shut): its turn about -y (its back end up) from where it spawned."""
    r = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))
    return float(math.degrees(math.atan2(-r[0, 2], r[0, 0])))


@on_step
def snap(env) -> None:
    """The lever's over-centre spring: past 30 degrees it pulls open, below it pulls shut."""
    from yam_arena.scene import object_states

    state = object_states(env, ["lever"])["lever"]
    if _finite(state):
        set_target("lever", OPEN_TARGET if lever_angle(state) > SNAP_DEG else SHUT_TARGET)


# -- scoring ---------------------------------------------------------------------------------------------------------
SHUT_DEG = 5.0  # the lever within 5 degrees of its stop is shut
BAND = (0.018, 0.024)  # m: the bare end's tip this far into the port: at most 4 mm of copper shows, at most 2 mm of
# insulation is in
TRAP = 0.026  # m: the tip this far in or more puts 4 mm or more of insulation in the clamp
STARTED = 0.006  # m: the tip this far in has entered the port
OFF_TABLE_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table
PORT_SLACK = 0.001  # m: how far outside the port's walls the tip may be read, for contact offsets


def tip(state) -> np.ndarray:
    """The bare end's tip: the wire's centre plus half its length along its own axis."""
    r = quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))
    return np.asarray(state.pos, float) + r[:, 0] * WIRE_LEN / 2


def depth(objects) -> float:
    """How far the tip is into the near port (m, along the port's axis), 0 when it is not in the port."""
    x, y, z = tip(objects["wire"])
    d = x - FACE_X
    inside = (abs(y - CONN_Y) <= BORE_HALF + PORT_SLACK
              and BASE_TOP + FLOOR_T - PORT_SLACK <= z <= BASE_TOP + BORE_TOP + PORT_SLACK)
    return float(d) if d > 0 and inside else 0.0


def lever_shut(objects) -> bool:
    return lever_angle(objects["lever"]) <= SHUT_DEG


def _void(objects) -> bool:
    if not _finite(objects["wire"], objects["lever"]):
        return True
    return bool(objects["wire"].pos[2] < TOP + OFF_TABLE_Z)


def still(objects) -> bool:
    """The wire's centre moves under 1 cm/s. (Its spin is not read: clamped, a thin rigid wire jitters about its own
    axis in PhysX without going anywhere.)"""
    return bool(np.linalg.norm(objects["wire"].lin_vel) < 0.01)


def success(objects) -> bool:
    """The bare end 18 to 24 mm into the port, the lever shut, the wire still."""
    if _void(objects):
        return False
    d = depth(objects)
    return bool(BAND[0] <= d <= BAND[1] and lever_shut(objects) and still(objects))


def score(objects) -> float:
    """1.0: success. 0.5: the bare end 18 to 24 mm in, the lever open (more than 5 degrees from shut). 0.25: the
    lever open and the tip at least 6 mm in, but not in that range (and under 26 mm). 0 otherwise: including the tip
    26 mm or more in (insulation in the clamp), the lever shut on a wire not in range, the wire off the table."""
    if _void(objects):
        return 0.0
    d = depth(objects)
    if d >= TRAP:
        return 0.0
    in_band = BAND[0] <= d <= BAND[1]
    if lever_shut(objects):
        return 1.0 if in_band and still(objects) else 0.0
    if in_band:
        return 0.5
    return 0.25 if d >= STARTED else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm, fingers closing along y. 1: straight down, pinch the lever's sides lightly 2.4 cm behind the hinge, then
# carry the pinch round the hinge to 42 degrees, the wrist tilting forward with the lever (``wrist.TiltWaypoint``), and
# let go: the spring takes it on to its 60-degree stop. 2: straight down again, take the wire 6 cm behind its tip,
# lift it, line it up with the port 1.5 cm in front of the face, push it in until the tip is 21 mm in, and let go (it
# drops 2 mm onto the board and the contact bar). 3: come in behind the open lever tilted 60 degrees, pinch it again,
# carry it back down to 3 degrees and let go: the spring shuts it on the copper. The left arm stays where it is.
# Numbers are TCP positions. A pinch at the full 50 N holds the lever so hard that it can't turn between the pads, so
# the pads only squeeze it lightly, and the wrist turns with it.
PINCH_U = 0.024  # m from the face: where the pads pinch the lever
PINCH_W = PIVOT[2] + 0.002  # the pad tips 2 mm above the lever's bottom (2.5 mm above the housing)
PINCH_OPEN = -0.02  # finger joint: pads 4 cm apart round the 1.4 cm lever
PINCH = -0.0055  # finger joint: pads sent to 1.1 cm apart, so they squeeze the 1.4 cm lever lightly (about 7 N each)
SWING = (15.0, 30.0, 42.0)  # degrees: the pinch is carried round the hinge through these, the wrist tilting as much
SWING_BACK = (40.0, 20.0, 3.0)  # and back down, from the 60-degree stop: let go at 3 degrees, the spring shuts it
GRIP_BACK = 0.06  # m: the wire is held this far behind its tip (the pads reach 1.8 cm ahead of the TCP)
GRIP_TIPS = 0.002  # m: the pad tips this far above the table when they close on the lying wire
WIRE_BELOW_TCP = PAD_TIP_PAST_TCP + GRIP_TIPS - INS_D / 2  # the held wire's axis under the TCP
BORE_AXIS_W = (FLOOR_T + BORE_TOP) / 2
INSERT_Z = BASE_TOP + BORE_AXIS_W + WIRE_BELOW_TCP  # the held wire's axis on the port's axis
LINE_UP = 0.015  # m: the tip stops this far in front of the face before it goes in
PUSH_TO = 0.021  # m: the tip this far into the port
TRAVEL_Z = TOP + 0.08


def _turn(du: float, dw: float, degrees: float) -> tuple[float, float]:
    """A point (u, w) from the hinge, carried round it as the lever opens ``degrees``."""
    a = math.radians(degrees)
    return du * math.cos(a) - dw * math.sin(a), du * math.sin(a) + dw * math.cos(a)


def swing_point(degrees: float) -> tuple[float, float, float]:
    """The pinching TCP carried round the hinge with the lever, ``degrees`` open."""
    du, dw = _turn(PINCH_U - PIVOT[0], PINCH_W + PAD_TIP_PAST_TCP - PIVOT[2], degrees)
    return at(PIVOT[0] + du, 0.0, PIVOT[2] + dw)


def _behind(degrees: float, back: float = 0.03) -> tuple[float, float, float]:
    """The pinch's TCP pose for the lever ``degrees`` open, backed off ``back`` along the tilted fingers."""
    x, y, z = swing_point(degrees)
    a = math.radians(degrees)
    return (x - back * math.sin(a), y, z + back * math.cos(a))


def demo():
    w, t = Waypoint, TiltWaypoint
    px, py, pz = swing_point(0.0)
    gx, gy = WIRE_XY[0] + WIRE_LEN / 2 - GRIP_BACK, WIRE_XY[1]
    gz = TOP + GRIP_TIPS + PAD_TIP_PAST_TCP
    lx = FACE_X - LINE_UP - GRIP_BACK  # the TCP with the tip lined up in front of the face
    ix = FACE_X + PUSH_TO - GRIP_BACK  # and pushed in
    opening = [t(1.0, swing_point(a), PINCH, f"swing to {a:.0f} degrees", pitch=math.radians(a)) for a in SWING]
    closing = [t(1.0, swing_point(a), PINCH, f"swing back to {a:.0f} degrees", pitch=math.radians(a))
               for a in SWING_BACK]
    up, end, last = math.radians(OPEN_DEG), math.radians(SWING[-1]), math.radians(SWING_BACK[-1])
    lx_, _, lz_ = swing_point(SWING_BACK[-1])
    return {
        "right": [
            w(1.2, (px, py, TRAVEL_Z), PINCH_OPEN, "over the lever"),
            w(1.0, (px, py, pz), PINCH_OPEN, "down round the lever"),
            w(0.6, (px, py, pz), PINCH, "pinch"),
            *opening,
            t(0.5, swing_point(SWING[-1]), PINCH_OPEN, "let the lever go", pitch=end),
            t(0.6, _behind(SWING[-1]), PINCH_OPEN, "back off", pitch=end),
            w(0.8, (_behind(SWING[-1])[0], py, TRAVEL_Z), PINCH_OPEN, "up"),
            w(1.3, (gx, gy, TRAVEL_Z), PINCH_OPEN, "over the wire"),
            w(1.0, (gx, gy, gz), PINCH_OPEN, "down round the wire"),
            w(0.6, (gx, gy, gz), FINGER_CLOSED, "grip the wire"),
            w(0.9, (gx, gy, TRAVEL_Z), FINGER_CLOSED, "lift the wire"),
            w(1.5, (lx, CONN_Y, INSERT_Z + 0.02), FINGER_CLOSED, "over the port"),
            w(0.9, (lx, CONN_Y, INSERT_Z), FINGER_CLOSED, "line up"),
            w(1.8, (ix, CONN_Y, INSERT_Z), FINGER_CLOSED, "push in"),
            w(0.3, (ix, CONN_Y, INSERT_Z), FINGER_CLOSED, "hold"),
            w(0.5, (ix, CONN_Y, INSERT_Z), PINCH_OPEN, "let the wire go"),
            w(0.7, (ix, CONN_Y, TRAVEL_Z), PINCH_OPEN, "up off the wire"),
            t(1.2, _behind(OPEN_DEG), PINCH_OPEN, "behind the open lever", pitch=up),
            t(0.8, swing_point(OPEN_DEG), PINCH_OPEN, "round the open lever", pitch=up),
            t(0.5, swing_point(OPEN_DEG), PINCH, "pinch it again", pitch=up),
            *closing,
            t(0.4, (lx_, py, lz_), PINCH_OPEN, "let the lever shut", pitch=last),
            w(0.8, (lx_, py, TRAVEL_Z), PINCH_OPEN, "lift away"),
        ],
    }
