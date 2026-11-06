"""On the station table, a 1 kg spool of orange 3D-printer filament lies flat on a narrow cardboard parts box in front of
the robot: 20 cm across, 7 cm wide, with a 52 mm bore and black flanges, the filament wound 13 cm across. Its rim
overhangs the box on the left and the right. Behind it stands a grey spool holder: a square post on a foot plate with
a 45 mm rod 25 cm above the table, pointing at the robot, and a stop collar where the rod meets the post.

The task: lift the spool with both hands, stand it up and slide it onto the rod until it touches the stop.

The spool is one rigid body of box colliders around an open bore (``spool.py``), the rod and the collar are cylinder
colliders on the fixed post. The bore clears the rod by 3.5 mm all round, and the rod's last 2 cm are chamfered.

Run it:  inspect-robots run --embodiment yam_arena -E task=spool_onto_holder.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import on_reset, on_step  # noqa: E402
from grip import HandWaypoint, joint_moves, solve  # noqa: E402
from spool import Rod, Spool, rod, spool  # noqa: E402
from looks import dress  # noqa: E402  (after spool: it hides the colliders spool.py adds)
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, READY  # noqa: E402
from yam_arena.station import BASES  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- a cardboard parts box, fixed, and the spool lying flat on it, its axis vertical ----------------------------------
# The box is narrower than the spool, so the rim overhangs it 4.5 cm on the left and the right: a hand can take the
# spool there across its whole width, a finger under each flange. It lifts the spool 10 cm, into the reach of a level
# side grip (7 cm above the table, the wrists can't turn the hand level).
CARTON_SIZE = (0.22, 0.11, 0.10)
BOX_TOP = TOP + CARTON_SIZE[2]
SPOOL_D = 0.20  # flange diameter
SPOOL_W = 0.07  # width, flange to flange
BORE_R = 0.026  # m: the bore's radius (52 mm across)
CORE_R = 0.065  # m: the wound filament's surface (13 cm across): 3.5 cm of each flange stands proud of it
FLANGE_T = 0.006
SPOOL_MASS = 1.0
SPOOL_XY = (0.21, 0.0)
CARTON = Box("carton", pos=(*SPOOL_XY, TOP + CARTON_SIZE[2] / 2), size=CARTON_SIZE, color=(0.72, 0.53, 0.33))
SPOOL = spool(Spool(
    "spool",
    pos=(*SPOOL_XY, BOX_TOP + SPOOL_W / 2),
    size=(SPOOL_D, SPOOL_D, SPOOL_W),
    mass=SPOOL_MASS,
    color=(0.95, 0.45, 0.08),
    bore=BORE_R,
    core=CORE_R,
    flange=FLANGE_T,
    sides=24,
))
SPOOL_R = SPOOL_D / 2

# -- the holder: a post on a foot plate, the rod pointing at the robot, a stop collar at its base ---------------------
POST_X = 0.47
POST_SIZE = (0.04, 0.04, 0.29)
POST_FACE_X = POST_X - POST_SIZE[0] / 2  # 0.45: the post's face toward the robot
ROD_Z = TOP + 0.25  # the rod's axis
ROD_R = 0.0225  # 45 mm across
ROD_LEN = 0.12  # from the post's face
ROD_TIP_X = POST_FACE_X - ROD_LEN  # 0.33
COLLAR_R = 0.035  # 70 mm across: wider than the bore, so the spool stops on it
COLLAR_T = 0.01
STOP_X = POST_FACE_X - COLLAR_T  # 0.44: the collar's face toward the robot, the stop
POST = Box("holder", pos=(POST_X, 0.0, TOP + POST_SIZE[2] / 2), size=POST_SIZE, color=(0.55, 0.57, 0.6))
FOOT_SIZE = (0.14, 0.16, 0.012)
FOOT = Box("holder_foot", pos=(POST_X + 0.02, 0.0, TOP + FOOT_SIZE[2] / 2), size=FOOT_SIZE, color=(0.55, 0.57, 0.6))
ROD_FRICTION = 0.3  # a smooth rod: the spool slides along it (and turns on it)
ROD_NOSE = (0.02, 0.014)  # m: the rod's last 2 cm narrow to a 28 mm nose (a chamfered end, as on most holders)
rod(POST, Rod("rod", ROD_TIP_X, POST_FACE_X, 0.0, ROD_Z, ROD_R, *ROD_NOSE), ROD_FRICTION)
rod(POST, Rod("collar", STOP_X, POST_FACE_X, 0.0, ROD_Z, COLLAR_R), ROD_FRICTION)

OBJECTS = [SPOOL, POST, FOOT, CARTON]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The holder's
# mesh draws the post, the foot plate, the rod and the collar, so the foot's own box is only hidden.
LOOKS = {"spool": "spool.usdc", "holder": "holder.usdc", "carton": "carton.usdc"}
dress(LOOKS, hide=["holder_foot"])

# -- scoring ---------------------------------------------------------------------------------------------------------
AXIS_TOL = math.radians(10)  # the spool's axis within this of the rod's
SEATED = 0.01  # m: the spool's inner flange within this of the stop
NEAR_TIP = 0.05  # m: the spool's centre within this of the rod tip's centre
LET_GO = 0.005  # m: no finger point this close to the spool's outside once it has been let go
OFF_TABLE_Z = -0.02  # m above the table: a spool whose centre is lower has fallen off the table
ROD_TIP = np.array([ROD_TIP_X, 0.0, ROD_Z])

# What the episode knows that the objects don't: whether a finger is on the spool, read every control step.
STATE = {"held": False}


@on_reset
def reset_memory(env=None) -> None:
    STATE.update(held=False)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _finite(objects) -> bool:
    s = objects["spool"]
    return bool(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) and np.all(np.isfinite(s.lin_vel)))


def spool_axis(objects) -> np.ndarray:
    """The spool's axis (a unit vector, either way along it)."""
    return _rot(objects["spool"].quat_xyzw)[:, 2]


def axis_angle(objects) -> float:
    """The angle between the spool's axis and the rod's (rad), 0 to 90 degrees."""
    return float(math.acos(min(1.0, abs(float(spool_axis(objects)[0])))))


def off_axis(objects) -> float:
    """How far the spool's centre is from the rod's axis line (m)."""
    c = objects["spool"].pos
    return float(math.hypot(c[1], c[2] - ROD_Z))


def faces_x(objects) -> tuple[float, float]:
    """Where the spool's two flange faces cross the rod's line, along x: (near the robot, near the post)."""
    c, a = objects["spool"].pos, abs(float(spool_axis(objects)[0]))
    return float(c[0] - SPOOL_W / 2 * a), float(c[0] + SPOOL_W / 2 * a)


def on_rod(objects) -> bool:
    """The rod runs through the bore across the spool's whole width: the spool's centre within 2.6 cm (the bore's
    radius) of the rod's axis, its axis within 10 degrees of the rod's, and both its faces between the rod's tip and
    the stop."""
    near, far = faces_x(objects)
    return bool(off_axis(objects) <= BORE_R and axis_angle(objects) <= AXIS_TOL
                and near >= ROD_TIP_X and far <= STOP_X + 0.005)


def seated(objects) -> bool:
    """On the rod, its inner flange within 1 cm of the stop collar."""
    return bool(on_rod(objects) and STOP_X - faces_x(objects)[1] < SEATED)


def at_tip(objects) -> bool:
    """The spool's centre (the bore's middle) within 5 cm of the centre of the rod's tip."""
    return bool(np.linalg.norm(objects["spool"].pos - ROD_TIP) <= NEAR_TIP)


def fingers_on(objects, fingers) -> bool:
    """Any finger point within 5 mm of the spool's outside (a cylinder 20 cm across and 7 cm wide)."""
    if fingers is None or len(fingers) == 0:
        return False
    s = objects["spool"]
    local = (np.asarray(fingers) - s.pos) @ _rot(s.quat_xyzw)
    radial = np.hypot(local[:, 0], local[:, 1])
    return bool(np.any((np.abs(local[:, 2]) <= SPOOL_W / 2 + LET_GO) & (radial <= SPOOL_R + LET_GO)))


@on_step
def watch(env) -> None:
    """Every control step: is a finger on the spool?"""
    from fingers import finger_points
    from yam_arena.scene import object_states

    objects = object_states(env, ["spool"])
    STATE["held"] = fingers_on(objects, finger_points(env)) if _finite(objects) else False


def _violation(objects) -> bool:
    return bool(not _finite(objects) or objects["spool"].pos[2] < TOP + OFF_TABLE_Z)


def success(objects) -> bool:
    """Seated against the stop, let go (no finger within 5 mm of the spool) and still."""
    if _violation(objects):
        return False
    return bool(seated(objects) and not STATE["held"] and objects["spool"].resting(max_speed=0.01, max_spin=0.5))


def score(objects) -> float:
    """1.0: success (held 1 s). 0.5: on the rod, but not at the stop, or still held, or still moving. 0.25: the
    spool's centre within 5 cm of the rod's tip. 0 otherwise, and 0 if the spool fell off the table."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    if on_rod(objects):
        return 0.5
    return 0.25 if at_tip(objects) else 0.0


def distance(objects) -> float:
    """From the spool's centre to where it sits at the stop."""
    goal = np.array([STOP_X - SPOOL_W / 2, 0.0, ROD_Z - (BORE_R - ROD_R)])
    return float(np.linalg.norm(objects["spool"].pos - goal))


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Both hands come at the spool level from the sides, where its rim overhangs the box (y = +-0.10), the left hand
# pointing along -y and the right along +y, so the wrists stay 18 cm apart. Each takes the spool across its whole width,
# one finger on top of the top flange and one under the bottom flange (the full 50 N grip). The two grips' line runs
# through the spool's centre. They lift it 9 cm and roll both wrists a quarter turn together: the spool turns about
# that line and stands up, its bore along x, still held by its rims. They raise it to the rod's height, slide it on
# until it meets the stop, let go and draw back. Numbers are TCP positions.
LEFT, RIGHT = 1.0, -1.0
DEPTH = 0.025  # m: the pad tips this far inside the rim
TCP_Y = SPOOL_R + 0.013 - DEPTH  # the TCP's distance from the spool's axis (the pad tips are 13 mm past it)
GRIP_Z = BOX_TOP + SPOOL_W / 2  # the spool's middle, lying on the box
LIFT = 0.09
CARRY_Z = GRIP_Z + LIFT  # the spool sweeps 10.6 cm round its centre as it turns: clear of the box and the rod's tip
SAG = 0.016  # m: how far the held spool hangs below the TCPs (the arms give under its weight; measured)
INSERT_Z = ROD_Z + SAG
START_X = SPOOL_XY[0]  # stood up there, the spool's far face is 8.5 cm short of the rod's tip
SEAT_X = STOP_X - SPOOL_W / 2 + 0.004  # 4 mm past the stop: the arms lean the spool on the collar
OUT = 0.032  # m: how far a hand backs off sideways to let go (the pad tips then clear the rim)
AWAY_X = 0.30  # where the hands wait once the spool is on
OVER_ROLL = math.radians(3.5)  # the hands roll this much past a quarter turn: the held spool lags it (measured)
# The left arm's posture for the level side grip (the right arm's mirrors it): shoulder and elbow raised, the wrist
# turned so the hand lies level. The first move goes there in joint space, so the IK keeps this posture all along.
POSTURE = (-0.13, 1.75, 0.53, 1.21, 1.44, -1.57)
_ARM = {LEFT: "left", RIGHT: "right"}


def _hand(side: float, closing) -> dict:
    return {"approach": (0.0, -side, 0.0), "closing": closing}


def _posture(side: float, pos, hand: dict):
    """The arm's joint angles for this grip, in the level-grip posture (mirrored for the right arm)."""
    q0 = np.array(POSTURE) * np.array([side, 1, 1, 1, side, side])
    return solve(pos, hand["approach"], hand["closing"], q0, BASES[_ARM[side]])


def _arm(side: float, wait: float) -> list:
    w = HandWaypoint
    lying = _hand(side, (0.0, 0.0, side))  # fingers closing up and down, across the lying spool's width
    # The same hand rolled a quarter turn (and a little more): fingers closing along x.
    standing = _hand(side, (-side * math.cos(OVER_ROLL), 0.0, -side * math.sin(OVER_ROLL)))
    y_in, y_out = side * TCP_Y, side * (TCP_Y + OUT)
    x0 = SPOOL_XY[0]
    return [
        *joint_moves(READY, _posture(side, (x0, side * 0.15, GRIP_Z + 0.03), lying), BASES[_ARM[side]], 2.4, 3,
                     FINGER_OPEN, "level beside the spool"),
        w(1.4, (x0, y_in, GRIP_Z), FINGER_OPEN, "in over the rim", **lying),
        w(0.8, (x0, y_in, GRIP_Z), FINGER_CLOSED, "take it across its width", **lying),
        w(1.5, (x0, y_in, CARRY_Z), FINGER_CLOSED, "lift", **lying),
        w(3.0, (x0, y_in, CARRY_Z), FINGER_CLOSED, "roll it upright", **standing),
        w(2.0, (START_X, y_in, INSERT_Z), FINGER_CLOSED, "up to the rod", **standing),
        w(0.8, (START_X, y_in, INSERT_Z), FINGER_CLOSED, "line up", **standing),
        w(3.5, (SEAT_X, y_in, INSERT_Z), FINGER_CLOSED, "slide it on to the stop", **standing),
        w(0.8, (SEAT_X, y_in, INSERT_Z), FINGER_OPEN, "let go", **standing),
        w(0.8, (SEAT_X, y_out, INSERT_Z), FINGER_OPEN, "back off", **standing),
        w(1.2, (AWAY_X, y_out, INSERT_Z), FINGER_OPEN, "draw back", **standing),
        w(wait, (AWAY_X, y_out, INSERT_Z), FINGER_OPEN, "wait", **standing),
    ]


def demo():
    return {"left": _arm(LEFT, 2.0), "right": _arm(RIGHT, 2.0)}
