"""On the station table, centred in front of the robot, a desktop CO2 laser cutter (52 x 40 x 22 cm, fixed) with its
lid shut. The lid is a hood: a top panel with a tinted window and a front flap, hinged across the back, with a black
handle bar along the flap's lower edge. It is heavy, and a gas strut on each side helps only once it is open past
30 degrees: from about 36 degrees up the struts lift it the rest of the way to its open stop at 80 degrees and hold it
there.

The task: open the lid with both hands at the two ends of the handle, until the struts hold it fully open, without
letting it fall back.

The lid is one rigid body on a revolute joint (``bonds.Hinged``) whose collider is boxes (``plate.collider``). The
struts are a torque about the hinge, set every control step from the lid's angle and speed (``strut``). Scoring reads the lid's angle from its
orientation, and remembers the highest angle and whether it fell back or slammed shut.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_laser_cutter_lid.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cutter as C  # noqa: E402
import numpy as np  # noqa: E402
import wrist  # noqa: E402, F401  (lets ArmPlan play PinchWaypoint)
from bonds import Hinged, on_reset, on_step  # noqa: E402
from looks import dress  # noqa: E402
from plate import collider, surface  # noqa: E402
from wrist import PinchWaypoint  # noqa: E402
from yam_arena.demo import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
SUCCESS_HOLD_S = 1.0
TOP = C.TOP
G = 9.81

# -- the laser cutter: its body fixed, centred on the robot's centre line, its front face 36 cm out -------------------
XF, YC = 0.36, 0.0
BODY_COLOR = (0.13, 0.14, 0.15)
BODY = [Box(name, pos=pos, size=size, color=BODY_COLOR) for name, pos, size in C.placed_body(XF, YC)]

# -- the lid ------------------------------------------------------------------------------------------------------------
LID_MASS = 6.0  # kg: a steel hood with a glass window. One arm cannot lift it past about 12 degrees (README)
HINGE = C.at(XF, YC, C.HINGE_U, 0.0, C.HINGE_W)
_cu, _cw = C.LID_CENTRE[0], C.LID_CENTRE[2]
HINGE_DAMPING = 0.05  # N m s per rad and per kg of lid: the bare hinge (its joint drive, which has no spring)
STRUT_DAMPING = 1.0  # N m s per rad and per kg of lid: the struts' own damping, once they push
LID = Hinged(
    "lid",
    pos=C.at(XF, YC, _cu, 0.0, _cw),
    size=C.LID_SIZE,
    color=BODY_COLOR,
    mass=LID_MASS,
    axis="Y",
    pivot=HINGE,
    lower=0.0,
    upper=C.OPEN_DEG,
    stiffness=0.0,
    damping=HINGE_DAMPING * LID_MASS * math.pi / 180.0,  # the joint takes N m s per degree
    start_target=0.0,
)
collider("lid", [((u, v, w), size, 0.0) for (u, v, w), size in C.lid_pieces()])
surface("lid", 1.0)  # the handle's rubber grip; the rest of the lid touches nothing but the body

OBJECTS = [*BODY, LID]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The machine's
# body mesh rides on its front wall and draws every fixed part.
LOOKS = {"cutter_front": "cutter_body.usdc", "lid": "cutter_lid.usdc"}
dress(LOOKS, hide=[name for name, _, _ in C.body_boxes() if name != "cutter_front"])

# -- the struts ---------------------------------------------------------------------------------------------------------
STRUT_FROM = 30.0  # degrees: below this the struts give nothing (their line passes close to the hinge)
STRUT_FULL = 38.0  # degrees: their full lift from here up
STRUT_GAIN = 1.2  # their full torque is this times the lid's weight torque at 30 degrees
_CM = (C.HINGE_U - C.lid_volume_centre()[0], C.lid_volume_centre()[1] - C.HINGE_W)  # the centre of mass from the hinge


def weight_torque(deg: float, mass: float | None = None) -> float:
    """The lid's weight about the hinge, N m, closing it, open ``deg`` degrees."""
    a = math.radians(deg)
    m = LID_MASS if mass is None else mass
    return m * G * (_CM[0] * math.cos(a) - _CM[1] * math.sin(a))


def strut_torque(deg: float, mass: float | None = None) -> float:
    """The struts' torque about the hinge, N m, opening the lid: none below 30 degrees, rising to full at 38."""
    frac = min(max((deg - STRUT_FROM) / (STRUT_FULL - STRUT_FROM), 0.0), 1.0)
    return frac * STRUT_GAIN * weight_torque(STRUT_FROM, mass)


def balance_deg(mass: float | None = None) -> float:
    """The angle above which the struts lift the lid by themselves."""
    lo, hi = STRUT_FROM, STRUT_FULL
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if strut_torque(mid, mass) < weight_torque(mid, mass) else (lo, mid)
    return (lo + hi) / 2


def lid_angle(state) -> float:
    """How far the lid is open, in degrees (0 shut): its turn about +y (front edge up) from where it spawned."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    r00 = 1 - 2 * (y * y + z * z)
    r02 = 2 * (x * z + y * w)
    return math.degrees(math.atan2(r02, r00))


def lid_rate(state) -> float:
    """How fast the lid opens, degrees per second (negative closing): its spin about y."""
    return math.degrees(float(state.ang_vel[1]))


def _finite(state) -> bool:
    return bool(np.all(np.isfinite(state.pos)) and np.all(np.isfinite(state.quat_xyzw))
                and np.all(np.isfinite(state.ang_vel)))


# -- what the trial remembers ----------------------------------------------------------------------------------------
DROP_DEG = 20.0  # the lid falling this far below the highest it has been is a drop
SHUT_DEG = 3.0  # within this of shut, the lid is at its closed stop ...
SLAM_RATE = 60.0  # ... and arriving there closing faster than this (degrees per second) is a slam
MEMORY = {"highest": 0.0, "dropped": False, "slammed": False, "last": 0.0}


def forget() -> None:
    MEMORY.update(highest=0.0, dropped=False, slammed=False, last=0.0)


def observe(state, dt: float = 0.05) -> None:
    """Record the lid's highest angle, and whether it has fallen 20 degrees below it or hit its closed stop faster
    than 60 degrees per second. Called every control step."""
    if not _finite(state):
        return
    deg = lid_angle(state)
    closing = max(-lid_rate(state), (MEMORY["last"] - deg) / dt)
    MEMORY["highest"] = max(MEMORY["highest"], deg)
    if MEMORY["highest"] - deg > DROP_DEG:
        MEMORY["dropped"] = True
    if deg < SHUT_DEG and closing > SLAM_RATE:
        MEMORY["slammed"] = True
    MEMORY["last"] = deg


@on_reset
def _reset(env) -> None:
    forget()


def strut_command(deg: float, rate: float, mass: float | None = None) -> float:
    """The torque (N m, opening) the struts put on the lid at ``deg`` degrees, opening at ``rate`` degrees per
    second: their push, less their damping once they push."""
    if deg <= STRUT_FROM:
        return 0.0
    m = LID_MASS if mass is None else mass
    return strut_torque(deg, m) - STRUT_DAMPING * m * math.radians(rate)


def _apply_torque(env, torque: float) -> None:
    """Hold ``torque`` (N m, about +y, opening) on the lid until the next call: Isaac Lab adds it every physics
    step."""
    import torch

    lid = env.unwrapped.scene.rigid_objects["lid"]
    device = env.unwrapped.device
    zero = torch.zeros((1, 1, 3), device=device)
    torques = torch.tensor([[[0.0, torque, 0.0]]], device=device)
    lid.permanent_wrench_composer.set_forces_and_torques_index(forces=zero, torques=torques, is_global=True)


@on_step
def strut(env) -> None:
    """The struts: every control step, the torque about the hinge for the lid's angle and speed now. The trial's
    memory is updated too."""
    from yam_arena.scene import object_states

    state = object_states(env, ["lid"])["lid"]
    if not _finite(state):
        return
    observe(state)
    _apply_torque(env, strut_command(lid_angle(state), lid_rate(state)))


# -- scoring -------------------------------------------------------------------------------------------------------------
OPEN_ENOUGH = 75.0  # degrees: at the open stop (80), where the struts hold it
HELD_UP = 37.0  # degrees: past where the struts lift it by themselves (36.4)
STARTED = 10.0  # degrees


def _failed(objects) -> bool:
    return not _finite(objects["lid"]) or MEMORY["dropped"] or MEMORY["slammed"]


def success(objects) -> bool:
    """The lid at its open stop (75 degrees or more), still, and it never fell back or slammed."""
    if _failed(objects):
        return False
    s = objects["lid"]
    return bool(lid_angle(s) >= OPEN_ENOUGH and abs(lid_rate(s)) < 3.0)


def score(objects) -> float:
    """1.0: open 75 degrees or more and still. 0.5: open 36 degrees or more. 0.25: open 10 degrees or more. 0 if the
    lid ever fell 20 degrees below the highest it reached, or hit its closed stop faster than 60 degrees per second.
    Angles are the lid's at the end of the trial."""
    if _failed(objects):
        return 0.0
    deg = lid_angle(objects["lid"])
    if success(objects):
        return 1.0
    if deg >= HELD_UP:
        return 0.5
    if deg >= STARTED:
        return 0.25
    return 0.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Each gripper points forward with its jaws closing up and down, comes in at the handle bar 4.5 cm inside its posts,
# closes round it and lifts the lid along its arc, turning with it, to 44 degrees. There it lets go, backs off down
# and away, and the struts lift the lid on to its stop.
GRIP_V = C.HANDLE_V - 0.045  # m either side of the centre line
OPEN = -0.022  # finger target before the grip: the pads 44 mm apart round the 16 mm bar
LIFT_TO = 44.0  # degrees
LIFT_STEP = 4.0  # degrees between lift waypoints
LIFT_S = 0.5  # s per step
LEAD = 0.0  # degrees the arms aim ahead of the lid while lifting
ROLL = -math.pi / 2
HANDS = 2  # 1 in the one-handed ablation: the left arm stays at its ready pose (see README)
BACK = 0.04  # m the grippers back off along their own axis after letting go


def grip_pose(deg: float, side: int, back: float = 0.0, drop: float = 0.0):
    """The TCP on the handle bar with the lid open ``deg`` degrees, on the robot's ``side`` (-1 right, +1 left), moved
    ``back`` along the gripper's own axis toward the robot and ``drop`` down; and the grip's pitch."""
    u, w = C.handle_point(deg)
    pitch = math.pi / 2 - math.radians(deg)
    x, y, z = C.at(XF, YC, u, side * GRIP_V, w)
    return (x - back * math.sin(pitch), y, z + back * math.cos(pitch) - drop), pitch


def demo():
    plans = {}
    for arm, side in (("right", -1), ("left", 1))[:HANDS]:

        def w(t, deg, finger, label, back=0.0, drop=0.0):
            pos, pitch = grip_pose(deg, side, back, drop)
            return PinchWaypoint(t, pos, finger, label, 0.0, pitch, ROLL)

        plan = [
            w(2.0, 0.0, OPEN, "in front of the handle", back=0.05),
            w(1.2, 0.0, OPEN, "round the bar"),
            w(0.8, 0.0, FINGER_CLOSED, "grip"),
        ]
        deg = 0.0
        while deg < LIFT_TO - 1e-9:
            deg = min(deg + LIFT_STEP, LIFT_TO)
            plan.append(w(LIFT_S, min(deg + LEAD, C.OPEN_DEG), FINGER_CLOSED, "lift"))
        plan += [
            w(0.5, LIFT_TO + LEAD, FINGER_CLOSED, "hold"),
            w(0.4, LIFT_TO + LEAD, OPEN, "let go"),
            w(0.8, LIFT_TO, OPEN, "back off", back=BACK, drop=0.03),
            w(1.5, LIFT_TO, OPEN, "away", back=BACK + 0.08, drop=0.10),
        ]
        plans[arm] = plan
    return plans
