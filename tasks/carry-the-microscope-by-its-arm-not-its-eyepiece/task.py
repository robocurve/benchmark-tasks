"""On the station table, to the robot's left: a bench teaching microscope standing upright on its base. Its arm
rises at the back and bends forward over the base to carry the head; the head is over the middle of the base, with
the objective and the stage under it. A black eyepiece tube stands in a silver collar on top of the head and is the
highest part. In front of the robot is a fixed grey shelf with a white mark on its top.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

CREAM, BLACK, SILVER = (0.90, 0.88, 0.82), (0.08, 0.08, 0.09), (0.80, 0.81, 0.83)
DARK = (0.30, 0.32, 0.36)
SHELF_GREY, WHITE = (0.45, 0.47, 0.50), (0.96, 0.96, 0.96)

# Microscope. Part frame: origin on the base's underside, under the eyepiece; +x toward the arm (the microscope's
# back), y across. The microscope stands side-on: turned 90 degrees, so its back (the arm) is toward the robot's
# left and its front toward the robot's right.
SCOPE_XY = (0.26, 0.12)
SCOPE_YAW = math.pi / 2  # part frame +x is world +y
BASE = (0.120, 0.090, 0.018)  # 120 mm front to back (part x) x 90 mm wide x 18 mm
BASE_CX = 0.006  # the base's centre is 6 mm behind the eyepiece
ARM_X, ARM_T, ARM_W = 0.053, 0.018, 0.026  # the upright arm at the back: centre, front-back thickness, width
ARM_TOP = 0.120  # nothing is above or in front of the arm's top 30 mm except the head, 27 mm in front of it
HEAD = (0.030, 0.040, 0.028)  # head, centred over the origin
HEAD_Z = (0.086, 0.114)
COLLAR_TOP = 0.121
EYE_R, EYE_LEN, EYE_STOP = 0.010, 0.040, 0.032
EYE_BOTTOM = 0.108  # the tube reaches 13 mm down into the collar and head; its top is at 148 mm
SCOPE_MASS, EYE_MASS = 0.25, 0.03

BODY_SHAPES = [
    parts.box(BASE, at=(BASE_CX, 0.0, BASE[2] / 2), color=DARK),  # base
    parts.box((ARM_T, ARM_W, ARM_TOP - BASE[2]), at=(ARM_X, 0.0, (ARM_TOP + BASE[2]) / 2), color=CREAM),  # arm
    parts.box((ARM_X - ARM_T / 2 - 0.010, 0.020, 0.010), at=((ARM_X - ARM_T / 2 + 0.010) / 2, 0.0, 0.081),
              color=CREAM),  # head bracket from the arm, 76 to 86 mm, under the head
    parts.box(HEAD, at=(0.0, 0.0, sum(HEAD_Z) / 2), color=CREAM),  # head, 86 to 114 mm
    parts.cylinder(0.008, 0.020, at=(0.0, 0.0, 0.076), color=SILVER, axis="z"),  # objective, 66 to 86 mm
    parts.box((0.045, 0.055, 0.006), at=(0.0, 0.0, 0.058), color=BLACK),  # stage, 55 to 61 mm
    parts.box((ARM_X - ARM_T / 2 - 0.0225, 0.020, 0.006), at=((ARM_X - ARM_T / 2 + 0.0225) / 2, 0.0, 0.058),
              color=BLACK),  # stage bracket to the arm
    parts.cylinder(0.013, COLLAR_TOP - HEAD_Z[1], at=(0.0, 0.0, (COLLAR_TOP + HEAD_Z[1]) / 2), color=SILVER,
                   axis="z"),  # eyepiece collar, 114 to 121 mm
]
EYEPIECE = parts.Slider(
    "eyepiece",
    shapes=[parts.cylinder(EYE_R, EYE_LEN, at=(0.0, 0.0, EYE_BOTTOM + EYE_LEN / 2), color=BLACK, axis="z")],
    axis=(0.0, 0.0, 1.0),  # it can only slide up out of its collar
    travel=EYE_STOP,
    mass=EYE_MASS,
    damping=0.05,
)
SCOPE = parts.Part("microscope", pos=(*SCOPE_XY, TOP), mass=SCOPE_MASS, yaw=SCOPE_YAW, shapes=BODY_SHAPES,
                   joints=[EYEPIECE])


def _world(xy_part, origin=SCOPE_XY):
    """A part-frame (x, y) offset as a world (x, y) point, for the microscope at `origin`."""
    c, s = math.cos(SCOPE_YAW), math.sin(SCOPE_YAW)
    return (origin[0] + c * xy_part[0] - s * xy_part[1], origin[1] + s * xy_part[0] + c * xy_part[1])


# Fixed shelf, 35 mm tall, with a white mark 2 mm thick on top.
SHELF_XY = (0.28, -0.05)
SHELF = (0.200, 0.200, 0.035)
MARK = (0.120, 0.150, 0.002)
MARK_TOP = TOP + SHELF[2] + MARK[2]
SHELF_BOX = Box("shelf", pos=(*SHELF_XY, TOP + SHELF[2] / 2), size=SHELF, color=SHELF_GREY)
MARK_BOX = Box("white_mark", pos=(*SHELF_XY, TOP + SHELF[2] + MARK[2] / 2), size=MARK, color=WHITE)

OBJECTS = [SCOPE, SHELF_BOX, MARK_BOX]

EPS = 1e-6
EYE_OUT = 0.003  # the eyepiece may rise at most 3 mm out of its collar at any point
EYE_SEATED = 0.001  # at the end: within 1 mm of fully down
MAX_TILT = math.radians(15.0)  # at any point
FLAT = math.radians(3.0)  # at the end: at most 3 degrees
FALL_SPEED = 0.6  # m/s downward: faster than a free fall of about 18 mm
ON_MARK = 0.001  # the base's lowest corner at most 1 mm above the mark (and at most 2 mm below: contact depth)
CENTRED = 0.020  # the base's centre within 20 mm of the mark's centre
CLEAR = 0.005  # released: no part of either gripper within 5 mm of the microscope


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tilt(scope) -> float:
    return math.acos(float(np.clip(_rot(scope.quat_xyzw)[2, 2], -1.0, 1.0)))


def _base_bottom(scope) -> float:
    r = _rot(scope.quat_xyzw)
    return min(float((scope.pos + r @ np.array([BASE_CX + sx * BASE[0] / 2, sy * BASE[1] / 2, 0.0]))[2])
               for sx in (-1, 1) for sy in (-1, 1))


def _base_centre(scope) -> np.ndarray:
    return scope.pos + _rot(scope.quat_xyzw) @ np.array([BASE_CX, 0.0, BASE[2] / 2])


# Whole-trial record: did the eyepiece ever rise over 3 mm, the microscope tilt over 15 degrees, or fall faster than
# 0.6 m/s? Cleared only by a scene reset: the microscope back within 0.5 mm of its start and at rest, after a jump of
# over 20 mm in one control step (a robot cannot move it back that way).
_START = np.array((*SCOPE_XY, TOP))
_record = {"fault": False, "prev": None}
# Released: in the simulator, an on_step callback measures every step how close the fingers and gripper of both arms
# are to the microscope's shapes. Offline (near_misses.py) there is no robot, so "clear" stays True.
_grip = {"clear": True}


def _update(objects) -> None:
    scope = objects["microscope"]
    pos = np.asarray(scope.pos, float)
    prev = _record["prev"]
    if prev is not None and np.linalg.norm(pos - _START) < 0.0005 and scope.resting():
        if np.linalg.norm(pos - prev) > 0.020:
            _record["fault"] = False
    _record["prev"] = pos
    if (scope.joints["eyepiece"] > EYE_OUT + EPS or _tilt(scope) > MAX_TILT + EPS
            or scope.lin_vel[2] < -FALL_SPEED - EPS):
        _record["fault"] = True


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    scope = sim.states()["microscope"]
    r = _rot(scope.quat_xyzw)
    links = {name: (scope.pos if name == "base" else scope.links[name][0],
                    r if name == "base" else _rot(scope.links[name][1])) for name, _, _ in SCOPE.links}
    boxes = SCOPE.world_boxes(links)
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = all(analysis.hulls_to_box(hulls, c, rot, size)[0] > CLEAR for _, c, rot, size in boxes)


def _placed(objects) -> bool:
    scope = objects["microscope"]
    bottom = _base_bottom(scope)
    off = np.linalg.norm(_base_centre(scope)[:2] - np.array(SHELF_XY))
    return bool(
        _tilt(scope) < FLAT + EPS
        and MARK_TOP - 0.002 <= bottom <= MARK_TOP + ON_MARK + EPS
        and off <= CENTRED + EPS
        and scope.joints["eyepiece"] <= EYE_SEATED + EPS
        and scope.resting()
        and _grip["clear"]
    )


def success(objects):
    """Standing flat on the mark with its eyepiece seated, still and let go; the eyepiece never pulled out, the
    microscope never tipped or dropped."""
    _update(objects)
    return bool(not _record["fault"] and _placed(objects))


def distance(objects):
    return float(np.linalg.norm(objects["microscope"].pos[:2] - np.array(SHELF_XY)))


def score(objects):
    """Pass/fail: 1.0 on success, otherwise 0."""
    _update(objects)
    return 1.0 if not _record["fault"] and _placed(objects) else 0.0


# --- demo: left arm only ----------------------------------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
GRIP_DZ = 0.092 + TIP  # TCP above the base's underside: tips 92 mm up, 6 mm above the head bracket
LIFT = 0.045  # base 45 mm above the table while carried (the shelf is 37 mm with its mark)
RELEASE = 0.015  # commanded: base 15 mm above the mark when the fingers open (the loaded arm sags about 5 mm)
WIDE, RELEASE_OPEN = -0.020, -0.020  # pads 40 mm apart around the arm's 18 mm thickness
TRAVEL = 0.16
DEMO_BY_EYEPIECE = False  # counter-test only: grip the eyepiece and lift


def _w(x, y, z):
    return (x, y, TOP + z)


def demo():
    """Left arm: come down on the upright arm, low down, with a finger on its front and back faces; lift, carry over
    the mark and let go just above it."""
    sx, sy = SCOPE_XY
    if DEMO_BY_EYEPIECE:
        ez = COLLAR_TOP + 0.004 + TIP
        return {"left": [
            Waypoint(1.5, _w(sx, sy, ez + 0.005), WIDE, "over the eyepiece"),
            Waypoint(1.0, _w(sx, sy, ez), WIDE, "down to the eyepiece"),
            Waypoint(0.8, _w(sx, sy, ez), FINGER_CLOSED, "grip"),
            Waypoint(2.0, _w(sx, sy, ez + LIFT), FINGER_CLOSED, "lift"),
            Waypoint(2.0, _w(sx, sy, ez + LIFT), FINGER_CLOSED, "hold"),
        ]}
    gx, gy = _world((ARM_X, 0.0))
    # Body origin over the mark so that the base's centre lands on the mark's centre.
    ox, oy = np.array(SHELF_XY) - (np.array(_world((BASE_CX, 0.0))) - np.array(SCOPE_XY))
    tx, ty = _world((ARM_X, 0.0), origin=(ox, oy))
    up = GRIP_DZ + SHELF[2] + MARK[2] + RELEASE
    return {"left": [
        Waypoint(1.5, _w(gx, gy, TRAVEL), WIDE, "over the arm"),
        Waypoint(1.5, _w(gx, gy, GRIP_DZ), WIDE, "down at the arm's front and back"),
        Waypoint(0.8, _w(gx, gy, GRIP_DZ), FINGER_CLOSED, "grip"),
        Waypoint(1.5, _w(gx, gy, GRIP_DZ + LIFT), FINGER_CLOSED, "lift"),
        Waypoint(2.5, _w(tx, ty, GRIP_DZ + LIFT), FINGER_CLOSED, "carry"),
        Waypoint(1.5, _w(tx, ty, up), FINGER_CLOSED, "lower"),
        Waypoint(0.6, _w(tx, ty, up), RELEASE_OPEN, "release"),
        Waypoint(1.5, _w(tx, ty, ARM_TOP + 0.025 + TIP), RELEASE_OPEN, "straight up, tips 25 mm over the arm"),
        Waypoint(1.0, _w(tx - 0.05, ty, 0.15), RELEASE_OPEN, "back toward the robot"),
    ]}
