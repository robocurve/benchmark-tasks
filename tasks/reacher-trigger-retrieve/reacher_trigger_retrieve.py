"""Retrieve a remote with a tabletop trigger-action reacher."""
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

TOP = TABLE_TOP["station"]
X, Y = .230, -.260
RIGHT_YAW = math.pi / 2
MAT_X, MAT_Y = .267, .108
MAT_SIZE = (.180, .192, .002)
PARK_X = .183
REMOTE_Y = .244
REMOTE_SIZE = (.042, .142, .019)
BLACK = (.10, .11, .115)
RED = (.56, .10, .07)
STEEL = (.38, .40, .42)
RUBBER = (.25, .26, .27)
kit = mk.Kit()


def world(x, y, z):
    return X - y, Y + x, TOP + z


def device_part(name, x, y, boxes, mass, color):
    return kit.part(name, world(x, y, .010), boxes, mass, color, yaw=RIGHT_YAW)


parts = [
    mk.box((.314, .016, .017), at=(.230, 0, 0), color=STEEL),
    mk.box((.103, .022, .020), at=(.041, .008, 0), yaw=.05),
    mk.box((.049, .027, .020), at=(.088, .004, 0)),
    mk.box((.051, .015, .019), at=(.405, -.021, 0), yaw=-.27),
    mk.box((.037, .012, .018), at=(.429, -.027, 0), color=RUBBER),
]
body = device_part("reacher", 0, 0, parts, .077, BLACK)
trigger = device_part("trigger", .083, 0, [
    mk.box((.064, .010, .019), at=(-.037, -.036, 0), yaw=.35),
    mk.box((.038, .009, .018), at=(-.014, -.016, 0), yaw=.85),
    mk.box((.007, .014, .012), at=(0, .020, 0)),
], .011, RED)
pushrod = device_part("pushrod", .236, .019, [
    mk.box((.296, .006, .009)),
], .011, STEEL)
jaw = device_part("moving_jaw", .396, -.005, [
    mk.box((.056, .013, .019), at=(.025, .035, 0), yaw=.54),
    mk.box((.028, .014, .018), at=(.048, .046, 0), color=RUBBER),
    mk.box((.009, .013, .012), at=(-.006, .025, 0)),
], .012, BLACK)

kit.hinge("trigger_stroke", body, trigger, world(.083, 0, .010), (0, 0, 1),
          -26, 0, stiffness=.55, damping=.025)
kit.slider("rod_travel", body, pushrod, (0, 1, 0), 0, .014,
           stiffness=35, damping=1.2)
kit.hinge("jaw_stroke", body, jaw, world(.396, -.005, .010), (0, 0, 1),
          -33, 0, stiffness=.22, damping=.015)

remote = kit.part("remote", (X, REMOTE_Y, TOP+.010), [
    mk.box(REMOTE_SIZE),
], .095, BLACK)

mat = kit.part("mat", (MAT_X, MAT_Y, TOP-.00132), [
    mk.box(MAT_SIZE, at=(0, 0, .001)),
], color=(.35, .53, .57))

OBJECTS = kit.objects()
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.6

def _angles(objects):
    return kit.q_deg(objects, "trigger_stroke"), kit.q_deg(objects, "jaw_stroke")


def _pose_metrics(objects):
    target, quat = kit.pose(objects, remote)
    up = 1 - 2 * float(quat[0]**2 + quat[1]**2)
    part = objects[remote]
    half_x = MAT_SIZE[0] / 2 - .0025
    half_y = MAT_SIZE[1] / 2 - .004
    inside = (part.low[0] >= MAT_X-half_x and part.high[0] <= MAT_X+half_x
              and part.low[1] >= MAT_Y-half_y and part.high[1] <= MAT_Y+half_y)
    flat = abs(target[2]-(TOP+.01018)) < .003 and up >= math.cos(math.radians(10))
    settled = np.linalg.norm(part.lin_vel) <= .015
    return bool(inside and flat and settled), target


def _released(objects, tcps):
    stroke, jaw_angle = _angles(objects)
    handle = kit.point(objects, body, (.045, 0, 0))
    body_state = objects[body]
    return (handle[0] <= PARK_X and abs(stroke) <= 4 and abs(jaw_angle) <= 5
            and abs(handle[2]-TOP-.010) <= .006
            and np.linalg.norm(body_state.lin_vel) <= .025
            and all(np.linalg.norm(handle-c) >= .105 for c in tcps))


def _value(objects, tcps):
    if mk.MEMORY.get("linked_travel_m", 0.) < .065:
        return 0.0
    if not _pose_metrics(objects)[0]:
        return .4
    return 1.0 if _released(objects, tcps) else .8


def _observe(objects):
    tcps = np.asarray(mk.tcps())
    target, _ = kit.pose(objects, remote)
    tip = kit.point(objects, body, (.435, 0, 0))
    stroke, jaw_angle = _angles(objects)
    rod_m = kit.q(objects, "rod_travel")
    previous_y = mk.MEMORY.get("previous_remote_y", float(target[1]))
    if (len(tcps) == 2 and np.all(np.linalg.norm(tcps-target, axis=1) >= .105)
            and np.linalg.norm((target-tip)[:2]) < .084
            and stroke <= -15 and jaw_angle <= -6 and rod_m >= .003):
        mk.MEMORY["linked_travel_m"] = mk.MEMORY.get("linked_travel_m", 0.) + previous_y - float(target[1])
    mk.MEMORY["previous_remote_y"] = float(target[1])

    step = mk._EP["step"]
    dt = float(getattr(mk._EP["env"].unwrapped, "step_dt", .05))
    held = (mk.MEMORY.get("full_steps", 0)+1) if _value(objects, tcps) == 1.0 else 0
    mk.MEMORY["full_steps"] = held
    terminal = held == max(1, round(SUCCESS_HOLD_S / dt))
    if terminal or step % max(1, round(1/dt)) == 0:
        prefix = "SUBACTION_END" if terminal else "SUBACTION"
        for name, body_name, value, unit in (
            ("retrieve remote using linked jaws", remote, mk.MEMORY.get("linked_travel_m", 0.) * 1000, "mm toward mat"),
            ("leave remote flat on marker", remote, int(_pose_metrics(objects)[0]), "placed"),
            ("set reacher down with trigger open", trigger, stroke, "degrees"),
        ):
            record = {"name": name, "joint_or_body": body_name, "end_value": round(float(value), 3),
                      "unit": unit, "step": step, "run_end": terminal}
            if body_name == trigger:
                record["handle_xyz_m"] = np.round(kit.point(objects, body, (.045, 0, 0)), 4).tolist()
                record["jaw_degrees"] = round(float(jaw_angle), 3)
            else:
                part = objects[remote]
                record["remote_xyz_m"] = np.round(target, 4).tolist()
                record["bounds_low_m"] = np.round(part.low, 4).tolist()
                record["bounds_high_m"] = np.round(part.high, 4).tolist()
            print(prefix + " " + json.dumps(record), file=sys.stderr, flush=True)


kit.on_step(_observe)


def score(objects):
    return _value(objects, np.asarray(mk.tcps()))


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    target, _ = kit.pose(objects, remote)
    return float(np.linalg.norm(target-np.array((MAT_X, MAT_Y, TOP+.01018))))


def demo():
    x, y = .254, -.208
    grasp_z, travel_z, lower_z = .020, .073, .018
    travel = .136
    travel_yaw = RIGHT_YAW-.10
    shaft_x, shaft_y, shaft_z, put_x = .242, -.216, .017, .126
    return {"right": [
        Waypoint(1.4, (x, y, TOP+.115), FINGER_OPEN, "approach handle", RIGHT_YAW),
        Waypoint(1.2, (x, y, TOP+grasp_z), FINGER_OPEN, "straddle grip", RIGHT_YAW),
        Waypoint(1.2, (x, y, TOP+grasp_z), FINGER_CLOSED, "squeeze trigger", RIGHT_YAW),
        Waypoint(.7, (x, y, TOP+travel_z), FINGER_CLOSED, "raise the remote off the surface", RIGHT_YAW),
        Waypoint(2.8, (x, y-travel, TOP+travel_z), FINGER_CLOSED, "retrieve remote to marker", travel_yaw),
        Waypoint(.8, (x, y-travel, TOP+lower_z), FINGER_CLOSED, "set the reacher down", travel_yaw),
        Waypoint(.7, (x, y-travel, TOP+lower_z), FINGER_OPEN, "release trigger", travel_yaw),
        Waypoint(.6, (x, y-travel, TOP+.145), FINGER_OPEN, "raise open gripper", travel_yaw),
        Waypoint(.9, (shaft_x, shaft_y, TOP+.11), FINGER_OPEN, "approach shaft", travel_yaw),
        Waypoint(.8, (shaft_x, shaft_y, TOP+shaft_z), FINGER_OPEN, "straddle shaft", travel_yaw),
        Waypoint(.8, (shaft_x, shaft_y, TOP+shaft_z), FINGER_CLOSED, "grip the shaft", travel_yaw),
        Waypoint(.8, (shaft_x, shaft_y, TOP+.066), FINGER_CLOSED, "lift the empty reacher", travel_yaw),
        Waypoint(1.2, (put_x, shaft_y, TOP+.066), FINGER_CLOSED, "move jaws clear of remote", travel_yaw),
        Waypoint(.9, (put_x, shaft_y, TOP+shaft_z), FINGER_CLOSED, "set reacher aside", travel_yaw),
        Waypoint(.6, (put_x, shaft_y, TOP+shaft_z), FINGER_OPEN, "release reacher", travel_yaw),
        Waypoint(.6, (put_x, shaft_y, TOP+.11), FINGER_OPEN, "raise open gripper", travel_yaw),
        Waypoint(1.0, (.26, -.15, TOP+.17), FINGER_OPEN, "park away from result", RIGHT_YAW),
    ]}
