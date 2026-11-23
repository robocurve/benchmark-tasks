"""Wind an unplugged extension cord onto an open hand-cranked reel."""
from __future__ import annotations

import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
from yam_arena.task_file import TABLE_TOP

from scenekit import BodyState, KINDS, UsdBody, _state, _usd_object
from tiltkit import grip

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Wind the loose extension cord evenly onto the reel, then release it."
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 880

REEL_X = 0.320
REEL_Y = 0.0
AXLE_Z = TOP + 0.160
CRANK_RADIUS = 0.088
HUB_RADIUS = 0.048
FLANGE_RADIUS = 0.082
HALF_WIDTH = 0.056
CORD_RADIUS = 0.004
LINK_LENGTH = 0.020
LINKS = 45
LAYOUT_ANGLE = math.radians(25)
EXIT_AXIS = np.array([math.cos(LAYOUT_ANGLE), math.sin(LAYOUT_ANGLE), 0.0])
AXLE_AXIS = np.array([math.sin(LAYOUT_ANGLE), -math.cos(LAYOUT_ANGLE), 0.0])
CENTER = np.array([REEL_X, REEL_Y, AXLE_Z])
CRANK_CENTER = CENTER + 0.213 * AXLE_AXIS
CRANK_GRIP_OFFSET = 0.004 * AXLE_AXIS
WIND_DIRECTION = 1


class Reel(UsdBody):
    self_collision = True
    actuator_groups = {
        "cord": dict(joint_names_expr=["bend_.*"], stiffness=0.01, damping=0.001,
                     armature=5e-5, effort_limit_sim=0.15, velocity_limit_sim=20.0),
        "spinner": dict(joint_names_expr=["spinner_axle"], stiffness=0.0, damping=0.002,
                        armature=1e-5, effort_limit_sim=0.1, velocity_limit_sim=20.0),
        "handle": dict(joint_names_expr=["handle_fold"], stiffness=2.0, damping=0.12,
                       armature=0.0001, effort_limit_sim=1.0, velocity_limit_sim=3.0),
        "axle": dict(joint_names_expr=["drum_axle"], stiffness=0.0, damping=0.03,
                     armature=0.001, effort_limit_sim=6.0, velocity_limit_sim=8.0),
    }


class ReelState(BodyState):
    def __init__(self, base, cord, crank_angle, crank_speed, tcp):
        super().__init__(**vars(base))
        object.__setattr__(self, "cord", cord)
        object.__setattr__(self, "crank_angle", crank_angle)
        object.__setattr__(self, "crank_speed", crank_speed)
        object.__setattr__(self, "tcp", tcp)


def crank(angle):
    return CRANK_CENTER - CRANK_RADIUS * math.sin(angle) * EXIT_AXIS + (0, 0, CRANK_RADIUS * math.cos(angle))


def crank_angle_at(point):
    delta = point - CRANK_CENTER
    return math.atan2(-float(delta @ EXIT_AXIS), delta[2])


def crank_grip(angle, joint=False):
    blend = min(1.0, math.sin(angle)**2 / 0.72)
    pitch = math.radians(-66 + 29 * blend)
    approach = math.cos(pitch) * EXIT_AXIS + (0, 0, math.sin(pitch))
    return grip(approach, -EXIT_AXIS, joint=joint)

def _reel_state(environment, spec):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    base = environment.unwrapped
    art = base.scene[spec.name]

    def array(value):
        return getattr(value, "torch", value).detach().cpu().numpy()

    origin = array(base.scene.env_origins)[0]
    indices = [art.body_names.index(f"cord_{i:02d}") for i in range(LINKS)]
    cord = array(art.data.body_pos_w)[0, indices] - origin
    axle = art.joint_names.index("drum_axle")
    angle = float(array(art.data.joint_pos)[0, axle])
    speed = float(array(art.data.joint_vel)[0, axle])
    tcp = []
    for side in ("left", "right"):
        arm = base.scene[f"{side}_robot"]
        ids = arm.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        tcp.append(ik.tcp_pose(array(arm.data.joint_pos)[0, ids], station.BASES[side])[0])
    return ReelState(_state(environment, spec), cord, angle, speed, np.stack(tcp))


KINDS[Reel] = (_usd_object, _reel_state)
asset_pos = np.array([REEL_X, REEL_Y, TOP]) + 0.008 * EXIT_AXIS
yaw = LAYOUT_ANGLE - math.pi / 2
OBJECTS = [Reel("reel", pos=tuple(map(float, asset_pos)), size=(0.20, 0.29, 0.32),
                usd="assets/reel.usdc", rot_xyzw=(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)),
                articulation=True, fix_root=True, material=False, solver_iters=32)]

def winding(points):
    delta = points - CENTER
    radial = np.column_stack((delta @ EXIT_AXIS, delta[:, 2]))
    axial = delta @ AXLE_AXIS
    radius = np.linalg.norm(radial, axis=1)
    inside = ((radius >= HUB_RADIUS - 0.001) & (radius <= FLANGE_RADIUS - CORD_RADIUS)
              & (np.abs(axial) <= HALF_WIDTH - CORD_RADIUS))
    outside = np.flatnonzero(~inside)
    count = int(outside[0]) if outside.size else len(points)
    if count < 2:
        return 0.0, 0.0, 0.0
    angles = np.unwrap(np.arctan2(radial[:count, 1], radial[:count, 0]))
    turns = abs(float(angles[-1] - angles[0])) / math.tau
    return count * LINK_LENGTH, turns, float(np.ptp(axial[:count]))


def score(objects):
    state = objects["reel"]
    length, turns, spread = winding(state.cord)
    released = (np.linalg.norm(state.tcp[:, None] - state.cord[None], axis=2).min() >= 0.03
                and np.linalg.norm(state.tcp - crank(state.crank_angle), axis=1).min() >= 0.04)
    if not released or length < 0.42 or turns < 0.85:
        return 0.0
    if length >= 0.84 and turns >= 1.65 and spread >= 0.016:
        return 1.0
    return 0.5


def success(objects):
    return score(objects) == 1.0 and abs(objects["reel"].crank_speed) < 0.05


def demo(start_angle=-0.45):
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint
    from tiltkit import install

    install()
    mode = os.environ.get("REEL_MODE", "guided")
    phase = start_angle
    tilt = crank_grip(phase)
    home_tilt = crank_grip(phase, joint=True)
    start = crank(phase) + CRANK_GRIP_OFFSET
    retreat = np.array([0, 0, -0.028])
    crank_pinch = -0.007

    def point(t, pos, finger, label, yaw=tilt):
        return Waypoint(t, tuple(map(float, pos)), finger, label, yaw)

    right = [
        point(2, start - retreat, FINGER_OPEN, "approach", home_tilt),
        point(1, start, FINGER_OPEN, "around knob"),
        point(0.8, start, crank_pinch, "pinch knob"),
    ]
    for angle in np.linspace(phase, phase + 2.95*math.tau, 173)[1:]:
        right.append(point(0.10, crank(angle) + CRANK_GRIP_OFFSET, crank_pinch, "crank", crank_grip(angle)))
    right.extend([
        point(0.5, crank(angle) + CRANK_GRIP_OFFSET, FINGER_OPEN, "release", crank_grip(angle)),
        point(1, crank(angle) + 0.040 * AXLE_AXIS + (0, 0, 0.035), FINGER_OPEN, "clear", crank_grip(angle)),
        point(2, crank(angle) + 0.040 * AXLE_AXIS + (0, 0, 0.035), FINGER_OPEN, "settle", crank_grip(angle)),
    ])
    left = []
    if mode == "guided":
        direction = (0.4096, -0.7094, -0.5736)
        guide_tilt = grip(direction, (0, -1, 0))
        guide_home = grip(direction, (0, -1, 0), joint=True)
        guide = CENTER + 0.224 * EXIT_AXIS - 0.038 * AXLE_AXIS + (0, 0, -0.122)
        sweep = 0.042 * AXLE_AXIS
        left = [
            point(2, guide + (0, 0.005, 0.055), FINGER_OPEN, "above incoming cord", guide_home),
            point(1, guide, FINGER_OPEN, "straddle cord", guide_tilt),
            point(0.8, guide, -0.0038, "form guide", guide_tilt),
            point(16.5, guide + sweep, -0.0038, "guide laterally", guide_tilt),
            point(0.5, guide + sweep, FINGER_OPEN, "release cord", guide_tilt),
            point(1.5, guide + sweep + (0, 0, 0.055), FINGER_OPEN, "clear", guide_tilt),
        ]
    return {"right": right, "left": left}
