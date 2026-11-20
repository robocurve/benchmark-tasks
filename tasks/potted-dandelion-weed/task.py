"""Potted planting: "Weed the pot."

A young dandelion has come up in an 8 in nursery pot of potting mix: toothed leaves round a crown at the soil,
a yellow flower, a 63 mm taproot. The gardener holds the pot down by its rim with
one hand (the root holds harder than the pot weighs) and pulls the weed by the crown with the other, close to
the soil, straight up and steadily (pulling the leaves tears them off; a sideways yank snaps the root), then
puts it on the tarp.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

import simkit  # noqa: E402
import weed as S  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "potted-dandelion-weed"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 900
OBJECT_NAMES = ("pot", "root", "weed") + S.LEAVES

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    objects += [yscene.task_object(s) for s in S.scene_objects()]
    cfg = IsaacLabArenaEnvironment(name="yam_potted_dandelion_weed", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


# -- measured hands --------------------------------------------------------------------------------
PAD_T = 0.008
PAD_W = 0.024
PAD_L = 0.045


def _bases():
    from yam_arena import station
    return {side: np.asarray(station.BASES[side], float) for side in ("left", "right")}


def hand_points(robot):
    """Per hand: the finger pads as oriented boxes (centre, axes, half-sizes), from the measured joints and
    finger travel by the URDF's forward kinematics at the runner's arm bases."""
    from yam_arena import ik
    bases = _bases()
    out = {}
    for side in ("left", "right"):
        q = np.asarray(robot[side]["joints"], float)
        pos, R = ik.tcp_pose(q, bases[side])
        half = abs(float(robot[side]["fingers"][0]))
        tip = pos + R[:, 2] * PAD_TIP
        boxes = []
        for sgn in (-1.0, 1.0):
            c = tip + R[:, 0] * sgn * (half + PAD_T / 2) - R[:, 2] * (PAD_L / 2)
            boxes.append((c, R.copy(), np.array([PAD_T / 2, PAD_W / 2, PAD_L / 2])))
        out[side] = boxes
    return out


class Dynamics:
    """Reports the finger pads (from the measured joints) to the scorer. Nothing here moves a body: the root,
    its neck and each leaf are held by breakable PhysX fixed joints (weed.py) that let go only under load."""

    def __init__(self, env):
        pass

    def reset(self):
        pass

    def before_step(self, step, action):
        pass

    def after_step(self, step, objects, root_poses, robot):
        hands = hand_points(robot)
        return {"pad_boxes": [[list(map(float, c)), [list(map(float, r)) for r in R], list(map(float, hh))]
                              for boxes in hands.values() for c, R, hh in boxes]}


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- scripted demo -----------------------------------------------------------------------------------
# Two hands. The right hand comes down on the near (-y) rim of the pot, one finger inside the wall and
# one outside, and pinches the rim: it holds the pot down. The left hand comes down over the crown,
# jaws across it (closing along x, between the leaves), pad tips 7 mm over the soil, and closes. It pulls
# straight up, slowly (95 mm in 3.5 s): the soil lets go at 13 N, more than the pot weighs, so without the right
# hand the pot would come up with the weed. The root slides out whole. The right hand lets go of the rim, and
# the left hand carries the weed over the tarp, lowers it and lets go.
Z = np.array([0.0, 0.0, 1.0])
REST = {"left": (0.20, 0.24, S.TOP + 0.22), "right": (0.20, -0.24, S.TOP + 0.22)}
APPROACH = -0.025                     # left fingers part open for the approach (half-gap 25 mm)
RIM_OPEN = -0.012                     # right fingers just open round the 7 mm rim
PULL = 0.095
ONE_HAND = False                      # True replays the one-hand baseline: same left-hand squeeze and pull,
                                      # right hand parked at rest (README: it scores 0)

def _rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])


# The left hand is straight down; the right hand leans 17 deg outward (toward -y), so the two wrists clear
# each other over the pot (checked offline with the URDF meshes: 50 mm or more apart). A tilted left hand
# struck the crown with the slanted finger edge and drove it down into the mix.
ACROSS_X = _rz(-math.pi / 2) @ DOWN   # left: jaws closing along x, across the crown
RIM_ROT = _rx(0.30) @ DOWN            # right: jaws closing across the rim, leaning out


def grip_tcp():
    """Pad tips 4 mm over the soil, so the whole crown sits in the flat, upright part of the pads: low on the 15 mm crown, clear of the soil (the last 25 mm are taken slowly;
    a fast descent overshoots)."""
    return np.array([S.WX, S.WY, S.SOIL + 0.004 + PAD_TIP])


def rim_tcp():
    """Right hand on the -y rim, jaws across the wall (closing along y), pad tips 9 mm below the rim top."""
    return np.array([S.POT_C[0], S.POT_C[1] - (S.R_TOP + 0.0005), S.TOP + S.POT_H - 0.009 + PAD_TIP])


def _w(t, p, f, label, rot):
    return Waypoint(t, tuple(float(v) for v in p), f, label, rot)


def demo():
    g = grip_tcp()
    h = rim_tcp()
    tc = np.array([S.TARP_C[0], S.TARP_C[1], 0.0])
    up = g + Z * PULL
    A = ACROSS_X
    left = [_w(1.0, REST["left"], FINGER_OPEN, "rest", A),
            _w(2.0, g + Z * 0.08, APPROACH, "over the crown", A),
            _w(1.2, g + Z * 0.025, APPROACH, "fingers down either side of the crown", A),
            _w(1.5, g, APPROACH, "slowly down to the soil", A),
            _w(0.6, g, -0.0135, "jaws in to 2 mm round the crown", A),
            _w(1.2, g, FINGER_CLOSED, "grip the crown at the soil, gently", A),
            _w(0.8, g, FINGER_CLOSED, "settle the grip", A),
            _w(3.5, up, FINGER_CLOSED, "pull straight up, slowly", A),
            _w(1.6, up, FINGER_CLOSED, "root out; the other hand lets go of the pot", A),
            _w(2.0, np.array([tc[0], tc[1], up[2]]), FINGER_CLOSED, "over the tarp", A),
            _w(1.2, np.array([tc[0], tc[1], S.TOP + 0.095]), FINGER_CLOSED, "lower it", A),
            _w(0.8, np.array([tc[0], tc[1], S.TOP + 0.095]), FINGER_OPEN, "let go", A),
            _w(1.0, np.array([tc[0], tc[1], S.TOP + 0.17]), FINGER_OPEN, "hand up", A),
            _w(1.5, REST["left"], FINGER_OPEN, "rest", A)]
    right = [_w(1.0, REST["right"], FINGER_OPEN, "rest", DOWN),
             _w(1.6, h + Z * 0.06, RIM_OPEN, "over the near rim", RIM_ROT),
             _w(1.4, h, RIM_OPEN, "down onto the rim, one finger each side of the wall", RIM_ROT),
             _w(1.0, h, FINGER_CLOSED, "pinch the rim: hold the pot down", RIM_ROT),
             _w(6.8, h, FINGER_CLOSED, "hold the pot while the weed is pulled", RIM_ROT),
             _w(0.6, h, RIM_OPEN, "let go of the rim", RIM_ROT),
             _w(0.8, h + Z * 0.06, RIM_OPEN, "hand up", RIM_ROT),
             _w(1.4, REST["right"], FINGER_OPEN, "rest", DOWN)]
    if ONE_HAND:
        right = [_w(1.0, REST["right"], FINGER_OPEN, "rest (one-hand baseline)", DOWN)]
    tl = sum(w.duration for w in left)
    tr = sum(w.duration for w in right)
    left.append(_w(2.5, REST["left"], FINGER_OPEN, "hold", A))
    right.append(_w(tl + 2.5 - tr, REST["right"], FINGER_OPEN, "rest", DOWN))
    return {"left": left, "right": right}


def plan_poses():
    out = []
    for side, wps in demo().items():
        for w in wps:
            out.append((side, w.label, np.asarray(w.pos, float), np.asarray(w.yaw, float)))
    return out


SEEDS = ([0, 1, 1, -0.6, 0, 0], [0.5, 1.5, 1.2, -1, 0, 0], [-0.5, 1.5, 1.2, -1, 0, 0], [0, 2, 2, -1.5, 0, 0],
         [0.8, 1.2, 0.8, -0.5, 1, 0], [-0.8, 1.2, 0.8, -0.5, -1, 0], [0.8, 1.2, 0.8, -0.5, -1, 0],
         [-0.8, 1.2, 0.8, -0.5, 1, 0], [0, 1.2, 1.0, -0.3, 0, 1.5], [0, 1.2, 1.0, -0.3, 0, -1.5])


class DemoController:
    """The waypoint plans, each arm's warm-started IK first seeded on the elbow branch that reaches its
    first grasp (a multi-seed solve), since from READY it can settle on one that cannot."""

    def __init__(self, plans):
        from yam_arena import ik
        self.plans = plans
        for side, plan in plans.items():
            w = plan.waypoints[min(2, len(plan.waypoints) - 1)]
            pos, rot = np.asarray(w.pos, float), np.asarray(w.yaw, float)
            best = None
            for sd in SEEDS:
                q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float), base_pos=plan.base_pos, iters=200)
                if best is None or pe + 0.02 * re < best[0]:
                    best = (pe + 0.02 * re, q)
            plan.q = best[1]

    def action(self, clock):
        return np.concatenate([self.plans[side].action7(clock)[0] for side in ("left", "right")])


def make_demo_controller(env, plans):
    return DemoController(plans)
