"""ceiling-tile-lay-in: "Put the ceiling tile back into its opening in the grid."

A 2 ft x 2 ft (603 mm) mineral-fibre lay-in tile has been pushed up out of its bay of a 15/16 in exposed-tee
suspended ceiling and left lying askew (turned 12 degrees) across the tops of the four tees round its opening, as
it is left after plenum access. Re-laying it is done from below, with the hands up in the opening: the tile is
turned back square on the tee tops with the fingertips under it, centred over the opening (it has only about
2 mm to spare between the tees' bulbs), let down 38 mm between the tee webs and set flat on the 24 mm flanges,
which it overlaps by 8.85 mm a side, without knocking the loose neighbouring tiles. It is 0.6 m across and rests
on four narrow steel edges, so one fingertip under it only spins or tips it: two hands are needed, one each side.

The scripted demo (``DemoController``) is closed loop on the tile's measured pose: each hand's fingertips are
held a few millimetres up against the tile's underside at a fixed place in the tile's frame, and moved each step a
little further toward square and centred than the tile now is, so the tile is carried round by friction only.
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
import geom as G  # noqa: E402
from scoring import TrialScorer, quat_matrix  # noqa: E402

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "ceiling-tile-lay-in"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
SCENE_ONLY = False                     # True: arms just look; for setup screenshots / settling checks
MAX_STEPS = 80 if SCENE_ONLY else 800
OBJECT_NAMES = G.OBJECT_NAMES

DOWN = simkit.patch_demo()             # straight-down grip; waypoints may also pass a 3x3 rotation


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    aim_overview(embodiment)
    objects = yscene.task_room("station", embodiment.rig.top_cam)
    objects += [yscene.task_object(s) for s in G.scene_objects()]
    cfg = IsaacLabArenaEnvironment(name="yam_" + NAME.replace("-", "_"), scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


def aim_overview(embodiment):
    """Re-aim the recording-only overview camera under the ceiling (it is not a model observation)."""
    from yam_arena import ik
    eye, target = np.array(G.OVERVIEW_EYE), np.array(G.OVERVIEW_TARGET)
    fwd = (target - eye) / np.linalg.norm(target - eye)
    left = np.cross([0.0, 0.0, 1.0], fwd)
    left /= np.linalg.norm(left)
    up = np.cross(fwd, left)
    cam = embodiment.camera_config.overview_cam
    cam.offset.pos = tuple(float(v) for v in eye)
    cam.offset.rot = tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.column_stack([fwd, left, up])))


def make_scorer():
    return TrialScorer()


# -- the scripted demo ---------------------------------------------------------------------------------------
REST = {"left": (0.18, 0.20, G.TOP + 0.18), "right": (0.18, -0.20, G.TOP + 0.18)}
PITCH = math.radians(15.0)             # hands come in level from the robot's side, tipped up 15 degrees
CONTACT_UP = 0.055                     # highest point of the hand (front top of the gripper) above the TCP, tipped up
PRESS = 0.000                          # how far the fingertips are held up into the tile's underside
PALM_LOCAL = {"left": np.array([0.03, 0.20]), "right": np.array([0.03, -0.20])}   # under the tile, its own frame
LIFT = 0.003                           # how far the hands hold the tile up off the tee tops while turning it
YAW_RATE = math.radians(6.0)           # per second: how far ahead of the tile the hands lead it round
MOVE_RATE = 0.030                      # m/s: the same for centring
T_IN, T_UP = 3.5, 1.5                  # reach in under the tile, then up to it
T_TURN = 10.0                          # turning and centring (closed loop), at most
SQUARE_TOL = (math.radians(0.3), 0.0015)   # square and centred enough to let it down
T_LOWER = 3.0                          # let it down onto the flanges
T_OUT = 3.0


def frame(a, close):
    a = np.asarray(a, float)
    a = a / np.linalg.norm(a)
    c = np.asarray(close, float)
    c = c - (c @ a) * a
    c = c / np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


PALM_R = frame((math.cos(PITCH), 0.0, math.sin(PITCH)), (0.0, 0.0, 1.0))


def _w(t, p, f, label, rot=None):
    return Waypoint(t, tuple(float(v) for v in p), f, label, DOWN if rot is None else rot)


def demo():
    """Nominal plan (used for SCENE_ONLY and for the run's duration); the real demo is DemoController."""
    if SCENE_ONLY:
        return {s: [_w(1.5, REST[s], FINGER_OPEN, "look"), _w(MAX_STEPS / HZ, REST[s], FINGER_OPEN, "hold")]
                for s in ("left", "right")}
    total = T_IN + T_UP + T_TURN + T_LOWER + T_OUT + 3.0
    return {s: [_w(1.0, REST[s], 0.0, "start"), _w(total, REST[s], 0.0, "closed-loop demo")] for s in ("left", "right")}


def tile_yaw(R):
    """The tile's turn from square (rad), folded into -45..45 deg."""
    a = math.atan2(R[1, 0], R[0, 0])
    return (a + math.pi / 4) % (math.pi / 2) - math.pi / 4


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class DemoController:
    """Closed-loop scripted demo on the measured tile pose (the Guide allows a demo to read simulator state)."""

    def __init__(self, env, plans):
        from yam_arena import station
        self.env = env
        self.base = {s: np.asarray(station.BASES[s], float) for s in ("left", "right")}
        self.q = {}
        self.target = {}
        self.log_t = -1.0
        self.palm = {}
        self.press = PRESS
        self.z_rest = None
        self.t3 = None
        self.ok_run = 0

    def _joints(self, side):
        robot = self.env.unwrapped.scene[f"{side}_robot"]
        q = robot.data.joint_pos
        q = q.torch[0] if hasattr(q, "torch") else q[0]
        q = q.cpu().numpy()
        names = robot.joint_names
        return np.array([q[names.index(f"joint{i}")] for i in range(1, 7)])

    def _tile(self):
        p = self.env.unwrapped.arena_world.get_pose_w("tile")[0].cpu().numpy()
        return np.asarray(p, float)

    def _ik(self, side, pos, R):
        from yam_arena import ik
        if side not in self.q:
            self.q[side] = self._joints(side)
        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(R), self.q[side], base_pos=self.base[side], iters=60)
        self.q[side] = q
        return q

    def _goal(self, t, tile):
        """Per hand: desired TCP position. Heights are absolute (never follow the tile up); in the turning phase
        each hand's place under the tile is carried round the tile's measured centre by a small lead each step."""
        c = tile[:3]
        R = quat_matrix(tile[3:7])
        yaw = tile_yaw(R)
        t1, t2 = T_IN, T_IN + T_UP
        if self.t3 is None and t >= t2:
            ok = abs(yaw) <= SQUARE_TOL[0] and np.linalg.norm(c[:2] - np.array([G.XC, G.YC])) <= SQUARE_TOL[1]
            self.ok_run = self.ok_run + 1 if ok else 0
            if self.ok_run >= 6 or t >= t2 + T_TURN:
                self.t3 = t
        t3 = self.t3 if self.t3 is not None else 1e9
        t4 = t3 + T_LOWER
        z_touch = G.FLANGE_Z + G.WEB_H - CONTACT_UP + self.press
        if t >= t2 - 0.3 and t < t3:
            # feel for the tile: raise the hands until it is just lifted LIFT above its rest on the tee tops,
            # so they carry most of its weight (and their friction turns it), never more
            lift = c[2] - self.z_rest
            if lift < LIFT - 0.0005:
                self.press = min(self.press + 0.0004, 0.030)
            elif lift > LIFT + 0.001:
                self.press = max(self.press - 0.0004, -0.010)
        if self.z_rest is None:
            self.z_rest = float(c[2])
        if not self.palm:
            for side in ("left", "right"):
                self.palm[side] = c[:2] + rz(yaw) @ PALM_LOCAL[side]
        if t2 <= t < t3:
            lead_yaw = float(np.clip(-yaw, -YAW_RATE / HZ, YAW_RATE / HZ))
            dc = np.array([G.XC, G.YC]) - c[:2]
            n = np.linalg.norm(dc)
            lead_c = dc if n <= MOVE_RATE / HZ else dc / n * MOVE_RATE / HZ
            for side in ("left", "right"):
                self.palm[side] = c[:2] + rz(lead_yaw) @ (self.palm[side] - c[:2]) + lead_c
        out = {}
        for side in ("left", "right"):
            xy = self.palm[side]
            if t < t1:
                z = z_touch - 0.050
            elif t < t2:
                z = z_touch - 0.050 * (1 - (t - t1) / T_UP)
            elif t < t3:
                z = z_touch
            elif t < t4:
                z = max(G.FLANGE_Z - CONTACT_UP - 0.030, self.target[side][2] - 0.020 / HZ)
            else:
                xy = self.target[side][:2] + np.array([-0.03 / HZ, 0.0])
                z = max(G.FLANGE_Z - 0.20, self.target[side][2] - 0.06 / HZ)
            out[side] = np.array([xy[0], xy[1], z])
        return out

    def action(self, clock):
        from yam_arena import ik
        tile = self._tile()
        goal = self._goal(clock, tile)
        acts = []
        for side in ("left", "right"):
            if side not in self.target:
                self.target[side], _ = ik.tcp_pose(self._joints(side), self.base[side])
            prev = np.asarray(self.target[side], float)
            step = goal[side] - prev
            vmax = 0.25 / HZ
            n = np.linalg.norm(step)
            self.target[side] = prev + (step if n <= vmax else step / n * vmax)
            q = self._ik(side, self.target[side], PALM_R)
            acts.append(np.r_[q, 0.0])
        if clock - self.log_t >= 0.5:
            self.log_t = clock
            R = quat_matrix(tile[3:7])
            print(f"[demo] t={clock:.2f} tile=({tile[0]:.4f},{tile[1]:.4f},{tile[2] - G.FLANGE_Z:+.4f}) "
                  f"yaw={math.degrees(tile_yaw(R)):.2f} L={np.round(self.target['left'], 3)} "
                  f"R={np.round(self.target['right'], 3)}", flush=True)
        return np.concatenate(acts)


def make_demo_controller(env, plans):
    return DemoController(env, plans)
