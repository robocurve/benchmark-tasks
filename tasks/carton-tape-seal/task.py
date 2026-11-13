"""Packing bench: "Tape the carton shut."

A filled carton whose long top flaps spring back up a little, and a hand tape gun. One strip of tape
goes along the flaps' seam, the flaps held flat under it, lapping down both end faces: the gun is set
down at the near end with its leader hanging over it, run along the seam and on past the far edge,
lifted so the stretched tape tears on its blade, and both hanging ends are smoothed down by hand.
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
import tape as T  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "carton-tape-seal"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 1600
OBJECT_NAMES = ("carton", "flap_l", "flap_r", "gun", "roller")

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (T.carton_object(), T.flap_object("flap_l"), T.flap_object("flap_r"), T.gun_object(), T.roller_object())
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_carton_tape_seal", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


class Dynamics:
    """Packing tape, from measured poses only (a disclosed stand-in for an adhesive film).

    The strip is cells 8 mm long along the seam and down both end faces. The gun dispenses tape as its
    roller moves along the strip; a cell is stuck where the roller pressed it on (axle within 3 mm of
    its working height on the board). Cells dispensed with the roller lifted bridge, and the tape's
    free ends hang: both are drawn 4 mm off the board. A hanging cell on an end face sticks when a
    fingertip (pad within 15 mm of the face) is on it and the cell between it and the edge is already
    down, so the tabs are smoothed on from the edge outward. The gun carries 75 mm of free tape past
    the roller: set down within 30 mm of the near edge, it hangs down the near end. Run on 8 mm past
    the far edge and lifted 12 mm, the tape tears on the blade (a position rule, not a contact), leaving
    a tail of up to 60 mm hanging down the far end. Once three cells are stuck the tape holds the
    flaps: their spring hinges are set closed. A flap standing open more than 3 degrees when a cell
    went on is recorded as a gap.
    """

    def __init__(self, env):
        import omni.usd
        self.base = env.unwrapped
        stage = omni.usd.get_context().get_stage()
        self.cells = [None] * T.CELLS
        self.flap_target = {}
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            for k in range(T.CELLS):
                if path.endswith(f"/carton/piece{T.TAPE_PIECE0 + k}"):
                    self.cells[k] = prim
            for n in T.FLAPS:
                if path.endswith(f"/{n}/hinge_joint"):
                    self.flap_target[n] = prim.GetAttribute("drive:angular:physics:targetPosition")
        if any(c is None for c in self.cells) or len(self.flap_target) != 2:
            raise RuntimeError("tape cells or flap hinges not found on the stage")

    def _show(self, k, offset, hanging=False):
        """Draw cell k: stuck flat on the board, or hanging / bridging 4 mm off it."""
        from pxr import Gf, UsdGeom
        prim = self.cells[k]
        local, where = T.cell_local(k, offset)
        if hanging:
            local = local + {"near": np.array([-0.004, 0.0, 0.0]), "far": np.array([0.004, 0.0, 0.0]),
                             "top": np.array([0.0, 0.0, 0.004])}[where]
        ops = [op for op in UsdGeom.Xformable(prim).GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        attr = ops[0].GetAttr()
        attr.Set(Gf.Vec3f(*map(float, local)) if attr.GetTypeName() == "float3" else Gf.Vec3d(*map(float, local)))
        UsdGeom.Imageable(prim).MakeVisible()

    def reset(self):
        from pxr import UsdGeom
        for prim in self.cells:
            UsdGeom.Imageable(prim).MakeInvisible()
        self.state = ["none"] * T.CELLS          # none | hanging | stuck
        self.offset = [0.0] * T.CELLS
        self.gap = [0.0] * T.CELLS
        self.started = False
        self.cut = False
        self.cut_at = None
        self.head = None                         # how far along the strip tape has been dispensed
        self.held_down = False
        for n in T.FLAPS:
            self.flap_target[n].Set(float(-T.flap_sign(n) * T.FLAP_POP_DEG))

    def before_step(self, step, action):
        pass

    def _hang(self, k, offset):
        if 0 <= k < T.CELLS and self.state[k] == "none":
            self.state[k] = "hanging"
            self.offset[k] = offset
            self._show(k, offset, hanging=True)

    def _stick(self, k, offset, gap):
        if 0 <= k < T.CELLS and self.state[k] != "stuck":
            self.state[k] = "stuck"
            self.offset[k] = offset
            self.gap[k] = gap
            self._show(k, offset)

    def after_step(self, step, objects, root_poses, robot):
        carton = np.asarray(root_poses["carton"], float)
        gun = np.asarray(root_poses["gun"], float)
        roller, blade, fwd, up = T.gun_points(gun)
        gap = max(T.flap_angle(n, root_poses[n]) for n in T.FLAPS)
        contact = T.roller_contact(carton, roller)
        if contact is not None and not self.cut:
            s, off = contact
            if not self.started and 0.0 <= s <= 0.03:
                self.started = True
                self.head = s
                for k in range(T.cell_of(s - T.LEADER), T.cell_of(s) + 1):
                    if 0 <= k < T.CELLS:
                        if T.cell_s(k) >= 0:
                            self._stick(k, off, gap)          # the leader end pressed onto the top by the roller
                        else:
                            self._hang(k, off)
            if self.started:
                lo, hi = sorted((self.head, s))
                for k in range(T.cell_of(lo), T.cell_of(hi) + 1):
                    self._hang(k, off)                        # dispensed while the roller passed: bridges
                self._stick(T.cell_of(s), off, gap)           # the cell under the roller is pressed on
                self.head = max(self.head, s)
        if self.started and not self.cut:
            # tape off the roll follows the gun past the last pressed cell; the blade biting it cuts it
            bite = T.blade_cut(carton, roller, blade, fwd)
            if bite is not None and bite > self.head - 0.004:
                self.cut = True
                self.cut_at = bite
                for k in range(T.cell_of(self.head), T.cell_of(bite) + 1):
                    self._hang(k, self.offset[T.cell_of(self.head)] if 0 <= T.cell_of(self.head) < T.CELLS else 0.0)
        # a fingertip smooths hanging tape down an end face, cell by cell from the edge outward: a cell
        # sticks when the pad is on it and the cell between it and the edge is already down
        for side in ("left", "right"):
            tcp = np.asarray(robot[side]["tcp"], float)[:3]
            s = T.hand_press(carton, tcp - np.array([0.0, 0.0, PAD_TIP]))
            if s is None:
                continue
            k = T.cell_of(s)
            if not (0 <= k < T.CELLS) or self.state[k] != "hanging":
                continue
            if s < 0:
                inner = k + 1                                  # toward the near edge
                ok = inner == T.TAB_CELLS or self.state[inner] == "stuck"
            else:
                inner = k - 1                                  # toward the far edge
                ok = inner == T.TAB_CELLS + T.TOP_CELLS - 1 or self.state[inner] == "stuck"
            if ok:
                self._stick(k, self.offset[k], 0.0)
        top = [k for k in range(T.TAB_CELLS, T.TAB_CELLS + T.TOP_CELLS) if self.state[k] == "stuck"]
        if not self.held_down and len(top) >= 3:
            self.held_down = True
            for n in T.FLAPS:
                self.flap_target[n].Set(0.0)                  # the tape holds both flaps down now
        left = sum(1 for k in range(T.TAB_CELLS) if self.state[T.TAB_CELLS - 1 - k] == "stuck"
                   and all(self.state[T.TAB_CELLS - 1 - j] == "stuck" for j in range(k + 1)))
        right = sum(1 for k in range(T.TAB_CELLS) if all(self.state[T.TAB_CELLS + T.TOP_CELLS + j] == "stuck"
                                                          for j in range(k + 1)))
        offs = [abs(self.offset[k]) for k in top]
        return {"top_stuck": len(top), "top_cells": T.TOP_CELLS,
                "max_offset_mm": round(1000 * max(offs), 1) if offs else 0.0,
                "max_gap_deg": round(max((self.gap[k] for k in top), default=0.0), 1),
                "near_tab_mm": left * T.CELL * 1000, "far_tab_mm": right * T.CELL * 1000,
                "unstuck_top": sum(1 for k in range(T.TAB_CELLS, T.TAB_CELLS + T.TOP_CELLS) if self.state[k] == "hanging"),
                "started": bool(self.started), "cut": bool(self.cut), "flaps_held": bool(self.held_down),
                "flap_open_deg": round(gap, 1)}


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------

REST = {"left": (0.16, 0.24, T.TOP + 0.20), "right": (0.16, -0.24, T.TOP + 0.20)}
HANDLE_FINGER = -(2 * T.HANDLE[0] - 0.010) / 2
GRIP0 = DOWN                                           # the hand's rotation on the standing gun's handle
GUN0 = T.gun_rot()
GRIP_LOCAL = np.array([T.HANDLE_AT[0], 0.0, T.HANDLE_AT[1]])
ROLLER_LOCAL = np.array([T.ROLLER_AT[0], 0.0, T.ROLLER_AT[1]])


def _gun_hand(seconds, roller_xyz, pitch, finger, label, lift=0.0):
    """TCP that puts the gun's roller axis at ``roller_xyz`` with its nose pitched down by ``pitch``."""
    rg = T.gun_rot(pitch=pitch)
    centre = np.asarray(roller_xyz, float) - rg @ ROLLER_LOCAL
    grip = centre + rg @ GRIP_LOCAL + np.array([0.0, 0.0, lift])
    rot = rg @ GUN0.T @ GRIP0
    tcp = grip - rot @ np.array([0.0, 0.0, PAD_TIP])
    return Waypoint(seconds, tuple(map(float, tcp)), finger, label, rot)


def _press(seconds, xyz, label, finger=FINGER_CLOSED, rot=None):
    """Closed fingertips at ``xyz``."""
    rot = DOWN if rot is None else rot
    tip = np.asarray(xyz, float)
    return Waypoint(seconds, tuple(map(float, tip - rot @ np.array([0.0, 0.0, PAD_TIP]))), finger, label, rot)


def demo():
    r = T.ROLLER[0]
    top = T.TOP_SURF
    sy = T.SEAM_Y
    x_far, x_near = T.CARTON_C[0] + T.END_X, T.CARTON_C[0] - T.END_X
    rest_roller = np.array([T.GUN_REST[0], T.GUN_REST[1], T.TOP + r]) + GUN0 @ np.array([T.ROLLER_AT[0], 0.0, 0.0])
    press = 0.0015                                   # roller pressed this far into the top
    x_start = x_near + 0.010
    x_end = x_far - 0.004
    f = HANDLE_FINGER
    right = [
        Waypoint(1.5, REST["right"], FINGER_OPEN, "look", DOWN),
        _gun_hand(2.0, rest_roller, 0.0, FINGER_OPEN, "over the tape gun's handle", lift=0.05),
        _gun_hand(1.2, rest_roller, 0.0, FINGER_OPEN, "down round it"),
        _gun_hand(0.8, rest_roller, 0.0, f, "take it"),
        _gun_hand(1.5, rest_roller + np.array([0.0, 0.0, 0.06]), 0.0, f, "lift it"),
        _gun_hand(2.0, (x_start, sy, top + r + 0.03), 0.0, f, "over the near end of the seam"),
        _gun_hand(1.2, (x_start, sy, top + r - press), 0.0, f, "roller down on the seam, leader over the end"),
        _gun_hand(0.6, (x_start, sy, top + r - press), 0.0, f, "press the leader end on"),
    ]
    n = 8
    for k in range(1, n + 1):
        x = x_start + (x_end - x_start) * k / n
        right.append(_gun_hand(0.5, (x, sy, top + r - press), 0.0, f, "run it along the seam"))
    right += [
        _gun_hand(1.2, (x_far + 0.040, sy, top + r + 0.004), 0.0, f, "run it on past the far edge, the tape stretched over it"),
        _gun_hand(0.6, (x_far + 0.040, sy, top + r + 0.016), 0.0, f, "snap it up: the blade tears the tape"),
        _gun_hand(1.0, (x_far - 0.02, sy, top + r + 0.02), 0.0, f, "lift it away"),
        _gun_hand(2.0, rest_roller + np.array([0.0, 0.0, 0.04]), 0.0, f, "back to the bench"),
        _gun_hand(1.0, rest_roller, 0.0, f, "stand it down"),
        _gun_hand(0.6, rest_roller, 0.0, FINGER_OPEN, "let go"),
        _gun_hand(0.8, rest_roller, 0.0, FINGER_OPEN, "lift off", lift=0.06),
        Waypoint(1.5, REST["right"], FINGER_OPEN, "stand clear", DOWN),
    ]
    t_start = sum(w.duration for w in right[:7])       # the roller is down on the seam
    t_roll = t_start + 0.6                              # it starts along the seam
    t_cut = sum(w.duration for w in right[:8 + n + 2])
    hold_x = x_far - 0.015
    left = [
        Waypoint(1.5, REST["left"], FINGER_OPEN, "look", DOWN),
        _press(max(0.5, t_start - 1.5 - 1.0 - 0.6), (hold_x, sy, top + 0.03), "over the far half of the seam"),
        _press(0.6, (hold_x, sy, top - 0.002), "hold the flaps down for the gun"),
        _press(1.0 + 0.6 + 0.9, (hold_x, sy, top - 0.002), "hold the flaps down while the tape takes them"),
        _press(0.8, (x_far + 0.05, sy + 0.10, top + 0.06), "out of the gun's way, beyond the far left corner"),
    ]
    t_l = sum(w.duration for w in left)
    left += [
        _press(max(0.5, t_cut + 1.2 - t_l), (x_far + 0.04, sy + 0.06, top + 0.05), "wait beyond the far end"),
        _press(1.2, (x_far + FINGER_HALF, sy, top + 0.02), "beyond the far end", rot=ALONG_X),
        _press(0.8, (x_far + FINGER_HALF, sy, top - 0.004), "fingers on the tail", rot=ALONG_X),
        _press(1.6, (x_far + FINGER_HALF - 0.002, sy, top - 0.047), "smooth it down the end", rot=ALONG_X),
        _press(0.8, (x_far + 0.03, sy + 0.10, top + 0.06), "off it"),
        _press(1.8, (x_near - 0.02, sy + 0.12, top + 0.06), "round the left of the carton"),
        _press(1.0, (x_near - 0.05, sy + 0.03, top + 0.05), "to the near end"),
        _press(0.8, (x_near - FINGER_HALF, sy, top - 0.004), "fingers on the hanging leader", rot=ALONG_X),
        _press(1.6, (x_near - FINGER_HALF + 0.002, sy, top - 0.047), "smooth it down the end", rot=ALONG_X),
        _press(0.8, (x_near - 0.05, sy + 0.06, top + 0.06), "clear"),
        Waypoint(1.5, REST["left"], FINGER_OPEN, "stand clear", DOWN),
    ]
    t_r = sum(w.duration for w in right)
    t_l = sum(w.duration for w in left)
    (left if t_l < t_r else right).append(Waypoint(abs(t_r - t_l) + 1.5, REST["left" if t_l < t_r else "right"],
                                                   FINGER_OPEN, "done", DOWN))
    return {"left": left, "right": right}


FINGER_HALF = 0.010                                    # closed fingers' half thickness along the jaw axis
ALONG_X = simkit.rot_z(math.pi / 2) @ DOWN             # jaws along x: the closed pair's flat face toward an end face
