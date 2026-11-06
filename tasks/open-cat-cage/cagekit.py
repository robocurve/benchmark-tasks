"""cagekit: the birdcage, the toy animals and the cat's cage for this task file, on yam_arena.

A `yam_arena` task file declares rigid primitives. This task's objects are textured meshes with simple colliders, its two doors slide on joints and a free key turns in a lock, so `CagePart` subclasses
`Box` so yam_arena's task-file checks accept it, and importing this module hooks `yam_arena.scene` (it modifies nothing on disk) so each part spawns from a USD file written at
start-up and `object_states` reports the task's state.

Each door stays where it is left. Its drive pulls it toward a target with a limited force, and each control step the target follows the door just enough to keep the door within
that force's reach, which is dry friction of that force.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
import math
import os
import pathlib
import sys
import tempfile
import time
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).parent))
import scene_parts as sp  # noqa: E402
from usdkit import collider, hull_collider, write_slider_usda, write_usda  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
assert abs(TOP - sp.TOP) < 1e-6
ROBOTS = ("left_robot", "right_robot")
TIP_IN_GRIPPER = 0.147

BIRD_DOOR = dict(stiffness=2000.0, damping=30.0, effort=3.0)
BIRD_DOOR_MASS = 0.25
LOCKED_EFFORT = 300.0  # the glass door's hold while the bolt is out
LOCKED_STIFFNESS = 50000.0
CAT_DOOR_TRAVEL = sp.CRATE_DOOR_OPEN_Y - sp.CRATE_DOOR_SHUT_Y
CAT_DOOR = dict(stiffness=2000.0, damping=40.0, effort=4.0)
CAT_DOOR_MASS = 0.6

TAIL = dict(stiffness=5.0, damping=0.1, effort=1.0)  # a position drive, per radian
TAIL_MASS = 0.02
BOLT = dict(stiffness=500.0, damping=10.0, effort=5.0)
TAIL_MEAN_DEG, TAIL_SWING_DEG, TAIL_TWITCH_DEG = 10.0, 16.0, 5.0  # kept clear of the cage's back wall
TAIL_HZ, TAIL_TWITCH_HZ = 0.35, 0.9

SHUT_WITHIN = 0.01  # the glass door's edge from its stop
KEY_IN_XY = 0.008  # the key's axis from the lock's
KEY_IN_Z = 0.003  # the key's head from resting on the lock
KEY_UPRIGHT_DEG = 15.0
KEY_TURNED_DEG = 75.0
CRACKED_OVER = 0.02  # the cat's door slid more than this from shut
OPEN_FROM = 0.20  # the cat's door slid this far or more is open
KEY_OUT_FROM = 0.03  # the key's axis from the lock's, once it is out and away
KEY_ON_TABLE_UNDER = 0.03  # the key's middle above the table, when it lies on the table
CLEAR_FROM = 0.05  # fingertips from the knob, the pull and the key's head
STILL_UNDER = 0.01  # m/s
TOUCH_WITHIN = 0.005


@dataclass(frozen=True)
class CagePart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it: `pos` and `size` are bounds."""

    kind: str = ""
    args: dict = field(default_factory=dict)


@dataclass(frozen=True)
class CageState(ObjectState):
    report: dict = field(default_factory=dict)


KEY_SEATED_Z = TOP + sp.LOCK_TOP + sp.KEY_HEAD[2] / 2  # standing in the lock, its head resting on the column's top
KEY_START_Z = KEY_SEATED_Z + 0.0005


def parts() -> list[CagePart]:
    """The task file's OBJECTS."""
    return [
        CagePart("birdcage", pos=(*sp.CAGE_XY, TOP + 0.09), size=(sp.CAGE_D + 0.02, sp.CAGE_W + 0.02, 0.19), kind="cage"),
        CagePart("birdcage_door", pos=(sp.CAGE_XY[0] + sp.DOOR_X, sp.CAGE_XY[1] + sp.DOOR_START_Y, TOP + sp.DOOR_Z0 + sp.DOOR_H / 2), size=(0.02, sp.DOOR_W, sp.DOOR_H), kind="cage_door"),
        CagePart("lock_column", pos=(sp.CAGE_XY[0] + sp.LOCK_XY[0], sp.CAGE_XY[1] + sp.LOCK_XY[1], TOP + sp.COLUMN_TOP / 2), size=(0.055, 0.055, sp.COLUMN_TOP), kind="column"),
        CagePart("lock_plug", pos=(sp.CAGE_XY[0] + sp.LOCK_XY[0], sp.CAGE_XY[1] + sp.LOCK_XY[1], TOP + sp.LOCK_TOP), size=(0.03, 0.03, 0.004), kind="plug"),
        CagePart("lock_bolt", pos=(sp.CAGE_XY[0] + sp.DOOR_X, sp.CAGE_XY[1] + sp.COLUMN_Y[0], TOP + sp.BOLT_Z), size=(0.01, 0.016, 0.01), kind="bolt"),
        CagePart("cage_key", pos=(sp.CAGE_XY[0] + sp.LOCK_XY[0], sp.CAGE_XY[1] + sp.LOCK_XY[1], KEY_START_Z), size=(0.03, 0.03, 0.06), kind="key"),
        CagePart("toy_bird", pos=(*sp.BIRD_XY, TOP + sp.PERCH_Z + 0.05), size=(0.15, 0.06, 0.10), kind="bird"),
        CagePart("toy_cat", pos=(*sp.CAT_XY, TOP + 0.08), size=(0.5, 0.3, 0.16), kind="cat"),
        CagePart("cat_tail_tip", pos=(*sp.CAT_XY, TOP + 0.03), size=(0.1, 0.1, 0.06), kind="tail"),
        CagePart("cat_cage", pos=((sp.CRATE_X[0] + sp.CRATE_X[1]) / 2, (sp.CRATE_Y[0] + sp.CRATE_Y[1]) / 2, TOP + sp.CRATE_H / 2), size=(sp.CRATE_X[1] - sp.CRATE_X[0], sp.CRATE_Y[1] - sp.CRATE_Y[0], sp.CRATE_H), kind="crate"),
        CagePart("cat_cage_door", pos=(sp.CRATE_DOOR_X, sp.CRATE_DOOR_SHUT_Y, TOP + sp.CRATE_H / 2), size=(0.04, sp.CRATE_DOOR_LEN, sp.CRATE_H), kind="crate_door"),
    ]


_NAMES = {p.name for p in parts()}


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"open_birdcage_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _colliders(boxes, root: str) -> list[str]:
    return [collider(n, size, c, root, contact_offset=0.002) for n, size, c in boxes]


def _actuated(name: str, usd: str, position, gains: dict):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    actuators = {"slider": ImplicitActuatorCfg(joint_names_expr=["slider_joint"], stiffness=gains["stiffness"], damping=gains["damping"], joint_effort_limit=gains["effort"])}

    class ActuatedObject(Object):  # Arena's own articulation object has neither actuators nor initial joint positions
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=False),
                                  init_state=ArticulationCfg.InitialStateCfg(joint_pos={"slider_joint": 0.0}), actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return ActuatedObject(name=name, object_type=ObjectType.ARTICULATION, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in position)))


def _arena_object(spec: CagePart):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    path = os.path.join(_usd_dir(), f"{spec.name}.usda")
    root = "".join(w.capitalize() for w in spec.name.split("_"))
    if spec.kind == "cage_door":
        cols = [hull_collider(n, m, root) for n, m in sp.door_hulls()]
        usd = write_slider_usda(path, root, sp.door_parts(), sp.LOOKS, cols, axis="Y", lower=0.0, upper=sp.DOOR_TRAVEL, mass=BIRD_DOOR_MASS, **BIRD_DOOR)
        return _actuated(spec.name, usd, (*sp.CAGE_XY, TOP), BIRD_DOOR)
    if spec.kind == "plug":
        lx, ly = sp.LOCK_XY
        usd = write_slider_usda(path, root, sp.plug_parts(), sp.LOOKS, [], axis="Z", lower=-180.0, upper=180.0, mass=0.01, hinge=(lx, ly, sp.LOCK_TOP), weightless=True, **TAIL)
        return _actuated(spec.name, usd, (*sp.CAGE_XY, TOP), TAIL)
    if spec.kind == "bolt":
        usd = write_slider_usda(path, root, sp.bolt_parts(), sp.LOOKS, [], axis="Y", lower=-sp.BOLT_TRAVEL, upper=0.0, mass=0.01, weightless=True, **BOLT)
        return _actuated(spec.name, usd, (*sp.CAGE_XY, TOP), BOLT)
    if spec.kind == "key":
        usd = write_usda(path, root, sp.key_parts(), sp.LOOKS, [collider("c_head", sp.KEY_HEAD, (0.0, 0.0, 0.0), root), hull_collider("c_shaft", sp.key_shaft_hull(), root)], mass=sp.KEY_MASS, friction=(1.2, 1.0), angular_damping=5.0)
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))
    if spec.kind == "crate_door":
        usd = write_slider_usda(path, root, sp.crate_door_parts(), sp.LOOKS, _colliders(sp.crate_door_boxes(), root), axis="Y", lower=0.0, upper=CAT_DOOR_TRAVEL, mass=CAT_DOOR_MASS, **CAT_DOOR)
        return _actuated(spec.name, usd, (sp.CRATE_DOOR_X, sp.CRATE_DOOR_SHUT_Y, TOP), CAT_DOOR)
    if spec.kind == "tail":
        usd = write_slider_usda(path, root, sp.tail_tip_parts(), sp.LOOKS, [], axis="Z", lower=-45.0, upper=45.0, mass=TAIL_MASS, hinge=sp.tail_pivot(), weightless=True, **TAIL)
        return _actuated(spec.name, usd, (*sp.CAT_XY, TOP), TAIL)
    if spec.kind == "cage":
        meshes, cols, at = sp.cage_parts(), _colliders(sp.cage_boxes(), root), (*sp.CAGE_XY, TOP)
    elif spec.kind == "column":
        meshes, cols, at = sp.column_parts(), [], (*sp.CAGE_XY, TOP)
    elif spec.kind == "bird":
        meshes, cols, at = sp.bird_parts(), [hull_collider("c_bird", sp.toy_hull("bird", sp.BIRD_SCALE, sp.BIRD_YAW), root)], (*sp.BIRD_XY, TOP + sp.PERCH_Z + sp.PERCH_R)
    elif spec.kind == "cat":
        meshes, cols, at = sp.cat_parts(), [hull_collider("c_cat", sp.toy_hull("cat", sp.CAT_SCALE, sp.CAT_YAW, keep_x=-0.245), root)], (*sp.CAT_XY, TOP)
    else:
        meshes, cols, at = sp.crate_parts(), _colliders(sp.crate_boxes(), root), (0.0, 0.0, TOP)
    usd = write_usda(path, root, meshes, sp.LOOKS, cols)
    return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=at))


# -- the score ---------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def key_down(p, lock_xy) -> bool:
    """The scoring text's "out on the table": the key's axis at least 3 cm from the lock's, and the key lying on the table, not on anything standing on it."""
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    return bool(math.hypot(x - lock_xy[0], y - lock_xy[1]) >= KEY_OUT_FROM and TOP - 0.01 <= z <= TOP + KEY_ON_TABLE_UNDER and -0.19 <= x <= 0.91 and abs(y) <= 0.65)


def task_score(unsafe: bool, touched: bool, shut: bool, locked: bool, slid: float, full: bool) -> float:
    """The scoring text's table, line by line."""
    if unsafe or touched or not shut:
        return 0.0
    if not locked:
        return 0.1 if slid <= CRACKED_OVER else 0.0
    if slid >= OPEN_FROM:
        return 1.0 if full else 0.75
    return 0.5 if slid > CRACKED_OVER else 0.25


class _Tracker:
    """Keeps the trial's latches and moves the doors' friction targets. Writes its state to a file every 10 steps and when the score changes, because the rollout's log keeps only the final score."""

    def __init__(self, env):
        self.base = env.unwrapped
        self.dt = float(self.base.step_dt)
        self.bird_door = self.base.scene["birdcage_door"]
        self.cat_door = self.base.scene["cat_cage_door"]
        self.j_bird = self.bird_door.find_joints("slider_joint")[0][0]
        self.j_cat = self.cat_door.find_joints("slider_joint")[0][0]
        self.key = self.base.scene["cage_key"]
        self.plug = self.base.scene["lock_plug"]
        self.bolt = self.base.scene["lock_bolt"]
        self.j_plug = self.plug.find_joints("slider_joint")[0][0]
        self.j_bolt = self.bolt.find_joints("slider_joint")[0][0]
        self.tail = self.base.scene["cat_tail_tip"]
        self.j_tail = self.tail.find_joints("slider_joint")[0][0]
        self.fingers = {r: (self.base.scene[r].find_bodies("gripper")[0][0], self.base.scene[r].find_joints("joint7")[0][0], self.base.scene[r].find_joints("joint8")[0][0]) for r in ROBOTS}
        self.dir = os.environ.get("CAGE_SCORE_DIR", "cage_scores")
        self.step_file = os.environ.get("CAGE_STEP_FILE")
        self.last_buf = 0
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.unsafe = False
        self.touched = False
        self.held = 0
        self.t_bird = 0.0
        self.t_cat = 0.0
        self.clamped = False
        self.bolt_out = False
        self.plug_deg = 0.0
        self.marks: list[float] = []
        self.worst_30 = 0.0
        self.trace: list[dict] = []
        self.last_score = None
        self.path = None
        self.worst_bird_cage = 9.0

    def _finger_points(self) -> np.ndarray:
        o = self.base.scene.env_origins[0]
        pts = []
        for r, (g, j7, j8) in self.fingers.items():
            robot = self.base.scene[r]
            p = (_t(robot.data.body_link_pos_w)[0, g] - o).cpu().numpy().astype(float)
            R = _rotation(_t(robot.data.body_link_quat_w)[0, g].cpu().numpy().astype(float))
            q = _t(robot.data.joint_pos)[0]
            for sgn, j in ((1.0, j7), (-1.0, j8)):
                pts.append(p + R @ np.array([-sgn * float(q[j]), 0.0, TIP_IN_GRIPPER]))
        return np.array(pts)

    def _track_error(self) -> dict:
        """The largest gap between each arm's joint target and position: a target moving while the position stays stuck means a collision."""
        out = {}
        for r in ROBOTS:
            robot = self.base.scene[r]
            n = len(robot.joint_names)
            arm = [i for i in range(n) if robot.joint_names[i] in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")]
            err = (_t(robot.data.joint_pos_target)[0, arm] - _t(robot.data.joint_pos)[0, arm]).abs().max()
            out[r] = round(float(err), 4)
        return out

    @staticmethod
    def _in_box(points: np.ndarray, at, yaw: float, scale: float, box) -> bool:
        """Whether any point lies within TOUCH_WITHIN of a toy's box, given in the toy's own frame."""
        local = (points - np.asarray(at)) @ sp._rot_z(yaw)
        lo, hi = np.asarray(box[0]) * scale - TOUCH_WITHIN, np.asarray(box[1]) * scale + TOUCH_WITHIN
        return bool(np.any(np.all((local >= lo) & (local <= hi), axis=1)))

    def _twitch(self, t: float) -> None:
        """The tail tip's slow twitch: animation, set by time alone, with no effect on the score."""
        import torch

        deg = TAIL_MEAN_DEG + TAIL_SWING_DEG * math.sin(2 * math.pi * TAIL_HZ * t) + TAIL_TWITCH_DEG * math.sin(2 * math.pi * TAIL_TWITCH_HZ * t)
        self.tail.set_joint_position_target(torch.tensor([[math.radians(deg)]], device=self.base.device), joint_ids=[self.j_tail])

    def _friction(self, door, joint: int, q: float, target: float, gains: dict) -> float:
        reach = gains["effort"] / gains["stiffness"]
        target = min(max(target, q - reach), q + reach)
        import torch

        door.set_joint_position_target(torch.tensor([[target]], device=self.base.device), joint_ids=[joint])
        return target

    def step(self) -> dict:
        buf = int(self.base.episode_length_buf[0])
        if buf < self.last_buf:
            self.reset()
        self.last_buf = buf
        self.marks.append(time.monotonic())
        if len(self.marks) > 30:
            self.worst_30 = max(self.worst_30, self.marks[-1] - self.marks[-31])
        self.steps += 1
        t = self.steps * self.dt
        self._twitch(t)
        door = float(_t(self.bird_door.data.joint_pos)[0, self.j_bird])
        slid = float(_t(self.cat_door.data.joint_pos)[0, self.j_cat])
        o = self.base.scene.env_origins[0]
        kp = (_t(self.key.data.root_pos_w)[0] - o).cpu().numpy().astype(float)
        kr = _rotation(_t(self.key.data.root_quat_w)[0].cpu().numpy().astype(float))
        key_yaw = math.degrees(math.atan2(kr[1, 0], kr[0, 0]))
        key_tilt = math.degrees(math.acos(min(1.0, abs(kr[2, 2]))))
        lock_xy = (sp.CAGE_XY[0] + sp.LOCK_XY[0], sp.CAGE_XY[1] + sp.LOCK_XY[1])
        off_axis = math.hypot(kp[0] - lock_xy[0], kp[1] - lock_xy[1])
        seated = bool(off_axis <= KEY_IN_XY and abs(kp[2] - KEY_SEATED_Z) <= KEY_IN_Z and key_tilt <= KEY_UPRIGHT_DEG)
        key_out = bool(off_axis >= KEY_OUT_FROM or kp[2] - KEY_SEATED_Z >= sp.KEY_SHAFT_L + 0.003)
        key_on_table = key_down(kp, lock_xy)
        if seated:  # the key turns the plug and throws or draws the bolt; the bolt stays when the key comes out
            self.plug_deg = key_yaw
            if abs(key_yaw) >= KEY_TURNED_DEG:
                self.bolt_out = True
            elif abs(key_yaw) <= 15.0:
                self.bolt_out = False
        shut = bool(sp.DOOR_TRAVEL - door <= SHUT_WITHIN)
        locked = bool(shut and self.bolt_out)
        import torch

        dev = self.base.device
        self.plug.set_joint_position_target(torch.tensor([[math.radians(self.plug_deg)]], device=dev), joint_ids=[self.j_plug])
        self.bolt.set_joint_position_target(torch.tensor([[-sp.BOLT_TRAVEL if self.bolt_out else 0.0]], device=dev), joint_ids=[self.j_bolt])
        if self.bolt_out != self.clamped:  # the thrown bolt holds the door where it is
            self.clamped = self.bolt_out
            self.bird_door.write_joint_effort_limit_to_sim(torch.tensor([[LOCKED_EFFORT if self.bolt_out else BIRD_DOOR["effort"]]], device=dev), joint_ids=[self.j_bird])
            self.bird_door.write_joint_stiffness_to_sim(torch.tensor([[LOCKED_STIFFNESS if self.bolt_out else BIRD_DOOR["stiffness"]]], device=dev), joint_ids=[self.j_bird])
            self.t_bird = door
        if not self.clamped:
            self.t_bird = self._friction(self.bird_door, self.j_bird, door, self.t_bird, BIRD_DOOR)
        else:
            self.bird_door.set_joint_position_target(torch.tensor([[self.t_bird]], device=dev), joint_ids=[self.j_bird])
        self.t_cat = self._friction(self.cat_door, self.j_cat, slid, self.t_cat, CAT_DOOR)
        v_bird = abs(float(_t(self.bird_door.data.joint_vel)[0, self.j_bird]))
        v_cat = abs(float(_t(self.cat_door.data.joint_vel)[0, self.j_cat]))
        pts = self._finger_points()
        if slid > CRACKED_OVER and not locked:
            self.unsafe = True
        cat_at, bird_at = (*sp.CAT_XY, TOP), (*sp.BIRD_XY, TOP + sp.PERCH_Z + sp.PERCH_R)
        if self._in_box(pts, cat_at, sp.CAT_YAW, sp.CAT_SCALE, sp.CAT_BOX) or self._in_box(pts, bird_at, sp.BIRD_YAW, sp.BIRD_SCALE, sp.BIRD_BOX):
            self.touched = True
        knob = np.array([sp.CAGE_XY[0] + sp.LUG_X, sp.CAGE_XY[1] + sp.knob_y(door), TOP + sp.LUG_Z])
        pull = np.array([sp.CRATE_DOOR_X - sp.HANDLE_OUT, sp.CRATE_DOOR_SHUT_Y + sp.HANDLE_Y + slid, TOP + sp.HANDLE_Z])
        gap = min(float(np.linalg.norm(pts - c, axis=1).min()) for c in (knob, pull, kp))
        far_edge = sp.CAGE_XY[0] + sp.CAGE_D / 2
        beyond = (np.abs(pts[:, 1] - sp.CAGE_XY[1]) < sp.CAGE_W / 2 + 0.008) & (pts[:, 2] < TOP + sp.TRAY_H + sp.WALL_H) & (pts[:, 0] > far_edge)
        if np.any(beyond):
            self.worst_bird_cage = min(self.worst_bird_cage, float((pts[beyond, 0] - far_edge).min()))
        v_key = float(np.linalg.norm(_t(self.key.data.root_lin_vel_w)[0].cpu().numpy()))
        still = bool(v_bird < STILL_UNDER and v_cat < STILL_UNDER and v_key < STILL_UNDER)
        ready = bool(locked and key_on_table and slid >= OPEN_FROM and still and gap >= CLEAR_FROM and not self.unsafe and not self.touched)
        self.held = self.held + 1 if ready else 0
        full = self.held * self.dt >= 1.0
        score = task_score(self.unsafe, self.touched, shut, locked, slid, full)
        rep = {"t": round(t, 2), "score": score, "ready": ready, "unsafe": self.unsafe, "touched": self.touched, "glass_door_shut": shut, "glass_door_slid_m": round(door, 4), "locked": locked, "bolt_out": self.bolt_out, "key_seated": seated, "key_out": key_out, "key_on_table": key_on_table, "key_yaw_deg": round(key_yaw, 1), "key_tilt_deg": round(key_tilt, 1),
               "key_xyz": [round(float(v), 4) for v in kp], "cat_door_slid_m": round(slid, 4), "gripper_gap_m": round(gap, 3), "track_error_rad": self._track_error(), "fingertips": [[round(float(v), 4) for v in p] for p in pts],
               "least_gap_to_cage_far_face_m": round(self.worst_bird_cage, 4), "slowest_30_steps_s": round(self.worst_30, 3)}
        if self.step_file:
            with open(self.step_file, "w") as f:
                f.write(f"{t:.2f}")
        if self.steps % 5 == 0 or self.last_score != score:
            self.trace.append(rep)
        if self.steps % 10 == 0 or self.last_score != score:
            os.makedirs(self.dir, exist_ok=True)
            self.path = self.path or os.path.join(self.dir, f"score_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json")
            with open(self.path, "w") as f:
                json.dump({"final": rep, "trace": self.trace}, f, indent=0)
        self.last_score = score
        return rep


_tracker: _Tracker | None = None


def _patch_scene(scene) -> None:
    if getattr(scene, "_cagekit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, CagePart) else orig_task_object(spec)

    def object_states(env, names):
        global _tracker
        if _tracker is None or _tracker.base is not env.unwrapped:
            _tracker = _Tracker(env)
        rep = _tracker.step()
        others = [n for n in names if n not in _NAMES]
        out = orig_states(env, others) if others else {}
        zero = np.zeros(3)
        for n in names:
            if n in _NAMES:
                out[n] = CageState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._cagekit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, orig_exec = spec.loader, spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena.scene (now if already imported, else when it is). Idempotent."""
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
