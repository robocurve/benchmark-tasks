"""alarmkit: the fire alarm on its wall section, and the fire, for this task file, on yam_arena.

A `yam_arena` task file declares rigid shapes, so `AlarmPart` subclasses `Box` to pass its checks, and importing this module hooks `yam_arena.scene` (it modifies nothing on disk), so each part spawns from its USD
file and `object_states` reports the task's state. The glass is only seen: when a fingertip goes through it, the kit hides it and shows the shards. The handle is an articulation with one prismatic joint. When the handle counts as pulled, it latches and the strobes flash; the strobes are only seen, and nothing reads them.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
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
import alarm_usd as u  # noqa: E402
from alarm_score import Scorer  # noqa: E402

TOP = TABLE_TOP["station"]
ENV_PATH = "/World/envs/env_0"
ROBOTS = {"left": "left_robot", "right": "right_robot"}
FINGER_TIP = 0.147  # fingertips along the `gripper` link's z
FLASH_STEPS = 5  # the strobes are on for this many control steps, then off for as many: twice a second at 20 Hz


@dataclass(frozen=True)
class AlarmPart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it."""

    kind: str = ""


@dataclass(frozen=True)
class AlarmState(ObjectState):
    report: dict = field(default_factory=dict)


def parts() -> list[AlarmPart]:
    def p(name):
        return AlarmPart(name, pos=(0.3, -0.1, TOP + 0.05), size=(0.1, 0.1, 0.1), kind=name)

    return [p("alarm_wall"), p("alarm_box"), p("alarm_pane"), p("alarm_shards"), p("alarm_handle"), p("alarm_strobes"), p("alarm_strobes_lit"), p("alarm_fire")]


_NAMES = {p.name for p in parts()}


def _arena_object(spec: AlarmPart):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    d = os.path.join(tempfile.gettempdir(), f"fire_alarm_{os.getpid()}")
    os.makedirs(d, exist_ok=True)
    on_table = Pose(position_xyz=(0.0, 0.0, float(TOP)))
    root = "".join(w.capitalize() for w in spec.name.split("_"))
    path = os.path.join(d, f"{spec.name}.usda")
    if spec.kind == "alarm_handle":
        return _handle(spec.name, u.write_handle_usda(path))
    if spec.kind == "alarm_wall":
        usd = u.write_part(path, root, u.wall_parts(), u.wall_colliders(root))
    elif spec.kind == "alarm_box":
        usd = u.write_part(path, root, u.box_parts(), u.box_colliders(root))
    elif spec.kind == "alarm_pane":
        usd = u.write_part(path, root, u.pane_parts())
    elif spec.kind == "alarm_shards":
        usd = u.write_part(path, root, u.shard_parts(), hidden=True)
    elif spec.kind == "alarm_strobes":
        usd = u.write_part(path, root, u.strobe_parts(), u.strobe_colliders(root))
    elif spec.kind == "alarm_strobes_lit":
        usd = u.write_part(path, root, u.strobes_lit_parts(), hidden=True, extra=u.strobe_lights())
    else:
        usd = u.write_part(path, root, u.fire_parts())
    return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=on_table)


def _handle(name: str, usd: str):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    actuators = {"slider": ImplicitActuatorCfg(joint_names_expr=["slider_joint"], stiffness=40.0, damping=5.0, joint_effort_limit=20.0)}

    class ActuatedObject(Object):  # Arena's own articulation object has neither actuators nor initial joint positions
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=False), init_state=ArticulationCfg.InitialStateCfg(joint_pos={"slider_joint": 0.0}), actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    o = u.ORIGIN
    w, x, y, z = u.SLOPE_WXYZ
    return ActuatedObject(name=name, object_type=ObjectType.ARTICULATION, usd_path=usd, initial_pose=Pose(position_xyz=(float(o[0]), float(o[1]), float(TOP + o[2])), rotation_xyzw=(float(x), float(y), float(z), float(w))))


# -- the state --------------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotate(q, v):
    """Rotate v by the unit quaternion q (x, y, z, w)."""
    a, w = q[:3], q[3]
    return v + 2.0 * np.cross(a, np.cross(a, v) + w * v)


_VISIBLE: dict[str, bool] = {}


def set_visible(path: str, visible: bool) -> None:
    """Show or hide a prim, only when that changes."""
    import isaaclab.sim as sim_utils
    from pxr import UsdGeom

    if _VISIBLE.get(path) == visible:
        return
    _VISIBLE[path] = visible
    prim = sim_utils.get_current_stage().GetPrimAtPath(path)
    if prim.IsValid():
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if visible else img.MakeInvisible()


class _Report:
    """Reads the scene once per control step, hands it to the scorer, acts on what it says, and writes the task's state to alarm_scores/ (or $ALARM_SCORE_DIR), because the rollout's log keeps only the final score."""

    def __init__(self, env):
        self.base = env.unwrapped
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        root = os.environ.get("ALARM_SCORE_DIR", str(pathlib.Path(__file__).parent / "alarm_scores"))
        os.makedirs(root, exist_ok=True)
        self.path = os.path.join(root, f"score_{stamp}.json")
        self.scorer = Scorer(self.base.step_dt)
        self.handle = self.base.scene["alarm_handle"]
        self.joint = self.handle.find_joints("slider_joint")[0][0]
        self.last_buf = -1
        self.reset()

    def reset(self) -> None:
        """A new trial: the glass whole, the handle free, the strobes off."""
        self.scorer.reset()
        self.trace: list[dict] = []
        self.last: dict | None = None
        self.gone = False
        self.lit = False
        self.lit_at = 0
        self.wall: list[float] = []
        self.worst_30 = 0.0
        set_visible(f"{ENV_PATH}/alarm_pane/visual", True)
        set_visible(f"{ENV_PATH}/alarm_shards/visual", False)
        set_visible(f"{ENV_PATH}/alarm_strobes_lit/visual", False)

    def _fingertips(self, name: str) -> np.ndarray:
        robot = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        i = robot.find_bodies("gripper")[0][0]
        gp = (_t(robot.data.body_pos_w)[0, i] - o).cpu().numpy().astype(float)
        gq = _t(robot.data.body_quat_w)[0, i].cpu().numpy().astype(float)
        f = abs(float(_t(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]].item()))
        return np.array([gp + _rotate(gq, np.array((s * f, 0.0, FINGER_TIP))) for s in (-1, 1)])

    def _break_glass(self) -> None:
        set_visible(f"{ENV_PATH}/alarm_pane/visual", False)
        set_visible(f"{ENV_PATH}/alarm_shards/visual", True)
        self.gone = True

    def _latch(self) -> None:
        import torch

        self.handle.set_joint_position_target(torch.tensor([[-u.TRAVEL]], device=self.base.device), joint_ids=[self.joint])
        self.lit = True

    def step(self) -> dict:
        buf = int(self.base.episode_length_buf[0])
        if buf < self.last_buf:
            self.reset()
        self.last_buf = buf
        now = time.monotonic()
        self.wall.append(now)
        if len(self.wall) > 30:
            span = now - self.wall.pop(0)
            if self.scorer.steps > 90:
                self.worst_30 = max(self.worst_30, span)
        tips = {side: self._fingertips(name) for side, name in ROBOTS.items()}
        slid = -float(_t(self.handle.data.joint_pos)[0, self.joint].item())
        rep = self.scorer.update(np.vstack(list(tips.values())), slid)
        if rep["broken"] and not self.gone:
            self._break_glass()
        if rep["pulled"]:  # the score's own word, so the lights come on exactly when the alarm counts as set off. Nothing reads the lights
            if not self.lit:
                self._latch()
                self.lit_at = rep["step"]
            set_visible(f"{ENV_PATH}/alarm_strobes_lit/visual", (rep["step"] - self.lit_at) % (2 * FLASH_STEPS) < FLASH_STEPS)
        rep["tips"] = {k: [[round(float(x), 4) for x in p] for p in v] for k, v in tips.items()}
        rep["wall_s_per_30_steps_worst"] = round(self.worst_30, 2)
        if rep["step"] % 5 == 0 or self.last is None or self.last["score"] != rep["score"]:
            rep["wall_s"] = round(now, 2)
            self.trace.append(rep)
            with open(self.path, "w") as f:
                json.dump({"final": rep, "trace": self.trace}, f, indent=0)
        self.last = rep
        return rep


_report: _Report | None = None


def _patch_scene(scene) -> None:
    if getattr(scene, "_alarmkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, AlarmPart) else orig_task_object(spec)

    def object_states(env, names):
        global _report
        if _report is None or _report.base is not env.unwrapped:
            _report = _Report(env)
        rep = _report.step()
        others = [n for n in names if n not in _NAMES]
        out = orig_states(env, others) if others else {}
        zero = np.zeros(3)
        for n in names:
            if n in _NAMES:
                out[n] = AlarmState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._alarmkit_patched = True


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


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Finder) for f in sys.meta_path):
    sys.meta_path.insert(0, _Finder())
