"""Inspect Robots embodiment ``yam_pair``: two bimanual YAM stations (four arms) on one table in Isaac Lab-Arena.

    inspect-robots run --embodiment yam_pair -E task=task.py --policy dream_pair --instruction "..." ...

Observation: ``a_top_cam a_left_cam a_right_cam b_top_cam b_left_cam b_right_cam`` (+ ``overview_cam`` with
``-E overview=true``), state ``a_joint_pos``/``b_joint_pos`` (14-D each, yam_arena's layout). Action: 28-D, A's 14-D
then B's 14-D, absolute joint targets. Each step holds the target for one control period (240 Hz physics, 20 Hz
control by default, one render per step). See CONTRACT.md.

Importing this module is cheap: Isaac Sim starts on the first ``reset()``.
"""

from __future__ import annotations

import os
import sys
from typing import Any

import numpy as np
from inspect_robots import Scene
from inspect_robots.embodiment import PRIVILEGED_SUCCESS, RESETTABLE, SEEDABLE, EmbodimentBase, EmbodimentInfo
from inspect_robots.types import Observation, StepResult

from yam_pair import boundary, taskfile
from yam_pair.layout import SIDES, STATIONS, Layout
from yam_pair.task import PairState

SETTLE_S = 0.5
RESET_RENDERS = 4
DEFAULT_CONTROL_HZ = 20.0

DOCS = """Two bimanual YAM stations, A and B, on one table: four 6-DoF arms with parallel-jaw grippers. Each station
is a standard bimanual YAM (left and right arm, a top camera on a stand behind the arms and a camera on each wrist).
Each station has its own controller, which sees only its own three cameras, its own 14-D joint state and its own
instruction; the 28-D action is A's 14-D then B's 14-D, each [left j0..j5, left gripper, right j0..j5, right
gripper], joints in rad, grippers 0 closed .. 1 open."""


def environment_id(task: taskfile.PairTask, layout: Layout) -> str:
    return f"isaaclab-arena-0.3.1-yam-pair-{layout.arrangement}-task-{task.name}"


def _flag(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class YamPairEmbodiment(EmbodimentBase):
    """Two bimanual YAM stations in Isaac Lab-Arena, loaded from a yam_pair task file."""

    def __init__(
        self,
        *,
        task: str | None = None,
        layout: str | None = None,
        separation: float | None = None,
        offset: float | None = None,
        yaw_a: float | None = None,
        yaw_b: float | None = None,
        control_hz: float = DEFAULT_CONTROL_HZ,
        cam_width: int = 320,
        cam_height: int = 176,
        object_jitter: float = 0.0,
        overview: bool | str = False,
        overview_width: int = 640,
        overview_height: int = 400,
        headless: bool = True,
        device: str = "cuda:0",
    ) -> None:
        if not task:
            raise ValueError("yam_pair needs a task file: -E task=path/to/task.py")
        self.task = taskfile.load(task)
        self.layout = self.task.layout.with_overrides(arrangement=layout, separation=separation, offset=offset,
                                                      yaw_a=yaw_a, yaw_b=yaw_b)
        self.object_jitter = float(object_jitter)
        if not self.object_jitter >= 0:
            raise ValueError(f"object_jitter must be >= 0 metres, got {object_jitter!r}")
        self.headless, self.device = _flag(headless), device
        self.cam_width, self.cam_height = int(cam_width), int(cam_height)
        self.overview = (int(overview_width), int(overview_height)) if _flag(overview) else None
        from yam_arena.inspect_embodiment import _urdf_arm_limits
        from inspect_robots_yam.packing import pack

        lower, upper = _urdf_arm_limits()
        arm_low, arm_high = np.append(lower, 0.0), np.append(upper, 1.0)
        low14, high14 = pack(arm_low, arm_low), pack(arm_high, arm_high)
        self._low, self._high = np.concatenate([low14, low14]), np.concatenate([high14, high14])
        scored = self.task.hooks.get("success") is not None
        docs = "\n\n".join(d for d in (DOCS, f"Layout: {self.layout.describe()}.", self.task.docs) if d)
        self.info = EmbodimentInfo(
            name="yam_pair",
            action_space=boundary.pair_action_box(low14, high14),
            observation_space=boundary.pair_observation_space(self.cam_height, self.cam_width, self.overview),
            control_hz=float(control_hz),
            is_simulated=True,
            capabilities=frozenset({SEEDABLE, RESETTABLE} | ({PRIVILEGED_SUCCESS} if scored else set())),
            docs=docs,
            environment_id=environment_id(self.task, self.layout),
            environment_revision=self.task.sha256[:12],
        )
        taskfile.register(self.info.environment_id, self.task, self)
        self._env: Any = None
        self._base: Any = None
        self._instruction: str | None = None
        self._t, self._step = 0.0, 0
        self._held, self._best_score = 0, 0.0
        self._contact_peak = 0.0
        self.timing: dict[str, float] = {"steps": 0, "step_s": 0.0}

    # -- Isaac lifecycle -------------------------------------------------------------------------------------------
    def _ensure_sim(self) -> None:
        if self._env is not None:
            return
        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        import yam_arena.inspect_embodiment as yie

        if yie._APP is None:  # one Kit app per process, shared with yam_arena; closed (never hung) at exit
            import atexit

            from isaaclab.app import AppLauncher

            kwargs: dict[str, Any] = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                kwargs["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = yie.window_display(os.environ)
            yie._APP = AppLauncher(**kwargs).app
            atexit.register(yie._close_app_at_exit)
        import torch

        from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
        from yam_pair import sim

        self._sim = sim
        self._torch = torch
        self._env, self._arena_emb, self._extra_names = sim.make_env(
            self.task, self.layout, control_hz=self.info.control_hz, device=self.device, overview=self.overview)
        self._base = self._env.unwrapped
        ready = [*READY, 1.0, *READY, 1.0]
        self._ready = torch.tensor([ready * 2], dtype=torch.float32, device=self._base.device)
        self._ids = {
            f"{s}_{side}": (
                self._base.scene[f"{s}_{side}_robot"].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[f"{s}_{side}_robot"].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for s in STATIONS for side in SIDES
        }
        self._hook("on_build")

    def close(self) -> None:
        """Drop the env; Kit closes at interpreter exit (closing it here would end the process)."""
        self._env = self._base = None

    # -- hooks -------------------------------------------------------------------------------------------------------
    def _hook(self, name: str) -> None:
        fn = self.task.hooks.get(name)
        if fn is not None:
            fn(self._env)
        sk = sys.modules.get("scenekit")  # a task-folder scenekit's decorators
        if sk is not None:
            if hasattr(sk, "_ENV"):
                sk._ENV["env"] = self._env
            lists = {"before_reset": "_PRE_RESET", "on_reset": "_RESET", "on_step": "_STEP"}
            for f in getattr(sk, lists.get(name, "_none"), []):
                f(self._env)

    # -- Embodiment contract -----------------------------------------------------------------------------------------
    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        self._ensure_sim()
        torch, base = self._torch, self._base
        self._instruction = scene.instruction
        self._held, self._best_score = 0, 0.0
        self._hook("before_reset")  # on the first reset: right after the scene was built, as scenekit does
        self._env.reset()
        self._sim.restore_usd_joints(self._env, self.task)  # UsdBody joint_pos holds at every reset
        if seed is not None and self.object_jitter > 0:
            rng = np.random.default_rng(seed)
            for name in (*self.task.free_objects, *self._extra_free()):
                obj = base.scene[name]
                pose = obj.data.default_root_pose.torch.clone()
                pose[:, :3] += base.scene.env_origins
                jitter = rng.uniform(-self.object_jitter, self.object_jitter, size=2)
                pose[:, :2] += torch.tensor(jitter, dtype=pose.dtype, device=pose.device)
                obj.write_root_pose_to_sim_index(root_pose=pose)
                obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
        for _ in range(max(1, round(SETTLE_S / base.step_dt))):
            self._env.step(self._ready)
        for _ in range(RESET_RENDERS):
            base.sim.render()
        self._t, self._step, self._contact_peak = 0.0, 0, 0.0
        self.timing = {"steps": 0, "step_s": 0.0}
        self._sim.read_contacts(base)  # the settle's contacts are not the trial's
        self._hook("on_reset")
        return self._observation()

    def _extra_free(self) -> list[str]:
        rigid = self._base.scene.rigid_objects
        return [n for n in self._extra_names if n in rigid]

    def step(self, action: Any) -> StepResult:
        import time

        tick = time.monotonic()
        target = np.clip(np.asarray(action.data, dtype=np.float64).reshape(-1), self._low, self._high)
        boundary.split_action(target)  # validates 28-D
        a = self._torch.tensor(target, dtype=self._torch.float32, device=self._base.device).unsqueeze(0)
        self._env.step(a)
        self._t += self._base.step_dt
        self._step += 1
        contacts = self._sim.read_contacts(self._base)
        now = max(contacts.values(), default=0.0)
        self._contact_peak = max(self._contact_peak, now)
        self._hook("on_step")
        result = self._task_step(contacts, now)
        self.timing["steps"] += 1
        self.timing["step_s"] += time.monotonic() - tick
        return result

    # -- internals -----------------------------------------------------------------------------------------------------
    def object_states(self) -> dict:
        """Every task object's state now (world frame). Privileged: for scoring and scripted demos only."""
        return self._sim.object_states(self._env, self.task, self._extra_names)

    def arm_states(self) -> dict[str, np.ndarray]:
        from yam_arena.constants import FINGER_OPEN

        out = {}
        for s in STATIONS:
            per_arm = []
            for side in SIDES:
                arm_ids, finger_ids = self._ids[f"{s}_{side}"]
                q = self._base.scene[f"{s}_{side}_robot"].data.joint_pos.torch[0]
                joints = q[arm_ids].cpu().numpy().astype(np.float64)
                grip = float(np.clip(q[finger_ids].mean().item() / FINGER_OPEN, 0.0, 1.0))
                per_arm.append(np.append(joints, grip))
            out[s] = np.concatenate(per_arm)
        return out

    def pair_state(self, contacts: dict[str, float] | None = None, now: float = 0.0) -> PairState:
        return PairState(step=self._step, t=self._t, contact_n=now, contact_peak_n=self._contact_peak,
                         contact_by_arm=dict(contacts or {}), arms=self.arm_states(), layout=self.layout, env=self._env)

    def _task_step(self, contacts: dict[str, float], now: float) -> StepResult:
        task = self.task
        states = self.object_states()
        pair = self.pair_state(contacts, now)
        info: dict[str, Any] = {"cross_contact_n": now, "cross_contact_peak_n": self._contact_peak}
        success = False
        if task.hooks.get("success") is not None:
            self._held = self._held + 1 if task.call("success", states, pair) else 0
            success = self._held >= max(1, round(task.success_hold_s * self.info.control_hz))
            info["success"] = success
        if task.hooks.get("distance") is not None:
            info["distance"] = float(task.call("distance", states, pair))
        if task.hooks.get("score") is not None:
            value = min(max(float(task.call("score", states, pair)), 0.0), 1.0)
            self._best_score = max(self._best_score, value)
            info["score"] = self._best_score if task.score_over == "best" else value
        if task.hooks.get("report") is not None:
            info["report"] = {k: (float(v) if isinstance(v, int | float | np.floating | np.integer | bool) else v)
                              for k, v in dict(task.call("report", states, pair) or {}).items()}
        return StepResult(observation=self._observation(), terminated=success,
                          termination_reason="success" if success else None, info=info)

    def _observation(self) -> Observation:
        import cv2

        base = self._base
        images = {}
        for s in STATIONS:
            for cam in boundary.ARM_CAMERAS:
                name = boundary.camera_name(s, cam)
                out = base.scene[name].data.output["rgb"]
                rgb = getattr(out, "torch", out)[0, ..., :3].cpu().numpy()
                if rgb.shape[:2] != (self.cam_height, self.cam_width):
                    rgb = cv2.resize(rgb, (self.cam_width, self.cam_height))
                images[name] = np.ascontiguousarray(rgb, dtype=np.uint8)
        if self.overview is not None:
            out = base.scene["overview_cam"].data.output["rgb"]
            images["overview_cam"] = np.ascontiguousarray(getattr(out, "torch", out)[0, ..., :3].cpu().numpy(),
                                                          dtype=np.uint8)
        arms = self.arm_states()
        return Observation(
            images=images,
            state={boundary.state_key(s): arms[s] for s in STATIONS},
            instruction=self._instruction,
            image_times=dict.fromkeys(images, self._t),
            state_time=self._t,
        )


def yam_pair_embodiment(**kwargs: Any) -> YamPairEmbodiment:
    """Embodiment ``yam_pair``: ``-E task=task.py`` (required); ``-E layout=facing|side_by_side -E separation=0.8
    -E offset=0 -E yaw_a=0 -E yaw_b=0`` (degrees) move the stations; ``-E overview=true`` adds a third-person view."""
    return YamPairEmbodiment(**kwargs)
