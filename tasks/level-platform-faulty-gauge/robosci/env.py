"""SimEnv: one task instance on the base scene, stepped at the control rate.

A task supplies the scene objects, a per-step monitor and the rubric scorer.
The env owns physics, rendering and optional frame recording for videos.
"""

from __future__ import annotations

from typing import Any, Protocol

import mujoco
import numpy as np

from robosci import scene


class SimTask(Protocol):
    """What every robosci task implements."""

    key: str

    def build(self, spec: mujoco.MjSpec, rng: np.random.Generator) -> None:
        """Add this task's objects to the base scene; sample hidden variables."""

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Called once after compile and home; set object state if needed."""

    def monitor(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Called every physics step group; updates the event log."""

    def score(self, model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, Any]:
        """Rubric stage, success flags and task metrics at this moment."""


def _jpeg(img: np.ndarray) -> bytes:
    import io

    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(img).save(buf, format="JPEG", quality=96)
    return buf.getvalue()


def decode_frame(b: bytes) -> np.ndarray:
    import io

    from PIL import Image

    return np.asarray(Image.open(io.BytesIO(b)).convert("RGB"))


class SimEnv:
    """Physics + rendering for one task and one seed."""

    def __init__(self, task: SimTask, seed: int = 0, *, width: int = 640, height: int = 360,
                 record: tuple[str, ...] = (), record_fps: float = 30.0) -> None:
        self.task = task
        self.seed = seed
        spec = scene.base_spec()
        task.build(spec, np.random.default_rng(seed))
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        scene.set_home(self.model, self.data)
        task.reset(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.width, self.height = width, height
        self._renderer: mujoco.Renderer | None = None
        self._hero: mujoco.Renderer | None = None
        self.record = record
        self.frames: dict[str, list[np.ndarray]] = {c: [] for c in record}
        self.hud: list[list[str]] = []
        self.n_sub = int(round(1.0 / (scene.CONTROL_HZ * self.model.opt.timestep)))
        self.rec_every = max(1, int(round(1.0 / (record_fps * self.model.opt.timestep))))
        self._k = 0
        self.steps = 0

    @property
    def renderer(self) -> mujoco.Renderer:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.height, self.width)
        return self._renderer

    def render(self, camera: str) -> np.ndarray:
        """RGB image from a named camera (the presentation camera "hero" renders at 1280x720)."""
        r = self.renderer
        if camera == "hero":
            if self._hero is None:
                self._hero = mujoco.Renderer(self.model, 720, 1280)
            r = self._hero
        r.update_scene(self.data, camera=camera)
        return r.render().copy()

    def settle(self, seconds: float = 1.0) -> None:
        """Let objects come to rest before the episode starts (not recorded)."""
        for _ in range(int(seconds / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)

    def step(self, action: np.ndarray) -> None:
        """Apply a 14-D action for one control period (1 / CONTROL_HZ s)."""
        scene.apply_action(self.model, self.data, action)
        for _ in range(self.n_sub):
            mujoco.mj_step(self.model, self.data)
            self._k += 1
            if self._k % 10 == 0:
                self.task.monitor(self.model, self.data)
            if self.record and self._k % self.rec_every == 0:
                for c in self.record:
                    self.frames[c].append(_jpeg(self.render(c)))  # compressed: long episodes stay small
                hud = getattr(self.task, "hud", None)
                self.hud.append(hud(self.model, self.data) if hud else [])
        self.steps += 1

    def observe(self) -> dict[str, Any]:
        """Camera images plus the 14-D joint state."""
        obs: dict[str, Any] = {c: self.render(c) for c in scene.CAMERAS}
        obs["joint_pos"] = scene.current_action(self.model, self.data)
        return obs

    def score(self) -> dict[str, Any]:
        out = self.task.score(self.model, self.data)
        out["episode_steps"] = self.steps
        return out

    def close(self) -> None:
        for attr in ("_renderer", "_hero"):
            if getattr(self, attr) is not None:
                getattr(self, attr).close()
                setattr(self, attr, None)
