# SPDX-License-Identifier: MIT
"""Play a fixed-layout calibration trajectory at one joint target per tick."""
from __future__ import annotations

from hashlib import sha256
from importlib.resources import files
import json
from typing import TYPE_CHECKING

import numpy as np
from yam_arena import ik

if TYPE_CHECKING:
    from inspect_robots.embodiment import EmbodimentInfo
    from inspect_robots.scene import Scene
    from inspect_robots.types import ActionChunk, Observation


CONTROL_HZ = 20.0
ACTION_LABELS = tuple(
    f"{arm}_{channel}"
    for arm in ("left", "right")
    for channel in ("j0", "j1", "j2", "j3", "j4", "j5", "gripper")
)
ACTION_LOW = np.tile(np.r_[ik.LOWER, 0.0], 2)
ACTION_HIGH = np.tile(np.r_[ik.UPPER, 1.0], 2)


def _load_actions(program: bytes) -> np.ndarray:
    document = json.loads(program)
    if not isinstance(document, dict) or document.get("schema") != "bevel-angle-transfer-fixed-demo-v1":
        raise ValueError("invalid fixed demo program schema")
    if document.get("control_hz") != CONTROL_HZ:
        raise ValueError("fixed demo program requires 20 Hz")
    if document.get("labels") != list(ACTION_LABELS):
        raise ValueError("invalid fixed demo action channel order")

    rows = document.get("actions")
    if not isinstance(rows, list) or not rows or len(rows) != document.get("sample_count"):
        raise ValueError("invalid fixed demo action count")
    if any(
        not isinstance(row, list)
        or len(row) != len(ACTION_LABELS)
        or any(type(value) not in (int, float) for value in row)
        for row in rows
    ):
        raise ValueError("fixed demo actions must be 14-D numeric rows")

    actions = np.asarray(rows, dtype=np.float64)
    if not np.isfinite(actions).all():
        raise ValueError("fixed demo actions must be finite")
    if np.any(actions < ACTION_LOW) or np.any(actions > ACTION_HIGH):
        raise ValueError("fixed demo actions exceed YAM joint or gripper limits")

    # A bytes backing store keeps the calibration immutable even if a caller
    # tries to change the NumPy writeability flag.
    return np.frombuffer(actions.tobytes(), dtype=np.float64).reshape(actions.shape)


class BevelAngleTransferReplay:
    """Implement the Inspect policy protocol using an episode-local tick counter."""

    def __init__(self) -> None:
        from inspect_robots.policy import PolicyConfig, PolicyInfo
        from inspect_robots_yam.config import action_box

        program = files(__package__).joinpath("data", "fixed_demo_actions.json").read_bytes()
        self._actions = _load_actions(program)
        self._step = 0
        self.config = PolicyConfig()
        self.info = PolicyInfo(
            name="bevel_angle_transfer_demo",
            action_space=action_box(ACTION_LOW.copy(), ACTION_HIGH.copy()),
            control_hz=CONTROL_HZ,
            checkpoint=sha256(program).hexdigest(),
        )

    def bind(self, embodiment_info: EmbodimentInfo) -> None:
        if embodiment_info.control_hz != CONTROL_HZ:
            raise ValueError("bevel angle transfer fixed demo requires a 20 Hz embodiment")

    def reset(self, scene: Scene) -> None:
        self._step = 0

    def act(self, observation: Observation) -> ActionChunk:
        from inspect_robots.types import Action, ActionChunk

        final_step = len(self._actions) - 1
        if self._step == final_step:
            meta = {"request_stop": True, "stop_reason": "fixed_demo_motion_complete"}
        else:
            meta = {}
        action = Action(data=self._actions[self._step].copy(), meta=meta)
        self._step = min(self._step + 1, final_step)
        return ActionChunk(actions=[action], control_hz=CONTROL_HZ)


def make_demo_policy() -> BevelAngleTransferReplay:
    return BevelAngleTransferReplay()
