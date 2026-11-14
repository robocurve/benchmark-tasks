"""Precalibrated joint motion for a fixed fixture; elapsed-time input only.

The motion files contain ordinary fourteen-value robot commands. They were
calibrated in development and must succeed again under native physics. Neither
task-body state nor evaluation metrics are available to this policy.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

from .config import CONTROL_HZ


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        if mode not in {"demo", "hold", "connector", "ends", "unlocked"}:
            raise ValueError("Unknown reference mode")
        self.mode = mode
        self.dt = 1.0 / CONTROL_HZ
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name="chain_link_scripted", action_space=action_box(low, high)
        )
        path = Path(__file__).parent / "motions" / (
            "unlocked.json" if mode == "unlocked" else "demo.json"
        )
        raw = path.read_bytes()
        program = json.loads(raw)
        self.program_sha256 = hashlib.sha256(raw).hexdigest()
        self.actions = np.array(program["actions"], dtype=float)
        if (
            program["control_hz"] != CONTROL_HZ
            or self.actions.ndim != 2
            or self.actions.shape[1] != 14
            or len(self.actions) == 0
            or not np.isfinite(self.actions).all()
            or (self.actions < low - 1e-9).any()
            or (self.actions > high + 1e-9).any()
        ):
            raise ValueError("Invalid calibrated robot program")
        self.phases = program["phases"]
        if mode in {"connector", "ends"}:
            stop_phase = (
                "settle connector in cradle"
                if mode == "connector"
                else "let chain settle onto backing"
            )
            stop = next(p["last_step"] for p in self.phases if p["name"] == stop_phase)
            self.actions = self.actions[:stop]
        elif mode == "hold":
            self.actions = np.tile(np.r_[READY, 1.0, READY, 1.0], (100, 1))
        self.duration = len(self.actions) * self.dt + 3.0
        self.reset(None)

    def bind(self, embodiment_info):
        if float(embodiment_info.control_hz) != CONTROL_HZ:
            raise ValueError("Calibrated reference requires 20 Hz")

    def reset(self, scene):
        self.phase = "reset"

    def action_at(self, time):
        if not np.isfinite(time) or time < 0:
            raise ValueError("Invalid reference time")
        step = max(1, int(np.floor(float(time) * CONTROL_HZ + 1e-6)))
        if self.mode == "hold":
            self.phase = "hold reset"
        elif step > len(self.actions):
            self.phase = "hold final robot command"
        else:
            self.phase = next(
                p["name"]
                for p in self.phases
                if p["first_step"] <= step <= p["last_step"]
            )
        return self.actions[min(step, len(self.actions)) - 1].copy()

    def act(self, observation):
        time = float(observation.state_time) + self.dt
        action = self.action_at(time)
        meta = {"request_stop": True} if time > self.duration + self.dt else {}
        return ActionChunk(
            actions=[Action(data=action, meta=meta)], control_hz=CONTROL_HZ
        )


def scripted_policy(**kwargs):
    return ScriptedPolicy(**kwargs)
