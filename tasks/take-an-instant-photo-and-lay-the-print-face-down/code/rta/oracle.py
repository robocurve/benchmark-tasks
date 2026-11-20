"""Inspect Robots policy ``rta_oracle``: the task file's privileged scripted solution. NumPy only.

It reads the episode's hidden state (published by ``yam_memory`` at reset) on its first ``act()`` and plays
``demo(state)``: per arm, ``(start_s, waypoints)`` with ``start_s`` counted from the task's ``ACT_AFTER_S``.
Before an arm's start it holds the ready pose with the gripper open. When every plan has ended it holds the
last pose for ``hold_s`` seconds and asks the rollout to stop (unless the episode already ended).
"""

from __future__ import annotations

from typing import Any

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk, Observation
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import pack
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ, LEFT_BASE_POS, READY, RIGHT_BASE_POS
import dataclasses

from yam_arena.demo import Waypoint

PARK_MOVE_S = 1.5  # a trailing wait reaches the park spot in this time, then holds


def park_fast(waypoints) -> list[Waypoint]:
    """A ``wait`` waypoint of several seconds is played as a quick move to its spot plus a hold there. Played as
    one waypoint, the arm crawled to the park spot over the whole wait (1 cm/s for 16 s) and was still crossing
    the workspace when the other arm came in; the two locked and a carry went 17 cm off (A5, a05)."""
    out = []
    for w in waypoints:
        if w.label == "wait" and w.duration > 2 * PARK_MOVE_S:
            out.append(dataclasses.replace(w, duration=PARK_MOVE_S, label="to park"))
            out.append(dataclasses.replace(w, duration=w.duration - PARK_MOVE_S))
        else:
            out.append(w)
    return out

from rta import registry
from rta.plans import PosePlan
from rta.spec import ARMS, check_demo


class OraclePolicy(PolicyBase):
    def __init__(self, hold_s: float = 2.0) -> None:
        low, high = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        self.info = PolicyInfo(name="rta_oracle", action_space=action_box(pack(low, low), pack(high, high)))
        self.hold_s = float(hold_s)
        self._dt = 1.0 / CONTROL_HZ
        self._environment_id: str | None = None
        self._bases = {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
        self._plans: dict[str, tuple[float, PosePlan]] | None = None
        self._end = 0.0

    def bind(self, embodiment_info: Any) -> None:
        if embodiment_info.control_hz:
            self._dt = 1.0 / float(embodiment_info.control_hz)
        self._environment_id = embodiment_info.environment_id
        if "-station-" in (self._environment_id or ""):
            self._bases = dict(station.BASES)

    def reset(self, scene: Any) -> None:
        self._plans = None  # built on the first act(), once the embodiment has published the episode's state

    def _build(self) -> None:
        task = registry.task_for(self._environment_id)
        state = registry.state_for(self._environment_id)
        if task is None or state is None:
            raise RuntimeError("rta_oracle needs the yam_memory embodiment (no memory task state was published)")
        plan = check_demo(task, task.demo(state))
        self._plans = {
            arm: (task.act_after_s + start, PosePlan(park_fast(waypoints), self._bases[arm])) for arm, (start, waypoints) in plan.items()
        }
        self._end = max(start + p.duration for start, p in self._plans.values())

    def act(self, observation: Observation) -> ActionChunk:
        if self._plans is None:
            self._build()
        assert self._plans is not None
        t = float(observation.state_time) + self._dt
        per_arm = []
        for arm in ARMS:
            entry = self._plans.get(arm)
            if entry is None or t < entry[0]:
                per_arm.append(np.r_[READY, 1.0])
            else:
                per_arm.append(entry[1].action7(t - entry[0])[0])
        meta = {"request_stop": True} if t > self._end + self.hold_s else {}
        return ActionChunk(actions=[Action(data=pack(*per_arm), meta=meta)], control_hz=1.0 / self._dt)


def rta_oracle_policy(**kwargs: Any) -> OraclePolicy:
    """Policy ``rta_oracle``; ``-P hold_s=2`` sets how long it holds the last pose before stopping."""
    return OraclePolicy(**kwargs)
