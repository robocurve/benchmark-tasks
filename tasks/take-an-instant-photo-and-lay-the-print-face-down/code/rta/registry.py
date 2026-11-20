"""The hidden state of the running episode, per embodiment, for the privileged ``rta_oracle`` policy.

Inspect Robots resets the policy before the embodiment, so the oracle cannot read the episode's hidden
state at ``reset()``; the embodiment publishes it here and the oracle reads it on its first ``act()``.
"""

from __future__ import annotations

from typing import Any

_TASKS: dict[str, Any] = {}
_STATES: dict[str, dict] = {}


def register_task(environment_id: str, task: Any) -> None:
    _TASKS[environment_id] = task


def task_for(environment_id: str | None) -> Any | None:
    return _TASKS.get(environment_id or "")


def publish_state(environment_id: str, state: dict) -> None:
    _STATES[environment_id] = dict(state)


def state_for(environment_id: str | None) -> dict | None:
    return _STATES.get(environment_id or "")
