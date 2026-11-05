"""Task card format (StationeryBench style), plus the scientific ability each task tests.

The sim scores the 0-4 rubric automatically from simulator state; the same card is what
an operator reads when the task runs on the real ABC kit. Each task module defines its
own ``SPEC``; ``SPECS`` / ``SPEC_BY_KEY`` are collected by ``robosci.tasks``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TaskSpec:
    """An immutable task card with a reset checklist and five rubric stages."""

    key: str
    title: str
    ability: str
    instruction: str
    objects: tuple[str, ...]
    setup: tuple[str, ...]
    hidden: str
    rubric: tuple[str, ...]
    metrics: tuple[str, ...]
    report_fields: tuple[str, ...] = ()  # score() keys copied into scorer metadata
    max_seconds: float = 360.0
    limitations: str = ""  # where the sim simplifies the real setup
    version: int = 1
    instruction_hard: str = ""  # goal-only hard instruction; empty = same as easy
    hidden_hard: str = ""  # hidden variables in hard mode; empty = same as easy

    def instruction_for(self, difficulty: str) -> str:
        return self.instruction_hard if difficulty == "hard" and self.instruction_hard else self.instruction

    def hidden_for(self, difficulty: str) -> str:
        return self.hidden_hard if difficulty == "hard" and self.hidden_hard else self.hidden


def __getattr__(name: str) -> Any:
    if name in ("SPECS", "SPEC_BY_KEY"):
        from robosci import tasks

        return getattr(tasks, name)
    raise AttributeError(name)
