"""Inspect Robots scorers for memory tasks. Pure Python.

- ``q_score``: partial credit in 0..1 — 1.0 for a trial that ended with success, otherwise the last step's
  ``info["q_score"]`` (the latched fraction of the task's milestones).
- ``rta_success``: 1.0 iff the trial ended with ``termination_reason="success"``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from inspect_robots.scorer import Score


def _last(record):
    return record.steps[-1] if record.steps else None


def _succeeded(step) -> bool:
    return bool(step.result.terminated and step.result.termination_reason == "success")


@dataclass(frozen=True)
class QScore:
    name: str = "q_score"

    def __call__(self, record: Any, target: Any) -> Score:
        last = _last(record)
        if last is None:
            return Score(value=0.0, explanation="no steps")
        if _succeeded(last):
            return Score(value=1.0, explanation="success")
        q = last.result.info.get("q_score")
        if q is None:
            return Score(value=0.0, explanation="no success and no milestone progress reported")
        reached = [k for k, v in (last.result.info.get("milestones") or {}).items() if v]
        reason = last.result.termination_reason or "episode ended"
        zero = " (score set to 0)" if last.result.info.get("zeroed") else ""
        return Score(value=float(q), explanation=f"{reason}{zero}; milestones reached: {reached or 'none'}")


@dataclass(frozen=True)
class RtaSuccess:
    name: str = "rta_success"

    def __call__(self, record: Any, target: Any) -> Score:
        last = _last(record)
        if last is None:
            return Score(value=0.0, explanation="no steps")
        ok = _succeeded(last)
        return Score(value=1.0 if ok else 0.0, explanation="success" if ok else (last.result.termination_reason or "no success"))


@dataclass(frozen=True)
class MilestoneScore:
    """1.0 iff the named milestone was reached by the end of the trial: latched (an event) or holding at the last
    step (an end state, the task's LATCHED); 1.0 for a successful trial."""

    milestone: str
    name: str = ""

    def __post_init__(self) -> None:
        if not self.name:
            object.__setattr__(self, "name", f"milestone/{self.milestone}")

    def __call__(self, record: Any, target: Any) -> Score:
        last = _last(record)
        if last is None:
            return Score(value=0.0, explanation="no steps")
        if _succeeded(last):
            return Score(value=1.0, explanation="success")
        reached = (last.result.info.get("milestones") or {}).get(self.milestone)
        if reached is None:
            return Score(value=None, explanation=f"milestone {self.milestone!r} not reported")
        return Score(value=1.0 if reached else 0.0, explanation="reached" if reached else "not reached")


def milestone_scorers(names) -> list[MilestoneScore]:
    """One scorer per milestone, so the eval report shows each subtask's hit rate across scenes."""
    return [MilestoneScore(n) for n in names]


def q_score(*, name: str = "q_score") -> QScore:
    return QScore(name=name)


def rta_success(*, name: str = "rta_success") -> RtaSuccess:
    return RtaSuccess(name=name)
