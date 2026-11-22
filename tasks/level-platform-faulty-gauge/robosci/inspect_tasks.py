"""Inspect Robots task registration and rubric scorers (mirrors stationerybench/tasks.py)."""

from __future__ import annotations

import zlib
from dataclasses import dataclass

from inspect_robots import Epochs, Scene, Score, Scorer, Target, Task, TrialRecord, episode_length, task

from robosci.specs import TaskSpec
from robosci.tasks import SPECS

K_TRIALS = 20



@dataclass(frozen=True)
class _InfoScorer:
    """Reads one rubric field from the last step's info (the sim scores every step)."""

    name: str
    field: str
    scale: float = 1.0
    report: tuple[str, ...] = ()

    def __call__(self, record: TrialRecord, target: Target | None) -> Score:
        if not record.steps:
            return Score(value=0.0, explanation="no steps")
        info = record.steps[-1].result.info or {}
        v = float(info.get(self.field, 0.0)) / self.scale
        keys = ("stage", "procedure_violations", *self.report)
        meta = {k: info[k] for k in keys if k in info}
        return Score(value=v, explanation=f"{self.field}={info.get(self.field)}", metadata=meta)


def rubric_scorers(report: tuple[str, ...] = ()) -> list[Scorer]:
    return [
        _InfoScorer("task_success", "success", report=report),
        _InfoScorer("science_success", "science_success"),
        _InfoScorer("rubric_stage", "stage", scale=4.0),
        _InfoScorer("procedure_violations", "procedure_violations"),
        episode_length(),
    ]


def make_task(spec: TaskSpec, *, difficulty: str = "easy", max_seconds: float | None = None) -> Task:
    """One scene; each epoch reseeds the hidden variables (masses, order)."""
    scene = Scene(
        id=f"{spec.key}-{difficulty}",
        instruction=spec.instruction_for(difficulty),
        init_seed=zlib.crc32(spec.key.encode()),
        metadata={
            "benchmark": "robosci",
            "task": spec.key,
            "difficulty": difficulty,
            "title": spec.title,
            "ability": spec.ability,
            "objects": list(spec.objects),
            "setup": list(spec.setup),
            "hidden": spec.hidden_for(difficulty),
            "rubric": "\n".join(f"{i} - {line}" for i, line in enumerate(spec.rubric)),
            "rubric_lines": list(spec.rubric),
            "bimanual": True,
        },
    )
    return Task(
        name=f"robosci/{spec.key}",
        scenes=[scene],
        scorer=rubric_scorers(spec.report_fields),
        epochs=Epochs(count=K_TRIALS, reducer="mean"),
        max_seconds=max_seconds or spec.max_seconds,
        metadata={"benchmark": "robosci", "k_trials": K_TRIALS, "version": spec.version},
    )


def _register(spec: TaskSpec):  # noqa: ANN202
    def factory(difficulty: str = "easy", max_seconds: float | None = None) -> Task:
        return make_task(spec, difficulty=difficulty, max_seconds=max_seconds)

    factory.__name__ = factory.__qualname__ = spec.key
    factory.__doc__ = f"{spec.title} ({spec.ability})."
    return task(f"robosci/{spec.key}")(factory)


# One module-level factory per task; pyproject entry points name them robosci.inspect_tasks:<key>.
for _spec in SPECS:
    globals()[_spec.key] = _register(_spec)
