"""Inspect Robots scorer ``pair_task_score``. Pure Python.

The value is the same as yam_arena's ``task_score``: 1.0 for a trial that ended with ``termination_reason="success"``,
otherwise the last step's ``info["score"]`` (the task file's ``score(objects, pair)``), otherwise 0.0. The metadata
also carries the cross-station contact (peak normal force, N, over the whole trial) and the task's last
``report(objects, pair)``, so collision and other separately reported numbers reach the eval log.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from inspect_robots.scorer import Score

CONTACT_THRESHOLD_N = 0.5


@dataclass(frozen=True)
class PairTaskScore:
    name: str = "pair_task_score"
    contact_threshold_n: float = CONTACT_THRESHOLD_N

    def __call__(self, record: Any, target: Any) -> Score:
        steps = record.steps
        if not steps:
            return Score(value=0.0, explanation="no steps", metadata={"steps": 0})
        last = steps[-1].result
        info = dict(last.info or {})
        peak = max((float((s.result.info or {}).get("cross_contact_n", 0.0)) for s in steps), default=0.0)
        peak = max(peak, float(info.get("cross_contact_peak_n", 0.0)))
        meta = {
            "steps": len(steps),
            "cross_contact_peak_n": peak,
            "contact_threshold_n": self.contact_threshold_n,
            "inter_station_contact": peak > self.contact_threshold_n,
            "report": info.get("report", {}),
            "termination_reason": record.termination_reason,
            "status": getattr(record, "status", None),
        }
        if last.terminated and last.termination_reason == "success":
            return Score(value=1.0, explanation=f"success; cross-station peak {peak:.2f} N", metadata=meta)
        partial = info.get("score")
        if partial is None:
            return Score(value=0.0, explanation="no success and no partial credit", metadata=meta)
        return Score(value=float(partial),
                     explanation=f"no success; partial credit {float(partial):g}; cross-station peak {peak:.2f} N",
                     metadata=meta)


def summarize(record: Any, contact_threshold_n: float = CONTACT_THRESHOLD_N) -> tuple[dict, list[dict]]:
    """A trial's yam_pair summary (for ``record.metadata["yam_pair"]``, which Inspect keeps in the eval log) and its
    per-step series (score, cross-station contact, report), from the steps' ``info``."""
    series = []
    for s in record.steps:
        info = dict(s.result.info or {})
        row = {"t": s.t, "cross_contact_n": float(info.get("cross_contact_n", 0.0))}
        for key in ("score", "success", "distance"):
            if key in info:
                row[key] = info[key]
        if info.get("report"):
            row["report"] = info["report"]
        series.append(row)
    last = dict(record.steps[-1].result.info or {}) if record.steps else {}
    peak = max((r["cross_contact_n"] for r in series), default=0.0)
    over = [r["t"] for r in series if r["cross_contact_n"] > contact_threshold_n]
    summary = {
        "steps": len(series),
        "termination_reason": record.termination_reason,
        "status": getattr(record, "status", None),
        "final_score": last.get("score"),
        "final_report": last.get("report", {}),
        "cross_contact_peak_n": peak,
        "contact_threshold_n": contact_threshold_n,
        "steps_over_threshold": len(over),
        "first_step_over_threshold": over[0] if over else None,
    }
    return summary, series


def pair_task_score(*, name: str = "pair_task_score", contact_threshold_n: float = CONTACT_THRESHOLD_N) -> PairTaskScore:
    """Scorer ``pair_task_score``: task_score's value, plus cross-station contact and the task's report()."""
    return PairTaskScore(name=name, contact_threshold_n=float(contact_threshold_n))
