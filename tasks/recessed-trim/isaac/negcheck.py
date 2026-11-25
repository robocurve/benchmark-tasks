"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each run directory it reads result.json and trace.jsonl, checks that the stimulus the stage is meant to produce
actually happened (from the measured trim / leg state logged every control step and the demo's own LOOK notes), and
that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "hold": 0.0,
    "neg-no-turn": 0.0,
    "neg-clock-zero": 0.0,
    "neg-light-pinch": 0.0,
    "neg-no-push": 0.5,
    "replay": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return (r.get("detail") or {}).get("measure", {}) or {}


def look(res, tag):
    for row in res.get("looks", []):
        if row.get("at") == tag:
            return row
    return None


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = m_of(rows[-1]) if rows else {}
    if stage == "demo":
        return bool(res["success"]), dict(final=end)
    if stage in ("idle", "hold", "replay"):
        never = not any(any((m_of(r).get("captured") or {}).values()) for r in rows)
        return never if stage != "replay" else True, dict(never_captured=never, steps=len(rows), final=end)
    if stage in ("neg-no-turn", "neg-clock-zero"):
        lu = look(res, "lined up") or {}
        slot = lu.get("slot") or {}
        worst_t = max((abs(v[1]) for v in slot.values()), default=0.0)
        never = not any(any((m_of(r).get("captured") or {}).values()) for r in rows)
        # stimulus: squeezed tips more than 3 mm (tangentially) off the slot centre when raised
        return worst_t > 0.003, dict(tangential_offset_mm=round(worst_t * 1e3, 2), never_captured=never)
    if stage == "neg-light-pinch":
        lu = look(res, "lined up") or {}
        th = lu.get("theta") or {}
        min_open = min(th.values()) if th else None
        never = not any(any((m_of(r).get("captured") or {}).values()) for r in rows)
        # stimulus: legs still open more than 3 deg while held under the brackets
        return (min_open is not None and min_open > 3.0), dict(leg_theta_deg=th, never_captured=never)
    if stage == "neg-no-push":
        both = all((end.get("captured") or {}).values()) if end.get("captured") else False
        return both and end.get("max_gap_mm", 0) > 2.0, dict(final=end)
    return False, {}


def main():
    out = Path(sys.argv[1])
    report = []
    ok_all = True
    for d in map(Path, sys.argv[2:]):
        res = json.loads((d / "result.json").read_text())
        rows = load_trace(d)
        st = res["stage"]
        done, ev = stimulus(st, rows, res)
        exp = EXPECT.get(st)
        score_ok = exp is not None and (res["task_score"] == exp if st != "replay" else res["task_score"] < 1.0)
        ok = bool(done and score_ok)
        ok_all &= ok
        report.append(dict(run=str(d), stage=st, score=res["task_score"], expected=exp, steps=res["steps"],
                           stimulus=done, evidence=ev, ok=ok))
        print(("OK  " if ok else "FAIL"), d.name, st, res["task_score"], json.dumps(ev, default=float)[:300])
    out.write_text(json.dumps(dict(passed=ok_all, runs=report), indent=2, default=float))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
