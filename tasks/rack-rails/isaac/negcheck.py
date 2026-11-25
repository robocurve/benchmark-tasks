"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured rail-end states and the
latch events), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "replay": 0.0,
    "neg-wrong-u": 0.0,
    "neg-front-no-push": 0.5,
    "neg-rear-only": 0.25,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m = end.get("measure", {}) or {}
    ev_l = end.get("latch_events", []) or []
    tu = m.get("target_u")
    ever = lambda f: any(f(m_of(r)) for r in rows if m_of(r))  # noqa: E731
    min_gap = {e: min((m_of(r)[e]["gap"] for r in rows if m_of(r)), default=None) for e in ("rear", "front")}
    base = dict(target_u=tu, latch_events=ev_l, latched_u=m.get("latched_u"), min_gap_mm={
        e: (round(v * 1e3, 3) if v is not None else None) for e, v in min_gap.items()},
        grip_peak_n=end.get("grip_peak_n"), final_tip_mm=m.get("tip_mm"))
    if stage == "demo":
        return res["success"] and all(e["u"] == tu for e in ev_l) and len(ev_l) == 2, base
    if stage == "idle":
        return not ev_l, base
    if stage == "replay":
        return True, base
    if stage == "neg-wrong-u":
        # both ends latched, at target U + 1 (one U above the free U over the server)
        return len(ev_l) == 2 and all(e["u"] == tu + 1 for e in ev_l), base
    if stage == "neg-front-no-push":
        # rear latched at the target U; front pegs in the target U's holes at the end, never latched; the front end
        # was pushed only into the catch zone and sprang back
        front_end = m.get("front", {}) or {}
        base["front_final"] = {k: front_end.get(k) for k in ("gap", "u", "in_holes")}
        return (len(ev_l) == 1 and ev_l[0]["end"] == "rear" and ev_l[0]["u"] == tu and front_end.get("in_holes")
                and front_end.get("u") == tu and not (m.get("latched") or {}).get("front")), base
    if stage == "neg-rear-only":
        never_front = not ever(lambda mm: mm["front"].get("in_holes"))
        base["front_never_pegged"] = never_front
        return len(ev_l) == 1 and ev_l[0]["end"] == "rear" and ev_l[0]["u"] == tu and never_front, base
    return False, base


def check(d):
    res = json.loads((d / "result.json").read_text())
    stage = res["stage"]
    rows = load_trace(d)
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    ok = bool(done) and want is not None and abs(res["task_score"] - want) < 1e-9 and not res.get("nonfinite")
    return dict(dir=str(d), stage=stage, task_score=res["task_score"], expected=want, stimulus_done=bool(done),
                evidence=ev, steps=res["steps"], stop=res.get("stop"), nonfinite=res.get("nonfinite"), passed=ok)


def main(out, dirs):
    rows = []
    for x in dirs:
        p = Path(x)
        trials = sorted(p.glob("trial*_*")) if not (p / "result.json").exists() else [p]
        rows += [check(t) for t in trials]
    summary = dict(passed=all(r["passed"] for r in rows), n=len(rows), rows=rows)
    Path(out).write_text(json.dumps(summary, indent=1, default=float))
    for r in rows:
        print(("PASS" if r["passed"] else "FAIL"), r["stage"], r["task_score"], r["expected"], r["evidence"])
    return summary["passed"]


if __name__ == "__main__":
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
