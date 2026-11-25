"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured depth angle, lever angle,
blade projection and hand positions in the trace), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {"demo": 1.0, "idle": 0.0, "neg-unlocked": 0.0, "neg-wrong-board": 0.0, "neg-no-support": 0.0, "replay": 0.0}
THETA_MAX_DEG = 27.149


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return r["detail"].get("measure", {}) or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    veto = end.get("veto", {}) or {}
    ev_clamp = end.get("clamp_events", []) or []
    released = any(e["ev"] == "release" for e in ev_clamp)
    th0 = rows[0]["theta_deg"] if rows else 0.0
    if stage == "demo":
        return res["success"] and not any(veto.values()), dict(final=m_end, clamp_events=ev_clamp,
                                                                grip_peak_n=end.get("grip_peak_n"))
    if stage == "idle":
        moved = max(abs(r["theta_deg"] - th0) for r in rows) if rows else 0
        return moved < 0.05 and not released, dict(theta_moved_deg=round(moved, 4))
    if stage == "neg-unlocked":
        # lever released and still released at the end, hands off, motor dropped to full depth after the let-go
        return (released and m_end.get("phi_deg", 0) > 25 and m_end.get("clear_mm", 0) > 30
                and m_end.get("theta_deg", 0) > THETA_MAX_DEG - 0.5), dict(
            final_phi_deg=m_end.get("phi_deg"), final_theta_deg=m_end.get("theta_deg"),
            final_proj_mm=m_end.get("proj_mm"), clear_mm=m_end.get("clear_mm"), clamp_events=ev_clamp)
    if stage == "neg-wrong-board":
        # locked, flat, hands off, but the blade is out of the 1..9 mm band for this board
        p = m_end.get("proj_mm", 0)
        return (m_end.get("locked") and m_end.get("flat") and m_end.get("clear_mm", 0) > 30
                and not (1.0 <= p <= 9.0)), dict(final_proj_mm=p, t_board_mm=end.get("t_board_mm"),
                                                 locked=m_end.get("locked"), clamp_events=ev_clamp)
    if stage == "neg-no-support":
        dropped = max(r["theta_deg"] for r in rows) > THETA_MAX_DEG - 0.5
        return released and dropped and m_end.get("locked"), dict(
            theta_max_deg=max(r["theta_deg"] for r in rows), final_proj_mm=m_end.get("proj_mm"),
            locked=m_end.get("locked"), clamp_events=ev_clamp)
    if stage == "replay":
        start = res.get("start", {})
        return len(rows) > 100, dict(start=start, final=m_end, veto=veto)
    return False, {}


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
