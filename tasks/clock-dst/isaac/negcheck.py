"""Stimulus + reaction check of finished demo / negative / replay runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each run directory it reads result.json and trace.jsonl, checks that the stimulus the stage is meant to produce
actually happened (from the measured clock pose, knob angle and contact forces logged every control step), and that
the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": (1.0, 1.0),
    "idle": (0.0, 0.0),
    "neg-noset": (0.0, 0.0),
    "neg-fixed60": (0.0, 0.0),
    "neg-fixed50": (0.0, 0.0),
    "neg-miss": (0.0, 0.0),
    "replay": (0.0, 0.0),
}


def rows_of(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def m_of(r):
    return (r.get("detail") or {}).get("measure") or {}


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    end = rows[-1]["detail"] if rows else {}
    m_end = end.get("measure", {}) or {}
    turned = end.get("knob_turned_deg")
    off_nail = any(not m_of(r).get("on_nail", True) for r in rows)
    ev = dict(
        final_shown=m_end.get("shown_hm"),
        final_err_min=m_end.get("err_min"),
        knob_turned_deg=turned,
        ever_off_nail=off_nail,
        final_on_nail=m_end.get("on_nail"),
        final_nail_n=m_end.get("nail_n"),
        final_tilt_deg=m_end.get("tilt_deg"),
        dropped=end.get("dropped"),
        min_vz=end.get("min_vz"),
        peak_robot_n=end.get("peak_robot_n"),
    )
    if stage == "demo":
        return bool(res["success"] and off_nail and not end.get("dropped")), ev
    if stage == "idle":
        return bool(abs(turned or 0) < 0.5 and not off_nail), ev
    if stage == "neg-noset":
        # taken off and hung back, knob not turned (time still the start time)
        return bool(off_nail and abs(turned or 0) < 2.0 and m_end.get("on_nail")), ev
    if stage == "neg-fixed60":
        # knob turned one hour (360 deg) within 2 deg, clock back on the nail
        return bool(off_nail and abs((turned or 0) - 360.0) < 3.0 and m_end.get("on_nail")), ev
    if stage == "neg-fixed50":
        # knob turned 50 min (300 deg) within 3 deg, clock back on the nail
        return bool(off_nail and abs((turned or 0) - 300.0) < 3.0 and m_end.get("on_nail")), ev
    if stage == "neg-miss":
        # time set, but never back on the nail (the keyhole missed the nail)
        hung_after = any(m_of(r).get("on_nail") for r in rows[len(rows) // 2 :])
        return bool(off_nail and not m_end.get("on_nail") and not hung_after and abs(m_end.get("err_min", 99)) <= 2.0), ev
    if stage == "replay":
        return bool(len(rows) > 100), ev
    return False, ev


def main():
    out = Path(sys.argv[1])
    report = []
    for d in map(Path, sys.argv[2:]):
        res = json.loads((d / "result.json").read_text())
        stage = res["stage"]
        rows = rows_of(d)
        done, ev = stimulus(stage, rows, res)
        lo, hi = EXPECT.get(stage, (0.0, 1.0))
        score = float(res["task_score"])
        report.append(
            dict(
                run=d.name,
                stage=stage,
                setup=res.get("setup"),
                score=score,
                steps=res["steps"],
                expected=[lo, hi],
                score_ok=lo <= score <= hi,
                stimulus_ok=done,
                evidence=ev,
            )
        )
        print(json.dumps(report[-1]))
    out.write_text(json.dumps(report, indent=2))
    print("ALL OK" if all(r["score_ok"] and r["stimulus_ok"] for r in report) else "CHECK FAILED")


if __name__ == "__main__":
    main()
