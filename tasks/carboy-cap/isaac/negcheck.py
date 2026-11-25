"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured thread state, the funnel
position and the contact monitor, independent of the score), and that the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "idle": 0.0,
    "neg-stow-first": 0.0,
    "stow-then-cap": 1.0,
    "neg-stow-then-cap": 1.0,
    "cap-only": 0.5,
    "neg-cap-loose": 0.0,
    "replay": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    det = [r["detail"] for r in rows if r.get("detail")]
    end = det[-1] if det else {}
    gaps = [d["gap_mm"] for d in det if d.get("gap_mm") is not None]
    stow_rows = [d for d in det if d.get("stowed")]
    at_stow = stow_rows[0] if stow_rows else {}
    ev = dict(funnel_left_neck=any(not d.get("funnel_in_neck", True) for d in det),
              ever_captured=any(d.get("thread") in ("captured", "engaged") for d in det),
              ever_engaged=any(d.get("thread") == "engaged" for d in det),
              max_turns=max((d.get("turns") or 0.0 for d in det), default=0.0),
              min_gap_mm=min(gaps) if gaps else None, sealed_t=end.get("sealed_t"), stowed_t=end.get("stowed_t"),
              gap_at_stow_mm=at_stow.get("gap_mm"), thread_at_stow=at_stow.get("thread"),
              sealed_at_stow=end.get("sealed_at_stow"), veto=end.get("veto"), veto_t=end.get("veto_t"),
              max_bottle_tilt=end.get("max_tilt_deg"), contact_peak_n=end.get("contact_peak_n"))
    if stage == "demo":
        return bool(res["success"] and ev["sealed_t"] is not None and ev["stowed_t"] is not None
                    and ev["sealed_t"] < ev["stowed_t"] and not end.get("veto")), ev
    if stage == "idle":
        return bool(not ev["funnel_left_neck"] and not ev["ever_captured"]), ev
    if stage == "neg-stow-first":
        # the funnel was taken out and came to rest in the tray while the cap was never on the thread
        return bool(ev["funnel_left_neck"] and ev["stowed_t"] is not None and not ev["ever_captured"]), ev
    if stage == "cap-only":
        return bool(end.get("sealed") and not end.get("funnel_in_tray") and ev["funnel_left_neck"]), ev
    if stage in ("stow-then-cap", "neg-stow-then-cap"):
        # stowed with the bottle open, then the cap really was screwed down to the seat
        return bool(ev["stowed_t"] is not None and ev["sealed_at_stow"] is False and ev["sealed_t"] is not None
                    and ev["sealed_t"] > ev["stowed_t"]), ev
    if stage == "neg-cap-loose":
        # the cap was on the thread (engaged) but not down to the seat when the funnel was stowed
        return bool(ev["stowed_t"] is not None and ev["thread_at_stow"] == "engaged"
                    and (ev["gap_at_stow_mm"] or 0.0) > 0.25), ev
    if stage == "replay":
        return True, ev
    return False, ev


def trials(d):
    d = Path(d)
    if (d / "result.json").exists():
        return [d]
    return sorted(p for p in d.glob("trial*") if (p / "result.json").exists())


def main():
    out = Path(sys.argv[1])
    rows = []
    for d in sys.argv[2:]:
        for t in trials(d):
            res = json.loads((t / "result.json").read_text())
            stage = res["stage"]
            done, ev = stimulus(stage, load_trace(t), res)
            want = EXPECT.get(stage)
            rows.append(dict(run=str(t), stage=stage, score=res["task_score"], expected=want,
                             score_ok=want is not None and abs(res["task_score"] - want) < 1e-9,
                             stimulus_ok=done, steps=res["steps"], stop=res.get("stop"), evidence=ev))
            print(json.dumps({k: rows[-1][k] for k in ("run", "stage", "score", "score_ok", "stimulus_ok")}))
    out.write_text(json.dumps(dict(passed=all(r["score_ok"] and r["stimulus_ok"] for r in rows), rows=rows),
                              indent=2, default=float))


if __name__ == "__main__":
    main()
