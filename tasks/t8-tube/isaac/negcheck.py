"""Stimulus + reaction check of finished demo / negative runs (reads JSON only).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl,
checks that the stimulus the stage is meant to produce actually happened (from the measured lampholder state, the
lamp's spin, speed and position, the drop monitor and the robot-contact monitor, independent of the score), and that
the final score is the expected level.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    "demo": 1.0,
    "outside": 0.5,
    "idle": 0.0,
    "neg-let-go-open": 0.0,
    "neg-pull-down": 0.0,
    "neg-drop-high": 0.0,
    "replay": 0.0,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / "trace.jsonl")]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    det = [r["detail"] for r in rows if r.get("detail")]
    end = det[-1] if det else {}
    states = [d.get("holder") for d in det]
    first = lambda st: next((i + 1 for i, x in enumerate(states) if x == st), None)  # noqa: E731
    ev = dict(max_spin_deg=max((abs(d.get("spin_deg") or 0.0) for d in det if d.get("holder") in ("locked", "slot")),
                               default=0.0),
              slot_step=first("slot"), out_step=first("out"), broken_step=first("broken"),
              dropped=end.get("dropped"), broke=end.get("broke"), peaks=end.get("peaks"),
              lamp_contact_peak_n=(end.get("contact_peak_n") or {}).get("lamp"),
              fixture_contact_peak_n=(end.get("contact_peak_n") or {}).get("fixture"),
              in_carton_end=end.get("in_carton"), lamp_z_end=end.get("lamp_z"), veto=end.get("veto"),
              veto_t=end.get("veto_t"), events=end.get("events"))
    if stage == "demo":
        return bool(res["success"] and ev["out_step"] and end.get("in_carton") and not end.get("veto")), ev
    if stage == "outside":
        return bool(ev["out_step"] and end.get("on_bench") and end.get("clear_s", 0) >= 4 and not end.get("veto")), ev
    if stage == "idle":
        return bool(ev["max_spin_deg"] < 1.0 and ev["slot_step"] is None), ev
    if stage == "neg-let-go-open":
        # the lamp really was turned open (pins in line with the slots) and then fell with no hand on it
        return bool(ev["slot_step"] and ev["dropped"] and ev["dropped"]["v"] > 0.8), ev
    if stage == "neg-pull-down":
        # the pull really loaded the lampholders (>= 40 N on the lamp's hold) and the lamp never reached open
        return bool(ev["slot_step"] is None and (ev["peaks"] or {}).get("lat_n", 0) >= 40.0), ev
    if stage == "neg-drop-high":
        # taken out properly, then it fell with no hand on it and came to rest in the carton
        return bool(ev["out_step"] and ev["dropped"] and ev["dropped"]["v"] > 0.8 and end.get("in_carton")), ev
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
