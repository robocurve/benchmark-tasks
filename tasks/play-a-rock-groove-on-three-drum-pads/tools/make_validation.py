"""Write validation.json from the recorded runs in out/ (standard library only).

    python tools/make_validation.py --scripted scripted-1 --repeat scripted-2
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_drum_groove import fixed_scene as F  # noqa: E402

EXPECTED = {"flat": .9, "tempo82": .75, "no_fill": .75, "flam_bar3": .5, "one_arm": .25, "air": 0., "press": 0.,
            "rest": .75, "flail": 0., "no_stop": 0., "gripper": .5}
MUST = ("flat", "one_arm", "air", "press", "flail")
JUDGED = ("score", "bpm", "rms_ms", "max_error_ms", "H", "G", "E", "wrong", "extra", "arm_contacts", "rests",
          "accent", "accent_detail", "clean_end", "pair_spread_max_ms", "title")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rel(path):
    return str(Path(path).relative_to(ROOT))


def run_record(name):
    out = ROOT / "out" / name
    r = json.loads((out / "RESULT.json").read_text())
    judged = json.loads((out / "media" / "score-vs-hits.json").read_text())
    video = out / ("demo.mp4" if name.startswith("scripted") else "combined.mp4")
    rec = {"score": r["score"], "score_from_files": r["score_from_files"], "variant": r["variant"],
           "executed_steps": r["executed_steps"], "step_limit": r["step_limit"], "elapsed_s": r["elapsed_s"],
           "termination_reason": r["termination_reason"], "result": rel(out / "RESULT.json"),
           "grader_trace": rel(out / "media" / "grader-trace.jsonl"), "hits_mid": rel(out / "media" / "hits.mid"),
           "hits_mid_sha256": sha(out / "media" / "hits.mid"), "events": rel(out / "media" / "events.json"),
           "score_vs_hits": {"path": rel(out / "media" / "score-vs-hits.png"),
                             "sha256": sha(out / "media" / "score-vs-hits.png"),
                             "table": rel(out / "media" / "scored-hits.csv")},
           "judged": {k: judged.get(k) for k in JUDGED},
           "videos": {"overhead": rel(out / "media" / "top_cam.mp4"), "side": rel(out / "media" / "side_cam.mp4")}}
    if video.exists():
        rec["videos"]["combined_with_sound"] = rel(video)
        rec["videos"]["combined_sha256"] = sha(video)
        sync = out / "media" / "sync.json"
        if sync.exists():
            rec["sound_sync"] = json.loads(sync.read_text())
    return r, rec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scripted", default="scripted-1")
    parser.add_argument("--repeat", default="scripted-2")
    args = parser.parse_args()
    r, demo = run_record(args.scripted)
    r2, rep = run_record(args.repeat)
    trace = [sha(ROOT / "out" / n / "media" / "grader-trace.jsonl") for n in (args.scripted, args.repeat)]
    demo.update(machine="one NVIDIA A10 (24 GB), Ubuntu 22.04, headless, the GPU shared with up to two other Isaac Sim "
                        "runs; elapsed_s includes Isaac Sim start-up and scene creation",
                inspect_robots=r["inspect_robots"], yam_arena=r["yam_arena"], repeat=rep["result"],
                repeat_score=rep["score"], repeat_trace_identical=trace[0] == trace[1], trace_sha256=trace,
                repeat_hits_mid_identical=demo["hits_mid_sha256"] == rep["hits_mid_sha256"],
                stills=[rel(ROOT / "out" / args.scripted / "media" / n) for n in ("setup.png", "end_top_cam.png",
                                                                                   "end_side_cam.png")],
                screenshot=rel(ROOT / "out" / args.scripted / "setup.png"),
                screenshot_note="the first frame of demo.mp4: overhead and side together")
    controls = {}
    for path in sorted((ROOT / "out").glob("control-*/RESULT.json")):
        name = path.parent.name
        rr, rec = run_record(name)
        rec.update(expected=EXPECTED.get(rr["variant"]), native=True, must_run=rr["variant"] in MUST)
        controls[name] = rec
    validation = {
        "status": "Scripted demonstration and controls run natively in Isaac Sim (headless). The Dream-1 trial has "
                  "not been run: runs.dream is empty and nothing here is a Dream-1 result.",
        "seed": 0, "object_jitter": 0, "control_hz": F.CONTROL_HZ, "physics_hz": F.PHYSICS_HZ,
        "max_steps": F.MAX_STEPS,
        "runs": {"scripted": demo,
                 "dream": {"model": "dreamlabs/dream-1-beta", "score": None, "executed_steps": None,
                           "step_limit": F.MAX_STEPS, "elapsed_s": None, "status": "not run"}},
        "controls": controls,
        "recorded_hashes": {k: v for k, v in r.items() if k.endswith("_sha256")},
        "scripted_final_grade": {k: demo["judged"][k] for k in JUDGED if k != "title"},
    }
    (ROOT / "validation.json").write_text(json.dumps(validation, indent=1) + "\n")
    print(json.dumps({"scripted": demo["score"], "repeat_identical": demo["repeat_trace_identical"],
                      "controls": {k: (v["score"], v["expected"]) for k, v in controls.items()}}))


if __name__ == "__main__":
    main()
