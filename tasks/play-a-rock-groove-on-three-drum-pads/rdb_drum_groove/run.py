"""Run one fixed trial in Inspect Robots and keep its independently scored evidence: RESULT.json, the hits as MIDI
(media/hits.mid), the trial's events (media/events.json), the score-vs-hits plot and table, the scorer trace, the
overhead and side videos.

The ukulele and spark-plug packages' runner (``rdb_spark_plug/run.py``): a trial that crashes still leaves
RESULT.json saying how far it got.
"""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time
import traceback

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import fixed_scene as fixture
from . import layout as L
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

TASK_ID = "play-a-rock-groove-on-three-drum-pads"
MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 120
SCORER = "drum_groove_score"
HASHED = {"scene_sha256": "fixed_scene.py", "layout_sha256": "layout.py", "hits_sha256": "hits.py",
          "judge_sha256": "judge_midi.py", "midi_sha256": "midi.py", "grader_sha256": "grading.py",
          "adapter_sha256": "adapter.py", "poses_sha256": "poses.py", "scripted_sha256": "scripted.py"}


def live_dream(audit):
    import dream
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    client = BudgetedDreamClient(dream.DreamClient(), DREAM_REQUEST_CAP, audit)
    policy = DreamPolicy(session=DreamSession(make_client=lambda: client),
                         model=MODEL, record=False, timeout_s=90.)
    return policy, client


def experiment(embodiment, policy, log_dir, *, max_steps=fixture.MAX_STEPS, sinks=None):
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space, max_delta=1.0), names=["clamp", "delta-limit"])
    task = Task(TASK_ID, scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def write_evidence(embodiment, out, name):
    """hits.mid, events.json, scored-hits.csv, score-vs-hits.json and .png, from the trial's hits and events."""
    from .judge_midi import judge, title_line, write_csv
    from .midi import read_midi, write_midi
    from .plot import plot_score

    media = out / "media"
    media.mkdir(parents=True, exist_ok=True)
    write_midi(media / "hits.mid", embodiment.note_ons)
    events = dict(embodiment.events, end={"clean_from": embodiment.clean_from})
    (media / "events.json").write_text(json.dumps(events, indent=1) + "\n")
    result = judge(read_midi(media / "hits.mid"), events)        # what a reader of the files gets
    write_csv(result, media / "scored-hits.csv")
    plot_score(result, media / "score-vs-hits.png", name)
    summary = {k: v for k, v in result.items() if k not in ("classified", "note_rows")}
    summary["title"] = title_line(result, name)
    (media / "score-vs-hits.json").write_text(json.dumps(summary, indent=1) + "\n")
    return result, summary


def diagnostics(embodiment, reference):
    rule = getattr(embodiment, "trigger", None)
    return {
        "hits": rule.hits if rule is not None else None,
        "contacts_not_hits": [c for c in rule.contacts if c.get("rejected")] if rule is not None else None,
        "samples": embodiment.diagnostics,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
    }


def failure_record(args, embodiment, policy, error):
    failure = {"policy": args.policy, "failed": True, "error": error, "executed_steps": embodiment.tick,
               "final_grade": embodiment.final_grade and {k: v for k, v in embodiment.final_grade.items()
                                                          if k not in ("classified", "note_rows")},
               "state_after_reset": getattr(embodiment, "state_after_reset", None)}
    try:
        failure["diagnostics"] = diagnostics(embodiment, getattr(policy, "reference", None))
    except Exception as problem:      # never hide the original failure
        failure["diagnostics_error"] = repr(problem)
    (args.out / "RESULT.json").write_text(json.dumps(failure, indent=1, default=str) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=["scripted", "dream"])
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--live", action="store_true",
                        help="show the Isaac Sim window (browser desktop from ./setup.sh --desktop); same trial")
    parser.add_argument("--max-steps", type=int, default=fixture.MAX_STEPS,
                        help="a lower step cap, for the controls only (RDB_DRUM_DEMO=...), whose outcome is settled "
                             "once the demonstration has stopped; default: the task's run limit")
    args = parser.parse_args()
    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment = make_embodiment(media_dir=args.out / "media", headless=not args.live, step_limit=args.max_steps)
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(), None)
    result = None
    try:
        try:
            with tempfile.TemporaryDirectory(prefix="drum-groove-frames-") as tmp:
                sinks = [JsonLogSink(tmp), audit] if audit is not None else None
                result = experiment(embodiment, policy, Path(tmp), max_steps=args.max_steps, sinks=sinks)
                for path in Path(tmp).glob("*.json"):       # Inspect Robots' log, without the temporary folder's path
                    (args.out / path.name).write_text(path.read_text().replace(tmp, "<frames>"))
        except BaseException:
            failure_record(args, embodiment, policy, traceback.format_exc(limit=12))
            raise
    finally:
        try:
            if embodiment.recorder is not None:
                embodiment.recorder.close()
            evidence = write_evidence(embodiment, args.out, args.out.name)
        finally:
            embodiment.close()
            if client is not None:
                try:
                    policy.close()
                finally:
                    audit.close()
                (args.out / "DREAM_USAGE.json").write_text(json.dumps(
                    {"action_requests": client.calls, "cap": DREAM_REQUEST_CAP, "trials": 1,
                     "list_price_estimate_usd": round(client.calls * 3 / 2250, 6)}, indent=2) + "\n")
    if result is None or not result.samples:
        raise RuntimeError("Inspect Robots produced no trial")
    scene = result.samples[0]
    score = scene.reduced.get(SCORER)
    if result.status != "success" or score is None:
        detail = scene.error or result.error or result.status
        failure_record(args, embodiment, policy, str(detail))
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    package = Path(__file__).parent
    reference = getattr(policy, "reference", None)
    judged, summary = evidence
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "variant": reference.variant if reference is not None else None,
        "score": score, "scorer": SCORER, "score_from_files": judged["score"],
        "control_hz": fixture.CONTROL_HZ, "physics_hz": fixture.PHYSICS_HZ,
        "step_limit": args.max_steps, "task_run_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        **{key: hashlib.sha256((package / name).read_bytes()).hexdigest() for key, name in HASHED.items()},
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"), "yam_arena": version("yam-arena"),
        "seed": 0, "object_jitter": 0, "observation_state_keys": ["joint_pos"],
        "observation_cameras": ["top_cam", "left_cam", "right_cam"], "policy_inputs_camera_size": [320, 176],
        "wall_time_limit_is_external": True, "headless": not args.live,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
        "judged": summary,
        # Diagnostics only, none of them part of the score.
        "physics_settings": embodiment.settings, "state_after_reset": embodiment.state_after_reset,
        "grip_force_peak_n": {k: round(v, 1) for k, v in embodiment.grip_peak.items()},
        "hand_on_pad_peak_n": round(embodiment.hand_peak, 2),
        "min_stick_gap_mm": round(1000 * embodiment.min_stick_gap, 1),
        "stick_on_stick_or_other_hand_steps": embodiment.cross_steps,
    }
    print(json.dumps({k: v for k, v in status.items() if k != "judged"}), flush=True)
    status["diagnostics"] = diagnostics(embodiment, reference)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    return int(args.policy == "scripted" and reference is not None and reference.variant == "full" and score != 1.)


if __name__ == "__main__":
    code = main()
    os._exit(code)
