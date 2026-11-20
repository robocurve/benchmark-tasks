"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence.

A trial that crashes still leaves RESULT.json saying how far it got.
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

TASK_ID = "tune-a-ukulele-g-string-with-a-clip-on-tuner"
MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 128
SCORER = "ukulele_tuning_score"
HASHED = {"scene_sha256": "fixed_scene.py", "layout_sha256": "layout.py", "tuning_sha256": "tuning.py",
          "grader_sha256": "grading.py", "adapter_sha256": "adapter.py", "scripted_sha256": "scripted.py"}


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


def model_numbers():
    return {"cents_per_button_degree": L.CENTS_PER_DEG, "free_play_deg": L.FREE_PLAY_DEG,
            "button_drag_nm": L.BUTTON_DRAG, "button_mm": [1000 * L.BUTTON_LEN, 1000 * L.BUTTON_WIDE, 1000 * L.BUTTON_THICK],
            "g_start_cents": L.START_CENTS[L.TUNED], "pluck_out_mm": 1000 * L.PLUCK_OUT, "pluck_back_mm": 1000 * L.PLUCK_BACK,
            "pluck_return_s": L.PLUCK_RETURN_S, "pluck_gap_s": L.PLUCK_GAP_S, "string_stiffness_n_per_m": L.PLUCK_STIFFNESS,
            "tuner_reads_s": L.TUNER_ACTIVE_S, "needle_deg_at_50_cents": L.NEEDLE_MAX_DEG, "in_tune_cents": L.IN_TUNE_CENTS,
            "final_check_turn_deg": L.FREE_PLAY_DEG,
            "instrument_origin_mm": [1000 * v for v in fixture.ORIGIN]}


def diagnostics(embodiment, reference):
    """Everything logged for whoever reads the run afterwards. Nothing here is ever given to a model policy."""
    phase_ends = []
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in embodiment.diagnostics if first < row["tick"] <= after]
            if rows:      # the state at the end of each phase
                phase_ends.append({"phase": name, "first_tick": first + 1, **rows[-1]})
    milestones, seen = [], -1.
    for row in embodiment.diagnostics:     # when each score was first reached
        if row["score"] > seen:
            seen = row["score"]
            milestones.append(row)
    return {
        "phase_ends": phase_ends,
        "score_steps": milestones,
        "plucks": getattr(embodiment, "pluck_events", []),
        "samples": embodiment.diagnostics,
        "skin_error": getattr(embodiment, "skin_error", None),
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "regrips": reference.regrips if reference is not None else None,
        "cycles": reference.cycles if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
        "worst_joint6_shift_deg": round(float(reference.j6_shift) * 57.29578, 2) if reference is not None else None,
    }


def failure_record(args, embodiment, policy, error):
    failure = {"policy": args.policy, "failed": True, "error": error,
               "executed_steps": embodiment.tick, "final_grade": embodiment.final_grade,
               "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
               "state_after_reset": getattr(embodiment, "state_after_reset", None),
               "model_numbers": model_numbers()}
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
    args = parser.parse_args()
    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment = make_embodiment(media_dir=args.out / "media", headless=not args.live)
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(), None)
    result = None
    try:
        try:
            with tempfile.TemporaryDirectory(prefix="ukulele-tuning-frames-") as tmp:
                sinks = [JsonLogSink(tmp), audit] if audit is not None else None
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
        except BaseException:
            failure_record(args, embodiment, policy, traceback.format_exc(limit=12))
            raise
    finally:
        try:
            embodiment.close()
        finally:
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
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "variant": reference.variant if reference is not None else None,
        "score": score, "scorer": SCORER, "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        **{key: hashlib.sha256((package / name).read_bytes()).hexdigest() for key, name in HASHED.items()},
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "object_jitter": 0, "observation_state_keys": ["joint_pos"],
        "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostics only, none of them part of the score.
        "peak_interarm_contact_n": embodiment.peak_interarm_n,
        "peaks": {name: round(value, 3) for name, value in embodiment.peak.items()},
        "state_after_reset": embodiment.state_after_reset,
        "model_numbers": model_numbers(),
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    print(json.dumps(status), flush=True)
    status["diagnostics"] = diagnostics(embodiment, reference)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    return int(args.policy == "scripted" and reference is not None and reference.variant == "full" and score != 1.)


if __name__ == "__main__":
    code = main()
    if code:
        # The result and videos are already closed. An unsuccessful Isaac trial can hang in Kit's exit
        # callbacks; return the recorded failure instead of waiting.
        os._exit(code)
