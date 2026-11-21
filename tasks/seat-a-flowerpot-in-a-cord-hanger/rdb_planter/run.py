"""Run one fixed planter trial through Inspect Robots."""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time

from . import fixed_scene as fixture

MODEL = "dreamlabs/dream-1-beta"
DREAM_CHUNK_STEPS = 48
DREAM_REQUEST_CAP = 4 * -(-fixture.MAX_STEPS // DREAM_CHUNK_STEPS)
VARIANTS = ("idle", "left_only", "right_only")


def outcome(result, failure, final_grade):
    """(score, completed, why not) of a trial, whether or not Inspect Robots finished it.

    A finished trial is scored by the scorer. One stopped by an error keeps the grade of its last executed step,
    which is what the scorer would have read; with no executed step there is no score.
    """
    sample = result.samples[0] if result is not None and result.samples else None
    scored = sample.reduced.get(fixture.SCORER) if sample is not None else None
    if failure is None and result is not None and result.status == "success" and scored is not None:
        return scored, True, None
    if failure is not None:
        detail = f"{type(failure).__name__}: {failure}"
    elif sample is None:
        detail = "Inspect Robots produced no trial"
    else:
        detail = str(sample.error or result.error or result.status)
    return (final_grade["score"] if final_grade else None), False, detail


def live_dream(audit):
    import dream
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    from .dream_audit import BudgetedDreamClient

    client = BudgetedDreamClient(dream.DreamClient(), DREAM_REQUEST_CAP, audit)
    policy = DreamPolicy(session=DreamSession(make_client=lambda: client), model=MODEL, record=False, timeout_s=90.)
    return policy, client


def experiment(embodiment, policy, log_dir, *, max_steps=fixture.MAX_STEPS, sinks=None):
    from inspect_robots import Scene, eval as inspect_eval
    from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
    from inspect_robots.task import Task

    from .grading import make_scorer

    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space, max_delta=1.0), names=["clamp", "delta-limit"])
    task = Task(fixture.TASK_ID, scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=["scripted", "dream"])
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--variant", choices=VARIANTS, default=None, help="scripted control run (see module doc)")
    args = parser.parse_args()
    from inspect_robots.logging import JsonLogSink           # lazily: needs the simulator stack

    from .arena import make_embodiment
    from .reference import make_scripted
    from .dream_audit import DreamAudit, TRANSITION

    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment = make_embodiment(out=args.out)
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(args.variant), None)
    result = failure = None
    try:
        with tempfile.TemporaryDirectory(prefix="planter-frames-") as tmp:
            sinks = [JsonLogSink(tmp), audit] if audit is not None else None
            try:
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
            finally:
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
    except Exception as error:      # reported below, after the result file has been written
        failure = error
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
                    {"action_requests": client.calls, "cap": DREAM_REQUEST_CAP,
                     "cap_reached_at_step": client.capped_at, "trials": 1,
                     "list_price_estimate_usd": round(client.calls * 3 / 2250, 6)}, indent=2) + "\n")
    final_grade = getattr(embodiment, "final_grade", None)
    score, completed, detail = outcome(result, failure, final_grade)
    scene = result.samples[0] if result is not None and result.samples else None
    reference = getattr(policy, "log", None)
    package = Path(__file__).parent
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None, "variant": args.variant,
        "score": score, "scorer": fixture.SCORER, "completed": completed, "error": detail,
        "control_hz": fixture.CONTROL_HZ, "step_limit": fixture.MAX_STEPS,
        "executed_steps": getattr(embodiment, "tick", 0), "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "layout_sha256": hashlib.sha256((package / "fixed_scene.py").read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256((package / "grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"), "yam_arena": version("yam-arena"),
        "seed": 0, "observation_state_keys": getattr(embodiment,"policy_state_keys",[]), "observation_cameras": sorted(getattr(embodiment,"policy_input_shapes",{})),
        "policy_input_shapes": getattr(embodiment,"policy_input_shapes",{}), "policy_inputs_camera_size": list(getattr(embodiment,"policy_input_shapes",{}).get("top_cam",[])[:2][::-1]), "wall_time_limit_is_external": True,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        "dream_request_cap": DREAM_REQUEST_CAP if client is not None else None,
        "peak_interarm_contact_n": getattr(embodiment,"peak_interarm_contact_n",None),
        "interarm_samples": getattr(embodiment,"interarm_samples",0),
        "final_grade": final_grade,
        "termination_reason": scene.termination_reasons[0] if scene is not None and scene.termination_reasons else None,
        "reference_waypoints": reference,
    }
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    print(json.dumps({k: v for k, v in status.items() if k not in ("diagnostics", "demo")}, default=float), flush=True)
    if not completed:
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail} (RESULT.json has the score reached)")
    # An isolated-arm solution counts as a bimanual counterexample.
    return int(args.policy == "scripted" and (
        (args.variant is None and score != 1.) or (args.variant is not None and score == 1.)
    ))


if __name__ == "__main__":
    code = main()
    if code:
        # The result and video are already closed. An unsuccessful Isaac trial can hang in Kit's atexit callbacks;
        # return the recorded failure to the batch gate instead of starting a model trial.
        os._exit(code)
