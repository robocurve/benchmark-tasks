"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence.

``python -m rdb_filter_clamp.run scripted|dream --out DIR``. ``DEMO_VARIANT=no_hold`` (scripted only) is the
control: the hand that seated the funnel lets go of it before the clamp is fitted; it cannot demonstrate the upper-only lift.
RESULT.json is written even when the trial fails with an error (``error`` and the diagnostics gathered so far).
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
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 160
SCORER = "filter_clamp_score"


def live_dream(audit):
    import dream
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession

    client = BudgetedDreamClient(dream.DreamClient(), DREAM_REQUEST_CAP, audit)
    policy = DreamPolicy(session=DreamSession(make_client=lambda: client), model=MODEL, record=False, timeout_s=90.)
    return policy, client


def experiment(embodiment, policy, log_dir, *, max_steps=fixture.MAX_STEPS, sinks=None):
    space = embodiment.info.action_space
    guards = GuardChain(ClampGuard(space), DeltaLimitGuard(space, max_delta=1.0), names=["clamp", "delta-limit"])
    task = Task("fit-the-spring-clamp-on-a-glass-vacuum-filter-holder",
                scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _diagnostics(embodiment, policy):
    reference = getattr(policy, "reference", None)
    phase_ends = []
    rows = getattr(embodiment, "diagnostics", [])
    if reference is not None:
        starts = reference.phases + [(getattr(embodiment, "tick", 0) + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            inside = [row for row in rows if first < row["tick"] <= after]
            if inside:
                phase_ends.append({"phase": name, **inside[-1]})
    return {
        "after_reset": getattr(embodiment, "after_reset", None),
        "phase_ends": phase_ends,
        "samples": rows,
        # every step while the clamp is round the joint (and every 2nd while the funnel sits unclamped):
        # poses, jaw angles, forces of fingertips on funnel and jaws, of the jaws on the glass, funnel on base
        "contact_trace": getattr(embodiment, "contact_trace", []),
        "peak_contacts_n": getattr(embodiment, "peaks", {}),
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=["scripted", "dream"])
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment = make_embodiment(media_dir=args.out / "media")
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(), None)
    result, error = None, None
    package = Path(__file__).parent
    status = {"policy": args.policy, "model": MODEL if args.policy == "dream" else None,
              "demo_variant": (os.environ.get("DEMO_VARIANT") or None) if args.policy == "scripted" else None,
              "scorer": SCORER, "control_hz": fixture.CONTROL_HZ, "step_limit": fixture.MAX_STEPS,
              "scene_sha256": _hash(fixture.__file__), "asset_sha256": _hash(fixture.USD),
              "layout_sha256": _hash(package / "layout.py"), "grader_sha256": _hash(package / "grading.py"),
              "inspect_robots": version("inspect-robots"), "yam_arena": version("yam-arena"), "seed": 0,
              "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
              "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
              "scripted_demo_reads_simulator_state": args.policy == "scripted"}
    try:
        with tempfile.TemporaryDirectory(prefix="filter-clamp-frames-") as tmp:
            sinks = [JsonLogSink(tmp), audit] if audit is not None else None
            try:
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
            finally:
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
    except Exception as exc:          # write the failure record before raising (first runs must be informative)
        error = exc
        status["error"] = "".join(traceback.format_exception(exc))[-6000:]
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
    score = None
    if error is None:
        if result is None or not result.samples:
            status["error"] = "Inspect Robots produced no trial"
        else:
            sample = result.samples[0]
            score = sample.reduced.get(SCORER)
            if result.status != "success" or score is None:
                status["error"] = f"Inspect Robots trial did not complete: {sample.error or result.error or result.status}"
            status["termination_reason"] = sample.termination_reasons[0] if sample.termination_reasons else None
    status.update({
        "score": score, "executed_steps": getattr(embodiment, "tick", 0), "elapsed_s": round(time.monotonic() - start, 2),
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostic only: arm-to-arm contact is not part of the score.
        "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
        "body_masses_kg": getattr(embodiment, "masses", None),
        "clamp_dof_names": getattr(embodiment, "dof_names", None),
        "final_grade": getattr(embodiment, "final_grade", None),
    })
    print(json.dumps({k: v for k, v in status.items() if k != "error"}), flush=True)
    status["diagnostics"] = _diagnostics(embodiment, policy)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float, allow_nan=True) + "\n")
    if error is not None:
        raise error
    if "error" in status:
        raise RuntimeError(status["error"])
    return int(args.policy == "scripted" and score != 1.)


if __name__ == "__main__":
    raise SystemExit(main())
