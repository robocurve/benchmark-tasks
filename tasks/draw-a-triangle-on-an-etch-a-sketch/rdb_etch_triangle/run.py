"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time
import traceback

import numpy as np

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import fixed_scene as fixture
from . import layout as L
from .adapter import CONTACT_TRACE_COLUMNS, make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 128
SCORER = "etch_triangle_score"


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
    task = Task("draw-a-triangle-on-an-etch-a-sketch",
                scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


def diagnostics(embodiment, reference):
    """Everything logged for whoever reads the run afterwards. Nothing here is ever given to a model policy."""
    phase_ends = []
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in embodiment.diagnostics if first < row["tick"] <= after]
            trace = [row for row in embodiment.contact_trace if first < row[0] <= after]
            if rows:
                peaks = ({col: max(r[i] for r in trace) for i, col in enumerate(CONTACT_TRACE_COLUMNS)
                          if col.endswith("_n")} if trace else {})
                phase_ends.append({"phase": name, "first_tick": first + 1, "peak_forces_n": peaks, **rows[-1]})
    return {
        "phase_ends": phase_ends,
        "samples": embodiment.diagnostics,
        "contact_trace_columns": CONTACT_TRACE_COLUMNS,
        "contact_trace": embodiment.contact_trace,
        "drawn_path_toy_mm": [[round(1000 * float(v), 2) for v in p] for p in getattr(embodiment, "path", [])],
        "skin_error": getattr(embodiment, "skin_error", None),
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "regrips": reference.regrips if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
        "worst_joint6_shift_deg": (round(float(np.degrees(reference.worst_j6_shift)), 2)
                                   if reference is not None else None),
    }


def model_numbers():
    return {"mm_per_knob_turn": 1000 * L.MM_PER_TURN, "backlash_deg": 20., "knob_turning_torque_nm": L.KNOB_FRICTION,
            "knob_d_mm": 1000 * L.KNOB_D, "knob_h_mm": 1000 * L.KNOB_H, "toy_mass_kg": L.MASS,
            "line_width_mm": 1000 * L.LINE_WIDTH, "start_on_screen_mm": [1000 * v for v in L.START_ON_SCREEN]}


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
    result = None
    try:
        try:
            with tempfile.TemporaryDirectory(prefix="etch-triangle-frames-") as tmp:
                sinks = [JsonLogSink(tmp), audit] if audit is not None else None
                result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
                for path in Path(tmp).glob("*.json"):
                    (args.out / path.name).write_bytes(path.read_bytes())
        except BaseException:
            # Failure record first, so a crashed trial still tells us how far it got.
            failure = {"policy": args.policy, "failed": True, "error": traceback.format_exc(limit=12),
                       "executed_steps": embodiment.tick, "final_grade": embodiment.final_grade,
                       "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
                       "state_after_reset": getattr(embodiment, "state_after_reset", None),
                       "model_numbers": model_numbers()}
            try:
                failure["diagnostics"] = diagnostics(embodiment, getattr(policy, "reference", None))
            except Exception as error:      # never hide the original failure
                failure["diagnostics_error"] = repr(error)
            (args.out / "RESULT.json").write_text(json.dumps(failure, indent=1, default=str) + "\n")
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
        # A policy or environment error inside the trial: keep the failure record and diagnostics before raising.
        failure = {"policy": args.policy, "failed": True, "error": str(detail),
                   "executed_steps": embodiment.tick, "final_grade": embodiment.final_grade,
                   "peak_interarm_contact_n": getattr(embodiment, "peak_interarm_n", None),
                   "state_after_reset": getattr(embodiment, "state_after_reset", None),
                   "model_numbers": model_numbers()}
        try:
            failure["diagnostics"] = diagnostics(embodiment, getattr(policy, "reference", None))
        except Exception as error:      # never hide the original failure
            failure["diagnostics_error"] = repr(error)
        (args.out / "RESULT.json").write_text(json.dumps(failure, indent=1, default=str) + "\n")
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    package = Path(__file__).parent
    reference = getattr(policy, "reference", None)
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": SCORER, "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "layout_sha256": hashlib.sha256((package / "layout.py").read_bytes()).hexdigest(),
        "stylus_sha256": hashlib.sha256((package / "stylus.py").read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256((package / "grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostics only, none of them part of the score. The first is read by the batch harness.
        "peak_interarm_contact_n": embodiment.peak_interarm_n,
        "peak_contact_n": {k: round(v, 2) for k, v in embodiment.peak_forces.items()},
        "state_after_reset": embodiment.state_after_reset,
        "model_numbers": model_numbers(),
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    print(json.dumps(status), flush=True)
    status["diagnostics"] = diagnostics(embodiment, reference)
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1, default=float) + "\n")
    return int(args.policy == "scripted" and score != 1.)


if __name__ == "__main__":
    code = main()
    if code:
        # The result and videos are already closed. An unsuccessful Isaac trial can hang in Kit's exit
        # callbacks; return the recorded failure to the batch gate instead of starting a model trial.
        os._exit(code)
