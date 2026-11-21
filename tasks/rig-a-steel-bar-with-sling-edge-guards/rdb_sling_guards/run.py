"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence.

The spark-plug package's runner (rdb_spark_plug/run.py, itself rdb_ukulele_tuning/run.py): a trial that crashes still
leaves RESULT.json saying how far it got.
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
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION, request_budget_spent
from .grading import make_scorer

TASK_ID = "rig-a-steel-bar-with-sling-edge-guards"
MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 300             # 6,000 steps at 20 Hz; the spark-plug trial used 373 requests for 9,000 steps
SCORER = "sling_guards_score"
HASHED = {"scene_sha256": "fixed_scene.py", "layout_sha256": "layout.py", "hook_sha256": "hook.py",
          "magnet_sha256": "magnet.py", "grader_sha256": "grading.py", "adapter_sha256": "adapter.py",
          "scripted_sha256": "scripted.py"}


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
    return {"bar_mm": [1000 * L.BAR_S, 1000 * L.BAR_S, 1000 * L.BAR_L], "bar_z_mm": [1000 * L.Z_BOT, 1000 * L.Z_TOP],
            "sling_plane_x_mm": 1000 * L.XS, "web_links": L.N_LINKS, "link_mm": round(1000 * L.LINK_L, 2),
            "link_mass_kg": round(L.LINK_MASS, 5), "hinge_k_nm_per_rad": round(L.HINGE_K, 5), "fitting_kg": L.FIT_MASS,
            "guard_mm": [1000 * L.GUARD_LEN, 1000 * L.GUARD_FL, 1000 * L.GUARD_T], "guard_kg": round(L.GUARD_MASS, 4),
            "magnet": L.MAGNET, "magnet_pull_n": round(L.MAGNET_PULL_N, 2), "magnet_g0_mm": 1000 * L.MAG_G0,
            "hook_bowl_bottom_mm": 1000 * L.HOOK_Z, "hook_bowl_centres_mm": 1000 * L.RAM_B, "bowl_radius_mm": 1000 * L.BOWL_R_IN,
            "hook_kg": L.HOOK_MASS, "pull_n": L.PULL_N, "hold_s": L.HOLD_S, "edge_zone_mm": 1000 * L.EDGE_ZONE,
            "bare_limit_n": L.BARE_LIMIT_N, "seat_tol_mm": 1000 * L.SEAT_TOL, "seat_angle_deg": L.SEAT_ANGLE_DEG}


def diagnostics(embodiment, reference):
    """Everything logged for whoever reads the run afterwards. Nothing here is ever given to a model policy."""
    phase_ends = []
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in embodiment.diagnostics if first < row["tick"] <= after]
            if rows:
                phase_ends.append({"phase": name, "first_tick": first + 1, **rows[-1]})
    milestones, seen = [], -1.
    for row in embodiment.diagnostics:
        if row["score"] > seen:
            seen = row["score"]
            milestones.append(row)
    return {
        "phase_ends": phase_ends,
        "score_steps": milestones,
        "samples": embodiment.diagnostics,
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
    }


def failure_record(args, embodiment, policy, error):
    failure = {"policy": args.policy, "failed": True, "error": error,
               "executed_steps": embodiment.tick, "final_grade": embodiment.final_grade,
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
    parser.add_argument("--max-steps", type=int, default=fixture.MAX_STEPS,
                        help="a lower step cap, for the controls only (RDB_RIGGING_DEMO=...), whose outcome is "
                             "settled once the demonstration is at rest; default: the task's run limit")
    args = parser.parse_args()
    start = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=False)
    fast = os.environ.get("RDB_RIGGING_FAST") == "1"         # development only: no review camera and no media
    embodiment = make_embodiment(media_dir=None if fast else args.out / "media", headless=not args.live)
    audit = DreamAudit(args.out) if args.policy == "dream" else None
    policy, client = live_dream(audit) if audit is not None else (make_scripted(), None)
    if client is not None:              # all requests used: end and score the trial instead of failing it
        embodiment.stop_check = lambda: request_budget_spent(client, policy, DREAM_REQUEST_CAP)
    result = None
    try:
        try:
            with tempfile.TemporaryDirectory(prefix="sling-guards-frames-") as tmp:
                sinks = [JsonLogSink(tmp), audit] if audit is not None else None
                result = experiment(embodiment, policy, Path(tmp), max_steps=args.max_steps, sinks=sinks)
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
                     "capacity_waits": client.capacity_waits, "capacity_wait_s": client.capacity_wait_s,
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
        "step_limit": args.max_steps, "task_run_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        **{key: hashlib.sha256((package / name).read_bytes()).hexdigest() for key, name in HASHED.items()},
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "object_jitter": 0, "observation_state_keys": ["joint_pos"],
        "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "headless": not args.live,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        "dream_capacity_waits": client.capacity_waits if client is not None else 0,
        # Diagnostics only, none of them part of the score.
        "peaks": embodiment.peaks,
        "timing_s": {k: round(v, 1) for k, v in embodiment.timing.items()},
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
        # callbacks (rdb_etch_triangle/run.py); return the recorded failure instead of waiting.
        os._exit(code)
