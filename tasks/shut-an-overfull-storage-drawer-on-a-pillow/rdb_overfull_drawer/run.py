"""Run one fixed trial in Inspect Robots and preserve its independently scored evidence."""

import argparse
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import tempfile
import time

from inspect_robots import Scene, eval as inspect_eval
from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
from inspect_robots.logging import JsonLogSink
from inspect_robots.task import Task

from . import fixed_scene as fixture
from . import layout
from .adapter import make_embodiment, make_scripted
from .dream_audit import BudgetedDreamClient, DreamAudit, TRANSITION
from .grading import make_scorer

MODEL = "dreamlabs/dream-1-beta"
DREAM_REQUEST_CAP = 96
SCORER = "overfull_drawer_score"


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
    task = Task("shut-an-overfull-storage-drawer-on-a-pillow",
                scenes=[Scene(id="fixed-start", instruction=fixture.INSTRUCTION)],
                scorer=make_scorer(), max_steps=max_steps)
    return inspect_eval(task, policy, embodiment, log_dir=str(log_dir), guard=guards, seed=0,
                        sinks=sinks, grader=None, store_frames=True, store_records=True, fail_on_error=True)[0]


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
        with tempfile.TemporaryDirectory(prefix="overfull-drawer-frames-") as tmp:
            sinks = [JsonLogSink(tmp), audit] if audit is not None else None
            result = experiment(embodiment, policy, Path(tmp), sinks=sinks)
            for path in Path(tmp).glob("*.json"):
                (args.out / path.name).write_bytes(path.read_bytes())
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
        raise RuntimeError(f"Inspect Robots trial did not complete: {detail}")
    package = Path(__file__).parent
    reference = getattr(policy, "reference", None)
    variant = getattr(policy, "variant", None)
    status = {
        "policy": args.policy, "model": MODEL if args.policy == "dream" else None,
        "score": score, "scorer": SCORER, "control_hz": fixture.CONTROL_HZ,
        "step_limit": fixture.MAX_STEPS, "executed_steps": embodiment.tick,
        "elapsed_s": round(time.monotonic() - start, 2),
        "scene_sha256": hashlib.sha256(Path(fixture.__file__).read_bytes()).hexdigest(),
        "asset_sha256": hashlib.sha256(fixture.USD.read_bytes()).hexdigest(),
        "layout_sha256": hashlib.sha256((package / "layout.py").read_bytes()).hexdigest(),
        "grader_sha256": hashlib.sha256((package / "grading.py").read_bytes()).hexdigest(),
        "inspect_robots": version("inspect-robots"),
        "yam_arena": version("yam-arena"),
        "seed": 0, "observation_state_keys": ["joint_pos"], "observation_cameras": ["top_cam", "left_cam", "right_cam"],
        "policy_inputs_camera_size": [320, 176], "wall_time_limit_is_external": True,
        "scripted_demo_reads_simulator_state": args.policy == "scripted",
        "scripted_demo_variant": variant,
        "dream_chunk_anchor": TRANSITION if audit is not None else None,
        "dream_action_requests": client.calls if client is not None else 0,
        # Diagnostics only, none of them part of the score. The first is read by the batch harness.
        "peak_interarm_contact_n": embodiment.peak_interarm_n,
        "peak_arm_on_pillow_n": [round(v, 2) for v in embodiment.peak_arm_on_pillow_n],      # left arm, right arm
        "peak_arm_on_drawer_n": [round(v, 2) for v in embodiment.peak_arm_on_drawer_n],
        # A pad under the frame's top should never be above its underside (0 or a little below): more than about
        # 1 mm means the contact between pad and frame gave way.
        "peak_pad_above_opening_under_frame_mm": round(embodiment.peak_pad_above_opening_under_frame_mm, 2),
        "pad_tops_after_reset_mm": embodiment.pad_tops_after_reset_mm,
        "pillow_surface_drawn": embodiment.skin is not None, "pillow_surface_error": embodiment.skin_error,
        "model_numbers": {"rail_z_mm": 1000 * layout.RAIL_Z, "rows_proud_mm": [round(1000 * v, 1) for v in layout.ROW_PROUD],
                          "rows_catch_at_open_mm": [round(1000 * layout.blocking_open(r), 1) for r in range(layout.ROWS)],
                          "press_force_per_row_n": round(layout.press_force(2, .025), 2),
                          "runner_friction_n": layout.RUNNER_FRICTION_N},
        "final_grade": embodiment.final_grade,
        "termination_reason": scene.termination_reasons[0] if scene.termination_reasons else None,
    }
    print(json.dumps(status), flush=True)
    # Diagnostics for whoever reads the run afterwards: sampled every 5 steps by the environment, the state at the
    # end of each phase of the demonstration, and the demonstration's own log of each press-and-feed cycle.
    # Nothing here is ever given to a model policy.
    phase_ends = []
    if reference is not None:
        starts = reference.phases + [(embodiment.tick + 1, "end")]
        for (first, name), (after, _) in zip(starts, starts[1:]):
            rows = [row for row in embodiment.diagnostics if first < row["tick"] <= after]
            if rows:
                phase_ends.append({"phase": name, "first_tick": first + 1,
                                   "peak_arm_on_pillow_n": [max(r["arm_on_pillow_n"][i] for r in rows) for i in range(2)],
                                   "peak_arm_on_drawer_n": [max(r["arm_on_drawer_n"][i] for r in rows) for i in range(2)],
                                   **rows[-1]})
    status["diagnostics"] = {
        "phase_ends": phase_ends,
        "samples": embodiment.diagnostics,
        "phases": [list(row) for row in reference.phases] if reference is not None else None,
        "reference": json.loads(json.dumps(reference.log, default=float)) if reference is not None else None,
        "worst_ik_mm": round(1000 * reference.worst_ik, 3) if reference is not None else None,
    }
    (args.out / "RESULT.json").write_text(json.dumps(status, indent=1) + "\n")
    return int(args.policy == "scripted" and (score != 1. or variant != "full"))


if __name__ == "__main__":
    code = main()
    if code:
        # The result and videos are already closed. An unsuccessful Isaac trial can hang in Kit's exit
        # callbacks; return the recorded failure to the batch gate instead of starting a model trial.
        os._exit(code)
