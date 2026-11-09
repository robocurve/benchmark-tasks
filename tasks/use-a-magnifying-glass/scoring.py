"""Complete magnifier task scorer: image hold, measured clearance and review.

Run without a completed review to obtain the numeric checks and an uncertified
zero. A reviewer supplies the observations that pose telemetry cannot establish.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

try:
    from .common import table_clearance, tool_vertices
    from .score_hold import audit_run, inspect_trace, sha256
except ImportError:
    from common import table_clearance, tool_vertices
    from score_hold import audit_run, inspect_trace, sha256

TABLE_Z_M = .7475
CLEARANCE_M = .020
RUN_LIMIT = 1200
REVIEW_CLAIMS = {
    "prescribed_initial_layout": "The complete video begins in the prescribed reset layout, with the magnifier resting on the table.",
    "picked_up_by_handle": "The robot picks the magnifier up by its handle using an ordinary gripper grasp.",
    "held_only_by_gripper": "Throughout the scored interval, only the gripper supports the tool at its handle; the glass/frame are not propped against a camera, robot link, table or other object.",
    "tool_and_target_intact": "The tool and printed target remain intact and unmodified, and the target stays fixed throughout the trial.",
    "ordinary_physics_and_camera": "The recorded trial uses ordinary robot contact, no object attachment/teleport/applied object force, and unchanged native camera settings without digital crop, zoom or image replacement.",
    "continuous_hold_visible": "The video shows an uninterrupted handle grasp and at least 20 mm whole-tool table clearance throughout the scored three-second interval, with no visible interruption.",
}


def evidence_hashes(run, target_json, video_name):
    run = Path(run)
    return {"trace": sha256(run/"trace.jsonl"), "run_record": sha256(run/"pickup.json"),
            "target_json": sha256(target_json), "video": sha256(run/video_name)}


def review_template(run, target_json, video_name="demo.mp4"):
    record = json.loads((Path(run)/"pickup.json").read_text())
    end = record["steps"]
    return {"schema": "magnifier-physical-review-v1", "reviewer": "",
            "reviewed_full_trial": False, "video_file": video_name,
            "source_sha256": evidence_hashes(run, target_json, video_name),
            "interval_steps": [end-60, end],
            "observations": {name: None for name in REVIEW_CLAIMS},
            "claim_definitions": REVIEW_CLAIMS, "notes": "Complete only after reviewing the actual video and simulator provenance; unknown observations remain null."}


def check_review(review, expected_hashes, interval):
    errors, failed = [], []
    if review.get("schema") != "magnifier-physical-review-v1":
        errors.append("Review has an unsupported or missing schema")
    if not isinstance(review.get("reviewer"), str) or not review["reviewer"].strip():
        errors.append("Review must identify its reviewer")
    if review.get("reviewed_full_trial") is not True:
        errors.append("Complete trial video has not been reviewed")
    if review.get("source_sha256") != expected_hashes:
        errors.append("Review evidence hashes do not match this run, target and video")
    if review.get("interval_steps") != interval:
        errors.append("Review does not cover the scored interval")
    observations = review.get("observations", {})
    for claim in REVIEW_CLAIMS:
        value = observations.get(claim)
        if type(value) is not bool:
            errors.append("Review observation is missing or unknown: " + claim)
        elif not value:
            failed.append(claim)
    return {"valid": not errors, "passes": not errors and not failed,
            "errors": errors, "failed_conditions": failed,
            "reviewer": review.get("reviewer"), "interval_steps": interval}


def clearance_audit(trace, record):
    window, interval, errors = inspect_trace(trace, record["steps"])
    result = {"valid": False, "passes": False, "minimum_required_m": CLEARANCE_M,
              "table_top_z_m": TABLE_Z_M, "errors": errors, "samples": []}
    if errors:
        return result
    try:
        local = tool_vertices()
        # Canonical mesh vertices include the handle, entire rim and glass.
        # Recompute from recorded poses rather than trusting a clearance label.
        for state in window:
            clearance = table_clearance(local, state["tool_pose_xyzw"], TABLE_Z_M)
            item = {"step": state["step"], "clearance_m": clearance}
            if "tool_table_clearance_m" in state:
                recorded = float(state["tool_table_clearance_m"])
                if not np.isfinite(recorded) or abs(recorded-clearance) > 1e-6:
                    raise ValueError("Recorded whole-tool clearance differs from mesh/pose recomputation at step " + str(state["step"]))
                item["recorded_clearance_matches"] = True
            result["samples"].append(item)
        minimum = min(item["clearance_m"] for item in result["samples"])
        result.update(valid=True, passes=minimum >= CLEARANCE_M, minimum_clearance_m=minimum,
                      minimum_step=min(result["samples"], key=lambda x: x["clearance_m"])["step"],
                      geometry="All canonical collision-solid vertices, transformed by the actual measured rigid-tool pose.",
                      scope="61 sampled states over three seconds. The separately required video review checks visible grasp/support continuity; neither 20 Hz record proves motion between samples.")
    except (ValueError, KeyError, TypeError) as error:
        result["errors"].append(str(error))
    return result


def score_run(run, target_json, review_path=None):
    run, target_json = Path(run), Path(target_json)
    result = {"schema": "magnifier-task-score-v1", "score": 0.0, "certified": False,
              "reasons": [], "rule": "All physical conditions must pass throughout the final three seconds; then the task score is the minimum photo score over those same 61 states."}
    try:
        result["photo_hold"] = audit_run(run, target_json)
        record = json.loads((run/"pickup.json").read_text())
        trace = [json.loads(line) for line in (run/"trace.jsonl").read_text().splitlines() if line.strip()]
        if not isinstance(record.get("steps"), int) or not 60 <= record["steps"] <= RUN_LIMIT:
            raise ValueError("Scored trial must have 60 to 1200 control steps at 20 Hz")
        if [state.get("step") for state in trace] != list(range(record["steps"]+1)):
            raise ValueError("Complete trial trace must contain every ordered state from reset through termination")
        result["clearance"] = clearance_audit(trace, record)
        result["source_sha256"] = {"scoring_py": sha256(__file__), "common_py": sha256(Path(__file__).with_name("common.py")),
                                   "tool_py": sha256(Path(__file__).with_name("tool.py")), "lens_py": sha256(Path(__file__).with_name("lens.py"))}
        if review_path is None:
            result["reasons"].append("A completed, evidence-bound physical review is required")
        else:
            review = json.loads(Path(review_path).read_text())
            video_name = review.get("video_file", "")
            if not video_name or Path(video_name).is_absolute() or ".." in Path(video_name).parts:
                raise ValueError("Review video_file must be a relative path inside the run directory")
            interval = [record["steps"]-60, record["steps"]]
            result["review"] = check_review(review, evidence_hashes(run, target_json, video_name), interval)
            result["source_sha256"]["review_json"] = sha256(review_path)
            result["reasons"].extend(result["review"]["errors"])
        if not result["photo_hold"]["certified"]:
            result["reasons"].append("Photo hold is not certified; see photo_hold.reasons")
        if not result["clearance"]["valid"]:
            result["reasons"].append("Whole-tool clearance evidence is invalid")
        if result["reasons"]:
            return result
        result["certified"] = True
        if result["clearance"]["passes"] and result["review"]["passes"]:
            result["score"] = result["photo_hold"]["photo_hold_score"]
        else:
            result["failed_conditions"] = ([] if result["clearance"]["passes"] else ["whole-tool clearance below 20 mm"]) + result["review"]["failed_conditions"]
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        result["reasons"].append(type(error).__name__ + ": " + str(error))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--target-json", type=Path, required=True)
    parser.add_argument("--review", type=Path)
    parser.add_argument("--write-review-template", action="store_true")
    parser.add_argument("--video-file", default="demo.mp4")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = review_template(args.run, args.target_json, args.video_file) if args.write_review_template else score_run(args.run, args.target_json, args.review)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({key: result[key] for key in ("schema", "score", "certified", "reasons") if key in result}, indent=2))
