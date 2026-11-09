"""Offline audit of a complete three-second magnifier photo hold.

This certifies image evidence and image registration, never physical grasp.
Small camera motion is allowed when all four actual outer marks remain within
one raw pixel of the frozen reference. ``scoring.py`` adds physical conditions.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

try:
    from .inspection_measure import compare_arrays
except ImportError:
    from inspection_measure import compare_arrays


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect_trace(trace, final_step):
    reasons = []
    first_step = final_step-60
    window = [state for state in trace if first_step <= state.get("step", -1) <= final_step]
    steps = [state.get("step") for state in window]
    expected = list(range(first_step, final_step+1))
    if first_step < 0 or steps != expected:
        reasons.append("Trace must contain exactly 61 ordered, consecutive states for the final three seconds")
    info = {"steps": [first_step, final_step], "state_count": len(window), "expected_control_hz": 20}
    if reasons:
        return window, info, reasons
    times = np.asarray([state.get("time_s", np.nan) for state in window], dtype=float)
    if not np.all(np.isfinite(times)) or not np.allclose(times, np.asarray(expected)/20, atol=1e-9, rtol=0):
        reasons.append("Trace times must equal their control step divided by 20")
    info["duration_s"] = float(times[-1]-times[0])
    if not np.isclose(info["duration_s"], 3.0, atol=1e-9, rtol=0):
        reasons.append("The trace interval does not span exactly three seconds")
    try:
        eyes = np.asarray([state["camera_eye"] for state in window], dtype=float)
        rotations = np.asarray([state["camera_rotation"] for state in window], dtype=float)
        if eyes.shape != (61, 3) or rotations.shape != (61, 3, 3) or not np.all(np.isfinite(eyes)) or not np.all(np.isfinite(rotations)):
            raise ValueError("Invalid camera pose shapes or non-finite values")
        if not np.allclose(rotations @ rotations.transpose(0, 2, 1), np.eye(3), atol=1e-6, rtol=0):
            raise ValueError("Camera rotations are not orthonormal")
        translation = float(np.linalg.norm(eyes-eyes[-1], axis=1).max())
        matrix_change = float(np.abs(rotations-rotations[-1]).max())
        info.update(maximum_camera_translation_from_final_m=translation,
                    maximum_camera_rotation_matrix_element_change=matrix_change,
                    camera_invariant=translation <= 1e-9 and matrix_change <= 1e-9,
                    pose_values_are_diagnostic=True,
                    camera_condition="All four actual outer-marker centres must remain within one native pixel of the same-pose frozen reference in every hold image.")
    except (KeyError, ValueError, TypeError) as error:
        reasons.append("Invalid camera trace: " + str(error))
    return window, info, reasons


def missing_frame_files(run, steps):
    return [f"hold-{step:04d}-{kind}.png" for step in steps for kind in ("raw", "policy")
            if not (run/f"hold-{step:04d}-{kind}.png").is_file()]


def load_image(path):
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"))


def check_policy_resize(raw, policy):
    """Independent no-antialias half-pixel bilinear check, within one RGB level.

    The runner's exact cv2 equality assertion is still required. This independent
    check allows integer implementation rounding, not image filtering or zoom.
    """
    if raw.shape != (360, 640, 3) or policy.shape != (224, 224, 3):
        return {"passes": False, "error": "Expected 640x360 raw and 224x224 policy RGB"}
    x = (np.arange(224)+.5)*raw.shape[1]/224-.5
    y = (np.arange(224)+.5)*raw.shape[0]/224-.5
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x-x0, y-y0
    x1 = np.clip(x0+1, 0, raw.shape[1]-1)
    y1 = np.clip(y0+1, 0, raw.shape[0]-1)
    x0, y0 = np.clip(x0, 0, raw.shape[1]-1), np.clip(y0, 0, raw.shape[0]-1)
    a = raw[y0[:, None], x0[None, :]].astype(float)
    b = raw[y0[:, None], x1[None, :]].astype(float)
    c = raw[y1[:, None], x0[None, :]].astype(float)
    d = raw[y1[:, None], x1[None, :]].astype(float)
    reference = ((1-fy)[:, None, None]*((1-fx)[None, :, None]*a+fx[None, :, None]*b)
                 + fy[:, None, None]*((1-fx)[None, :, None]*c+fx[None, :, None]*d))
    delta = np.abs(np.rint(reference).astype(int)-policy.astype(int))
    return {"passes": bool(delta.max() <= 1), "maximum_rgb_level_difference": int(delta.max()),
            "mean_rgb_level_difference": float(delta.mean()),
            "method": "Unfiltered half-pixel bilinear resize, allowing one integer RGB level of rounding; exact cv2 verification is additionally required from the run record."}


def quat_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("Invalid tool quaternion")
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def physical_trace_summary(window, record):
    result = {"physical_grasp_certified": False,
              "limit": "Positions and orientations describe the trace. They do not establish contact forces, grip integrity, collision validity or safe clearance."}
    try:
        offset = np.asarray(record["tool"]["lens_center_in_handle_m"], dtype=float)
        roots, centers, distances, offsets = [], [], [], []
        for state in window:
            pose = np.asarray(state["tool_pose_xyzw"], dtype=float)
            rotation = quat_matrix(pose[3:])
            center = pose[:3]+rotation@offset
            eye = np.asarray(state["camera_eye"])
            direction = -np.asarray(state["camera_rotation"])[:, 2]
            relative = center-eye
            roots.append(pose[:3]); centers.append(center)
            distances.append(float(relative@direction))
            offsets.append(float(np.linalg.norm(relative-direction*(relative@direction))))
        roots, centers = np.asarray(roots), np.asarray(centers)
        result.update(lens_center_world_min_m=centers.min(axis=0).tolist(),
                      lens_center_world_max_m=centers.max(axis=0).tolist(),
                      lens_camera_axial_distance_range_m=[min(distances), max(distances)],
                      maximum_lens_offset_from_camera_axis_m=max(offsets),
                      maximum_tool_root_translation_from_final_m=float(np.linalg.norm(roots-roots[-1], axis=1).max()),
                      minimum_tool_root_world_z_m=float(roots[:, 2].min()))
    except (KeyError, ValueError, TypeError) as error:
        result["error"] = str(error)
    return result


def audit_run(run, target_json):
    run, target_json = Path(run), Path(target_json)
    result = {"schema": "magnifier-photo-hold-audit-v2", "certified": False,
              "photo_hold_score": 0.0, "full_task_physical_certified": False,
              "reasons": [], "frames": [],
              "scope": "Exactly the final three seconds of image evidence, compared against the final frozen lens-free baseline. Physical pickup, grasp and collision checks remain separate."}
    try:
        record = json.loads((run/"pickup.json").read_text())
        trace = [json.loads(line) for line in (run/"trace.jsonl").read_text().splitlines() if line.strip()]
        target = json.loads(target_json.read_text())
        result["source_sha256"] = {"pickup_json": sha256(run/"pickup.json"),
                                    "trace": sha256(run/"trace.jsonl"), "target_json": sha256(target_json),
                                    "inspection_measure_py": sha256(Path(__file__).with_name("inspection_measure.py")),
                                    "score_hold_py": sha256(__file__)}
        comparison = record.get("optical_comparison", {})
        if record.get("complete") is not True:
            result["reasons"].append("Run record does not report successful completion")
        if comparison.get("same_camera_pose") is not True:
            result["reasons"].append("Run record does not certify a same-camera-pose reference")
        if not isinstance(comparison.get("same_physics_step"), int) or comparison["same_physics_step"] < 0:
            result["reasons"].append("Run record does not identify the frozen reference physics step")
        if comparison.get("actual_policy_resize_verified") is not True:
            result["reasons"].append("Run record lacks exact native-to-policy cv2 validation")
        final_step = record.get("steps")
        if not isinstance(final_step, int) or not trace or trace[-1].get("step") != final_step:
            raise ValueError("Run and trace final step do not agree")
        if record.get("final_state", {}).get("step") != final_step:
            result["reasons"].append("Run final_state does not match the final step")
        window, trace_info, trace_reasons = inspect_trace(trace, final_step)
        result["trace_interval"] = trace_info
        result["reasons"].extend(trace_reasons)
        if window:
            for field in ("camera_eye", "camera_rotation"):
                if not np.array_equal(record.get("final_state", {}).get(field), window[-1].get(field)):
                    result["reasons"].append("Recorded final camera differs from trace: " + field)
        steps = list(range(final_step-60, final_step+1))
        result["missing_frame_files"] = missing_frame_files(run, steps)
        if result["missing_frame_files"]:
            result["reasons"].append("Required native hold-frame PNGs are missing")
        result["physical_trace"] = physical_trace_summary(window, record) if window else {}
        if result["reasons"]:
            return result
        pairs = {}
        result["frozen_pair_checks"] = {}
        for label in ("baseline", "final"):
            with np.load(run/f"{label}.npz", allow_pickle=False) as arrays:
                pairs[label] = [load_image(run/f"{label}-{kind}.png") for kind in ("raw", "policy")]
                if any(not np.array_equal(arrays[kind], image) for kind, image in zip(("raw", "policy"), pairs[label])):
                    raise ValueError(f"{label} PNGs do not match the preserved camera arrays")
            resize = check_policy_resize(*pairs[label])
            result["frozen_pair_checks"][label] = {"png_matches_npz": True, "resize": resize,
                                                    "raw_sha256": sha256(run/f"{label}-raw.png"),
                                                    "policy_sha256": sha256(run/f"{label}-policy.png")}
            if not resize["passes"]:
                result["reasons"].append(f"{label} policy image is inconsistent with native camera resizing")
        if result["reasons"]:
            return result
        baseline_raw, baseline_policy = pairs["baseline"]
        for step in steps:
            frame = {"step": step, "time_s": step/20, "photo_score": 0.0, "valid": False}
            try:
                raw_path, policy_path = [run/f"hold-{step:04d}-{kind}.png" for kind in ("raw", "policy")]
                raw, policy = load_image(raw_path), load_image(policy_path)
                frame.update(raw_sha256=sha256(raw_path), policy_sha256=sha256(policy_path),
                             resize=check_policy_resize(raw, policy))
                if not frame["resize"]["passes"]:
                    raise ValueError("Actual policy image does not match the native camera resize")
                photo = compare_arrays(baseline_raw, raw, baseline_policy, policy, target)
                # An intact camera frame with an absent, occluded or unreadable
                # target is an observed task failure, not missing evidence.
                # File/resize/provenance errors still fail certification above.
                frame.update(valid=True,
                             optical_measurement_valid=bool(photo["valid"] and photo.get("registration", {}).get("valid", False)),
                             photo_score=photo["photo_score"],
                             minimum_edge_gain=photo.get("minimum_edge_gain_both_sizes"),
                             newly_readable_groups=photo.get("newly_readable_groups", []),
                             registration=photo.get("registration", {}),
                             candidate_policy_groups={group["name"]: {"readable": group["readable"],
                                                                      "scanlines": group["scanlines"]}
                                                      for group in photo["policy"]["candidate"].get("bar_groups", [])})
                if not frame["optical_measurement_valid"]:
                    frame["measurement_failure"] = photo.get("error") or photo.get("registration", {}).get("error") or "Target cannot be measured"
            except (OSError, ValueError, KeyError, TypeError) as error:
                frame["error"] = str(error)
            result["frames"].append(frame)
        invalid = [frame["step"] for frame in result["frames"] if not frame["valid"]]
        if invalid:
            result["invalid_frame_steps"] = invalid
            result["reasons"].append("One or more photo comparisons are invalid")
            return result
        gains = [frame["minimum_edge_gain"] for frame in result["frames"] if frame["minimum_edge_gain"] is not None]
        shifts = [frame["registration"]["maximum_shift_px"] for frame in result["frames"] if "maximum_shift_px" in frame["registration"]]
        result.update(certified=True, photo_hold_score=min(frame["photo_score"] for frame in result["frames"]),
                      minimum_edge_gain=min(gains) if gains else None,
                      maximum_registration_shift_px=max(shifts) if shifts else None,
                      unmeasurable_target_steps=[frame["step"] for frame in result["frames"] if not frame["optical_measurement_valid"]])
    except (OSError, ValueError, KeyError, TypeError, IndexError, json.JSONDecodeError) as error:
        result["reasons"].append(type(error).__name__ + ": " + str(error))
    return result


def self_test():
    import copy
    import tempfile
    trace = [{"step": i, "time_s": i/20, "camera_eye": [0, 0, 1],
              "camera_rotation": np.eye(3).tolist()} for i in range(61)]
    assert not inspect_trace(trace, 60)[2]
    assert inspect_trace(trace[:30]+trace[31:], 60)[2]
    bad = copy.deepcopy(trace); bad[20]["time_s"] += .001
    assert inspect_trace(bad, 60)[2]
    bad = copy.deepcopy(trace); bad[20]["camera_eye"][0] = .002
    assert not inspect_trace(bad, 60)[2] and not inspect_trace(bad, 60)[1]["camera_invariant"]
    bad = copy.deepcopy(trace); bad[20]["step"] = 19
    assert inspect_trace(bad, 60)[2]
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        for i in range(61):
            for kind in ("raw", "policy"):
                (root/f"hold-{i:04d}-{kind}.png").touch()
        assert not missing_frame_files(root, range(61))
        (root/"hold-0030-raw.png").unlink()
        assert missing_frame_files(root, range(61)) == ["hold-0030-raw.png"]
        failed = audit_run(root, root/"missing-target.json")
        assert not failed["certified"] and failed["photo_hold_score"] == 0 and failed["reasons"]
    return {"passed": True, "cases": 7,
            "scope": "Trace timing, missing states/files, camera-drift diagnostics and recoverable incomplete-artifact handling. Actual camera/target registration is gated by measured pixels in inspection_measure, not by exact pose equality."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--target-json", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        result = self_test()
    else:
        if not args.run or not args.target_json:
            parser.error("--run and --target-json are required")
        result = audit_run(args.run, args.target_json)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
