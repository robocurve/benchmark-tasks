# SPDX-License-Identifier: MIT
"""Regrade exported physical contacts on a CPU, without Isaac or yam-arena.

Usage: python -m square_cut_line.regrade /path/to/trial-0000
The observed force/point and tool root poses determine ink and guide use;
saved acceptance flags, stroke history and scores are comparison evidence only.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .contact import DEFAULT_CONFIG, config_dict, contact_gate, guide_metrics, lift_gate, advance_lift_hold, rotation_xyzw, source_manifest
from .scoring import score_strokes


def _target_local(board, config):
    mid = (np.asarray(config.TARGET_START)+np.asarray(config.TARGET_END))/2
    rotation = rotation_xyzw(board[3:])
    return tuple((rotation.T @ np.array([*(np.asarray(point)-mid), 0.0]))[:2].tolist()
                 for point in (config.TARGET_START, config.TARGET_END))


def reconstruct(records, config=DEFAULT_CONFIG, *, tolerance=1e-8):
    """Reconstruct ink and final grading from consecutive physics samples.

    Reject incomplete or inconsistent sample clocks rather than connecting
    unknown intervals. The trusted config comes from this version of the task,
    independently of any exported claims.
    """
    strokes, guided_strokes, stroke, guided = [], [], None, None
    mismatches = []
    last_ink_tick = None
    lift_hold_ticks = 0
    nib_count = guide_count = 0
    last = None
    for index, record in enumerate(records, start=1):
        tick, dt = record["tick"], float(record["dt"])
        if tick != index or not math.isfinite(dt) or not math.isclose(dt, config.PHYSICS_DT, abs_tol=1e-10, rel_tol=1e-8):
            raise ValueError(f"Sample {index}: expected consecutive ticks and {config.PHYSICS_DT} s physics dt")
        marker = np.asarray(record["marker_pose"], float)
        board = np.asarray(record["board_pose"], float)
        square = np.asarray(record["square_pose"], float)
        force = np.asarray(record["pair_force"], float)
        point = np.asarray(record["pair_point"], float)
        guide_force = np.asarray(record["guide_pair_force"], float)
        if guide_force.shape != (3,) or not np.isfinite(guide_force).all():
            raise ValueError(f"Sample {index}: invalid raw guide pair force")
        accepted, details = contact_gate(
            force, point, marker, board, nib_local=config.MARKER_NIB_LOCAL,
            nib_radius=config.MARKER_NIB_RADIUS, board_size=config.BOARD_SIZE)
        lift_hold_ticks = advance_lift_hold(lift_hold_ticks, accepted, marker, board, config)
        guide = guide_metrics(point, square, board, guide_force, config) if np.isfinite(point).all() else {"guide_used": False}
        if "ink" in record and record["ink"] != accepted:
            mismatches.append(f"sample {index}: saved ink acceptance differs")
        if "guide_used" in record and record["guide_used"] != guide["guide_used"]:
            mismatches.append(f"sample {index}: saved guide acceptance differs")
        last = (tick, dt, marker, board)
        if not accepted:
            stroke = guided = None
            continue
        rotation = rotation_xyzw(board[3:])
        mid_xy = (np.asarray(config.TARGET_START)+np.asarray(config.TARGET_END))/2
        mid_b = rotation.T @ (np.array([*mid_xy, config.BOARD_TOP])-board[:3])
        point_score = (np.asarray(details["point_board"])[:2]-mid_b[:2]).tolist()
        if "point_score" in record and not np.allclose(record["point_score"], point_score, atol=tolerance, rtol=0):
            mismatches.append(f"sample {index}: saved ink coordinate differs")
        if stroke is None or (stroke and np.linalg.norm(np.asarray(point_score)-stroke[-1]) > .01):
            stroke = []
            strokes.append(stroke)
            guided = None
        stroke.append(point_score)
        last_ink_tick = tick
        nib_count += 1
        guide_count += int(guide["guide_used"])
        if guide["guide_used"]:
            if guided is None:
                guided = []
                guided_strokes.append(guided)
            guided.append(point_score)
        else:
            guided = None
    board = last[3] if last is not None else np.array([*config.BOARD_CENTER, 0, 0, 0, 1])
    target_start, target_end = _target_local(board, config)
    metrics = score_strokes(strokes, target_start, target_end)
    guided_length = score_strokes(guided_strokes, target_start, target_end)["unique_ink_length"]
    fraction = min(1.0, guided_length/metrics["unique_ink_length"]) if metrics["unique_ink_length"] else 0.0
    tick = last[0] if last is not None else 0
    metrics.update(geometry_score=metrics["score"], unique_guided_ink_length=guided_length,
                   guide_used_fraction=fraction, physics_samples=tick,
                   geometry_evaluated_tick=tick, nib_contact_samples=nib_count, guide_used_samples=guide_count)
    if config.REQUIRE_GUIDE_USE:
        if fraction < .5:
            metrics["score"] = 0.0
        elif fraction < .9:
            metrics["score"] = min(.5, metrics["score"])
    quiet_s = (tick-last_ink_tick)*last[1] if last is not None and last_ink_tick is not None else 0.0
    lifted, lift = lift_gate(last[2], board, quiet_s,
                             nib_local=config.MARKER_NIB_LOCAL, nib_radius=config.MARKER_NIB_RADIUS,
                             board_size=config.BOARD_SIZE, clear_s=lift_hold_ticks*last[1]) if last is not None else (False, {})
    metrics.update(lift, marker_lift_complete=lifted)
    if not lifted:
        metrics["score"] = min(.5, metrics["score"])
    metrics["success"] = metrics["score"] >= 1.0
    return dict(metrics=metrics, strokes=strokes, guided_strokes=guided_strokes,
                target_start=target_start, target_end=target_end, differences=mismatches)


def _same(actual, saved, tolerance):
    if isinstance(actual, bool) or actual is None:
        return actual is saved
    if isinstance(actual, (float, int)):
        return (isinstance(saved, (float, int)) and math.isfinite(saved)
                and math.isclose(actual, saved, abs_tol=tolerance, rel_tol=1e-7))
    return actual == saved


def _same_strokes(actual, saved, tolerance):
    if not isinstance(saved, list) or len(actual) != len(saved):
        return False
    return all(np.asarray(a).shape == np.asarray(b).shape
               and np.allclose(a, b, atol=tolerance, rtol=0)
               for a, b in zip(actual, saved))


def regrade_trial(folder, *, check_ink=True, tolerance=1e-8):
    """Compare an independent final regrade with the trial's exported artifacts."""
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("Tolerance must be finite and nonnegative")
    folder = Path(folder)
    ink = json.loads((folder/"ink.json").read_text(encoding="utf-8"))
    saved = json.loads((folder/"metrics.json").read_text(encoding="utf-8"))
    with (folder/"contacts.jsonl").open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    result = reconstruct(records, tolerance=tolerance)
    differences = result["differences"]
    manifest_path = folder/"source_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest != source_manifest():
            differences.append("recorded source manifest differs from this installed task version")
        if ink.get("environment_revision") != manifest.get("revision"):
            differences.append("ink environment revision differs from source manifest")
    if "contact_config" in ink and json.dumps(ink["contact_config"], sort_keys=True) != json.dumps(config_dict(), sort_keys=True):
        differences.append("exported contact configuration differs from this task version")
    for name, actual in result["metrics"].items():
        if name not in saved or not _same(actual, saved[name], tolerance):
            differences.append(f"metric {name}: recomputed={actual!r}, saved={saved.get(name)!r}")
    for name in ("target_start", "target_end"):
        if name not in ink or not np.allclose(result[name], ink[name], atol=tolerance, rtol=0):
            differences.append(f"exported {name} differs from task target")
    if check_ink:
        for name in ("strokes", "guided_strokes"):
            if not _same_strokes(result[name], ink.get(name), tolerance):
                differences.append(f"exported {name} differs from reconstructed contacts")
    return dict(match=not differences, metrics=result["metrics"], differences=differences,
                checked_ink_history=check_ink, physics_samples=len(records))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_folder", type=Path)
    parser.add_argument("--skip-ink-check", action="store_true", help="Compare metrics only; still reconstruct contacts independently")
    parser.add_argument("--tolerance", type=float, default=1e-8)
    args = parser.parse_args(argv)
    try:
        report = regrade_trial(args.trial_folder, check_ink=not args.skip_ink_check, tolerance=args.tolerance)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps(dict(match=False, error=str(error))))
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
