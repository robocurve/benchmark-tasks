# SPDX-License-Identifier: MIT
"""Regrade a bevel angle transfer trial on a CPU.

Usage: python -m bevel_angle_transfer.regrade /path/to/trial-0000
Reconstruct the result from measured poses and pair contacts. Saved capture,
transfer, ink and guide acceptance flags are comparison evidence only.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .contact import DEFAULT_CONFIG, config_dict, source_manifest
from .scoring import TransferTracker


def reconstruct(records, config=DEFAULT_CONFIG, *, tolerance=1e-8):
    """Recompute score, physical ink and eligible marks from a complete sample clock."""
    tracker = TransferTracker(config)
    differences = []
    for record in records:
        sample = tracker.sample(record)
        for key in ("ink", "ink_segment_start", "guide_used", "capture_valid", "released", "transfer_valid"):
            if key in record and record[key] is not sample[key]:
                differences.append(f"sample {tracker.tick}: saved {key} differs")
        if "point_score" in record:
            actual, saved = sample["point_score"], record["point_score"]
            if actual is None or saved is None:
                same_point = actual is saved
            else:
                same_point = np.asarray(saved).shape == (2,) and np.allclose(actual, saved, atol=tolerance, rtol=0)
            if not same_point:
                differences.append(f"sample {tracker.tick}: saved ink coordinate differs")
    return dict(metrics=tracker.metrics(), strokes=tracker.strokes,
                target_start=tracker.target_start, target_end=tracker.target_end, differences=differences)


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
    if ink.get("schema") != "bevel-angle-transfer-contact-ink-v2":
        raise ValueError("This scorer requires v2 observations with measured gripper geometry; use the frozen v1 source for historical trials")
    saved = json.loads((folder/"metrics.json").read_text(encoding="utf-8"))
    with (folder/"contacts.jsonl").open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    if not records or any(record.get("trace_schema") != "bevel-visible-v2" for record in records):
        raise ValueError("Missing or incompatible bevel-visible-v2 measurement trace")
    result = reconstruct(records, tolerance=tolerance)
    differences = result["differences"]
    manifest_path = folder/"source_manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest != source_manifest():
            differences.append("recorded source manifest differs from this installed task version")
        if ink.get("environment_revision") != manifest.get("revision"):
            differences.append("ink environment revision differs from source manifest")
    else:
        differences.append("missing recorded source manifest")
    if json.dumps(ink.get("contact_config"), sort_keys=True) != json.dumps(config_dict(), sort_keys=True):
        differences.append("exported contact configuration differs from this task version")
    for name, actual in result["metrics"].items():
        if name not in saved or not _same(actual, saved[name], tolerance):
            differences.append(f"metric {name}: recomputed={actual!r}, saved={saved.get(name)!r}")
    for name in ("target_start", "target_end"):
        if name not in ink or not np.allclose(result[name], ink[name], atol=tolerance, rtol=0):
            differences.append(f"exported {name} differs from task target")
    if check_ink:
        for name in ("strokes",):
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

