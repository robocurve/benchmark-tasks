"""Independently replay the raw object/joint trace through the task evaluator."""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState

from .evaluation_io import RUN_LIMIT_STEPS, evaluator_fingerprint, release_state
from .task import SpacerTrialTracker
from .tracker import PUSHER_START, SPACER_START


def rescore(path: Path) -> dict:
    lines = path.read_text(encoding="utf-8").splitlines()
    records = [json.loads(line) for line in lines]
    if len(records) < 3 or records[0].get("type") != "header" or records[-1].get("type") != "end":
        raise ValueError("trace must contain a header, frames, and a completed end record")
    header = records[0]
    if header.get("schema") != 1 or header.get("control_hz") != 20:
        raise ValueError("unsupported trace schema/control rate")
    if header.get("run_limit_steps") != RUN_LIMIT_STEPS:
        raise ValueError("recorded run limit differs from the frozen task limit")
    if header.get("bases") != {"left": [0.0, 0.305, 0.75], "right": [0.0, -0.305, 0.75]}:
        raise ValueError("trace robot bases differ from the calibrated station")
    if header.get("task_sha256") != hashlib.sha256(Path(__file__).with_name("task.py").read_bytes()).hexdigest():
        raise ValueError("task file differs from the recorded task")
    if header.get("evaluator_sha256") != evaluator_fingerprint():
        raise ValueError("evaluator differs from the recorded evaluator")
    end = records[-1]
    if not end.get("complete") or end.get("reason") not in ("success", "run_limit"):
        raise ValueError("attempt was interrupted or truncated before completion")
    if type(end.get("steps")) is not int or not 0 <= end["steps"] <= RUN_LIMIT_STEPS:
        raise ValueError("invalid completed attempt step count")
    tracker = SpacerTrialTracker()
    previous = None
    comparisons = 0
    snapshot = None
    for record in records[1:-1]:
        if record.get("type") != "frame":
            raise ValueError("unexpected trace record")
        time_s = float(record["time_s"])
        if not math.isfinite(time_s) or time_s < 0:
            raise ValueError("invalid timestamp")
        if previous is None:
            if abs(time_s) > 1e-8:
                raise ValueError("trace must begin at reset time zero")
        elif not math.isclose(time_s - previous, 0.05, abs_tol=1e-7):
            raise ValueError("missing, duplicate, or nonmonotonic trace frame")
        previous = time_s
        objects = {}
        for name, data in record["objects"].items():
            fields = {key: np.asarray(data[key], dtype=float)
                      for key in ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")}
            for key, array in fields.items():
                if array.shape != ((4,) if key == "quat_xyzw" else (3,)) or not np.all(np.isfinite(array)):
                    raise ValueError(f"invalid {name}.{key}")
            objects[name] = ObjectState(**fields)
        if comparisons == 0:
            for name, expected in (("pusher", PUSHER_START), ("spacer", SPACER_START)):
                if np.linalg.norm(objects[name].pos - expected) > 0.001:
                    raise ValueError("trace does not begin from the declared fixed reset")
        release = release_state(objects, record["joint_pos"], header["bases"])
        snapshot = tracker.update(objects, time_s=time_s, released=release["released"])
        actual = dataclasses.asdict(snapshot)
        if actual != record["evaluation"]:
            raise ValueError(f"scorer/trace disagreement at {time_s:.3f}s")
        comparisons += 1
    if snapshot is None or not math.isclose(records[-1]["time_s"], previous, abs_tol=1e-7):
        raise ValueError("invalid end timestamp")
    if end.get("steps") != comparisons - 1:
        raise ValueError("end step count does not match the frames")
    if end["reason"] == "success" and not snapshot.success_ready:
        raise ValueError("invalid success termination")
    if end["reason"] == "run_limit" and end["steps"] != header.get("run_limit_steps"):
        raise ValueError("attempt did not reach its run limit")
    return {"frames": comparisons, "duration_s": previous, "score": snapshot.score,
            "success": snapshot.success_ready, "trace_agrees": True}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    print(json.dumps(rescore(args.trace), indent=2))


if __name__ == "__main__":
    main()
