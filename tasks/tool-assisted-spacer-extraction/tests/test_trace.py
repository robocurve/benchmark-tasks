from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from yam_arena.task_file import ObjectState
from yam_arena.constants import READY

from spacer_extraction.rescore import rescore
from spacer_extraction.evaluation_io import evaluator_fingerprint, release_state
from spacer_extraction.tracker import NEST, PUSHER_START, PUSHER_HALF_X, SPACER_START, TOP, SpacerTrialTracker

SPACER_X = float(SPACER_START[0])
TOOL_X = SPACER_X - 0.014 - PUSHER_HALF_X - 0.002


def _header() -> dict:
    task_file = Path(__file__).parents[1] / "src" / "spacer_extraction" / "task.py"
    return {
        "type": "header",
        "schema": 1,
        "control_hz": 20,
        "task_sha256": hashlib.sha256(task_file.read_bytes()).hexdigest(),
        "evaluator_sha256": evaluator_fingerprint(),
        "run_limit_steps": 2400,
        "bases": {"left": [0.0, 0.305, 0.75], "right": [0.0, -0.305, 0.75]},
    }


def _write(path, records):
    path.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")


def test_rescore_rejects_trace_without_frame_records(tmp_path):
    path = tmp_path / "empty.jsonl"
    _write(path, [_header(), {"type": "end", "complete": True, "reason": "run_limit"}])
    with pytest.raises(ValueError, match="header, frames, and a completed end"):
        rescore(path)


def test_rescore_rejects_truncated_or_interrupted_attempt(tmp_path):
    path = tmp_path / "interrupted.jsonl"
    _write(
        path,
        [
            _header(),
            {"type": "frame", "time_s": 0.0},
            {"type": "end", "time_s": 0.0, "steps": 0, "reason": "incomplete", "complete": False},
        ],
    )
    with pytest.raises(ValueError, match="interrupted or truncated"):
        rescore(path)


def test_rescore_rejects_frame_that_does_not_start_at_reset_time_zero(tmp_path):
    path = tmp_path / "bad-start.jsonl"
    _write(
        path,
        [
            _header(),
            {"type": "frame", "time_s": 0.05},
            {"type": "end", "time_s": 0.05, "steps": 0, "reason": "run_limit", "complete": True},
        ],
    )
    with pytest.raises(ValueError, match="begin at reset time zero"):
        rescore(path)


def test_rescore_rejects_invalid_timestamp_before_reading_frame_payload(tmp_path):
    path = tmp_path / "nan-time.jsonl"
    _write(
        path,
        [
            _header(),
            {"type": "frame", "time_s": float("nan")},
            {"type": "end", "time_s": 0.0, "steps": 0, "reason": "run_limit", "complete": True},
        ],
    )
    with pytest.raises(ValueError, match="invalid timestamp"):
        rescore(path)


def test_rescore_rejects_shortened_run_limit(tmp_path):
    path = tmp_path / "short-limit.jsonl"
    header = _header()
    header["run_limit_steps"] = 1
    _write(path, [header, {"type": "frame", "time_s": 0.0},
                  {"type": "end", "complete": True, "reason": "run_limit", "steps": 1}])
    with pytest.raises(ValueError, match="frozen task limit"):
        rescore(path)


def test_rescore_rejects_success_after_run_limit(tmp_path):
    path = tmp_path / "late-success.jsonl"
    _write(path, [_header(), {"type": "frame", "time_s": 0.0},
                  {"type": "end", "complete": True, "reason": "success", "steps": 2401}])
    with pytest.raises(ValueError, match="step count"):
        rescore(path)


def test_rescore_replays_a_complete_success_trace(tmp_path):
    path = tmp_path / "successful.jsonl"
    header = _header()
    bases = header["bases"]
    joints = [*READY, 1.0, *READY, 1.0]
    tracker = SpacerTrialTracker()
    records = [header]

    def make_objects(tool_pos, spacer_pos):
        result = {}
        for name, pos, half in (
            ("pusher", tool_pos, (PUSHER_HALF_X, 0.008, 0.008)),
            ("spacer", spacer_pos, (0.014, 0.012, 0.012)),
        ):
            center = np.asarray(pos, dtype=float)
            extent = np.asarray(half, dtype=float)
            result[name] = ObjectState(
                pos=center,
                quat_xyzw=np.array((0.0, 0.0, 0.0, 1.0)),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
                low=center - extent,
                high=center + extent,
            )
        return result

    def append_frame(time_s, tool_pos, spacer_pos):
        objects = make_objects(tool_pos, spacer_pos)
        release = release_state(objects, joints, bases)
        snapshot = tracker.update(objects, time_s=time_s, released=release["released"])
        records.append({
            "type": "frame",
            "time_s": time_s,
            "objects": {
                name: {
                    "pos": state.pos.tolist(),
                    "quat_xyzw": state.quat_xyzw.tolist(),
                    "lin_vel": state.lin_vel.tolist(),
                    "ang_vel": state.ang_vel.tolist(),
                    "low": state.low.tolist(),
                    "high": state.high.tolist(),
                }
                for name, state in objects.items()
            },
            "joint_pos": joints,
            "release": release,
            "evaluation": vars(snapshot),
        })
        return snapshot

    append_frame(0.0, PUSHER_START, SPACER_START)
    append_frame(0.05, (TOOL_X, 0.100, TOP + 0.008), (SPACER_X, 0.100, TOP + 0.012))
    for step in range(1, 42):
        delta = step * 0.0011
        append_frame((step + 1) * 0.05, (TOOL_X + delta, 0.100, TOP + 0.008),
                     (SPACER_X + delta, 0.100, TOP + 0.012))
    append_frame(2.15, PUSHER_START, NEST)
    final = None
    for step in range(1, 21):
        final = append_frame(2.15 + step * 0.05, PUSHER_START, NEST)
    assert final is not None and final.score == 1.0
    records.append({"type": "end", "time_s": 3.15, "steps": len(records) - 2,
                    "reason": "success", "complete": True})
    _write(path, records)

    result = rescore(path)
    assert result["trace_agrees"]
    assert result["success"]
    assert result["score"] == 1.0
