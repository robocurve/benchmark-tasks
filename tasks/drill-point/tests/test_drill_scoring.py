"""Drill scoring geometry and success gates, without starting Isaac Sim."""

import json
import sys
from dataclasses import replace
from itertools import product
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from inspect_robots.types import Observation
from yam_arena import ik
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.scoring import task_score
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import axis_angle_matrix

TASK_PATH = Path(__file__).resolve().parents[1] / "drill_task.py"


@pytest.fixture
def task():
    return load(TASK_PATH)


def drill_state(task, *, tilt=0.0, clearance=0.045, miss=0.0, speed=0.0, spin=0.0):
    """Pose the real support geometry at a known clearance and forward-ray miss distance."""
    m = task.module
    down = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -np.pi / 2)
    rotation = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), np.radians(tilt)) @ down
    local = m._DRILL_POINTS_LOCAL
    pos_z = m.TOP + clearance - (local @ rotation.T)[:, 2].min()
    tip_offset = rotation @ m._TIP_OFFSET_LOCAL
    axis = rotation @ m.AXIS_LOCAL
    travel = (m.TOP - pos_z - tip_offset[2]) / axis[2]
    pos_xy = m.TARGET_XY + np.array([miss, 0.0]) - tip_offset[:2] - travel * axis[:2]
    pos = np.array([*pos_xy, pos_z])
    corners = np.array(list(product(*zip(local.min(0), local.max(0)))))
    world_corners = corners @ rotation.T + pos
    return ObjectState(
        pos=pos,
        quat_xyzw=ik.matrix_to_quat_xyzw(rotation),
        lin_vel=np.array([speed, 0.0, 0.0]),
        ang_vel=np.array([spin, 0.0, 0.0]),
        low=world_corners.min(0),
        high=world_corners.max(0),
    )


def test_empty_box_corner_does_not_erase_a_valid_lift(task):
    drill = drill_state(task, tilt=-12.0)
    measured = task.module._measure(drill)
    # The old box gate failed even though the actual drill clears the table by 4.5 cm.
    assert drill.low[2] <= task.table_top + 0.02
    assert measured["clearance_m"] == pytest.approx(0.045)
    assert measured["tilt_deg"] == pytest.approx(12.0)
    assert task.score({"drill": drill}) == 0.75
    assert task.success({"drill": drill})


@pytest.mark.parametrize(
    "clearance, tilt, miss, speed, spin, partial, success",
    [
        (0.010, 0.0, 0.0, 0.0, 0.0, 0.0, False),
        (0.045, 16.0, 0.0, 0.0, 0.0, 0.25, False),
        (0.045, 0.0, 0.060, 0.0, 0.0, 0.50, False),
        (0.045, 0.0, 0.030, 0.0, 0.0, 0.75, True),
        (0.045, 0.0, 0.039, 0.1, 1.0, 0.75, True),
        (0.045, 0.0, 0.041, 0.0, 0.0, 0.75, False),
        (0.045, 0.0, 0.0, 0.011, 0.0, 0.75, True),
        (0.045, 0.0, 0.0, 0.0, 0.51, 0.75, True),
        (0.045, 0.0, 0.0, 0.0, 0.0, 0.75, True),
    ],
)
def test_score_uses_four_cm_aim_tolerance_without_movement_limits(
    task, clearance, tilt, miss, speed, spin, partial, success
):
    drill = drill_state(task, tilt=tilt, clearance=clearance, miss=miss, speed=speed, spin=spin)
    assert task.score({"drill": drill}) == partial
    assert task.success({"drill": drill}) is success
    assert task.distance({"drill": drill}) == pytest.approx(miss)


def test_actual_table_contact_still_scores_zero(task):
    drill = drill_state(task, clearance=0.0)
    assert task.score({"drill": drill}) == 0.0
    assert not task.success({"drill": drill})


def test_quaternion_normalization_does_not_change_the_score(task):
    drill = drill_state(task, tilt=-12.0)
    scaled = replace(drill, quat_xyzw=-2.0 * drill.quat_xyzw)
    assert task.module._measure(scaled)["clearance_m"] == pytest.approx(0.045)
    assert task.success({"drill": scaled})


@pytest.mark.parametrize("quaternion", [(0.0, 0.0, 0.0, 0.0), (np.nan, 0.0, 0.0, 1.0)])
def test_invalid_pose_does_not_silently_earn_credit(task, quaternion):
    drill = replace(drill_state(task), quat_xyzw=np.array(quaternion))
    with pytest.raises(ValueError, match="quat_xyzw"):
        task.score({"drill": drill})


def test_diagnostics_record_each_gate_and_the_old_box_measurement(task, tmp_path, monkeypatch, capsys):
    path = tmp_path / "scoring.jsonl"
    monkeypatch.setenv("DRILL_SCORING_LOG", str(path))
    task.score({"drill": drill_state(task, tilt=-12.0)})
    row = json.loads(path.read_text())
    assert row["clearance_m"] == pytest.approx(0.045)
    assert row["bbox_clearance_m"] < 0.020
    assert row["partial_score"] == 0.75
    assert row["success_now"] is True
    assert "clearance=" in capsys.readouterr().out


def test_full_credit_still_requires_a_continuous_second(task, monkeypatch):
    state = {"drill": drill_state(task, tilt=-12.0, miss=0.030, speed=0.05, spin=0.6)}
    monkeypatch.setitem(sys.modules, "yam_arena.scene", SimpleNamespace(object_states=lambda *_: state))
    emb = YamArenaEmbodiment(task=str(TASK_PATH), control_hz=20)
    monkeypatch.setattr(emb, "_observation", lambda: Observation())
    for _ in range(19):
        assert not emb._task_step().terminated
    on_target = state["drill"]
    state["drill"] = drill_state(task, tilt=-12.0, miss=0.045, speed=0.05, spin=0.6)
    assert not emb._task_step().terminated
    state["drill"] = on_target
    for _ in range(19):
        assert not emb._task_step().terminated
    result = emb._task_step()
    assert result.terminated and result.termination_reason == "success"
    record = SimpleNamespace(steps=[SimpleNamespace(result=result)])
    assert task_score()(record, None).value == 1.0
