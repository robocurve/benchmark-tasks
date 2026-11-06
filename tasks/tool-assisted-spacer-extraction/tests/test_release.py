"""Pure release-proof checks around the declared gripper and clearance limits."""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from spacer_extraction import evaluation_io


@pytest.fixture
def release_fixture(monkeypatch):
    objects = {
        "pusher": SimpleNamespace(low=np.array([0.0, 0.0, 0.0]),
                                  high=np.array([0.2, 0.2, 0.2])),
        "spacer": SimpleNamespace(low=np.array([1.0, 0.0, 0.0]),
                                  high=np.array([1.2, 0.2, 0.2])),
    }
    joints = np.zeros(14)
    joints[6] = joints[13] = 1.0
    bases = {"left": [0.0, 0.305, 0.75], "right": [0.0, -0.305, 0.75]}
    tcps = {"left": np.array([-0.1, 0.1, 0.1]), "right": np.array([1.3, 0.1, 0.1])}

    def tcp_pose(arm_joints, base):
        assert np.asarray(arm_joints).shape == (6,)
        return tcps["left" if base[1] > 0 else "right"], np.eye(3)

    monkeypatch.setattr(evaluation_io.ik, "tcp_pose", tcp_pose)
    return objects, joints, bases, tcps


@pytest.mark.parametrize("gripper_index", [6, 13])
@pytest.mark.parametrize("margin,accepted", [(-1e-6, False), (1e-6, True)])
def test_each_gripper_must_cross_eighty_percent_open(release_fixture, gripper_index, margin, accepted):
    objects, joints, bases, _ = release_fixture
    joints[gripper_index] = 0.8 + margin
    result = evaluation_io.release_state(objects, joints, bases)
    assert result["released"] is accepted


@pytest.mark.parametrize("arm", ["left", "right"])
@pytest.mark.parametrize("object_name", ["pusher", "spacer"])
@pytest.mark.parametrize("margin,accepted", [(-1e-6, False), (1e-6, True)])
def test_each_tcp_must_clear_each_complete_object_bound(
    release_fixture, arm, object_name, margin, accepted
):
    objects, joints, bases, tcps = release_fixture
    # Place one TCP just outside the complete selected object's lower x face.
    tcps[arm] = np.array([objects[object_name].low[0] - (0.08 + margin), 0.1, 0.1])
    result = evaluation_io.release_state(objects, joints, bases)
    assert result["released"] is accepted
    clearance_index = 0 if arm == "left" else 1
    assert result["clearances_m"][clearance_index] == pytest.approx(0.08 + margin)


def test_tcp_inside_object_bounds_is_not_released(release_fixture):
    objects, joints, bases, tcps = release_fixture
    tcps["left"] = np.array([0.1, 0.1, 0.1])
    result = evaluation_io.release_state(objects, joints, bases)
    assert not result["released"]
    assert result["clearances_m"][0] == 0.0


@pytest.mark.parametrize("joints", [np.zeros(13), np.zeros((2, 7)), np.full(14, np.nan), np.full(14, np.inf)])
def test_malformed_joint_measurements_are_rejected(release_fixture, joints):
    objects, _, bases, _ = release_fixture
    with pytest.raises(ValueError, match="14 finite measured joint values"):
        evaluation_io.release_state(objects, joints, bases)


@pytest.mark.parametrize("field,value", [
    ("low", np.zeros(2)),
    ("high", np.array([0.2, np.nan, 0.2])),
    ("low", np.array([0.3, 0.0, 0.0])),
])
def test_malformed_release_bounds_are_rejected(release_fixture, field, value):
    objects, joints, bases, _ = release_fixture
    setattr(objects["pusher"], field, value)
    with pytest.raises(ValueError, match="invalid object bounds"):
        evaluation_io.release_state(objects, joints, bases)


@pytest.mark.parametrize("base", [[0.0, 0.3], [0.0, np.nan, 0.75]])
def test_malformed_robot_base_is_rejected(release_fixture, base):
    objects, joints, bases, _ = release_fixture
    bases["left"] = base
    with pytest.raises(ValueError, match="invalid robot base position"):
        evaluation_io.release_state(objects, joints, bases)
