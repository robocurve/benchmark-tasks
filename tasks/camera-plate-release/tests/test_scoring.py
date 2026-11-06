"""Pure mathematical scorer tests; these do not replace native physics trials."""

from dataclasses import replace

import numpy as np
import pytest

from camera_release.config import MOUNT_POSITION, PAD_POSITION, PAD_TOP, PAYLOAD_POSITION
from camera_release.geometry import rotation_xyzw
from camera_release.scoring import Sample, Scorer


def measurement(**changes):
    sample = Sample(time=0, position=PAYLOAD_POSITION.copy(), quaternion=np.array([0., 0, 0, 1]),
                    velocity=np.zeros(6), cam_angle=0, stand_position=MOUNT_POSITION.copy(),
                    stand_quaternion=np.array([0., 0, 0, 1]), cam_anchor_error=0,
                    support_contact=True, robot_clearance=0.1)
    return replace(sample, **changes)


def placed(**changes):
    changes.setdefault("position", np.array([PAD_POSITION[0], PAD_POSITION[1], PAD_TOP]))
    return measurement(**changes)


def test_xyzw_identity_and_known_rotation():
    assert np.allclose(rotation_xyzw([0, 0, 0, 1]), np.eye(3))
    q = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
    assert np.allclose(rotation_xyzw(q) @ [0, 1, 0], [0, 0, 1])
    with pytest.raises(ValueError):
        rotation_xyzw([0, 0, 0, 0])


def test_initial_state_has_no_credit_and_released_cam_has_partial_credit():
    scorer = Scorer()
    assert scorer.evaluate(measurement())["score"] == 0
    assert scorer.evaluate(measurement(cam_angle=0.9))["score"] == 0.25


def test_placement_requires_a_full_second_of_release_and_stability():
    scorer = Scorer()
    for i in range(20):
        result = scorer.evaluate(placed(time=i/20))
        assert result["score"] == 0.75 and not result["success"]
    result = scorer.evaluate(placed(time=1.0))
    assert result["score"] == 1 and result["success"]


def test_touching_arm_cannot_receive_full_credit():
    scorer = Scorer()
    for i in range(40):
        result = scorer.evaluate(placed(time=i/20, robot_clearance=0.01))
    assert result["score"] == 0.75 and not result["success"]


def test_surface_containment_cannot_be_replaced_by_centre_only():
    scorer = Scorer()
    q = [0, 0, np.sin(np.pi/8), np.cos(np.pi/8)]
    result = scorer.evaluate(placed(position=np.array([PAD_POSITION[0], PAD_POSITION[1]+0.045, PAD_TOP]),
                                   quaternion=np.array(q)))
    assert not result["on_pad"]


def test_fixture_movement_permanently_invalidates_future_good_placement():
    scorer = Scorer()
    scorer.evaluate(measurement(stand_position=MOUNT_POSITION+np.array([0.006, 0, 0])))
    result = scorer.evaluate(placed(time=2))
    assert result["score"] == 0 and "fixture_moved" in result["violations"]


def test_drop_is_measured_across_physics_samples_and_latched():
    scorer = Scorer()
    initial = PAYLOAD_POSITION+np.array([0, 0, 0.1])
    scorer.observe_physics(initial, [0, 0, 0, 1], False)
    scorer.observe_physics(initial-[0, 0, 0.011], [0, 0, 0, 1], False)
    scorer.observe_physics([PAD_POSITION[0], PAD_POSITION[1], PAD_TOP], [0, 0, 0, 1], True)
    result = scorer.evaluate(placed(time=2))
    assert result["score"] == 0 and "drop" in result["violations"]
    scorer.reset()
    assert scorer.evaluate(placed())["score"] == 0.75


def test_workspace_escape_and_nonfinite_data_remain_invalid():
    scorer = Scorer()
    scorer.observe_physics([2, 0, 1], [0, 0, 0, 1], True)
    assert scorer.evaluate(placed())["score"] == 0
    scorer.reset()
    scorer.observe_physics([np.nan, 0, 1], [0, 0, 0, 1], False)
    assert scorer.evaluate(placed())["score"] == 0


def test_unstable_placement_cannot_complete():
    scorer = Scorer()
    for i in range(30):
        position = np.array([PAD_POSITION[0]+0.004*(i % 2), PAD_POSITION[1], PAD_TOP])
        result = scorer.evaluate(placed(time=i/20, position=position))
    assert result["score"] == 0.75 and not result["success"]
