import numpy as np
import pytest

from bevel_angle_transfer.hand_geometry import box_clearance, finger_clearances, finger_envelopes


def test_surface_clearance_not_distance_between_origins():
    identity = np.eye(3)
    assert box_clearance(np.zeros(3), identity, np.ones(3), np.array([2.01, 0., 0.]), identity, np.ones(3)) == pytest.approx(.01)
    assert box_clearance(np.zeros(3), identity, np.ones(3), np.array([1., 0., 0.]), identity, np.ones(3)) == 0.


def test_jaw_envelopes_include_model_offsets():
    left, right = finger_envelopes()
    assert left.shape[1] == right.shape[1] == 6
    assert len(left) > 1 and len(right) > 1
    assert np.min(left[:, 2] - left[:, 5]) < -.09
    identity = np.eye(3)
    values = [[0., 0., 0., 0., 0., 0., 1.], [1., 0., 0., 0., 0., 0., 1.]]
    box = left[0]
    gaps = finger_clearances(values, (np.zeros(3), identity), [(box[:3], box[3:])])
    assert gaps[0] == 0.
    assert gaps[1] > .8


def test_missing_measured_jaw_geometry_fails_closed():
    with pytest.raises(ValueError):
        finger_clearances(None, (np.zeros(3), np.eye(3)), [([0., 0., 0.], [.01, .01, .01])])
