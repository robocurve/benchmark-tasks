"""A seated bead can coexist with an inverted outlet. Use the collar's frame to tell them apart."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scoring import teat_fit
from specimen import specimen


@pytest.mark.parametrize('outlet_inverted', [False, True])
@pytest.mark.parametrize('angle_deg', [0, 100])
def test_outlet_independent_of_rim_and_world_pose(outlet_inverted, angle_deg):
    points, _, grid = specimen()
    points[:, 2] += .008
    if outlet_inverted:
        upper = grid[:, 9:, :].ravel()
        points[upper, 2] = .003 - abs(points[upper, 2] - .003)
    rotation = Rotation.from_euler('x', angle_deg, degrees=True)
    centre = np.array([.3, -.12, .82])
    collar = np.r_[centre, rotation.as_quat()]
    nodes = rotation.apply(points) + centre
    sectors, exposed = teat_fit(nodes[None], collar[None], grid)
    assert sectors.all()
    assert bool(exposed[0]) == (not outlet_inverted)
