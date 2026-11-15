"""Ordinary axial wrench withdrawal from a measured, latched tool pose.

Only the first observation selects the path. No object state is written. The
contact episode must independently prove that the adjustment survives release.
"""
import numpy as np


def axial_target(position, orientation, distance):
    position = np.asarray(position, float)
    orientation = np.asarray(orientation, float)
    if (position.shape != (3,) or orientation.shape != (3, 3)
            or not np.isfinite(position).all() or not np.isfinite(orientation).all()
            or not np.isfinite(distance) or not 0. <= distance <= .05
            or not np.allclose(orientation.T @ orientation, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(orientation), 1., atol=1e-6)):
        raise ValueError('Finite rigid tool pose and bounded withdrawal required')
    # The wrench mouth opens along local +Y. Moving the tool along -Y
    # therefore frees a stationary nipple without a commanded twist.
    return position - distance * orientation[:, 1], orientation.copy()
