"""Mesh-supported contact targets for an ordinary table turnover.

Planning only: no object pose, force, velocity or success state is written.
Initial candidate is a 180mm-high carton at (250,-100)mm. Physical feasibility
and every approach/withdrawal still require measured native validation.
"""
import math
import numpy as np


def fixed_wrist(side):
    """World-fixed 30degree downward tool axis, 90degree wrist polarity."""
    sign = -1. if side == 'left' else 1.
    z = np.array([0., sign*math.cos(math.pi/6), -.5])
    x = np.array([1., 0., 0.])
    base = np.column_stack((x, np.cross(z, x), z))
    return base @ np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


def tip_vertices(geometry, fingers=(.012, .012)):
    """Use the unchanged actual collision meshes at the supplied jaw state."""
    return np.concatenate([tri.reshape(-1, 3)
                           for _, name, tri in geometry.local_hand(fingers)
                           if name.startswith('tip_')])


def support_target(contact, inward_normal, wrist, vertices, press=.0002):
    """Align an actual tip support point with a specified physical plane.

    Near-coplanar support vertices are averaged within 50um. This operation
    can change selected facets as the carton rotates, so issued commands need
    existing speed/slew/clearance guards and contact tracking.
    """
    n = np.asarray(inward_normal, dtype=float)
    n /= np.linalg.norm(n)
    world = np.asarray(vertices) @ wrist.T
    extent = world @ n
    support = world[extent >= extent.max()-.00005].mean(axis=0)
    return np.asarray(contact)-support+n*press, wrist.copy()


def wall_target(origin, rotation, side, height, width, vertices, press=.0002):
    """Low single-wall brace and high folded-double-wall push contact."""
    left = side == 'left'
    sign = -1. if left else 1.
    local = np.array([0., -sign*(width/2+(.0015 if left else .0045)),
                      (.2 if left else .9)*height])
    return support_target(np.asarray(origin)+rotation@local,
                          rotation@np.array([0., sign, 0.]),
                          fixed_wrist(side), vertices, press)
