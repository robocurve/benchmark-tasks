"""Uniform solid-convex volume integrals and additive compound inertia.

Returns geometric volume, COM, and COM inertia. Convex inertia is per unit
mass; compound inertia uses its supplied total mass and uniform density.
Overlapping compound solids are counted additively, never as a solid union.
"""
import numpy as np
from scipy.spatial import ConvexHull, QhullError


def convex_mass_properties(vertices):
    points = np.asarray(vertices, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 4
            or not np.isfinite(points).all()):
        raise ValueError('A finite three-dimensional convex solid is required')
    origin = points.mean(axis=0)
    shifted = points-origin
    try:
        hull = ConvexHull(shifted)
    except QhullError as error:
        raise ValueError('Degenerate convex solid') from error
    volume = 0.
    first = np.zeros(3)
    second = np.zeros((3, 3))
    for indices, plane in zip(hull.simplices, hull.equations):
        a, b, c = shifted[indices]
        if np.dot(np.cross(b-a, c-a), plane[:3]) < 0:
            b, c = c, b
        tetra_volume = float(np.dot(a, np.cross(b, c))/6.)
        if tetra_volume < -np.finfo(float).eps*hull.volume*64:
            raise ValueError('Convex facet orientation or interior origin failed')
        s = a+b+c
        volume += tetra_volume
        first += tetra_volume*s/4.
        second += tetra_volume*(np.outer(s, s)+np.outer(a, a)
                                 + np.outer(b, b)+np.outer(c, c))/20.
    if not np.isfinite(volume) or volume <= 0:
        raise ValueError('Positive finite solid volume is required')
    centre = first/volume
    covariance = second/volume-np.outer(centre, centre)
    inertia = np.trace(covariance)*np.eye(3)-covariance
    inertia = (inertia+inertia.T)/2.
    if not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
        raise ValueError('Invalid solid inertia')
    return volume, origin+centre, inertia


def compound_mass_properties(solids, mass):
    if not np.isfinite(mass) or mass <= 0:
        raise ValueError('Positive finite total mass is required')
    properties = [convex_mass_properties(solid) for solid in solids]
    if not properties:
        raise ValueError('At least one convex solid is required')
    volume = sum(v for v, _, _ in properties)
    centre = sum(v*c for v, c, _ in properties)/volume
    inertia = np.zeros((3, 3))
    for v, c, unit_inertia in properties:
        d = c-centre
        inertia += mass*v/volume*(unit_inertia+np.dot(d, d)*np.eye(3)-np.outer(d, d))
    return volume, centre, (inertia+inertia.T)/2.
