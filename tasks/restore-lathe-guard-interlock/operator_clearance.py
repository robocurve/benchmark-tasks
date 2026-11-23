"""Conservative surface clearance from the fixed person to the swept machine.

All coordinates are fixture-local metres. A bounding-box gap is a lower
bound on the distance between the enclosed solids. Revolute joints use a
complete revolution, which also bounds motion between sampled guard angles.
"""
from itertools import product
from math import hypot, sqrt

import fixture


def bounds(center, size):
    return tuple((c - s / 2, c + s / 2) for c, s in zip(center, size))


def gap(a, b):
    return sqrt(sum(max(0., low_a - high_b, low_b - high_a) ** 2
                    for (low_a, high_a), (low_b, high_b) in zip(a, b)))


def sweep(shape, bodies):
    _, owner, center, size, _, _ = shape
    _, parent, origin, joint, kind, axis, limits, *_ = bodies[owner]
    if parent not in (None, "frame"):
        raise ValueError("Update the sweep for a nested fixture articulation")
    box = list(bounds(center, size))
    if joint:
        if axis not in ((1,0,0), (0,1,0), (0,0,1)):
            raise ValueError("Expected a positive coordinate axis")
        k = axis.index(1)
        if kind == "hinge":
            u, v = [i for i in range(3) if i != k]
            radius = max(hypot(x,y) for x,y in product(box[u],box[v]))
            box[u] = box[v] = (-radius, radius)
        elif kind == "slide":
            box[k] = (box[k][0] + limits[0], box[k][1] + limits[1])
        else:
            raise ValueError("Unsupported fixture joint")
    return tuple((lo + offset, hi + offset) for (lo,hi), offset in zip(box,origin))


def measure():
    bodies = {b[0]: b for b in fixture.BODIES}
    person = {}
    machine = {}
    for shape in fixture.SHAPES:
        name, owner, center, size, _, kind = shape
        if name.startswith("operator_"):
            if owner != "frame":
                raise ValueError("Expected a stationary operator on the fixture frame")
            person[name] = bounds(center, size)
        elif not kind.endswith("_visual") and name != "palm_rest":
            machine[name] = sweep(shape, bodies)
    # Bound the free insert at reset across all permitted pad heights. Dynamic
    # motion of a grasped or dropped insert is monitored by the contact reader.
    insert = list(bounds(fixture.KEY, fixture.KEY_SIZE))
    insert[2] = (insert[2][0] + bodies["testpad"][6][0], insert[2][1])
    machine["seated_insert"] = tuple(insert)
    nearest = {}
    for name, box in machine.items():
        distance, part = min((gap(box, p), part) for part, p in person.items())
        nearest[name] = {"operator_part": part, "lower_bound_mm": 1000 * distance}
    closest = min(nearest, key=lambda n: nearest[n]["lower_bound_mm"])
    return {"closest_machine_part": closest, "lower_bound_mm": nearest[closest]["lower_bound_mm"],
            "by_machine_part": nearest}
