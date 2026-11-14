"""Measured rack endpoint, independent of the syringe's axial direction."""
import math


def is_parked(position, axis, velocity, origin, tabletop):
    """Accept either end-for-end direction with the same pose/speed tolerances."""
    if not all(math.isfinite(v) for values in (position, axis, velocity, origin)
               for v in values) or not math.isfinite(tabletop):
        return False
    return (math.hypot(position[0] - origin[0], position[1] - origin[1]) < .025
            and abs(axis[2]) < .15 and abs(axis[1]) > .985
            and tabletop + .125 < position[2] < tabletop + .155
            and math.sqrt(sum(v * v for v in velocity)) < .025)
