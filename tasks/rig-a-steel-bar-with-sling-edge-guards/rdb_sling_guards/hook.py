"""The double hook's shape (Carl Stahl CondorLift 1,000 kg eye double hook) in its own frame, and the scorer's test of
whether a fitting hangs in a horn. Pure Python.

Hook frame: origin midway between the two bowl centres; y across the hook (the left horn at +y), z up; the hook's plane
is y-z (the sling's plane). Each horn: its bowl (inside radius R) from the middle, under and round the outside, rising
to a tip; the latch closes the opening from a pivot on the eye's side down to the tip.
"""

import math

from . import layout as L

C = L.RAM_B / 2                         # bowl centres at y = +-C


def bowl_centre(side):
    """side 'L' (+y) or 'R' (-y)."""
    return (0., C if side == "L" else -C, 0.)


def hook_capsules():
    """[(name, (y0, z0), (y1, z1), radius)] in the hook's plane (x = 0)."""
    caps = []
    for side, s in (("L", 1.), ("R", -1.)):
        n = 10
        angs = [math.radians(180. + (180. + L.TIP_DEG) * k / n) for k in range(n + 1)]
        pts = [(s * (C + L.RC * math.cos(a)), L.RC * math.sin(a)) for a in angs]
        for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
            caps.append((f"horn{side}_{k}", a, b, L.HOOK_R))
    eye = [(-.012, -.004), (-.037, .060), (-.026, .084), (0., .088), (.026, .084), (.037, .060), (.012, -.004)]
    for k, (a, b) in enumerate(zip(eye, eye[1:] + eye[:1])):
        caps.append((f"eye_{k}", a, b, .0095))
    caps.append(("neck", (0., -.020), (0., .010), .012))
    return caps


def latch(side):
    """(pivot (y, z), free end (y, z)) of a horn's latch when closed."""
    s = 1. if side == "L" else -1.
    return (s * L.LATCH_PIVOT[0], L.LATCH_PIVOT[1]), (s * L.LATCH_END[0], L.LATCH_END[1])


def in_bowl(apex_yz, side):
    """A fitting's apex-bar centre (y, z in the hook frame) lies in ``side``'s bowl: within the bowl's inside radius
    less the bar's (plus 3 mm) of its centre, and below the latch's free end. (The scorer also needs the
    bar's centre within APEX_OFF_PLANE of the hook's plane: grading.py.)"""
    cy = C if side == "L" else -C
    dy, dz = apex_yz[0] - cy, apex_yz[1]
    return math.hypot(dy, dz) < L.BOWL_R_IN - L.FIT_BAR_R + .003 and dz < L.LATCH_END[1]
