"""Motion targets for the unchanged YAM geometry and physically colliding controls."""

import os
from native.fixture_v21 import BASES, SEAT, grip, handle

MODES = (
    "reference",
    "alternate",
    "idle",
    "single_left",
    "single_right",
    "lever_only",
    "probe_home",
    "probe_left",
    "probe_right",
    "probe_valve",
)


def demo():
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_OPEN as op

    m = os.environ.get("TASK20_CONTROL", "reference")
    if m not in MODES:
        raise ValueError(m)
    paths = {s: [] for s in BASES}
    if m == "idle":
        return paths
    clamp = -0.0105  # Motor preload on a 34 mm elastomer-faced grip.

    def add(side, dt, p, f, label):
        paths[side].append(Waypoint(dt, p, f, label, 0.0))

    def hover(s):
        return (-0.35, 0.465 if s == "left" else -0.465, 0.864)

    for s in paths:
        add(s, 3.0, hover(s), op, "stage")
    if m == "probe_home":
        return paths
    for s in paths:
        if m not in ("reference", "alternate", "single_" + s, "probe_" + s):
            continue
        if m == "alternate" and s == "left":
            add(s, 2.4, hover(s), op, "stagger")
        x, y, z = grip(s, 90)
        add(s, 2.0, (x, y, z + 0.12), op, "above prop")
        add(s, 1.65, (x, y, z), op, "approach prop")
        add(s, 1.1, (x, y, z), clamp, "pinch external faces")
        for a in [*range(87, -1, -3), SEAT]:
            add(s, 0.21, grip(s, a), clamp, "fold prop")
        p = grip(s, SEAT)
        add(s, 0.7, p, clamp, "seat prop")
        add(s, 0.9, p, op, "release prop")
        x, y, z = p
        add(
            s,
            1.4,
            (x + 0.055, y + (0.010 if s == "left" else -0.010), z + 0.11),
            op,
            "clear prop",
        )
        add(s, 1.8, hover(s), op, "stage")
    if m in ("probe_left", "probe_right"):
        return paths
    clamp = -0.012
    x, y, z = handle(0)
    s = "right"
    add(s, 1.8, (x, y, z + 0.09), op, "above valve")
    add(s, 1.2, (x, y, z), op, "approach valve")
    add(s, 1.1, (x, y, z), clamp, "pinch valve")
    for a in range(5, 71, 5):
        add(s, 0.18, handle(a), clamp, "turn valve")
    p = handle(70)
    add(s, 0.8, p, clamp, "hold valve")
    add(s, 0.9, p, op, "release valve")
    add(s, 1.2, (p[0], p[1], p[2] + 0.08), op, "clear valve")
    add(s, 2.0, (-0.30, -0.31, 0.933), op, "park")
    return paths
