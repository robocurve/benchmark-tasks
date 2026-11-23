"""Motion targets for the unchanged YAM geometry and physically colliding controls."""

import os
from native.fixture import BASES, SEAT, STOW, grip, handle

MODES = (
    "reference", "late_clear", "occupied_release", "supports_only",
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
        return (-0.43, 0.475 if s == "left" else -0.475, 0.83)

    for s in paths:
        add(s, 3.0, hover(s), op, "stage")
    if m == "probe_home":
        return paths
    for s in paths:
        if m not in ("reference", "alternate","late_clear","occupied_release","supports_only", "single_" + s, "probe_" + s):
            continue
        if m == "alternate" and s == "left":
            add(s, 2.4, hover(s), op, "stagger")
        x, y, z = grip(s, STOW)
        add(s, 2.0, (x, y, z + 0.12), op, "above prop")
        add(s, 1.65, (x, y, z), op, "approach prop")
        add(s, 1.4, (x, y, z), clamp, "pinch external faces")
        for a in range(-45, 37, 3):
            x,y,z=grip(s,a)
            p=(x,y,z+.055*min(1.,(a+45.)/65.))
            add(s,0.21,p,clamp,"fold prop")
        add(s, 1.4, p, clamp, "seat prop")
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
    if m in ("probe_left", "probe_right", "supports_only"):
        return paths
    clamp = -0.012
    x, y, z = handle(0)
    s = "right"
    add(s, 1.8, (x, y, z + 0.09), op, "above valve")
    add(s, 1.2, (x, y, z), op, "approach valve")
    add(s, 1.4, (x, y, z), clamp, "pinch valve")
    for a in range(4,33,4):
        add(s, 0.18, handle(a), clamp, "turn valve")
    p = handle(32)
    add(s, 4.0, p, clamp, "hold valve")
    add(s, 0.9, p, op, "release valve")
    add(s, 1.2, (p[0], p[1], p[2] + 0.08), op, "clear valve")
    add(s, 2.0, (-0.48, -0.49, 0.86), op, "park")
    return paths
