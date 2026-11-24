"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object:
ready state}, expected score, expected success). Offline there is no robot, so the arms are set here as measured
clear; in the simulator they are measured every step and are None (no credit) until then.
"""

import math

import parts
from task import (BACK_T, CAL_XY, CALIPER, FACE0, LEAD, PAD_IN, PAD_OUT, PROUD, TOOL, TOP, Z_IN, Z_OUT)

grip = parts.__dict__.setdefault("_caliper_task_grip", {})
grip["clear"] = True

CX, CY = CAL_XY


def quat(axis, deg):
    a = math.radians(deg) / 2
    s = math.sin(a)
    return tuple([s * v for v in axis] + [math.cos(a)])


def cal(back=PROUD, vel=None):
    """The piston face ``back`` m lower than at the start, wound back by its thread."""
    turns = back / LEAD
    return {"caliper": CALIPER.state(lin_vel=vel, joints={"lift": -back, "feed": -back, "nut": -2 * math.pi * turns,
                                                          "piston": -2 * math.pi * turns})}


def pad(spec, dx=0.0, dy=0.0, dz=0.0, q=None, vel=None, home=True):
    z = TOP + (Z_IN if spec is PAD_IN else Z_OUT + BACK_T)
    x = CX if home else spec.pos[0]
    return {spec.name: spec.state(pos=(x + dx, CY + dy, z + dz), quat_xyzw=q, lin_vel=vel)}


BACK = cal()
BOTH = {**BACK, **pad(PAD_IN), **pad(PAD_OUT)}
ON_FACE = (CX, CY, TOP + FACE0 - PROUD)  # tool adapter resting on the wound-back face

CASES = [
    ("start: piston proud, pads outside", {}, 0.0, False),
    ("outer pad home, piston not wound back", pad(PAD_OUT), 0.0, False),
    (f"piston pushed 1 mm only (face {round((PROUD - 0.001) * 1000)} mm proud)", cal(back=0.001), 0.0, False),
    ("piston 0.1 mm proud of the ledges", cal(back=PROUD - 0.0001), 0.0, False),
    ("piston exactly at the ledges", cal(back=PROUD - 0.0), 0.5, False),
    ("piston wound back, no pads", BACK, 0.5, False),
    (f"piston wound fully back ({round((0.012 - PROUD) * 1000)} mm below the ledges)", cal(back=0.012), 0.5, False),
    ("piston back, outer pad home", {**BACK, **pad(PAD_OUT)}, 0.75, False),
    ("piston back, inner pad home", {**BACK, **pad(PAD_IN)}, 0.75, False),
    ("both pads home, piston back", BOTH, 1.0, True),
    ("inner pad 1.5 mm short of its stop (limit)", {**BOTH, **pad(PAD_IN, dx=-0.0015 + 1e-7)}, 1.0, True),
    ("inner pad 1.6 mm short of its stop", {**BOTH, **pad(PAD_IN, dx=-0.0016)}, 0.75, False),
    ("outer pad half in", {**BOTH, **pad(PAD_OUT, dx=-0.025)}, 0.75, False),
    ("inner pad riding 1.2 mm above its ledges", {**BOTH, **pad(PAD_IN, dz=0.0012)}, 0.75, False),
    ("inner pad tilted 4 degrees", {**BOTH, **pad(PAD_IN, q=quat((1, 0, 0), 4.0))}, 0.75, False),
    ("outer pad turned 4 degrees in the window", {**BOTH, **pad(PAD_OUT, q=quat((0, 0, 1), 4.0))}, 0.75, False),
    ("inner pad still sliding", {**BOTH, **pad(PAD_IN, vel=(0.03, 0.0, 0.0))}, 0.75, False),
    ("both pads home but the piston proud (forced)", {**cal(back=0.0), **pad(PAD_IN), **pad(PAD_OUT)}, 0.0, False),
    ("pads left where they started, piston back", {**BACK, **pad(PAD_IN, home=False), **pad(PAD_OUT, home=False)},
     0.5, False),
    ("both home, tool left standing on the piston", {**BOTH, "tool": TOOL.state(pos=ON_FACE)}, 0.0, False),
    ("both home, tool 4 mm above the outer pad",
     {**BOTH, "tool": TOOL.state(pos=(CX - 0.03, CY, TOP + Z_OUT + BACK_T + 0.004 + 0.0035))}, 0.0, False),
]
