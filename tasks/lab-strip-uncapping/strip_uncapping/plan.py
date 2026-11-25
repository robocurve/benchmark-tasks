"""Two-handed reference and the unbraced and stationary controls."""

import numpy as np

from . import config as C
from .motion import Program, rotation

TOP = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
LEFT_ROT = rotation("X", -0.74) @ rotation("Z", 0.5 * np.pi) @ TOP
TOOL_GRIP_OFFSET = np.array([-0.012, 0.0, 0.001])
TOOL_CLOSE = 0.23
EXTRACT_START = C.RACK + [
    -C.TIP_FROM_TOOL[0] - 0.015,
    0.0,
    C.RIM_Z + 0.00045 - C.TIP_FROM_TOOL[2],
]
EXTRACT_SEATED = EXTRACT_START + [0.015, 0.0, 0.0]


def tool_pose(origin, axes=None, opening=TOOL_CLOSE):
    axes = np.eye(3) if axes is None else axes
    return np.asarray(origin) + axes @ TOOL_GRIP_OFFSET, axes @ TOP, opening


def waypoints(mode="demo"):
    program = Program()
    if mode == "hold":
        return program.moves
    if mode not in ("demo", "unbraced"):
        raise ValueError(f"Unknown reference mode {mode!r}")
    brace = C.RACK + [0.0, 0.043, 0.0270]
    hover = brace - 0.090 * LEFT_ROT[:, 2]
    left_hover = (hover, LEFT_ROT, 1.0) if mode == "demo" else None
    left_brace = (brace, LEFT_ROT, 0.55) if mode == "demo" else None
    left_preload = (
        (brace - [0.0, 0.0, 0.010], LEFT_ROT, 0.55) if mode == "demo" else None
    )
    left_open = (brace, LEFT_ROT, 1.0) if mode == "demo" else None
    left_release = (hover, LEFT_ROT, 1.0) if mode == "demo" else None
    program.add(
        2.2,
        "approach the tool and retainer",
        left=left_hover,
        right=tool_pose(C.TOOL + [0, 0, 0.075], opening=1.0),
    )
    program.add(
        1.2, "lower the gripper to the tool", right=tool_pose(C.TOOL, opening=1.0)
    )
    program.add(0.9, "grasp the removal tool", right=tool_pose(C.TOOL))
    program.add(
        1.5,
        "lift the tool and approach the retainer",
        right=tool_pose(C.TOOL + [0, 0, 0.080]),
        left=left_open,
    )
    program.add(1.0, "grasp the outside rails of the retainer", left=left_brace)
    program.add(
        2.15,
        "approach the tube row with the retainer held down",
        right=tool_pose(EXTRACT_START + [0, 0, 0.048]),
        left=left_preload,
    )
    program.add(
        1.6, "register the prongs at the cap webbing", right=tool_pose(EXTRACT_START)
    )
    program.add(0.65, "register the prongs and settle the axes")
    program.add(
        1.7, "insert both prongs below the webbing", right=tool_pose(EXTRACT_SEATED)
    )
    program.add(0.65, "settle the prongs under the caps")
    program.add(
        1.5,
        "pry upward slowly against the plug fits",
        right=tool_pose(EXTRACT_SEATED + [0, 0, 0.0036]),
    )
    program.add(
        1.1,
        "pry beyond the plug fits",
        right=tool_pose(EXTRACT_SEATED + [0, 0, 0.0095]),
    )
    program.add(
        0.75,
        "lift the linked caps above the retainer",
        right=tool_pose(EXTRACT_SEATED + [0, 0, 0.037]),
    )
    program.add(
        2.3,
        "carry the caps above the discard dish",
        right=tool_pose(C.DISH + [-0.046, 0, 0.074]),
        left=left_open,
    )
    program.add(
        1.1,
        "lower the caps above the dish",
        right=tool_pose(C.DISH + [-0.048, 0.0, 0.0525]),
        left=left_release,
    )
    program.add(
        1.0,
        "tip the caps into the dish",
        right=tool_pose(C.DISH + [-0.048, 0.0, 0.0525], rotation("Y", 0.85)),
    )
    program.add(0.70, "let the cap strip slide off the prongs")
    program.add(
        0.8,
        "withdraw the tool clear of the dish",
        right=tool_pose(C.DISH + [-0.060, 0, 0.077]),
    )
    return program.moves
