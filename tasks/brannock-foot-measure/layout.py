"""Where the device and forms stand on the station table (world frame of yam_arena: x away from the arms, y left,
z up, metres). Pure Python.

The device lies with its long axis across the table: the heel cup on the left (toward the left arm), the toes
toward the right arm, its medial guide on the far side. Device frame (sizing.py): u = -y_world, v = +x_world.
"""

from __future__ import annotations

import math
import os

import numpy as np

from sizing import DEVICE, FORMS

TOP = 0.7475  # station table top (yam_arena.task_file.TABLE_TOP["station"])
X0, Y0 = 0.29, 0.13  # device origin (heel cup back wall on the centre line), world x, y
Z0 = TOP + DEVICE["base_t"] / 1000.0  # base plate's top surface

# Trial k uses LAYOUTS[k % 3]: which form, how far forward of the cup its heel starts (mm), how far off the
# centre line (mm, + medial) and its yaw (deg, + turns the toes medially).
LAYOUTS = (
    dict(form=0, fwd=22.0, side=-3.0, yaw=-1.5),
    dict(form=1, fwd=28.0, side=-4.0, yaw=-2.0),
    dict(form=2, fwd=25.0, side=-2.0, yaw=-1.0),
)


def trial_layout() -> int:
    if os.environ.get("BRANNOCK_LAYOUT"):
        return int(os.environ["BRANNOCK_LAYOUT"]) % 3
    return int(os.environ.get("TRIAL", "0")) % 3


def to_world(u_mm, v_mm, z_mm=0.0) -> np.ndarray:
    """Device frame (mm, z above the base top) -> world (m)."""
    return np.array([X0 + v_mm / 1000.0, Y0 - u_mm / 1000.0, Z0 + z_mm / 1000.0])


def to_device(p_world) -> np.ndarray:
    """World (m) -> device frame (mm)."""
    p = np.asarray(p_world, float)
    return np.array([(Y0 - p[1]) * 1000.0, (p[0] - X0) * 1000.0, (p[2] - Z0) * 1000.0])


# The form's own frame (footform.py) equals the device frame when seated. Its world rotation for yaw psi (about z,
# in the device frame, + toward medial) : device u -> world -y, v -> world +x.
def form_quat_xyzw(yaw_deg: float) -> tuple[float, float, float, float]:
    """World orientation of a form whose u axis is turned yaw_deg toward medial from the device's u axis."""
    # form frame -> world is Rz(theta): the form's u axis goes to (sin psi, -cos psi, 0) in the world.
    psi = math.radians(yaw_deg)
    theta = -math.pi / 2 + psi  # rotation taking world x (form u) to the u direction in the world
    return (0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))


def form_start(k: int):
    """(form dict, world position of the form frame origin, quat) for layout k."""
    lay = LAYOUTS[k]
    return FORMS[lay["form"]], to_world(lay["fwd"], lay["side"], 0.0), form_quat_xyzw(lay["yaw"])
