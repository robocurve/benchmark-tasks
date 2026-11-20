"""Fixed scene: three Alesis Nitro Max pads at real size on the calibrated dual-arm YAM station, set for open-handed
playing (the hi-hat in front of the left arm, the snare and the tom in front of the right), and a Vic Firth 5A stick
held by friction in each gripper, its finger pads on the stick's upper and lower sides.

Tabletop frame: x forward from the line joining the arm bases, y to the robot's left, z up from the table top; metres.
Importable without NumPy: the scorer's tests need only Python.
"""

import math
import os
from pathlib import Path

from . import layout as L

try:                                            # the station numbers come from yam_arena when it is installed
    from yam_arena import station as _station

    TOP = float(_station.TABLE_TOP)
    _BASES = {arm: tuple(float(v) for v in _station.BASES[arm]) for arm in ("left", "right")}
except ImportError:                             # the same numbers, so that the scorer's tests need only Python
    TOP = .7475
    _BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}

CONTROL_HZ = L.CONTROL_HZ
PHYSICS_HZ = L.PHYSICS_HZ
PHYSICS_DT = 1. / PHYSICS_HZ
MAX_STEPS = 1200
ARMS = ("left", "right")
BASES = {arm: (_BASES[arm][0], _BASES[arm][1], _BASES[arm][2] - TOP) for arm in ARMS}    # tabletop frame
USD = Path(os.environ.get("DRUM_USD") or Path(__file__).with_name("kit.usda"))

PADS = ("hihat", "snare", "tom")
PLAYS = {"hihat": "left", "snare": "right", "tom": "right"}      # open-handed: no crossing (not scored)
# Trigger zones, one kinematic body each. The 8 in tom and the hi-hat pad are single-zone pads: a stick on the tom's
# rim sounds the tom, as on the real kit. Only the 10 in snare's rim is its own zone (a wrong note in this groove).
SURFACES = ("hihat", "snare_head", "snare_rim", "tom_head", "tom_rim")
ZONE_PAD = {"hihat": "hihat", "snare_head": "snare", "snare_rim": "snare", "tom_head": "tom", "tom_rim": "tom"}
ZONE_NOTE = {"hihat": "hihat", "snare_head": "snare", "snare_rim": "snare_rim", "tom_head": "tom", "tom_rim": "tom"}
STICKS = {"left": "stick_left", "right": "stick_right"}
KIT, ACTORS = "Kit", "Sticks"

# The arms' start pose (radians, joints 1 to 6): each stick's tip 80 mm over its pad, ready to play (from
# ``scripted.contact_poses``, frozen here so the scene and the card state them). The grippers are closed on the sticks.
START_Q = {
    "left": (-0.32717, 0.93901, 0.21786, 0.72071, 0.96495, 1.56246),
    "right": (0.25913, 1.56991, 0.27403, 1.29533, -1.10679, -1.66646),
}
START_TIP_LIFT = .080


def pad_frame(kind):
    """(centre of the playing surface, unit normal) of a pad in the tabletop frame: tilted toward the robot (-x)."""
    (x, y), z, tilt = L.PAD_SPECS[kind]
    t = math.radians(tilt)
    return (x, y, z), (-math.sin(t), 0., math.cos(t))


def pad_quat_wxyz(kind):
    """The pad's orientation: a rotation about +y by -tilt (its normal leans toward the robot)."""
    t = math.radians(L.PAD_SPECS[kind][2])
    return (math.cos(-t / 2), 0., math.sin(-t / 2), 0.)


def pad_outer_r(kind):
    return {"hihat": L.HIHAT_R, "snare": L.SNARE_HEAD_R + L.RIM_W, "tom": L.TOM_HEAD_R + L.RIM_W}[kind]


def tip_clearance(point, kind):
    """Distance (m) from a stick's tip sphere centre to a pad's collider (shell and rim, or the hi-hat pad), less
    the tip's radius: how far the tip is from touching it."""
    c, n = pad_frame(kind)
    d = [point[i] - c[i] for i in range(3)]
    a = sum(d[i] * n[i] for i in range(3))
    radial = math.sqrt(max(sum(v * v for v in d) - a * a, 0.))
    top = L.HIHAT_BELL_TOP if kind == "hihat" else L.RIM_H
    bottom = -(L.HIHAT_T_EDGE if kind == "hihat" else L.SHELL_DEPTH)
    dr = radial - pad_outer_r(kind)
    da = max(bottom - a, a - top)
    if dr < 0 and da < 0:
        return max(dr, da) - L.TIP_R
    return math.hypot(max(dr, 0.), max(da, 0.)) - L.TIP_R


INSTRUCTION = ("Play this groove on the drum kit with the stick in each hand, at 75 beats per minute, counting "
               "1 & 2 & 3 & 4 &. Bars 1 to 3: hit the hi-hat on every count, loud on the numbers and soft on the ands, "
               "and the snare loud on 2 and 4 together with the hi-hat. Bar 4: the same up to 3 &, then only the tom, "
               "on 4 and on 4 &. End with the hi-hat and snare together on 1 of the next bar. Let each stroke come "
               "straight off the pad; at the end hold both sticks still, clear of the kit.")


def side_camera():
    """Review camera: in front of the kit, high, looking back at the pads and both hands. Not a policy input."""
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.rig import Camera, Lens

    eye = np.array((.98, -.05, TOP + .52))
    look = np.array((.40, 0., TOP + .10))
    back = eye - look
    back /= np.linalg.norm(back)
    right = np.cross((0., 0., 1.), back)
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(back, right), back))
    return Camera(tuple(eye), tuple(matrix_to_quat_xyzw(rotation)), "opengl", Lens.from_focal(14.))
