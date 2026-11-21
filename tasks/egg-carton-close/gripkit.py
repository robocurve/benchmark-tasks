"""gripkit: lets a scripted demo's Waypoint carry a full gripper rotation, so demos can grip from the side or with
the gripper pitched, not only straight down. Written from scratch for this kit (MIT). Same idea as the six-line patch
in builds/wardrobe-rail-v2/wardrobe_rail.py, moved here so every task shares one copy.

It only changes what a SCRIPTED DEMO can ask for. Dream-1 drives the joints itself and is not affected. Nothing is
teleported and the simulation is not edited.

Use:
    import gripkit
    Waypoint(1.5, (x, y, z), FINGER_OPEN, "reach in", gripkit.side())            # from the front, fingers top/bottom
    Waypoint(1.5, (x, y, z), FINGER_CLOSED, "tip it", gripkit.pitched(45))       # 45 degrees between down and forward
    Waypoint(1.5, (x, y, z), FINGER_OPEN, "above", 0.0)                          # a plain yaw still works
    Waypoint(1.5, (x, y, z), FINGER_CLOSED, "tip it", gripkit.pitched(45, flip=True))   # the same grip for the RIGHT arm

flip=True turns the grip 180 degrees about its own approach direction (R @ diag(-1, -1, 1)). The fingers close along
the same line, so the grip on the object is the same, but the wrist is the other way up. The right arm needs it for
side and pitched grips: measured 15 Nov, its IK is 50-67 degrees off without it and within 3 mm / 3 degrees with it.

Run `python ikcheck.py task.py` before any GPU run: a rotated grip that the arm cannot reach shows up there first.
"""

import math

import numpy as np
from yam_arena import demo as _demo

if not getattr(_demo, "_gripkit_installed", False):
    _plain = _demo.grip_rotation

    def _grip_rotation(yaw=0.0):
        """A 3x3 matrix is used as the TCP rotation (world frame); a number is a yaw, as before."""
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else _plain(yaw)

    _demo.grip_rotation = _grip_rotation
    _demo._gripkit_installed = True

R_DOWN = np.asarray(_demo.grip_rotation(0.0), float)   # straight down: approach -z, fingers closing along world y
FLIP = np.diag([-1.0, -1.0, 1.0])                       # half a turn about the gripper's own approach axis


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def grip(approach, closing, flip=False):
    """TCP rotation for a gripper that points along `approach` with its fingers closing along `closing` (world frame).
    `closing` is made square to `approach`. flip=True: the same grip turned 180 degrees about the approach (use it
    for the right arm)."""
    a = _unit(approach)
    c = np.asarray(closing, float)
    c = _unit(c - np.dot(c, a) * a)
    R = np.column_stack([np.cross(a, c), c, -a]) @ R_DOWN       # world -z -> a, world y -> c
    return R @ FLIP if flip else R


def side(yaw_deg=0.0, fingers="vertical", flip=False):
    """Gripper horizontal, pointing forward (+x) turned yaw_deg about z. fingers="vertical": they close on the top and
    bottom of the object; "horizontal": they close on its two sides. flip=True for the right arm."""
    y = math.radians(yaw_deg)
    a = (math.cos(y), math.sin(y), 0.0)
    c = (0.0, 0.0, 1.0) if fingers == "vertical" else (-math.sin(y), math.cos(y), 0.0)
    return grip(a, c, flip)


def pitched(pitch_deg, yaw_deg=0.0, flip=False):
    """Gripper tipped pitch_deg from straight down (0) toward forward (90), fingers closing horizontally across.
    flip=True for the right arm."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    a = (math.sin(p) * math.cos(y), math.sin(p) * math.sin(y), -math.cos(p))
    return grip(a, (-math.sin(y), math.cos(y), 0.0), flip)
