"""Reference actions through the two physical grippers."""

import numpy as np
from yam_arena import ik

from . import config as C
from .motion import Program, rotation


def pinch(lateral, approach):
    z = np.asarray(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.asarray(lateral) - np.dot(lateral, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


LEFT = pinch([-1., 0., 0.], [.21, -.60, -1.])
RIGHT = pinch([1., 0., 0.], [.18, .63, -1.])


def left(centre, offset=(0., 0., 0.), grip=.23):
    return np.asarray(centre) + [0., .020, 0.] + offset, LEFT, grip


def right(centre, offset=(0., 0., 0.), grip=.23):
    return np.asarray(centre) + [0., -.016, 0.] + offset, RIGHT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.5, "Approach whole egg", left=left(C.START, (0., 0., .105), .67))
    p.add(1.0, "Reach the narrow end", left=left(C.START, grip=.64))
    p.add(.6, "Hold egg with both pads", left=left(C.START))
    p.add(1.5, "Lift egg before tapping", left=left(C.START, (0., 0., .055)))
    p.add(.8, "Tap the shell on the board", left=left(C.START, (0., 0., -.0004)))
    p.add(1.5, "Lift cracked egg", left=left(C.WORK))
    if mode == "tap-only":
        p.add(2.0, "Return cracked but unpeeled egg to board", left=left(C.START))
        p.add(.5, "Release cracked egg", left=left(C.START, grip=.62))
        p.add(1.2, "Withdraw hand", left=left(C.START, (0., 0., .12), .62))
        return p.moves
    p.add(1.8, "Approach loose shell at blunt end",
          right=right(C.WORK, (0., -.075, .075), .61))
    p.add(.8, "Straddle the shell edge", right=right(C.WORK, (.0, .009, .002), .58))
    p.add(.6, "Grip loose shell", right=right(C.WORK, (.0, .009, .002), .18))
    p.add(1.1, "Pull loose shell away at slight angle",
          right=(right(C.WORK, (.010, -.077, .027), .18)[0], rotation("Y", .38) @ RIGHT, .18))
    p.add(.40, "Keep support while the shell pieces separate")
    return p.moves


def regrasp_band(actual, centre, rotation_in_world):
    """Regrasp the remaining shell along its measured rim and pull against the support."""
    p = Program()
    p.pose = actual.copy()
    right_pos, _, _ = actual["right"]
    grasp = centre + rotation_in_world @ [0., -.012, .0045]
    aligned = ik.slerp_matrix(RIGHT, rotation_in_world @ RIGHT, .70)
    hover = grasp - .048 * aligned[:, 2]
    p.add(.95, "Open and turn the right gripper toward the remaining shell",
          right=(right_pos + [0., .014, .022], RIGHT, .62))
    p.add(1.23, "Align the open fingers with the broken shell rim",
          right=(hover, aligned, .62))
    p.add(.66, "Reach onto the remaining shell", right=(grasp, aligned, .62))
    p.add(.64, "Pinch the remaining shell", right=(grasp, aligned, .23))
    root_pos, root_rot, root_grip = actual["left"]
    pull_axis = rotation_in_world @ [0., -.065, .020]
    p.add(1.38, "Peel the remaining shell while the other hand provides support",
          right=(grasp + pull_axis + [0., 0., .014], rotation("Y", .20) @ aligned, .23),
          left=(root_pos + [0., .016, .005], root_rot, root_grip))
    p.add(.4, "Keep the egg supported as the freed shell drops")
    return p.moves


def plate_reference(holder, actual, egg):
    """Carry the peeled egg using the hand that still supports it."""
    p = Program()
    p.pose = actual.copy()
    other = "right" if holder == "left" else "left"
    position, orientation, grip = actual[holder]
    offset = position - egg

    def target(position, openness):
        return (np.asarray(position) + offset, orientation, openness)

    side = 1. if other == "left" else -1.
    other_pos, other_orientation, _ = actual[other]
    retreat_rot = RIGHT if other == "right" else other_orientation
    retreat = (other_pos + [0., .039 * side, .045], retreat_rot, .66)
    p.add(1.1, "Withdraw the peeling hand while supporting the egg",
          **{other: retreat})
    p.add(2.25, "Carry the whole egg to the blue plate",
          **{holder: target(C.PLATE + [0., 0., .103], grip)})
    p.add(1.3, "Lower the egg gently to the plate",
          **{holder: target(C.PLATE + [0., 0., .025], grip)})
    p.add(.66, "Release the intact peeled egg",
          **{holder: target(C.PLATE + [0., 0., .025], .63)})
    p.add(1.2, "Withdraw both grippers",
          **{holder: target(C.PLATE + [0., 0., .131], .63)})
    return p.moves
