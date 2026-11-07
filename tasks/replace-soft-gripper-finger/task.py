"""On the station table, a spare parallel gripper lies flat in a steel fixture between the arms: white printed jaws on a linear rail, with two soft orange fin-ray
fingers pointing at the robot. Each finger lies in a shallow pocket in its jaw, its root under the loose upper plate of its fork. The left finger is worn: faded,
its ribs broken and its black grip strip mostly gone. The right finger is good. To the left and further out is a tray with two spare fingers, one new and bright, one
used and worn. Nearer on the left stands a red bin.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import fingerkit  # noqa: E402  (hooks yam_arena so the parts below are simulated)
from fingerkit import BED_Z, BIN_SIZE, BIN_XY, CLAMP, CLAMP_PINCH_X, CLAMP_X, SCREW_BLOCK, SCREW_BLOCK_HOLES, SCREW_BLOCK_XY, fork_holes, FIN_MIDDLE, FIN_T, NEW_XY, TOP, TRAY_FLOOR, USED_XY, seat_xy  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
INSTRUCTION = (
    "One soft finger on this spare gripper is worn. Its three screws are already out, in the white block. Replace the finger with a new one, put the worn finger in the red bin, "
    "put the fork plate back on the new finger and drop the three screws into their holes."
)
OBJECTS = fingerkit.parts()
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

OPEN_ROOT = -0.0235  # each finger 2.35 cm from the middle, round the 4 cm root
OPEN_CLAMP = -0.0235  # and round the 4 cm fork plate
GRIP = -0.015  # short of the 2 cm half width: about 25 N a finger, which keeps the part centred. Closed fully, both fingers reach their force limit and the part slides along them
ROOT_YAW = 0.0  # the fingers close along y, on the root's sides
CLAMP_YAW = 0.0
PINCH_BELOW_TOP = 0.003  # the TCP this far below the top of a finger
FINGERTIP_PAST_TCP = 0.013
HOVER = TOP + 0.15
OPEN_SCREW = -0.009  # each finger 9 mm from the middle, round the screw's head
SCREW_UP = TOP + 0.13  # the shank clears the plate
SCREW_HEAD_IN_TCP = FINGERTIP_PAST_TCP + 0.0012  # from the head's underside up to the TCP: the fingertips stay just above what the screw stands on
FINGERS_SHUT = 0.0  # the 8 mm head stops them about 20 N a finger short of their limit
HOLE_OFFSETS = ((0.0013, -0.002), (0.0, 0.0), (0.0, 0.0))  # the left arm stands this far off its target over each hole, from a logged run


def _report(objects) -> dict:
    return objects["spare_gripper"].report


def success(objects) -> bool:
    """The worn finger in the bin, the new finger home, a clamp in each place, grippers clear, all still."""
    return bool(_report(objects)["ready"])


def score(objects) -> float:
    """See the scoring text in task.json: fingerkit.task_score applies it."""
    return _report(objects)["score"]


def _left(duration, xy, z, finger, label) -> Waypoint:
    return Waypoint(duration, (float(xy[0]), float(xy[1]), float(z)), finger, label, ROOT_YAW)


def _right(duration, xy, z, finger, label) -> Waypoint:
    return Waypoint(duration, (float(xy[0]), float(xy[1]), float(z)), finger, label, CLAMP_YAW)


def demo() -> dict[str, list[Waypoint]]:
    """`FINGER_DEMO=new` (default): the right arm lifts the clamp and holds it, the left arm bins the worn finger and seats the new one, the right arm refits the
    clamp. `FINGER_DEMO=used`: the same moves with the used spare. Open loop, from the scene's starting layout; heights are of the TCP."""
    mode = os.environ.get("FINGER_DEMO", "new")
    if mode not in ("new", "used"):
        raise ValueError(f"FINGER_DEMO must be new or used, got {mode!r}")
    spare = NEW_XY if mode == "new" else USED_XY
    root = seat_xy(1.0)
    clamp = (root[0] + CLAMP_PINCH_X, root[1])
    clamp_grip = TOP + BED_Z + FIN_T + 0.0004 + FINGERTIP_PAST_TCP  # the fingertips just above the finger's top, so that a pinch on the fork plate leaves the finger alone
    clamp_up = clamp_grip + 0.035
    parked = (0.24, -0.16)
    root_grip = TOP + BED_Z + FIN_T - PINCH_BELOW_TOP
    tray_grip = TOP + TRAY_FLOOR + FIN_T - PINCH_BELOW_TOP
    carry = TOP + 0.125  # a held finger clears the holders; the arm droops about 1 cm under load this far out
    over_bin_xy = (BIN_XY[0] + FIN_MIDDLE, BIN_XY[1])  # the finger's middle over the bin's middle
    over_bin = TOP + BIN_SIZE[2] + FIN_T + 0.02
    right_first = [
        _right(1.8, clamp, clamp_up, OPEN_CLAMP, "above the clamp"),
        _right(1.2, clamp, clamp_grip, OPEN_CLAMP, "down to the clamp"),
        _right(0.6, clamp, clamp_grip, GRIP, "close on the clamp"),
        _right(0.4, clamp, clamp_grip, GRIP, "hold the clamp"),
        _right(1.4, clamp, clamp_up, GRIP, "lift the clamp"),
        _right(1.8, parked, HOVER, GRIP, "hold it aside"),
    ]
    t_right = sum(w.duration for w in right_first)
    left = [
        _left(2.0, (0.30, 0.20), HOVER, OPEN_ROOT, "stand clear of the other arm"),
        _left(t_right - 2.0, (0.30, 0.20), HOVER, OPEN_ROOT, "wait for the clamp"),
        _left(2.0, root, root_grip + 0.045, OPEN_ROOT, "above the worn finger"),
        _left(1.6, root, root_grip, OPEN_ROOT, "down to the worn finger"),
        _left(0.6, root, root_grip, GRIP, "close on the worn finger"),
        _left(0.3, root, root_grip, GRIP, "hold the worn finger"),
        _left(2.2, root, carry, GRIP, "lift it out"),
        _left(2.2, over_bin_xy, carry, GRIP, "over the bin"),
        _left(0.8, over_bin_xy, over_bin, GRIP, "lower into the bin"),
        _left(0.4, over_bin_xy, over_bin, -0.035, "drop it"),
        _left(0.8, over_bin_xy, over_bin + 0.03, -0.035, "rise from the bin"),
        _left(2.0, spare, tray_grip + 0.05, OPEN_ROOT, "above the spare"),
        _left(1.0, spare, tray_grip, OPEN_ROOT, "down to the spare"),
        _left(0.6, spare, tray_grip, GRIP, "close on the spare"),
        _left(0.3, spare, tray_grip, GRIP, "hold the spare"),
        _left(1.2, spare, tray_grip + 0.06, GRIP, "lift the spare"),
        _left(2.4, root, carry, GRIP, "over the jaw"),
        _left(2.0, root, root_grip + 0.001, GRIP, "into the pocket"),
        _left(0.5, root, root_grip + 0.001, OPEN_ROOT, "let go"),
        _left(1.2, root, carry, OPEN_ROOT, "rise from the jaw"),
        _left(1.5, (0.26, 0.31), HOVER, OPEN_ROOT, "clear"),
    ]
    t_left = sum(w.duration for w in left)
    right = right_first + [
        _right(max(0.1, t_left - t_right), parked, HOVER, GRIP, "wait"),
        _right(2.2, clamp, clamp_up, GRIP, "back over the jaw"),
        _right(1.6, clamp, clamp_grip + 0.001, GRIP, "onto the root"),
        _right(0.5, clamp, clamp_grip + 0.001, OPEN_CLAMP, "let go of the clamp"),
        _right(1.2, clamp, clamp_up, OPEN_CLAMP, "rise from the clamp"),
        _right(1.5, (0.26, -0.31), HOVER, OPEN_CLAMP, "clear"),
    ]
    # The screws, one at a time, once the other arm is clear.
    t_right_all = sum(w.duration for w in right)
    t_left_all = sum(w.duration for w in left)
    if t_right_all > t_left_all:
        left.append(_left(t_right_all - t_left_all, (0.26, 0.31), HOVER, OPEN_ROOT, "wait for the plate"))
    plate_top = TOP + BED_Z + FIN_T + CLAMP[2]
    holes = fork_holes(1.0)
    base = (root[0] + CLAMP_X, root[1])
    pick_z = TOP + SCREW_BLOCK[2] + SCREW_HEAD_IN_TCP
    drop_z = plate_top + 0.001 + SCREW_HEAD_IN_TCP
    for i, ((hx, hy), dx, off) in enumerate(zip(holes, SCREW_BLOCK_HOLES, HOLE_OFFSETS)):
        pick = (SCREW_BLOCK_XY[0] + dx, SCREW_BLOCK_XY[1])
        over = (base[0] + hx + off[0], base[1] + hy + off[1])
        left += [
            _left(1.8, pick, SCREW_UP, OPEN_SCREW, f"above screw {i}"),
            _left(1.2, pick, pick_z, OPEN_SCREW, f"down to screw {i}"),
            _left(0.5, pick, pick_z, FINGERS_SHUT, f"close on screw {i}"),
            _left(0.3, pick, pick_z, FINGERS_SHUT, f"hold screw {i}"),
            _left(1.4, pick, SCREW_UP, FINGERS_SHUT, f"lift screw {i}"),
            _left(2.0, over, SCREW_UP, FINGERS_SHUT, f"over hole {i}"),
            _left(1.6, over, drop_z + 0.012, FINGERS_SHUT, f"lower screw {i}"),
            _left(1.0, over, drop_z + 0.012, FINGERS_SHUT, f"settle over hole {i}"),
            _left(1.2, over, drop_z, FINGERS_SHUT, f"into hole {i}"),
            _left(0.6, over, drop_z, FINGERS_SHUT, f"settle in hole {i}"),
            _left(0.4, over, drop_z, OPEN_SCREW, f"drop screw {i}"),
            _left(0.8, over, SCREW_UP, OPEN_SCREW, f"rise from hole {i}"),
        ]
    left.append(_left(1.5, (0.26, 0.31), HOVER, OPEN_ROOT, "clear at last"))
    return {"left": left, "right": right}
