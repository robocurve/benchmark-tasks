"""Scripted shortcut policies for ``scoop_beads.py``, to show each of them falls short of a score.

This is a task file in its own right: it re-exports the real task's scene and scoring untouched and
only swaps in a different ``demo()``. Pick one with ``SCOOP_SHORTCUT``:

===============  ======================================================================
``hold``         Both arms stay in the ready pose. The floor of the scoring table.
``gripper``      The open jaws are driven into the tray and closed on the beads, then
                 carried to the bowl -- the "just pick them up" shortcut.
``spatula``      The flat spatula is taken off its rack and drawn through the beads the
                 way the scoop is: the wrong tool, with no walls to hold anything.
``shove``        Both arms push the tray, to show it cannot be slid over to the bowl or
                 tipped out.
===============  ======================================================================

    SCOOP_SHORTCUT=spatula inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=shortcuts.py --instruction "Move more than four beads into the bowl." ...
"""

import importlib.util
import math
import os
import pathlib
import sys

_NAME = "sb_main"
if _NAME not in sys.modules:
    _spec = importlib.util.spec_from_file_location(
        _NAME, pathlib.Path(__file__).with_name("scoop_beads.py")
    )
    _module = importlib.util.module_from_spec(_spec)
    sys.modules[_NAME] = _module
    _spec.loader.exec_module(_module)
task = sys.modules[_NAME]

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bench = task.bench
TOP = task.TOP
MODE = os.environ.get("SCOOP_SHORTCUT", "hold")

SRC = bench.SOURCE_XY
TGT = bench.TARGET_XY
FLOOR = bench.source_floor_z(TOP)
LEVEL = bench.scoop_rotation(0.0, task.DOWN, bench.SCOOP_YAW)


def _hold():
    return {}


def _gripper():
    """Drive the open jaws into the heap, close them, and carry whatever is held to the bowl.

    The pads are flat plates 94 mm apart closing on 16 mm spheres: they have nothing to close
    against but each other, and a sphere squeezed between two flat faces squirts out of them.
    """
    over = (SRC[0], SRC[1], TOP + 0.120)
    down = (SRC[0], SRC[1], FLOOR + bench.BEAD_R + PAD_TIP_PAST_TCP)
    return {
        "left": [
            Waypoint(2.6, over, FINGER_OPEN, "over the tray", LEVEL),
            Waypoint(2.2, down, FINGER_OPEN, "down into the beads", LEVEL),
            Waypoint(1.6, down, 0.0, "close on them", LEVEL),
            Waypoint(2.0, over, 0.0, "lift", LEVEL),
            Waypoint(3.0, (TGT[0], TGT[1], TOP + 0.120), 0.0, "over the bowl", LEVEL),
            Waypoint(1.6, (TGT[0], TGT[1], TOP + 0.120), FINGER_OPEN, "let go", LEVEL),
        ]
    }


def _spatula():
    """Take the flat spatula and draw it through the heap exactly as the scoop is drawn.

    It reaches the beads and pushes them against the tray's wall just as the scoop does. With no
    walls of its own, nothing stays on it: what rides up the blade rolls straight back off.
    """
    # Gripped across its blade, at the height its rack holds it at.
    grip_xy = bench.SPATULA_XY
    grip_z = TOP + bench.RACK_TOP + 0.006 + PAD_TIP_PAST_TCP
    finger = bench.finger_for(0.018)
    plan = [
        Waypoint(2.6, (*grip_xy, grip_z + 0.070), FINGER_OPEN, "above the spatula", LEVEL),
        Waypoint(1.6, (*grip_xy, grip_z), FINGER_OPEN, "down onto it", LEVEL),
        Waypoint(1.0, (*grip_xy, grip_z), finger, "take it", LEVEL),
    ]
    for i, (along, tilt, label) in enumerate(
        [(-0.045, task.DIG_TILT, "lip onto the bottom"),
         (-0.045, task.DIG_TILT, "settle"),
         (0.020, task.DRAW_TILT, "draw it through"),
         (0.053, task.DRAW_TILT, "run them up the wall"),
         (0.053, task.LEVEL_TILT, "lift them off the wall")]
    ):
        pan = task._pan(SRC, along, 0.0, bench.dig_pan_z(TOP, tilt, task.DRAW_PRESS))
        plan.append(Waypoint(2.6, bench.scoop_tcp(pan, tilt), finger, label,
                             bench.scoop_rotation(tilt, task.DOWN)))
    plan.append(Waypoint(2.6, (TGT[0], TGT[1], TOP + 0.130), finger, "over the bowl", LEVEL))
    return {"left": plan}


def _shove():
    """Push the tray from both sides. It has no mass, so it is a fixed collider: it cannot move."""
    z = TOP + bench.SOURCE_WALL_H / 2
    out = {}
    for arm, sign in (("left", 1.0), ("right", -1.0)):
        out[arm] = [
            Waypoint(2.0, (SRC[0], SRC[1] + sign * 0.230, z), 0.0, "line up", LEVEL),
            Waypoint(3.0, (SRC[0], SRC[1] + sign * 0.125, z), 0.0, "push in", LEVEL),
            Waypoint(3.0, (SRC[0], SRC[1] + sign * 0.110, z), 0.0, "push harder", LEVEL),
            Waypoint(2.0, (SRC[0], SRC[1] + sign * 0.110, z), 0.0, "hold", LEVEL),
        ]
    return out


# The scene, the scorer and the agent-facing description are the real task's, unchanged.
__doc__ = task.__doc__
SCENE = task.SCENE
OBJECTS = task.OBJECTS
SCORE_OVER = task.SCORE_OVER
success = task.success
distance = task.distance
score = task.score


def demo():
    return {"hold": _hold, "gripper": _gripper, "spatula": _spatula, "shove": _shove}[MODE]()
