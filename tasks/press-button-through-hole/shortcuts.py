"""Scripted shortcut policies for ``press_button_through_hole.py``, to show each of them scores 0.

This is a task file in its own right: it re-exports the real task's scene and scoring untouched and
only swaps in a different ``demo()``, so ``--policy yam_scripted`` plays the shortcut against exactly
the scene and scorer a model would face. Pick one with ``PBTH_SHORTCUT``:

===============  ======================================================================
``hold``         Both arms stay in the ready pose. The floor of the scoring table.
``gripper``      The wrist is rolled 90 degrees so the fingers point at the port, and the
                 closed gripper is driven straight at it -- the "just reach in" shortcut.
``rule``         The short rule is picked up and pushed at the port: the wrong tool, too short.
``baton``        The thick baton is picked up and pushed at the port: the wrong tool, too fat.
``throw``        The rod is lined up with the port and dropped instead of pushed, to show that
                 a bar that is not held never reaches or presses the button.
``shove``        Both arms push the enclosure sideways, to show it is bolted down.
===============  ======================================================================

    PBTH_SHORTCUT=gripper inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=shortcuts.py --instruction "Press the red button." --scorer task_score ...
"""

import importlib.util
import math
import os
import pathlib
import sys

_NAME = "pbth_main"
if _NAME not in sys.modules:
    _spec = importlib.util.spec_from_file_location(
        _NAME, pathlib.Path(__file__).with_name("press_button_through_hole.py")
    )
    _module = importlib.util.module_from_spec(_spec)
    sys.modules[_NAME] = _module
    _spec.loader.exec_module(_module)
task = sys.modules[_NAME]

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

cage = task.cage
simkit = task.simkit
TOP = task.TOP

# The scene, the scorer and the agent-facing description are the real task's, unchanged.
__doc__ = task.__doc__
SCENE = task.SCENE
OBJECTS = task.OBJECTS
SCORE_OVER = task.SCORE_OVER
success = task.success
distance = task.distance
score = task.score

MODE = os.environ.get("PBTH_SHORTCUT", "hold")
DOWN = simkit.patch_demo()
FORWARD = simkit.rot_y(-math.pi / 2) @ DOWN  # fingers pointing along +x, straight at the port

PORT_Z = TOP + cage.PORT_ZC_REL
NEAR_X = 0.105  # about as close to its own base as the arm reaches


def _hold():
    return {}


def _gripper():
    """Roll the wrist so the fingers point at the port and drive the closed gripper in.

    ``check_reach.py`` shows the closed hand needs 35.6 mm of clearance only 30 mm behind the pad
    tips, so a 34 mm port stops it about 25 mm in -- 65 mm short of the button.
    """
    approach = PORT_Z  # pointing forward, the pads lead along +x at the port's height
    return {
        "left": [
            Waypoint(2.0, (0.170, cage.CAGE_YC, TOP + 0.110), 0.0, "raise and roll", FORWARD),
            Waypoint(2.0, (0.200, cage.CAGE_YC, approach), 0.0, "line up with the port", FORWARD),
            Waypoint(3.0, (0.285, cage.CAGE_YC, approach), 0.0, "drive at the port", FORWARD),
            Waypoint(3.0, (0.300, cage.CAGE_YC, approach), 0.0, "press harder", FORWARD),
            Waypoint(2.0, (0.300, cage.CAGE_YC, approach), 0.0, "hold", FORWARD),
        ]
    }


def _wrong_tool(name: str):
    """Take a distractor off the rack and push it at the port the way the rod is pushed."""
    length, side, cx, y = {
        "rule": (cage.RULE_LEN, cage.RULE_T, cage.RULE_CX, cage.RULE_Y),
        "baton": (cage.BATON_LEN, cage.BATON_SIDE, cage.BATON_CX, cage.BATON_Y),
    }[name]
    grip_z = cage.bar_rest_z(TOP, side) + PAD_TIP_PAST_TCP
    finger = cage.finger_for(max(side - cage.GRIP_SQUEEZE, 0.004))
    stage_x = max(cage.CAGE_X0 - length / 2 - 0.010, NEAR_X)
    press_x = min(cage.BUTTON_FACE_X + 0.012 - length / 2, 0.38)
    arm = "left" if y > 0 else "right"
    # Crossing the bench from the rule's place to the port passes straight over the rod on its
    # rack.
    cross_z = max(grip_z, cage.bar_rest_z(TOP, cage.ROD_SIDE) + cage.ROD_SIDE / 2 + 0.030 + PAD_TIP_PAST_TCP)
    return {
        arm: [
            Waypoint(2.2, (cx, y, grip_z), FINGER_OPEN, "reach to the bar"),
            Waypoint(0.9, (cx, y, grip_z), finger, "close on it"),
            Waypoint(1.0, (cx, y, cross_z), finger, "lift clear of the rack"),
            Waypoint(1.4, (stage_x, y, cross_z), finger, "draw back"),
            Waypoint(1.8, (stage_x, y / 2, cross_z), finger, "swing across, over the rod"),
            Waypoint(1.6, (stage_x, cage.CAGE_YC, grip_z), finger, "settle on the port's axis"),
            Waypoint(3.0, (press_x, cage.CAGE_YC, grip_z), finger, "push in"),
            Waypoint(2.5, (press_x, cage.CAGE_YC, grip_z), finger, "hold"),
        ]
    }


def _throw():
    """Line the rod up with the port and let go of it, instead of pushing it in.

    This is the "throw the bar in" shortcut. A released rod has 90 mm of empty guard to cross on its
    own; it drops, and nothing presses the button. ``success()`` additionally requires the rod to be
    lying through the port with its near end still outside, which a dropped rod is not.
    """
    plan = list(task.demo()["left"])
    aimed = plan[-4]  # lined up on the port's axis, before the rod is pushed in
    return {
        "left": [
            *plan[:-3],
            Waypoint(0.6, aimed.pos, FINGER_OPEN, "let go of the rod", aimed.yaw),
            Waypoint(2.0, (task.STAGE_X - 0.020, cage.CAGE_YC, aimed.pos[2] + 0.060), FINGER_OPEN, "retreat",
                     aimed.yaw),
            Waypoint(3.0, (task.STAGE_X - 0.020, cage.CAGE_YC, aimed.pos[2] + 0.060), FINGER_OPEN, "wait",
                     aimed.yaw),
        ]
    }


def _shove():
    """Each arm leans on its own side of the enclosure, to show it is bolted down.

    Neither arm crosses the midline: an arm cannot reach in front of the other one, so each pushes
    the wall nearest itself 10 mm *into* the guard, which is what loads it.
    """
    z = TOP + 0.060
    return {
        "left": [
            Waypoint(2.0, (cage.CAGE_X0 + 0.030, cage.CAGE_Y1 + 0.070, z), 0.0, "beside the guard"),
            Waypoint(3.0, (cage.CAGE_X0 + 0.030, cage.CAGE_Y1 - 0.010, z), 0.0, "lean on it"),
            Waypoint(3.0, (cage.CAGE_X0 + 0.030, cage.CAGE_Y1 - 0.010, z), 0.0, "keep pushing"),
        ],
        "right": [
            Waypoint(2.0, (cage.CAGE_X1 - 0.030, cage.CAGE_Y0 - 0.070, z), 0.0, "beside the guard"),
            Waypoint(3.0, (cage.CAGE_X1 - 0.030, cage.CAGE_Y0 + 0.010, z), 0.0, "lean on it"),
            Waypoint(3.0, (cage.CAGE_X1 - 0.030, cage.CAGE_Y0 + 0.010, z), 0.0, "keep pushing"),
        ],
    }


PLANS = {
    "hold": _hold,
    "gripper": _gripper,
    "rule": lambda: _wrong_tool("rule"),
    "baton": lambda: _wrong_tool("baton"),
    "throw": _throw,
    "shove": _shove,
}


def demo():
    if MODE not in PLANS:
        raise ValueError(f"PBTH_SHORTCUT must be one of {sorted(PLANS)}, got {MODE!r}")
    return PLANS[MODE]()
