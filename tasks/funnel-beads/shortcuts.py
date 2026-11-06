"""Scripted shortcut policies for ``funnel_beads.py``, to show each of them falls short of a score.

This is a task file in its own right: it re-exports the real task's scene and scoring untouched and
only swaps in a different ``demo()``, so ``--policy yam_scripted`` plays the shortcut against exactly
the scene and scorer a model would face. Pick one with ``FB_SHORTCUT``:

===============  ======================================================================
``hold``         Both arms stay in the ready pose. The floor of the scoring table.
``tip``          The jug is carried over and poured straight at the bottle's mouth with the
                 funnel left on the bench -- the "just pour carefully" shortcut, given its
                 best shot: poured from as low as the jug's rim can go without fouling the
                 neck, and aimed by the same solved mouth offset the real demo uses.
``high``         The same pour from the height the real demo pours at, to show that the
                 funnel is not merely making an easy thing easier.
``shove``        Both arms push the bottle sideways, to show it cannot be walked under the
                 jug or tipped on its side to present a wider opening.
===============  ======================================================================

    FB_SHORTCUT=tip inspect-robots run --embodiment yam_arena --policy yam_scripted \\
      -E task=shortcuts.py --instruction "Fill the bottle with beads." --scorer task_score ...
"""

import importlib.util
import math
import os
import pathlib
import sys

_NAME = "fb_main"
if _NAME not in sys.modules:
    _spec = importlib.util.spec_from_file_location(
        _NAME, pathlib.Path(__file__).with_name("funnel_beads.py")
    )
    _module = importlib.util.module_from_spec(_spec)
    sys.modules[_NAME] = _module
    _spec.loader.exec_module(_module)
task = sys.modules[_NAME]

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bench = task.bench
TOP = task.TOP

# The scene, the scorer and the agent-facing description are the real task's, unchanged.
__doc__ = task.__doc__
SCENE = task.SCENE
OBJECTS = task.OBJECTS
SCORE_OVER = task.SCORE_OVER
success = task.success
distance = task.distance
score = task.score

MODE = os.environ.get("FB_SHORTCUT", "hold")

JUG_RISE = bench.jug_grip_z(TOP) - (TOP + 0.004) - PAD_TIP_PAST_TCP
JUG_FINGER = bench.finger_for(bench.jug_grip_opening())
MOUTH_Z = bench.bottle_mouth_z(TOP)

# How far the jug's lip hangs below the hand once the jug is over. The real demo clears the funnel's
# rim by this much; a pour at the bare bottle has to clear the neck by it instead.
LIP_DROP = (bench.JUG_R * math.sin(task.POUR_TILT)
            - (bench.JUG_H - JUG_RISE) * math.cos(task.POUR_TILT))


def _jug_wp(seconds, body, tilt, label, finger=JUG_FINGER):
    return Waypoint(seconds, task._tool_tcp(body, tilt, task.POUR_PSI, 0.0, JUG_RISE),
                    finger, label, task._rot(tilt, task.POUR_PSI))


def _hold():
    return {}


def _pour_at_the_bottle(pour_z: float):
    """The real demo's jug sequence, with the funnel left standing on the bench.

    Everything the jug does is unchanged -- the same grip, the same solved aim that lands its
    mouth on the bottle's axis, the same slow tip. The only difference is that nothing has been
    put in the bottle's mouth to catch what comes out.
    """
    jug_rest = (bench.JUG_XY[0], bench.JUG_XY[1], TOP + 0.004)
    jug_high = (jug_rest[0], jug_rest[1], jug_rest[2] + 0.100)
    approach = (task.POUR_XY[0], -0.160, max(pour_z - 0.095, MOUTH_Z + 0.035))
    over = (*task.POUR_XY, pour_z)
    return {
        "right": [
            _jug_wp(2.6, jug_high, task.LEVEL_TILT, "above the jug", FINGER_OPEN),
            _jug_wp(1.6, jug_rest, task.LEVEL_TILT, "down round it", FINGER_OPEN),
            _jug_wp(1.0, jug_rest, task.LEVEL_TILT, "take the jug"),
            _jug_wp(1.8, jug_high, task.CARRY_TILT, "lift it clear"),
            _jug_wp(3.0, approach, task.LEVEL_TILT, "over to the bottle"),
            _jug_wp(2.6, over, task.LEVEL_TILT, "square over the mouth"),
            _jug_wp(4.4, over, task.POUR_TILT, "tip it out"),
            _jug_wp(3.6, over, task.POUR_TILT, "let it all run in"),
            _jug_wp(1.8, approach, task.LEVEL_TILT, "come level"),
        ]
    }


def _tip():
    """As low as the jug's rim can go without fouling the neck: 15 mm of clearance over the mouth."""
    return _pour_at_the_bottle(MOUTH_Z + LIP_DROP + 0.015)


def _high():
    """From the height the real demo pours at, so the only variable is the funnel."""
    return _pour_at_the_bottle(task.POUR_Z)


def _shove():
    """Push the bottle from both sides. It has no mass, so it is a fixed collider: it cannot move."""
    z = TOP + bench.BODY_H / 2
    return {
        "left": [
            Waypoint(2.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] + 0.200, z), 0.0, "line up"),
            Waypoint(3.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] + 0.080, z), 0.0, "push in"),
            Waypoint(3.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] + 0.060, z), 0.0, "push harder"),
            Waypoint(2.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] + 0.060, z), 0.0, "hold"),
        ],
        "right": [
            Waypoint(2.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] - 0.200, z), 0.0, "line up"),
            Waypoint(3.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] - 0.080, z), 0.0, "push in"),
            Waypoint(3.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] - 0.060, z), 0.0, "push harder"),
            Waypoint(2.0, (bench.BOTTLE_XY[0], bench.BOTTLE_XY[1] - 0.060, z), 0.0, "hold"),
        ],
    }


def demo():
    return {"hold": _hold, "tip": _tip, "high": _high, "shove": _shove}[MODE]()
