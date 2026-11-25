"""Clock set to the time and hung back level, for yam_arena (Isaac Lab-Arena / PhysX): a 10 in quartz wall clock hangs
by its keyhole on a nail in a wall section in front of the robot, still showing standard time (one hour behind and a few
minutes off); its time is set with the setting knob behind its back (the crystal covers the hands). The robot takes it
off the nail, turns the knob until the hands show 10:10, and hangs it back on the nail, level.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 3000 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``clockkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured state plus the per-physics-step contact monitor; ``score()`` returns its level at the end of
the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the clock's start time uniformly from 8:55
to 9:25 and moves the nail by up to 15 mm sideways and 12 mm up or down; unseeded resets use the named setups in turn
(see task.json; CLOCK_SETUP picks the first and, when set, forces named setups). The full-score reference policy is
``official_run.py`` (registered as ``clock_dst_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from clockkit import ClockSet, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s hold is inside the scorer (every level needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = clock; size only for yam_arena's checks).
OBJECTS = [
    ClockSet(
        "clockset",
        pos=(L.WALL_X - 0.03, 0.0, L.NAIL_Z0 - 0.08),
        size=(0.25, 0.25, 0.05),
        mass=None,
        color=(0.1, 0.1, 0.1),
    )
]


def score(states):
    return float(STATE["score"])


def success(states):
    return STATE["score"] >= 1.0


def distance(states):
    return 1.0 - float(STATE["score"])


def demo():
    """For ``--policy yam_scripted`` (open-loop waypoints): no plan, both arms hold READY, so the official harness
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured state
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
