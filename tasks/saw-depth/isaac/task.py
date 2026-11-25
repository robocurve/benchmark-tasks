"""Circular-saw blade depth for yam_arena (Isaac Lab-Arena / PhysX): a DeWalt DWE575-class 7-1/4 in saw sits on a board
with its blade beside the board's edge; release the depth lever, set the blade so its lowest tooth is 1/8 to 1/4 in (3.2 to 6.4 mm) below
the board's underside, lock the lever and take the hands off the saw.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``sawkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses and joint angles; ``score()`` returns its level at the end of the trial
(SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the board (six stock thicknesses), the
start depth (saw left retracted, 0-2 mm, or deep, 59.5-64 mm, 1/2 each, independent of the board) and a +-12 mm shift of
the saw along the board; unseeded resets use the six named setups in turn (SAW_SETUP picks the first and, when set, forces named
setups). The full-score reference policy is ``official_run.py`` (registered as ``saw_depth_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from sawkit import SawBench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = board centre; size only for yam_arena's checks).
OBJECTS = [
    SawBench(
        "saw_bench",
        pos=(float(L.BOARD_X0 + L.BOARD_W / 2), 0.0, float(L.UNDERSIDE_Z)),
        size=(0.3, 0.5, 0.2),
        mass=None,
        color=(0.8, 0.66, 0.44),
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
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured poses
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
