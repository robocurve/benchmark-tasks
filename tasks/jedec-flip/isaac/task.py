"""JEDEC tray flip for yam_arena (Isaac Lab-Arena / PhysX): stack the empty JEDEC tray on the loaded one, squeeze the
two-tray stack at both ends, roll it over, set it down and lift the old tray off, so every 35 x 35 mm BGA package lies
ball side up in the new tray for ball inspection.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``flipkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses; ``score()`` returns its level at the end of the trial (SCORE_OVER = "end"); the trial
ends as soon as the full-score state has held for 1 s (inside the scorer). Start states: a reset with a seed
(inspect-robots always passes one) moves the loaded tray on its riser (up to 10 mm / 5 mm / 4 deg) and the empty tray
on the table (up to 12 mm / 15 mm / 8 deg), never leaving the empty tray within 8 mm and 3 deg of a straight
home-to-home move; unseeded resets use the named setups 0-4 in turn (JFLIP_SETUP picks the first and, when set, forces
named setups). The full-score reference policy is ``official_run.py`` (registered as ``jedec_flip_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from flipkit import STATE, TrayBench  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = loaded tray's nominal origin; size only for yam_arena's checks).
OBJECTS = [
    TrayBench(
        "bench",
        pos=tuple(float(v) for v in L.A_HOME),
        size=(0.3, 0.3, 0.05),
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
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured poses
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
