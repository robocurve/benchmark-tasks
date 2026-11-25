"""Impact-on-jobs task for yam_arena (Isaac Lab-Arena / PhysX): relamp a 2-ft T8 strip fixture under a shelf. A dead
F17T8 lamp is locked in the fixture's two rotor-lock G13 lampholders; it must be turned a quarter turn, which frees it
(the lampholder slots open downward), lowered out of the slots and laid in the lamp-recycling carton on the bench,
without dropping it.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1500 --store-frames --log-dir <dir>

The bench is custom-spawned PhysX bodies (``tubekit.py``: the fixture, shelf and lampholders as a fixed-base
articulation with the lamp on a jointed chain held by the lampholder model until it is out; the carton). The scorer
(``../src/scoring.py``) runs once per control step; ``score()`` returns its level at the end of the trial; the trial
ends as soon as the full-score state is reached (inside the scorer). Start states: a reset with a seed (inspect-robots
always passes one) places the fixture (with its shelf) 360-435 mm forward, +-40 mm sideways, 270-330 mm up and turned
4-8 deg either way, redrawn when a replayed default trajectory could still pinch the lamp, and the carton +-20 mm /
+-40 mm / +-6 deg (layout.setup_spec); unseeded resets use the named setups in turn (TUBE_SETUP picks the first and, when set, forces named setups). The full-score reference policy is
``official_run.py`` (registered as ``t8_tube_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from tubekit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 4 s hands-off hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = fixture origin; size only for yam_arena's checks).
OBJECTS = [Bench("bench", pos=(L.NOMINAL["fx"], L.NOMINAL["fy"], L.TOP + L.NOMINAL["fz"]), size=(0.1, 0.1, 0.1),
                 mass=None, color=(0.9, 0.9, 0.9))]


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
