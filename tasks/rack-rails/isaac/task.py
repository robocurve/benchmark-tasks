"""Server rail installation in a 12U four-post rack, for yam_arena (Isaac Lab-Arena / PhysX): take the closed sliding
rail from its cradle, hang it on the rack's near-side rear and front posts in the free U directly above the installed
1U server, both end latches engaged, and let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``rackkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step latch rule; ``score()`` returns its level at the end of the
trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the target U (U2, U3 or U4: the installed
server sits one U lower), moves the rack by up to 8 mm (x) and 6 mm (y) and the rail's cradle by up to 10 mm (x), 12 mm (y)
and 3 deg; unseeded resets use the named setups 0-4 in turn (layout.SETUPS; RACK_SETUP picks the first and, when set,
forces named setups). The full-score reference policy is ``official_run.py`` (registered as ``rack_rails_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from rackkit import Rack, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = rack hole column; size only for yam_arena's checks).
OBJECTS = [
    Rack(
        "rack",
        pos=(float(L.HOLE_COL_X), 0.0, float(L.u_center(3))),
        size=(0.5, 0.8, 0.1),
        mass=None,
        color=(0.1, 0.1, 0.11),
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
