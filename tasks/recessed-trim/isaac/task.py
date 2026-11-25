"""Recessed-light trim installation, for yam_arena (Isaac Lab-Arena / PhysX): lift the 6 in. torsion-spring baffle trim
off its carton, squeeze each spring's legs together and pass them up through the slot of its bracket in the housing
above the ceiling, then push the trim up until its flange sits flat on the ceiling, and let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 3000 --store-frames --log-dir <dir>

The rig is one custom-spawned PhysX assembly (``trimkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step robot-contact monitor; ``score()`` returns its level at the end
of the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the brackets' angle phi from 4..9 deg either
side of the robot's left-right line, the trim's yaw on its carton within +-5 deg and at least 4 deg away from phi, and
moves the carton by up to 15 mm; unseeded resets use the named setups in turn (see task.json; TRIM_SETUP picks the
first and, when set, forces named setups). The full-score reference policy is ``official_run.py`` (registered as
``recessed_trim_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from trimkit import STATE, TrimRig  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-released hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole rig (pose = carton top; size only for yam_arena's checks).
OBJECTS = [
    TrimRig(
        "rig",
        pos=tuple(float(v) for v in L.CARTON + [0.0, 0.0, L.CARTON_SIZE[2]]),
        size=(0.2, 0.2, 0.1),
        mass=None,
        color=(0.9, 0.9, 0.9),
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
