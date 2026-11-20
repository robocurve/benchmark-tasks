"""SO-101 joint 1 for yam_arena (Isaac Lab-Arena / PhysX): slide the printed shoulder bracket over both horns of the
base servo, fully home, with its top screw holes lined up over the top horn's holes, then let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``jointkit.py``): official SO-101 meshes (Apache-2.0) for the clamped
base and servo, the top horn fixed at its start angle, the bottom horn on a revolute joint, the bracket as a
free rigid body. The scorer (``../src/scoring.py``) runs once per control step on measured poses; ``score()`` returns
its level at the end of the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
The full-score reference policy is ``official_run.py`` (registered as ``so101_joint_scripted``).
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from jointkit import STATE, Joint1  # noqa: E402  (installs the yam_arena hooks)
import layout as L  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s release-and-still hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = servo axis; size only for yam_arena's checks).
OBJECTS = [Joint1("joint1", pos=tuple(float(v) for v in L.O_S), size=(.11, .11, .13), mass=None,
                  color=(1.0, .82, .12))]


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
