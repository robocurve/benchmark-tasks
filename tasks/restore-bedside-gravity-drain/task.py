"""Two YAM arms at a water training station with a supine mannequin and a paused bedside gravity drain."""

import sys
from pathlib import Path

HERE = Path(__file__).resolve()
if str(HERE.parent) not in sys.path:
    sys.path.insert(0, str(HERE.parent))
from yam_arena.task_file import Box

from evd.native import install

SCENE = "station"
OBJECTS = [Box("water_trainer", (0.0, 0.0, 0.0), size=(0.001, 0.001, 0.001))]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05
monitor = None


def score(objects):
    if monitor is None:
        raise RuntimeError("Trainer monitor unavailable")
    return monitor.core.score


def success(objects):
    return score(objects) == 1


def demo():
    from evd.policy import make_plan

    return make_plan("reference")


install(HERE)
