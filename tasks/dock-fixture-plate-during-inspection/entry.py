"""Install the changeover plate on the inspection fixture and leave both arms clear."""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from cover_bridge import install
from cover_geometry import ORIGIN
from cover_motion import demo as demo
from yam_arena.task_file import Box

SCENE = "station"
OBJECTS = [Box("cover_fixture", pos=ORIGIN, size=(0.001, 0.001, 0.001))]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05
monitor = None


def score(objects):
    if monitor is None:
        raise RuntimeError("Native observer missing")
    return monitor.trial.score


def success(objects):
    return score(objects) == 1.0 and monitor.trial.tick >= 7200


install(HERE / "entry.py")
