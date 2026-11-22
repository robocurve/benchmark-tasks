"""Remove the mechanical lock pin from the valve actuator and return the indicator to its closed mark."""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from valve_bridge import install
from valve_layout import ORIGIN
from valve_motion import demo as demo
from yam_arena.task_file import Box

SCENE = "station"
OBJECTS = [Box("valve_fixture", pos=ORIGIN, size=(0.001, 0.001, 0.001))]
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
