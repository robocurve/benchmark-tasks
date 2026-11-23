"""My heel is sore. I can lift my leg for a moment. Please settle my leg comfortably with the cushion, then give me room."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from isaac.bridge import install
from isaac.readout import ORIGIN
from yam_arena.task_file import Box

SCENE = "station"
OBJECTS = [Box("heel_fixture", pos=tuple(ORIGIN), size=(0.001, 0.001, 0.001))]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05
monitor = None


def score(objects):
    return monitor.score if monitor else 0.0


def success(objects):
    return bool(monitor and monitor.tick == 3360 and monitor.score == 1)


install(Path(__file__).resolve())
