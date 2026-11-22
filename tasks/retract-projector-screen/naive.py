"""Comparison that releases the tab while the screen is still extended."""
from dataclasses import replace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from yam_arena.constants import FINGER_OPEN

from task import OBJECTS, SCENE, SCORE_OVER, SUCCESS_HOLD_S, demo as controlled, score, success


def demo():
    moves = controlled()["right"]
    release = replace(moves[4], duration=.25, finger=FINGER_OPEN, label="release under tension")
    return {"right": [*moves[:5], release]}
