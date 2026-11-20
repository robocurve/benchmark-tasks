"""Exchange the spent cartridge and leave the trainer ready for repacking."""
import sys
from pathlib import Path
from yam_arena.task_file import Box

sys.path.insert(0, str(Path(__file__).resolve().parent))
import disposition_score as grading

SCENE = "station"
SCORE_OVER = "end"
OBJECTS = [Box("build_anchor", pos=(.48,.20,.748), size=(.005,.005,.001), color=(.3,.3,.3))]


def score(objects):
    return grading.CURRENT.score if grading.CURRENT is not None else 0.


def demo():
    return {}
