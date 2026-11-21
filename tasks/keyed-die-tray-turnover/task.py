"""Prepare a keyed tray of bare dies for backside inspection."""
from pathlib import Path
import sys
from yam_arena.task_file import Box

sys.path.insert(0, str(Path(__file__).resolve().parent))
import grading

SCENE = 'station'
SCORE_OVER = 'end'
OBJECTS = [Box('build_anchor', pos=(.48,.2,.748), size=(.005,.005,.001), color=(.3,.3,.3))]


def score(objects):
    if grading.CURRENT is None:
        raise RuntimeError('Start through run.py to install the native jig and contact scorer.')
    return grading.CURRENT.score


def demo():
    return {}
