"""Remove the orange service key and leave the shaft at its resting mark."""
import math
import os
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from yam_arena.task_file import Box,TABLE_TOP
from shaft_bridge import install
SCENE="station"
X=.335
OBJECTS=[Box("shaft_trainer",pos=(X,0,TABLE_TOP[SCENE]),size=(.001,.001,.001))]
SUCCESS_HOLD_S=3.6
SCORE_OVER="end"
monitor=None
def score(objects):
    if monitor is None:raise RuntimeError("Missing YAM physics observer")
    return float(monitor.trial.score)
def success(objects):return score(objects)==1.
from shaft_motion import demo
install(HERE/"entry.py")
