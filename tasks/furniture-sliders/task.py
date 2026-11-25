"""Fit a reusable slider under each leg, then slide the table into the outlined destination.
Leave it standing on all four sliders and let go.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from assets import build
import adapter

SCENE = 'station'
SUCCESS_HOLD_S = 0.
SCORE_OVER = 'end'
OBJECTS = build()


def success(objects):
    return adapter.monitor.success


def score(objects):
    return adapter.monitor.score
