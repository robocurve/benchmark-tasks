# SPDX-License-Identifier: MIT
"""Floating passive captive slider and a fixed, restrictive low dock."""
from pathlib import Path
from yam_arena.task_file import Box
from captive_crossbar_docking.contract import *
SCENE = 'station'
SUCCESS_HOLD_S = 1.0
SCORE_OVER = 'end'
ASSETS = Path(__file__).parent / 'assets'
CARRIER_ASSET = str(ASSETS / 'carrier.usda')
COMPOUND_ASSETS = {'dock': str(ASSETS / 'dock.usda')}
FIXED_ASSETS = REFERENCE_NAMES
OBJECTS = [Box('dock', pos=DOCK_POS, size=DOCK_SIZE, color=GREY)]
def success(objects):
    return False
def score(objects):
    return 0.
