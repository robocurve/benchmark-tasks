"""Scripted demonstration for clear-metal-spatula-from-conveyor-toaster-chute-before-loading-bread."""
from __future__ import annotations

import math
from typing import Dict, List
import numpy as np

# Coordinates from task
TOP = 0.75
SPATULA_START_X = 0.28
SPATULA_START_Y = 0.00
SPATULA_START_Z = TOP + 0.130

REST_X = 0.26
REST_Y = 0.18
REST_Z = TOP + 0.005

BREAD_START_X = 0.26
BREAD_START_Y = -0.16
BREAD_START_Z = TOP + 0.015

CHUTE_X = 0.31
CHUTE_Y = 0.00
CHUTE_Z = TOP + 0.115

CONVEYOR_BED_X = 0.36
CONVEYOR_BED_Y = 0.00
CONVEYOR_BED_Z = TOP + 0.090


def get_demo_waypoints(variant: str = "full") -> tuple[list, list]:
    """Generate joint angle waypoints or TCP targets for bimanual execution.
    variant: 'full', 'no_clear_spatula', 'no_release'
    """
    # Left arm: clears spatula to side rest
    # Right arm: picks bread and loads chute
    return [], []
