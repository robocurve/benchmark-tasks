"""Known-layout supported regrasp feasibility route, not an autonomous decision.

The initial grasp and later recovery are prescribed in this local probe. RGB
correction after regrasp uses the same observations as the end_visual controller.
"""
import json
from pathlib import Path
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from . import layout as L


def prefix(central_points):
    pick=np.array(L.HOME)+L.GRASPS['central']+[0,0,L.TCP_HEIGHT]
    goal=np.array(L.GOAL)+L.GRASPS['central']+[0,0,L.TCP_HEIGHT]
    hover=max(pick[2],goal[2])+.055
    return central_points[:7]+[
        Waypoint(8.5,tuple(goal+[0,0,.008]),-.006,'inspect central grasp; observer withdraws'),
        Waypoint(2.,(*goal[:2],hover),-.006,'raise before returning to support'),
        Waypoint(2.,(pick[0],goal[1],hover),-.006,'return outside screen post'),
        Waypoint(2.,(*pick[:2],hover),-.006,'return above blue support'),
        Waypoint(2.,tuple(pick),-.006,'lower onto blue support'),
        Waypoint(1.,tuple(pick),FINGER_OPEN,'release on blue support'),
        Waypoint(1.5,tuple(pick),FINGER_OPEN,'settle on blue support'),
        Waypoint(2.,(*pick[:2],hover),FINGER_OPEN,'clear plate before changing grasp'),
    ]


def initial_observer():
    return np.array(json.loads((Path(__file__).resolve().parents[1]/'observers/central.json').read_text())['q'])
