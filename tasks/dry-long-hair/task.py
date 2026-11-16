"""Dry every side and inner layer of the long-haired mannequin with the dryer.

Operate the real slider, expose wet regions by moving locks, avoid excessive
heating, then switch off and put the dryer down. Use run.py for articulated
locks, geometry-dependent heating, actual switch telemetry and scoring.
"""
from yam_arena.task_file import Box
from config import *
SCENE='station';SUCCESS_HOLD_S=0.;SCORE_OVER='end'
OBJECTS=[Box('mannequin',pos=(0,0,0),size=(.01,.01,.01)),
 Box('scalp_collision',pos=HEAD,size=(HEAD_RADIUS*2,)*3,mass=.05),
 Box('stand',pos=(HEAD[0],HEAD[1],TOP+.06),size=(.11,.11,.12),color=(.3,.32,.35)),
 Box('dryer_holder',pos=(0,0,0),size=(.01,.01,.01)),
 Box('dryer',pos=DRYER_START,size=(.12,.06,.14),mass=.48),
 Box('switch',pos=SWITCH_START,size=(.014,.010,.006),mass=.003),
 Box('dryer_pad',pos=(.23,0.,TOP+.001),size=(.18,.14,.002),color=(.75,.2,.35)),
 Box('mat',pos=(.32,0,TOP+.001),size=(.58,.60,.002),color=(.24,.27,.30))]
_tracker=None
_diagnostic=True
def score(objects):return 0. if _diagnostic or _tracker is None else _tracker.value
def success(objects):return score(objects)==1.
