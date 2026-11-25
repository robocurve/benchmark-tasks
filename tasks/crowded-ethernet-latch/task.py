"""Move the orange cable from the lower marked port to the upper marked port.

Press its normal spring clip before withdrawing; reinsert until the clip
re-engages, release it, and leave all six other cables connected. Use run.py
for native retention geometry, latch springs, flexible chains and scoring.
"""
from yam_arena.task_file import Box
from config import *
SCENE='station';SUCCESS_HOLD_S=0.;SCORE_OVER='end'
OBJECTS=[Box('switch',pos=(0,0,0),size=(.01,.01,.01)),
         Box('mat',pos=(.30,0,TOP+.001),size=(.58,.60,.002),color=(.24,.27,.30))]
for i in USED:
    p=port(i)
    OBJECTS.extend([Box(f'plug_{i}',pos=p,size=(.024,.012,.008),mass=.008),
                    Box(f'clip_{i}',pos=(p[0],p[1],p[2]+CLIP_Z),size=(.018,.005,.001),mass=.0003)])
    points=cable_points(i)
    for k,(a,b) in enumerate(zip(points,points[1:])):
        OBJECTS.append(Box(f'wire_{i}_{k}',pos=tuple((u+v)/2 for u,v in zip(a,b)),
                           size=(.016,.005,.005),mass=.001))
_tracker=None
_diagnostic=True
def score(objects):return 0. if _diagnostic or _tracker is None else _tracker.value
def success(objects):return score(objects)==1.
