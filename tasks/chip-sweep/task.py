"""Clear five loose chips from three undercut T-slots into the green collection bay.

Use the T-slot rake, leave every chip inside the green collection bay, and
return the rake to its two supports. The machining fixture is unpowered.
"""
import numpy as np
from yam_arena.task_file import Box,TABLE_TOP
from chip_sweep.scoring import History,evaluate
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Use the T-slot rake to clear all five metal chips from the three undercut channels into the green collection bay. Return the rake to its supports and open both grippers.'
OBJECTS=[Box('rake',pos=(.24,.27,TOP+.048),size=(.2475,.025,.095),mass=.08),
 Box('chip_tray',pos=(.35,.13,TOP),size=(.36,.20,.036)),
 Box('handle_support',pos=(.18,.27,TOP+.028),size=(.025,.055,.056),color=(.22,.28,.31)),
 Box('head_support',pos=(.34,.27,TOP+.014),size=(.025,.075,.028),color=(.22,.28,.31))]
CHIPS=[(.405,-.055,0.),(.455,-.055,.15),(.435,0.,-.2),(.405,.055,.2),(.475,.055,-.15)]
for i,(x,y,yaw) in enumerate(CHIPS):OBJECTS.append(Box(f'chip_{i}',pos=(x,y+.13,TOP+.015),size=(.012,.008,.006),yaw=yaw,mass=.006,color=(.6,.64,.67)))
HISTORY=History();SCORE_OVER='end';SUCCESS_HOLD_S=1.
def score(objects):return evaluate(objects,HISTORY,TOP)
def success(objects):return score(objects)>=1.
def demo():
 from chip_sweep.policy import make_plan
 return make_plan(TOP)
