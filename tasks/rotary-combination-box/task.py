"""Set the three numbered wheels to 3–1–7, then open the lid.

Three independent physical revolute wheels with equal soft detents. The hinged
lid is locked by its angular limit until all measured angles match the code.
This is a documented latch approximation, not a commercial lock's internals.
"""
from yam_arena.task_file import Box
from yam_arena.station import TABLE_TOP as TOP
from rotary_box.scoring import History,PITCH

SCENE='station'
SCORE_OVER='end'
CODE=(3,1,7)
INITIAL=(2,0,8)
CENTERS=[(.19,y,TOP+.040) for y in (.12,0,-.12)]
HINGE=(.25,0,TOP+.065)
LID_CENTER=(.32,0,TOP+.065)
OBJECTS=[Box(name='box',pos=(0,0,TOP),size=(.26,.4,.06),color=(.10,.24,.35)),Box(name='lid',pos=LID_CENTER,size=(.14,.40,.008),mass=.12,color=(.10,.24,.35))]
for i,(pos,digit) in enumerate(zip(CENTERS,INITIAL)):
    OBJECTS.append(Box(name=f'wheel_{i}',pos=pos,size=(.07,.07,.027),mass=.08,color=(.5,.55,.6),yaw=digit*PITCH))
HISTORY=History(CODE,CENTERS)

def score(objects):return HISTORY.score()
def demo():
    from rotary_box.policy import make_plan
    return make_plan(TOP,CENTERS,INITIAL,CODE)
