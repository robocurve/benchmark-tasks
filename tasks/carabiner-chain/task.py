"""Three loose spring-gate accessory carabiners and an empty fixed D eye."""
from yam_arena.task_file import Box
from chain_task.geometry import layout,STAND_POS,GATE_OFFSET
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=.05
HISTORY=None
INITIAL=layout(17)
OBJECTS=[Box(name='stand',pos=tuple(STAND_POS),size=(.16,.12,.30))]
for i in range(3):
 d=INITIAL['link'+str(i)];p,r=d['position'],d['rotation'];OBJECTS.append(Box(name='link'+str(i),pos=tuple(p),size=(.037,.065,.0045),mass=.018));OBJECTS.append(Box(name='gate'+str(i),pos=tuple(p+r@GATE_OFFSET),size=(.0045,.039,.0045),mass=.002))
def score(objects):return 0. if HISTORY is None else HISTORY.score
def success(objects):return HISTORY is not None and not HISTORY.invalid and HISTORY.score==1.
