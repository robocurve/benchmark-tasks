"""Function-test two recessed spring plungers in an unpowered service fixture.

Use the pin tool to press each recessed orange contact face at least 8 mm for
0.3 seconds. Release both plungers to their rest positions, park the tool in
its stand and withdraw with both grippers open. Do not contact pins directly.
"""
from yam_arena.task_file import Box,TABLE_TOP
from plunger_gauge.scoring import History,evaluate
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Use the pin tool to test both recessed spring plungers. Press each at least 8 mm, release both, return the tool to its stand, and open both grippers.'
OBJECTS=[Box('punch',pos=(.20,.27,TOP+.070),size=(.026,.026,.082),mass=.08),
 Box('punch_stand',pos=(.20,.27,TOP),size=(.07,.07,.055)),
 Box('fixture',pos=(.35,.09,TOP),size=(.12,.23,.051))]
for i,y in enumerate((.04,.14)):OBJECTS.append(Box(f'plunger_{i}',pos=(.35,y,TOP+.030),size=(.01,.01,.0122),mass=.03))
HISTORY=History();SCORE_OVER='end';SUCCESS_HOLD_S=1.
def score(objects):return evaluate(objects,HISTORY,TOP)
def success(objects):return score(objects)==1.
def demo():
 from plunger_gauge.policy import make_plan
 return make_plan(TOP)
