"""Transfer a loaded fixture pallet using the removable two-tine lifter.

Slide the fork under the pallet through its foot channels, lift the pallet,
carry it to the two green inspection rests, lower it onto the rests and
withdraw the fork. Return the lifter to its supports with open grippers.
"""
from yam_arena.task_file import Box,TABLE_TOP
from pallet_lifter.scoring import History,evaluate
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Use the forked lifter to move the loaded pallet onto the two green inspection rests. Withdraw and return the lifter to its supports, then open both grippers.'
OBJECTS=[Box('lifter',pos=(.24,.28,TOP+.035),size=(.305,.08,.025),mass=.08),
 Box('pallet',pos=(.40,.08,TOP+.023),size=(.12,.16,.038),mass=.12),
 Box('rests',pos=(.37,-.12,TOP),size=(.15,.158,.0166)),
 Box('handle_support',pos=(.18,.28,TOP+.02),size=(.025,.055,.04),color=(.20,.27,.30)),
 Box('tine_support',pos=(.34,.28,TOP+.0175),size=(.025,.075,.035),color=(.20,.27,.30))]
HISTORY=History();SCORE_OVER='end';SUCCESS_HOLD_S=1.
def score(objects):return evaluate(objects,HISTORY,TOP)
def success(objects):return score(objects)==1.
def demo():
 from pallet_lifter.policy import make_plan
 return make_plan(TOP)
