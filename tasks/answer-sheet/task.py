"""Pick up the pen and mark A for question 1, C for question 2, D for question 3, and B for question 4.

The sheet is fixed flat on the table. Four numbered rows each have A-D boxes.
An uncapped marker rests to the right. Ink is a permanent visual approximation
driven only by measured nib-to-paper contacts. Moving above the sheet makes no
ink. A dot earns no credit. An answer in any other box invalidates its row.
"""
from yam_arena.task_file import Box
from yam_arena.station import TABLE_TOP as TOP
from answer_sheet.scoring import Ink
from answer_sheet.layout import BOXES, ANSWERS, FORM_CENTER, FORM_SIZE

SCENE = 'station'
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 1.
PAPER_TOP = TOP + .001
OBJECTS = [
    Box(name='paper',pos=(*FORM_CENTER,TOP+.0005),size=(*FORM_SIZE,.001),color=(.96,.96,.94)),
    Box(name='marker',pos=(.24,-.24,TOP+.010),size=(.13,.02,.02),mass=.025,color=(.94,.94,.94)),
]
INK = Ink(BOXES,ANSWERS)

def score(objects):
    return INK.score()

def demo():
    return {}
