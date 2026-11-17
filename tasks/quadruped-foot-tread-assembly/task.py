"""Fit the replacement tread cover and both retaining clamp bars onto the supported robot foot. Leave all parts secured and release them. Keep the arms clear of each other and the table."""
from yam_arena.task_file import Box
from assembly import scoring
SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=1.
OBJECTS=[Box('build_anchor',pos=(.24,0,.751),size=(.01,.01,.005),color=(.18,.22,.26)),
 Box('pad_rest',pos=(.29,-.18,.785),size=(.03,.03,.075),color=(.35,.38,.4)),
 Box('left_clamp_rest_0',pos=(0.360,0.095,.82475),size=(.004,.010,.1545),color=(.35,.38,.4)),
 Box('left_clamp_rest_1',pos=(0.400,0.095,.82475),size=(.004,.010,.1545),color=(.35,.38,.4)),
 Box('right_clamp_rest_0',pos=(0.360,-0.095,.82475),size=(.004,.010,.1545),color=(.35,.38,.4)),
 Box('right_clamp_rest_1',pos=(0.400,-0.095,.82475),size=(.004,.010,.1545),color=(.35,.38,.4)),]
def score(*a,**kw):return 0. if scoring.ACTIVE is None else scoring.ACTIVE.value
def success(*a,**kw):return False
def distance(*a,**kw):return 1.
def demo(*a,**kw):return {}
