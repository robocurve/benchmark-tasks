"""Install the green suction cup through the plate and fit the white flow restrictor. Leave both installed and release them. Keep the arms clear of each other and the table."""
from yam_arena.task_file import Box
from assembly import scoring
SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=1.
OBJECTS=[Box('build_anchor',pos=(.24,0,.751),size=(.01,.01,.005),color=(.18,.22,.26)),
 Box('cup_rest',pos=(.38,-.09,.81375),size=(.022,.024,.1325),color=(.35,.38,.40)),
 Box('core_rest_back',pos=(.38,.080,.82050),size=(.008,.002,.146),color=(.35,.38,.40)),
 Box('core_rest_front',pos=(.38,.060,.82155),size=(.004,.002,.1481),color=(.35,.38,.40))]
def score(*a,**kw):return 0. if scoring.ACTIVE is None else scoring.ACTIVE.value
def success(*a,**kw):return False
def distance(*a,**kw):return 1.
def demo(*a,**kw):return {}
