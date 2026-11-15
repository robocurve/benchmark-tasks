"""Fixed scene; the flexible strip is integrated by run.py for every policy."""
from yam_arena.task_file import Box
from forms import Form
SCENE='station'
OBJECTS=[Form('silicone_leg',pos=(.36,0,.81),radius=.060,length=.34,axis='y',color=(.72,.48,.32)),Box('leg_rest',pos=(.36,0.,.755),size=(.065,.35,.015),color=(.28,.29,.31))]
def success(objects):return False
