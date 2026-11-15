"""Wrist form supported at its ends, leaving the bracelet path unobstructed."""
from yam_arena.task_file import Box
from forms import Form
SCENE='station'
OBJECTS=[Form('wrist_form',pos=(.34,0,.80),radius=.027,length=.22,axis='x',color=(.73,.50,.35)),
 Box('wrist_support_a',pos=(.235,0.,.760),size=(.025,.08,.025),color=(.30,.31,.33)),
 Box('wrist_support_b',pos=(.445,0.,.760),size=(.025,.08,.025),color=(.30,.31,.33))]
def success(objects):return False
