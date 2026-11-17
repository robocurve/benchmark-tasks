"""Wrist form supported at its ends, leaving the bracelet path unobstructed."""
from yam_arena.task_file import Box
from forms import Form
from bracelet_physics import Bracelet
from bracelet_geometry import END_Y
SCENE='station'
OBJECTS=[Box('clasp_rest_left',pos=(.34,END_Y,.7525),size=(.006,.010,.010),color=(.30,.40,.50)),
 Box('clasp_rest_right',pos=(.34,-END_Y,.7525),size=(.006,.010,.010),color=(.30,.40,.50)),Bracelet('bracelet',pos=(.34,0,.761)),Form('wrist_form',pos=(.34,0,.80),radius=.027,length=.22,axis='x',color=(.73,.50,.35)),
 Box('wrist_support_a',pos=(.235,0.,.760),size=(.025,.08,.025),color=(.30,.31,.33)),
 Box('wrist_support_b',pos=(.445,0.,.760),size=(.025,.08,.025),color=(.30,.31,.33))]
def success(objects):return False
