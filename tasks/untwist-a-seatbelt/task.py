"""Affordable full-size empty-seat mock-up, with colliding seat/pillar surfaces."""
SCENE='station'
from yam_arena.task_file import Box
from installation import BOXES,world
from model import CENTER,SPAN
OBJECTS=[Box(name,pos=pos,size=size,color=color) for name,pos,size,color in BOXES]
OBJECTS += [Box('lower_belt_anchor',pos=tuple(world(CENTER+[0,-SPAN/2,0])),size=(.04,.06,.025),color=(.60,.62,.65)),
 Box('shoulder_guide_mount',pos=tuple(world(CENTER+[0,SPAN/2,0])+[.025,.025,.020]),size=(.040,.040,.025),color=(.60,.62,.65)),
 Box('locked_retractor',pos=(.680,.155,.530),size=(.070,.065,.090),color=(.12,.13,.15))]
def success(objects):return False
