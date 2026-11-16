"""Seal a preloaded carton with an ordinary loose tape roll.

The two products already start inside and the four top flaps are folded closed but untaped.
Peel the roll end, lay one connected strip along the seam and both end faces,
cut it using the guarded cutter, release everything and leave the seal holding.
No pre-laid tape, prepared leader or extra handles. Use run.py for mechanics.
"""
from yam_arena.task_file import Box
from carton_scene import layout
from config import *
SCENE='station';SUCCESS_HOLD_S=0.;SCORE_OVER='end'
PARTS,HINGES=layout()
OBJECTS=[Box(p['name'],pos=tuple(u+v for u,v in zip(CENTER,p['pos'])),size=p['size'],mass=p['mass'],color=(.58,.36,.17)) for p in PARTS]
OBJECTS.append(Box('roll',pos=ROLL_START,size=(.10,.10,.048),mass=.16,color=(.72,.56,.27)))
for i in range(CELLS):
 p,yaw=tape_cell(i);OBJECTS.append(Box(f'tape_{i}',pos=p,size=(CELL_LENGTH,TAPE_THICKNESS,TAPE_WIDTH),yaw=yaw,mass=950.*CELL_LENGTH*TAPE_THICKNESS*TAPE_WIDTH,color=(.74,.58,.28)))
for n,p,s,m,c in PRODUCTS:OBJECTS.append(Box(n,pos=p,size=s,mass=m,color=c))
OBJECTS.extend([Box('mat',pos=(.32,0,TOP+.001),size=(.58,.60,.002),color=(.24,.27,.30)),Box('cutter_pad',pos=(.36,-.22,TOP+.001),size=(.16,.07,.002),color=(.85,.3,.08)),Box('cutter',pos=CUTTER_START,size=(.11,.025,.025),mass=.055,color=(1.,.3,.025))])
_tracker=None
_diagnostic=True
def score(objects):return 0. if _tracker is None or _diagnostic else _tracker.value
def success(objects):return score(objects)==1.
