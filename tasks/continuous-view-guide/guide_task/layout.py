"""Prototype geometry only; all coordinates are world metres."""
import math
import numpy as np
from yam_arena.task_file import Box, TABLE_TOP

TOP = TABLE_TOP['station']
CENTER = np.array([.22, -.10])
RADIUS = .090
HANDLE_OFFSET = .060
HEIGHT = TOP + .0825
START = math.radians(-10)
FINISH = math.radians(100)
ORANGE = (.95,.32,.025)
CYAN = (.02,.75,.85)
PINK = (.95,.04,.45)
WHITE = (.86,.88,.90)
HANDLE_SIZE = (.032,.024,.020)
APERTURE = .048
DEPTH = .080

def point(angle, radius=RADIUS, height=HEIGHT):
    return np.r_[CENTER + radius*np.array([math.cos(angle),math.sin(angle)]),height]

def handle(angle, height=HEIGHT):
    return point(angle,RADIUS+HANDLE_OFFSET,height)

HOME = tuple(map(float,handle(START)))
GOAL = tuple(map(float,handle(FINISH)))
FIXED = []
for i,(a,b) in enumerate(zip(np.linspace(math.radians(-45),math.radians(135),73)[:-1],
                              np.linspace(math.radians(-45),math.radians(135),73)[1:])):
    pa,pb=point(a),point(b)
    FIXED.append(Box(f'guide_{i:02d}',pos=tuple((pa+pb)/2),
                     size=(.008,float(np.linalg.norm(pb-pa))+.0004,.008),
                     yaw=(a+b)/2,color=CYAN))
for name,a in [('start',math.radians(-45)),('end',math.radians(135))]:
    pos=point(a)
    FIXED.append(Box(f'anchor_{name}',pos=(pos[0],pos[1],(TOP+HEIGHT)/2),
                     size=(.012,.012,HEIGHT-TOP),color=(.27,.32,.36)))
for name,a,color in [('start',START,(.15,.32,.55)),('goal',FINISH,(.12,.65,.20))]:
    pos=point(a)
    height=HEIGHT-TOP-.030
    FIXED.append(Box(f'dock_{name}',pos=(pos[0],pos[1],TOP+height/2),
                     size=(.060,.080,height),yaw=a,color=color))
FIXED.append(Box('base',pos=(*CENTER,TOP+.004),size=(.32,.32,.008),color=(.43,.46,.50)))
from dataclasses import replace
FIXED=[replace(o,pos=tuple(map(float,o.pos)),size=tuple(map(float,o.size)),yaw=float(o.yaw)) for o in FIXED]
OBJECTS=[*FIXED,Box('collar',pos=HOME,size=HANDLE_SIZE,mass=.080,yaw=START,color=ORANGE)]
PROTECTED=tuple(o.name for o in FIXED if o.name.startswith(('guide_','anchor_')))

# Four walls form a closed, 48 mm square through-aperture, 80 mm long.
# The root body is the graspable handle, 60 mm radially outside the aperture centre.
CHILDREN=[
 ('wall_inner',(-.087,0,0),(.006,DEPTH,.060),WHITE,True),
 ('wall_outer',(-.033,0,0),(.006,DEPTH,.060),WHITE,True),
 ('wall_top',(-.060,0,.027),(APERTURE,DEPTH,.006),WHITE,True),
 ('wall_bottom',(-.060,0,-.027),(APERTURE,DEPTH,.006),WHITE,True),
 ('handle_bridge',(-.022,0,0),(.028,.016,.014),ORANGE,True),
]
for sign in (-1,1):
    for x in (-.087,-.033):
        for z in (-.027,.027):
            CHILDREN.append((f'fid_{sign}_{x}_{z}'.replace('-','n').replace('.','p'),
                             (x,sign*(DEPTH/2+.0002),z),(.006,.0004,.006),PINK,False))

INSTRUCTION=('Move the orange-handled collar along the curved guide from the blue support to the green support, '
             'Leave the guide passing through the collar tunnel, and centre the collar upright on the green support, '
             'parallel to the support, release it, and leave it still for at least one second. '
             'Keep the collar clear of the guide and its anchors; keep both robots clear of the fixture '
             'and each other. Finger contact with the collar is allowed. Keep the fixture in place.')
