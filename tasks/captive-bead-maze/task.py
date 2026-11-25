"""A captive bored wooden bead follows a bent steel wire through native contact only.
Reproducible DIY toddler-style maze, not asserted to be a measured retail product.
"""
from collections import deque
import math, pathlib, sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
import meshkit as mk
from partkit import finger_points
from yam_arena.task_file import TABLE_TOP
SCENE='station'
TOP=TABLE_TOP[SCENE]
INSTRUCTION='Move the red wooden bead to the green end of its wire maze, leaving it there with both hands clear.'
# Coordinates relative to the wire origin 12 mm above the table; bend radius 50 mm.
pts=[]
for s in np.linspace(0,.05,26):pts.append([s,0,.10])
for a in np.linspace(-math.pi/2,0,61)[1:]:pts.append([.05+.05*math.cos(a),.05+.05*math.sin(a),.10])
for a in np.linspace(0,math.pi/2,61)[1:]:pts.append([.10,.05+.05*math.sin(a),.15-.05*math.cos(a)])
for a in np.linspace(math.pi,math.pi/2,61)[1:]:pts.append([.15+.05*math.cos(a),.10,.15+.05*math.sin(a)])
for x in np.linspace(.15,.23,41)[1:]:pts.append([x,.10,.20])
OFFSET=np.array([.22,-.23,TOP+.012])
PATH=np.asarray(pts)+OFFSET
START=PATH[16]
TARGET=PATH[-18]
# Round wooden bead, 30 mm OD, reamed straight 10 mm bore. Lathed axis initially x.
zmax=math.sqrt(.015**2-.005**2)
profile=[(math.sqrt(.015**2-z*z),float(z)) for z in np.linspace(-zmax,zmax,65)]
profile += [(.005,zmax),(.005,-zmax)]
BEAD=mk.mesh_part('bead',pos=tuple(map(float,START)),mass=.010,quat_xyzw=(0,math.sqrt(.5),0,math.sqrt(.5)),
    shapes=[mk.Shape(mk.revolve(profile,segments=96),color=(.85,.05,.03),sdf_resolution=256)],
    friction=.3,contact_offset=.0001,angular_damping=.1,solver_iterations=32)
WIRE=mk.mesh_part('wire',pos=(0,0,0),shapes=[mk.Shape(mk.tube(PATH,.002,sides=16),color=(.7,.7,.72),collider='exact')],friction=.3,contact_offset=.0001)
BASE=mk.mesh_part('base',pos=(.335,-.18,TOP),shapes=[mk.Shape(mk.box((.30,.22,.012)).moved((0,0,.006)),color=(.65,.44,.2),collider='hull')])
# End blocks close the path physically. Both ends are fixed into supports.
SUPPORTS=[]
for name,p,color in [('start',PATH[0],(.8,.2,.1)),('finish',PATH[-1],(.1,.75,.15))]:
    h=p[2]-TOP
    SUPPORTS.append(mk.mesh_part(name,pos=(float(p[0]),float(p[1]),float(TOP)),shapes=[mk.Shape(mk.box((.025,.025,h)).moved((0,0,h/2)),color=color,collider='hull')],contact_offset=.0001))
OBJECTS=[BASE,WIRE,BEAD,*SUPPORTS]
_STATE={'history':deque(maxlen=20),'invalid':False,'progress':0.0}
def clear(pos):
    e=mk.env()
    return e is None or all(np.linalg.norm(np.asarray(p)-pos)>.045 for p in finger_points(e))
def update(pos,released=True):
    p=np.asarray(pos)
    if p[2]<TOP or min(np.linalg.norm(PATH-p,axis=1))>.035:_STATE['invalid']=True
    if np.linalg.norm(p-TARGET)<.012 and released:_STATE['history'].append(p.copy())
    else:_STATE['history'].clear()
def still():
    h=_STATE['history']
    return len(h)==20 and max(np.linalg.norm(p-h[0]) for p in h)<=.001
@mk.on_reset
def reset(e):
    _STATE.update(history=deque(maxlen=20),invalid=False,progress=0.0)
@mk.on_step
def tick(e):
    from yam_arena.scene import object_states
    p=object_states(e,['bead'])['bead'].pos
    update(p,clear(p))
def success(objects):
    p=objects['bead'].pos
    return not _STATE['invalid'] and np.linalg.norm(p-TARGET)<.012 and clear(p) and still()
def score(objects):
    if _STATE['invalid']:return 0.0
    if success(objects) and still():return 1.0
    p=objects['bead'].pos
    if np.linalg.norm(p-TARGET)<.012 and clear(p):return .75
    i=int(np.argmin(np.linalg.norm(PATH-p,axis=1)))
    return .5 if i>=146 and clear(p) else .25 if i>=86 and clear(p) else 0.0
def distance(objects):return float(np.linalg.norm(objects['bead'].pos-TARGET))
def demo():return {}
