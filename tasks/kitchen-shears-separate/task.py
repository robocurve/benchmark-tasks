"""Separate real-scale take-apart shears through passive pivot contact geometry."""
import json,os,pathlib,sys
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import labkit
from labkit import part,on_reset,on_step
import geometry as g
import hands
from scoring import Observation,Scorer
import yam_arena.demo as _demo
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE='station'; TOP=TABLE_TOP[SCENE]; DT=.05; RUN_STEPS=1800
SUCCESS_HOLD_S=1e9  # record the whole placement and released finish
labkit.CONTACT_OFFSET=.0001

def meshes(solids):
    return [labkit.Mesh(points=s.vertices,tris=s.triangles,color=s.color,collide='hull') for s in solids]

HALVES=[part(f'shear_half_{i}',pos=(*g.PIVOT_XY,TOP),yaw=g.START_ANGLES[i],
             meshes=meshes(g.half(i)),mass=(.055,.050)[i],friction=.5) for i in range(2)]
TRAY=part('washing_tray',pos=(*g.TRAY_XY,TOP),meshes=meshes(g.tray()),mass=.220,friction=.6)
OBJECTS=[*HALVES,TRAY]
_scorer=Scorer(TOP); _step=0

@on_reset
def reset(env):
    global _step
    _step=0; _scorer.reset()

@on_step
def observe(env):
    global _step
    if abs(float(getattr(env.unwrapped,'step_dt',DT))-DT)>.001: raise RuntimeError('20Hz required')
    _step+=1
    states=[labkit._part_state(env,p) for p in OBJECTS]
    tips,centers,radii=hands.measure(env)
    _scorer.update(Observation(_step*DT,np.array([s.pos for s in states]),
                              np.array([s.rotation() for s in states]),tips,centers,radii))
    _feedback(_step*DT,states)
    if os.environ.get('SHEARS_DEBUG')=='1' and _step%10==0:
        print('SHEARSDBG '+json.dumps({'step':_step,'score':_scorer.result,'reason':_scorer.reason,
            'positions':[list(map(float,s.pos)) for s in states], 'rotations':[s.rotation().tolist() for s in states],
            'feedback':_pivot.diagnostic(), 'transfer_started':_transfer.started,
            'transfer_elapsed':_transfer.elapsed}),flush=True)

def score(objects): return float(_scorer.result)
def success(objects): return score(objects)==1.
def distance(objects): return float(np.linalg.norm(np.asarray(objects[HALVES[0].name].pos)-objects[HALVES[1].name].pos))


# Isolated table-supported diagnostic: no full demonstration/score claim.
from table_plan import object_plan,grasp_target
class Orient(float):
    def __new__(cls,rot):
        value=float.__new__(cls,0.);value.rot=np.asarray(rot);return value
_original=_demo.grip_rotation
if not getattr(_original,'_table_shears_orientation',False):
    def oriented(yaw=0.):return yaw.rot if hasattr(yaw,'rot') else _original(yaw)
    oriented._table_shears_orientation=True;_demo.grip_rotation=oriented

# -- visible-pose feedback: while the upper (slotted) half is held at its release angle, trim its
# commanded yaw until the halves are seen 90 deg apart, and its commanded position until its slot is seen centred on
# the head; the trims then stay fixed through the lift. It reads only the halves' poses (what a camera sees).
from pivot_feedback import PivotFeedback
_pivot=PivotFeedback()
from transfer import Transfer
_transfer=Transfer(TOP)
_FB={'on':None,'off':None}
_NOMINAL_P=np.array([*g.PIVOT_XY,TOP])
_NOMINAL_A=np.radians(-60.)
_NOMINAL_TCP,_NOMINAL_ROT=grasp_target(1,_NOMINAL_P,_NOMINAL_A)

def _feedback(t,states):
    if _FB['on'] is None or t<_FB['on']: return
    if _transfer.started is not None:
        _transfer.elapsed=t-_transfer.started
        return
    _pivot.update(t,[s.pos for s in states],[s.rotation() for s in states],_NOMINAL_TCP,_NOMINAL_ROT)
    if _pivot.lift_at is not None:
        hp=np.concatenate([s.vertices for s in g.half(0) if s.kind=='head'])
        sp=np.concatenate([s.vertices for s in g.half(1) if s.kind=='pivot'])
        head=g.world(hp,states[0].pos,states[0].rotation())
        ring=g.world(sp,states[1].pos,states[1].rotation())
        if ring[:,2].min()-head[:,2].max()>=.004 and abs(states[0].pos[2]-TOP)<.005:
            lower=grasp_target(0,_NOMINAL_P,np.radians(30.))
            upper=_pivot.target(_NOMINAL_TCP,_NOMINAL_ROT)
            _transfer.begin(t,[s.pos for s in states],[s.rotation() for s in states],[lower,upper])

@on_reset
def _reset_fb(env):
    _pivot.reset()
    _transfer.reset()

_original_target=_demo.ArmPlan.target
def _transfer_target(self,t):
    if _transfer.started is not None and not getattr(self,'_codex_offline',False):
        return _transfer.target(1 if self.base_pos[1]>0 else 0)
    return _original_target(self,t)
_demo.ArmPlan.target=_transfer_target

class _DynPos:
    def __init__(self,p,a): self.p,self.a=np.asarray(p,float),float(a)
    def __array__(self,dtype=None,copy=None):
        return np.asarray(_pivot.target(_NOMINAL_TCP,_NOMINAL_ROT)[0],dtype=dtype or float)

class _DynOrient(float):
    def __new__(cls,p,a):
        v=float.__new__(cls,0.); v.p,v.a=np.asarray(p,float),float(a); return v
    @property
    def rot(self): return _pivot.target(_NOMINAL_TCP,_NOMINAL_ROT)[1]

FEEDBACK_HOLD=16.0  # bounded alignment window; lifting depends on the seen gate, never the clock alone

def demo():
    """The table-supported plan, with the hold before the lift lengthened to FEEDBACK_HOLD and the upper half's
    targets from the hold on following the visible-pose trims."""
    plans={'left':[],'right':[]};phases=object_plan(TOP)
    phases[2]=dict(phases[2],duration=FEEDBACK_HOLD,label='hold at the release angle: trim yaw and centring as seen')
    t=5.0  # the two approach waypoints
    for k,ph in enumerate(phases):
        if k==2: _FB['on']=t+.3; _FB['off']=t+ph['duration']
        t+=ph['duration']
    plans={'left':[],'right':[]}
    for side,i in [('right',0),('left',1)]:
        tcp,hand=grasp_target(i,phases[0]['positions'][i],phases[0]['angles'][i])
        plans[side]+=[Waypoint(2.5,tcp+[0,0,.06],-.012,'approach side loop wall',Orient(hand)),
                      Waypoint(2.5,tcp,-.012,'lower one jaw inside loop',Orient(hand))]
    previous=phases[0]
    for k,phase in enumerate(phases):
        n=max(1,int(np.ceil(np.max(np.abs(phase['angles']-previous['angles']))/np.radians(1))))
        if k>=2: n=max(n,int(round(phase['duration']/.25)))  # short steps, so the trims show up promptly
        for f in np.linspace(0,1,n+1)[1:]:
            for side,i in [('right',0),('left',1)]:
                p=previous['positions'][i]*(1-f)+phase['positions'][i]*f
                a=previous['angles'][i]*(1-f)+phase['angles'][i]*f
                if i==1 and k>=2:  # the upper half from the hold on: follows the seen-pose trims
                    plans[side].append(Waypoint(phase['duration']/n,_DynPos(p,a),0.,phase['label'],_DynOrient(p,a)))
                else:
                    tcp,hand=grasp_target(i,p,a)
                    plans[side].append(Waypoint(phase['duration']/n,tcp,0.,phase['label'],Orient(hand)))
        previous=phase
    # The policy uses summed waypoint durations as its stopping clock, even when target() is dynamic.
    # Keep both plans alive through late alignment, the 37-second transfer, and the released finish.
    for side in plans:
        remaining=80.0-sum(w.duration for w in plans[side])
        if remaining>0:
            w=plans[side][-1]
            plans[side].append(Waypoint(remaining,w.pos,w.finger,'allow dynamic transfer to finish',w.yaw))
    return plans
