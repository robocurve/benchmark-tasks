"""Scoring counterexamples and measured reach checks for this task."""
from pathlib import Path
import copy,math,sys
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'runtime'))
from yam_arena.task_file import load,ObjectState
from yam_arena.demo import ArmPlan
from yam_arena import station
t=load(ROOT/'task.py');m=t.module
def state(pos,size):
    pos=np.array(pos,dtype=float);half=np.array(size,dtype=float)/2
    return ObjectState(pos,np.array([0.,0.,0.,1.]),np.zeros(3),np.zeros(3),pos-half,pos+half)
states={'cage':state(m.GOAL,(.052,.042,.048)),
        'rack_coupon':state(m.PANEL_START,(.08,.22,.08))}
m.reset_history()
assert not t.success(states),'An unobserved installation must not receive full score'
m._had_compression=True;m._q=np.zeros(2);m._open=True
for _ in range(20):m._recent.append(m.GOAL.copy())
assert t.score(states)==.5,'Earlier compression alone does not prove compressed passage'
m._crossed_compressed=True
for _ in range(20):m._recent.append(m.GOAL.copy())
assert t.score(states)==1.
m._open=False
assert t.score(states)==.75
m._open=True;m._q=np.array([-.006,.006])
assert not t.success(states),'Compressed hooks do not retain the panel'
m._q=np.zeros(2)
bad=copy.deepcopy(states);bad['cage'].quat_xyzw[:]=[0,math.sin(.2),0,math.cos(.2)]
assert not t.success(bad),'A tipped fastener must not pass'
bad=copy.deepcopy(states);bad['cage'].pos[2]+=.015
assert not t.success(bad),'A fastener above the panel must not pass'
bad=copy.deepcopy(states);bad['cage'].pos[0]=float('nan')
assert t.score(bad)==0.
m._violation=True
assert t.score(states)==0.
m.reset_history()
assert math.isclose(.052-.034,.018)
assert math.isclose(.010-.008/2-(-.003+.006/2),.006)
for arm,points in t.demo().items():
    plan=ArmPlan(points,station.BASES[arm])
    for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
    assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
    assert plan.duration+3<60
    print(arm,dict(worst_ik_mm=plan.worst_ik_mm,worst_ik_degrees=plan.worst_ik_deg,duration_s=plan.duration))
print('Scorer counterexamples and reach checks passed')
