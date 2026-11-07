from pathlib import Path
import sys,math
import numpy as np
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'runtime'))
from yam_arena.task_file import load,ObjectState
from yam_arena.demo import ArmPlan
from yam_arena import station
t=load(ROOT/'task.py');m=t.module
def state(pos):
    pos=np.array(pos,dtype=float);half=np.ones(3)*.01
    return ObjectState(pos,np.array([0.,0.,0.,1.]),np.zeros(3),np.zeros(3),pos-half,pos+half)
s={'marker':state((.13,.12,m.TOP+.011))}
m.reset_history();assert t.score(s)==0.
m._minimum=-np.array([.010,.003,.006]);m._maximum=-m._minimum;m._open=True
assert t.score(s)==1.
m._open=False;assert t.score(s)==.8
m._open=True;s['marker']=state((.13,0,m.TOP+.011));assert math.isclose(t.score(s),.6)
s['marker']=state((.13,.12,m.TOP+.011));m._maximum[1]=0.;assert math.isclose(t.score(s),.4)
m._maximum[:]=0.;assert t.score(s)==0.
m._violation=True;assert t.score(s)==0.
m.reset_history()
for arm,points in t.demo().items():
    plan=ArmPlan(points,station.BASES[arm])
    for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
    assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1 and plan.duration+3<60
    print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
print('Both stop coverage and correct identification and release checks passed')
