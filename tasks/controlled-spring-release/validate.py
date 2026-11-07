from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'runtime'))
from yam_arena.task_file import load,ObjectState
from yam_arena.demo import ArmPlan
from yam_arena import station
t=load(ROOT/'task.py');m=t.module
pos=np.array([.18,-.19,m.TOP+.011]);half=np.array([.015,.075,.010])
bar=ObjectState(pos,np.array([0.,0.,0.,1.]),np.zeros(3),np.zeros(3),pos-half,pos+half)
s={'retaining_bar':bar}
m.reset_history();assert t.score(s)==0.
m._bar_clear=True;m._q=.05019;m._speed=0.;m._open=True
assert t.score(s)==1.
m._open=False;assert t.score(s)==.75
m._open=True;m._q=.03;assert t.score(s)==.25
m._q=.05019;bar.low[1]=-.285;assert t.score(s)==.5
bar.low[1]=-.265;m._violation=True;assert t.score(s)==0.
m.reset_history()
for arm,points in t.demo().items():
    plan=ArmPlan(points,station.BASES[arm])
    for time in np.arange(.05,plan.duration+.1,.05):plan.action7(float(time))
    assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1 and plan.duration+3<60
    print(arm,plan.worst_ik_mm,plan.worst_ik_deg,plan.duration)
print('Unloaded state and return pad and latched safety checks passed')
