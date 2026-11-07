from pathlib import Path
import sys,copy,math
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'runtime'))
sys.path.insert(0,'/data/yam-isaac-lab-arena')
from yam_arena.task_file import load,ObjectState
from yam_arena.demo import ArmPlan
from yam_arena import station

t=load(ROOT/'task.py')
m=t.module
def state(o,pos=None):
    pos=np.array(o.pos if pos is None else pos,dtype=float); half=np.array(o.size)/2
    return ObjectState(pos,np.array([0.,0.,0.,1.]),np.zeros(3),np.zeros(3),pos-half,pos+half)
s={o.name:state(o) for o in t.objects}
assert t.score(s)==0
ideal=copy.deepcopy(s)
for n,g in m.GOALS.items(): ideal[n]=state(next(o for o in t.objects if o.name==n),g)
m._cleared={'blue_key','orange_key'}; m._open=True; m._stored_once=True
assert t.success(ideal) and t.score(ideal)==1
m._cleared=set()
assert t.score(ideal)==.5 and not t.success(ideal)
m.reset_history()
cheat=copy.deepcopy(s)
cheat['carrier'].pos[0]-=.02
m.observe(cheat,.05,None)
assert m._violation and t.score(ideal)==0
m.reset_history()
m._cleared={'blue_key','orange_key'};m._open=True;m._stored_once=True
bad=copy.deepcopy(ideal); bad['carrier'].quat_xyzw[:]=[0,0,math.sqrt(.5),math.sqrt(.5)]
assert not t.success(bad)
hover=copy.deepcopy(ideal)
for n in ['blue_key','orange_key']:
    hover[n].pos[2]+=.02; hover[n].low[2]+=.02; hover[n].high[2]+=.02
assert t.score(hover)==.5 and not t.success(hover)
m.reset_history()
unstored=copy.deepcopy(s)
unstored['carrier'].pos[0]-=.02
for n in ['blue_key','orange_key']:
    unstored[n].low[2]=m.TOP+.06
m.observe(unstored,.1,None)
assert m._violation
for arm,points in t.demo().items():
    plan=ArmPlan(points,station.BASES[arm])
    for time in np.arange(.05,plan.duration+.1,.05): plan.action7(float(time))
    print(arm,'mm',plan.worst_ik_mm,'degrees',plan.worst_ik_deg,'seconds',plan.duration,flush=True)
    assert plan.worst_ik_mm<2 and plan.worst_ik_deg<1
print('Transport key scorer and reach checks passed. Physics not yet verified.',flush=True)
