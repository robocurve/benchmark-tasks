"""Counterexamples for three scorer defects without simulating robot motion."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np

folder=Path(__file__).resolve().parent
slug=folder.name
sys.path.insert(0,str(folder/'runtime'))
sys.path.insert(0,str(folder))
spec=importlib.util.spec_from_file_location('revised_task',folder/'task.py')
task=importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)

class Tensor:
    def __init__(self,value):self.value=np.asarray(value)
    def __getitem__(self,key):return Tensor(self.value[key])
    def item(self):return self.value.item()
    def detach(self):return self
    def cpu(self):return self
    def numpy(self):return self.value

class Robot:
    def __init__(self,q):self.data=SimpleNamespace(joint_pos=SimpleNamespace(torch=Tensor([q])))
    def find_joints(self,*args,**kwargs):return ([0,1],[])

class Hub(Robot):
    def find_joints(self,*args,**kwargs):return ([0],[])

def state(pos,low=None,yaw=0):
    pos=np.asarray(pos,dtype=float)
    return SimpleNamespace(pos=pos,low=np.asarray(low if low is not None else pos-[.02,.02,.02]),
        high=pos+[.02,.02,.02],quat_xyzw=np.array([0,0,np.sin(yaw/2),np.cos(yaw/2)]),
        lin_vel=np.zeros(3),ang_vel=np.zeros(3),resting=lambda *args:True)

robots={'left_robot':Robot([-.04,-.04]),'right_robot':Robot([-.04,-.04])}
if slug=='insert-retaining-fork-into-shaft-groove':
    task._fork_start_low=task.TOP+.040
    initial=state((.19,-.14,task.TOP+.0665),(.163,-.26,task.TOP+.040))
    assert task.score({'fork':initial})==0.,'Idle scene must score zero'
    lifted=state((.19,-.14,task.TOP+.0865),(.163,-.26,task.TOP+.060))
    assert task.score({'fork':lifted})==.25,'A genuine lift retains partial credit'
elif slug=='seat-keyed-coupling-and-prove-drive-transfer':
    actual_contact_failed=task.drive_contact.failed
    task.drive_contact.bind=lambda base:None
    task.drive_contact.failed=lambda:False
    task.reset_history()
    hub=Hub([0.])
    base=SimpleNamespace(scene={**robots,'hub_fixture':hub})
    seated=state(task.GOAL)
    task.observe({'coupling':seated},0,base)
    assert task._engaged
    lifted=state(task.GOAL+[0,0,.05])
    task.observe({'coupling':lifted},.05,base)
    assert not task._engaged,'Removing the coupling must clear engagement'
    hub.data.joint_pos.torch=Tensor([[np.pi/2]])
    turned=state(task.GOAL,yaw=np.pi/2)
    task.observe({'coupling':turned},.10,base)
    assert task.score({'coupling':turned})==0.,'Disengage and turn and reseat must fail'
    task.reset_history()
    hub.data.joint_pos.torch=Tensor([[0.]])
    task.observe({'coupling':seated},0,base)
    hub.data.joint_pos.torch=Tensor([[np.pi/2]])
    task.observe({'coupling':turned},.05,base)
    assert task.score({'coupling':turned})==1.,'Continuous engagement remains valid'
    task.drive_contact.failed=actual_contact_failed
    class ContactView:
        def get_contact_data(self,dt):return None,None,None,np.array([0.]),np.array([1]),np.array([0])
    task.drive_contact._base=base
    task.drive_contact._view=ContactView()
    task.drive_contact.poll(SimpleNamespace(get_physics_dt=lambda:1/240))
    task.observe({'coupling':turned},.10,base)
    assert task.score({'coupling':turned})==0.,'Direct robot drive on the lower rotor must fail'
else:
    task.reset_history()
    states={'rolling_cradle':state((.28,0,task.TOP+.045)),
            'load':state((.28,0,task.TOP+.155),(.2575,-.0225,task.TOP+.105)),
            'left_chock':state((.28,.1145,task.TOP+.036),(.22,.10,task.TOP)),
            'right_chock':state((.28,-.1145,task.TOP+.036),(.22,-.13,task.TOP))}
    base=SimpleNamespace(scene=robots)
    task.observe(states,0,base)
    assert task._restrained
    states['right_chock']=state((.38,-.1145,task.TOP+.036),(.32,-.13,task.TOP))
    task.observe(states,.05,base)
    assert not task._restrained,'Removing a chock must clear restraint'
    states['load']=state((.30,0,task.TOP+.155),(.2775,-.0225,task.TOP+.105))
    task.observe(states,.10,base)
    assert task.score(states)==0.,'Low horizontal removal without chocks must fail'
print('PASS',slug)
