"""Counterexamples for premature pin return and final index capture."""
from pathlib import Path
from types import SimpleNamespace
import sys,numpy as np
root=Path(__file__).resolve().parent
sys.path[:0]=[str(root/'runtime'),str(root)]
import task
class Tensor:
 def __init__(self,value):self.value=np.asarray(value)
 def __getitem__(self,index):return Tensor(self.value[index])
 def item(self):return self.value.item()
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):return self.value
class Body:
 def __init__(self,q):self.set(q)
 def set(self,q):self.data=SimpleNamespace(joint_pos=SimpleNamespace(torch=Tensor([q])),joint_vel=SimpleNamespace(torch=Tensor([np.zeros(len(q))])))
 def find_joints(self,*args,**kwargs):return ([0],[])
class Robot(Body):
 def find_joints(self,*args,**kwargs):return ([0,1],[])
robots={'left_robot':Robot([-.04,-.04]),'right_robot':Robot([-.04,-.04])}
if root.name=='extend-training-handle-and-engage-detent':
 handle=Body([0.]);pin=Body([0.]);base=SimpleNamespace(scene={**robots,'adjustable_handle':handle,'locking_pin':pin})
 task.reset_history();task.observe({},0,base);assert task.score({})==0.
 pin.set([.05]);task.observe({},.05,base);assert task.score({})==.25
 handle.set([.04]);pin.set([0.]);task.observe({},.10,base);assert task.score({})==0.,'Premature pin return during extension must fail'
 task.reset_history();handle.set([0.]);pin.set([.05]);task.observe({},0,base)
 handle.set([.08]);task.observe({},.05,base);pin.set([0.]);task.observe({},.10,base);assert task.score({})==1.
else:
 hinge=Body([0.]);pin=Body([0.]);base=SimpleNamespace(scene={**robots,'footrest_module':hinge,'spring_latch':pin})
 task.reset_history();task.observe({},0,base);assert task.score({})==0.
 pin.set([.03]);task.observe({},.05,base);hinge.set([np.pi/4]);pin.set([0.]);task.observe({},.10,base);assert task.score({})==0.,'Premature latch return before target arrival must fail'
 task.reset_history();hinge.set([0.]);pin.set([.03]);task.observe({},0,base)
 hinge.set([np.pi/2]);task.observe({},.05,base);pin.set([0.]);task.observe({},.10,base);assert task.score({})==1.
 hinge.set([np.deg2rad(81.5)]);task.observe({},.15,base);assert not task._bad and task.score({})<1.,'Capture settling does not count as final success'
 hinge.set([np.pi/2]);task.observe({},.20,base);assert task.score({})==1.
 hinge.set([np.deg2rad(79)]);task.observe({},.25,base);assert task.score({})==0.,'Moving out of the capture band requires unlocking again'
print('PASS',root.name)
