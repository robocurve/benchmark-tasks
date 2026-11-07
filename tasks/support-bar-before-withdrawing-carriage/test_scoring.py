"""Reject carriage travel that does not remove support from the bar."""
from pathlib import Path
import sys,numpy as np
from types import SimpleNamespace
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
 def __init__(self,q):self.data=SimpleNamespace(joint_pos=SimpleNamespace(torch=Tensor([q])))
 def find_joints(self,*args,**kwargs):return ([0],[])
class Robot(Body):
 def find_joints(self,*args,**kwargs):return ([0,1],[])
def bar(y):
 pos=np.array([.26,y,task.TOP+.110])
 return SimpleNamespace(pos=pos,low=np.array([.165,y-.015,task.TOP+.080]),quat_xyzw=np.array([0.,0.,0.,1.]),resting=lambda *args:True)
carriage=Body([0.]);robots={name:Robot([-.04,-.04]) for name in ('left_robot','right_robot')}
base=SimpleNamespace(scene={**robots,'support_carriage':carriage})
task.reset_history();task.observe({'optical_bar':bar(0)},0,base)
carriage.data.joint_pos.torch=Tensor([[-.11]])
task.observe({'optical_bar':bar(-.11)},.05,base)
assert not task._cleared and task.score({'optical_bar':bar(-.11)})==0.,'Co-moving bar and carriage must not earn support removal credit'
task.reset_history();carriage.data.joint_pos.torch=Tensor([[0.]])
task.observe({'optical_bar':bar(0)},0,base)
carriage.data.joint_pos.torch=Tensor([[-.05]])
task.observe({'optical_bar':bar(0)},.05,base);assert task.score({'optical_bar':bar(0)})==.25
carriage.data.joint_pos.torch=Tensor([[-.11]])
task.observe({'optical_bar':bar(0)},.10,base);assert task._cleared and task.score({'optical_bar':bar(0)})==.5
print('PASS support relative clearance')
