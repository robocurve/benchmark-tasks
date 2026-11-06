"""Third-link motion added only after the released pair-024 scored 2/3.

All assembly uses ordinary joint actions and physical finger/hinge contacts.
"""
from dataclasses import replace
import numpy as np
from chain_task.pair import PairPolicy,SecondClipPolicy

class ThirdClipPolicy(SecondClipPolicy):
 child="link2";parent="link0";child_side="right";parent_side="left"
 parent_yaw=80.;center=np.array([.34,-.16,.94])
 counter_diagonal=np.diag([-1.,-1.,1.])

class ChainPolicy(PairPolicy):
 def __init__(self,**kwargs):
  super().__init__(**kwargs);self.third=ThirdClipPolicy(**kwargs);self.info=replace(self.info,name="carabiner_chain_scripted")
 def bind(self,info):super().bind(info);self.third.bind(info)
 def reset(self,scene):super().reset(scene);self._third_started=False
 def act(self,o):
  if self._third_started:return self.third.act(o)
  action=super().act(o)
  if self._second_started and self.second.stopping:
   from chain_task.embodiment import FEEDBACK as f
   if float(f.get('score',0.))>=2/3:
    self.third.reset(self._scene);self._third_started=True
    print('CHAIN_RELEASED_PAIR_BEFORE_THIRD',o.state_time,f['score'],flush=True)
    return self.third.act(o)
  return action

def make_policy(**kwargs):return ChainPolicy(**kwargs)
