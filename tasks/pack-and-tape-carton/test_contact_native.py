import unittest
import numpy as np
from contact_native import NativeAdhesion

I=np.eye(3)
R=np.array([[1,0,0],[0,0,1],[0,-1,0]])
class Attribute:
 def __init__(self):self.value=True
 def Set(self,value):self.value=value
 def Get(self):return self.value
class Joint:
 def __init__(self):self.enabled=Attribute()
 def GetJointEnabledAttr(self):return self.enabled

class NativeContactTests(unittest.TestCase):
 def setUp(self):
  self.created=[]
  def factory(*args):
   joint=Joint();self.created.append((args,joint));return joint
  self.native=NativeAdhesion(None,'/test',joint_factory=factory)
  self.films={'tape_0':dict(pos=np.array((0,0,.001525)),rotation=R)}
  self.targets={'north_top':dict(pos=np.zeros(3),rotation=I,size=(.01,.048,.003))}
 def advance(self,n,force=(0,0,1)):
  for _ in range(n):self.native.step(.0005,self.films,self.targets,{('tape_0','north_top'):force})
 def test_actual_normal_force_and_continuous_dwell_required(self):
  self.advance(200,force=(1,0,0));self.assertFalse(self.created)
  self.advance(199);self.assertFalse(self.created)
  self.advance(1,force=(0,0,0));self.advance(199);self.assertFalse(self.created)
  self.advance(1);self.assertEqual(len(self.created),1)
  self.assertEqual(len(self.native.active_patches()[('tape_0','north_top')]),12)
 def test_gap_and_force_sign_cannot_create_adhesion(self):
  self.advance(250,force=(0,0,-1));self.assertFalse(self.created)
  self.films['tape_0']['pos'][2]+=.002
  self.advance(250);self.assertFalse(self.created)
 def test_measured_separation_irreversibly_breaks_and_removes_coverage(self):
  self.advance(200)
  self.films['tape_0']['pos'][2]+=.002;self.advance(1)
  self.assertFalse(self.created[0][1].enabled.value)
  self.assertFalse(self.native.active_patches())
  self.films['tape_0']['pos'][2]-=.002;self.advance(250)
  self.assertEqual(len(self.created),1)
 def test_disabled_native_joint_loses_credit_even_while_coincident(self):
  self.advance(200);self.created[0][1].enabled.Set(False);self.advance(1)
  self.assertFalse(self.native.active_patches())
 def test_control_frequency_dwell_rejected(self):
  with self.assertRaises(ValueError):self.native.step(.05,self.films,self.targets,{})
