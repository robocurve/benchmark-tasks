import math
import numpy as np
import pytest
from pen_capping.scoring import RestMotion
from pen_capping.alignment import matrix

def test_translation_and_full_axis_spin_are_measured():
 m=RestMotion();p=np.zeros(3);r=np.eye(3)
 assert math.isinf(m.speed('pen',p,r,.05))
 assert m.speed('pen',p+np.array([.001,0,0]),r,.05)==pytest.approx(.02)
 # Rotating around its long axis changes no pen axis vector but must count.
 assert m.speed('pen',np.array([.001,0,0]),matrix(np.array([.02,0,0])),.05)==pytest.approx(.024)

def test_oscillation_cannot_cancel_across_intervals():
 m=RestMotion();m.speed('pen',[0,0,0],np.eye(3),.05)
 for x in [.002,0,.002,0]:assert m.speed('pen',[x,0,0],np.eye(3),.05)==pytest.approx(.04)

def test_micron_contact_pose_noise_is_rest_and_reset_clears_history():
 m=RestMotion();m.speed('pen',[0,0,0],np.eye(3),.05)
 for sign in [1,-1]*30:
  assert m.speed('pen',[sign*.000001,0,0],matrix(np.array([0,sign*.00001,0])),.05)<.015
 assert math.isinf(RestMotion().speed('pen',[0,0,0],np.eye(3),.05))

def test_independent_objects_and_pi_rotation():
 m=RestMotion();m.speed('a',[0,0,0],np.eye(3),.05)
 assert math.isinf(m.speed('b',[2,0,0],np.eye(3),.05))
 assert m.speed('a',[0,0,0],matrix(np.array([math.pi,0,0])),.05)==pytest.approx(.06*math.pi/.05)
