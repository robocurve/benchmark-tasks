"""Ordinary joint/gripper target plan. No simulated object writes.
Actual native contact/force must validate the top-card drag and extraction.
CPU plan uses nominal poses only; no scripted card motion is executed.
"""
import numpy as np
import geometry as g
from ordinary_arm import Arm,wait,width
from yam_arena import station,ik
from yam_arena.constants import DOWN_XYZW,READY
from yam_arena.urdf import quat_xyzw_to_matrix

def phases():
 down=quat_xyzw_to_matrix(np.array(DOWN_XYZW));down=np.linalg.svd(down)[0]@np.linalg.svd(down)[2]
 pinch=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
 # Geometry audit may select a more appropriate yaw before freeze.
 hold=g.ORIGIN+np.array([-.025,.044,.008]);side=np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])
 top=g.ORIGIN+np.array([-.018,0,g.ROOF+.0128225-.0040]);push=np.diag([1.,-1.,-1.])
 lr=ik.tcp_pose(np.array(READY),station.BASES['left']);rr=ik.tcp_pose(np.array(READY),station.BASES['right'])
 return [('left','case_approach',hold+np.array([0,.085,.08]),side,.032,2.),
 ('left','case_grasp',hold,side,.022,2.),
 ('left','case_hold',hold,side,0.0,1.),
 ('right','thumb_approach',top+np.array([0,0,.035]),push,0.,2.),
 ('right','thumb_contact',top,push,0.,2.),
 ('right','drag_top_card',top+np.array([.036,0,0]),push,0.,3.),
 ('right','thumb_lift',top+np.array([.036,0,.020]),push,.04,1.5),
 ('right','thumb_retreat',top+np.array([.036,-.150,.020]),push,.040,2.),
 ('right','edge_rotate',g.TRAY+np.array([.037,-.031,.060]),pinch,.025,2.),
 ('right','edge_pinch_approach',g.ORIGIN+np.array([.093,-.070,g.ROOF-.00038]),pinch,.025,2.),
 ('right','edge_pinch',g.ORIGIN+np.array([.073,-.031,g.ROOF-.00038]),pinch,0.,2.),
 ('right','extract',g.ORIGIN+np.array([.140,-.031,g.ROOF-.00038]),pinch,0.,3.),
 ('right','card_above_mat',g.TRAY+np.array([.037,-.031,.025]),pinch,0.,2.),
 ('right','prepare_release',g.TRAY+np.array([.037,-.031,.005]),pinch,0.,1.5),
 ('right','release',g.TRAY+np.array([.037,-.031,.005]),pinch,.015,1.),
 ('right','right_retreat',g.TRAY+np.array([.037,-.031,.065]),pinch,.015,1.5),
 ('right','right_clear',rr[0],rr[1],.094,2.),
 ('left','let_go',hold,side,.030,1.),
 ('left','left_clear',lr[0],lr[1],.094,2.)]

JOINT_FREE={'case_approach','thumb_approach','edge_rotate','edge_pinch_approach','card_above_mat','right_clear','left_clear'}

def run():
 import os
 cheat = os.environ.get('CARD_CHEAT')
 arms={n:Arm(n,station.BASES[n])for n in ['left','right']}
 for side,name,p,R,w,t in phases():
  if cheat == 'noop':
   break
  if cheat == 'no_drag' and name == 'drag_top_card':
   break
  if cheat == 'drag_only' and name == 'edge_rotate':
   break
  if cheat == 'no_release' and name == 'release':
   break
  print('REFERENCE',name,flush=True)
  method=arms[side].joint_move if name in JOINT_FREE else arms[side].move
  for _ in method(p,R,t,width(w)):
   yield np.r_[arms['left'].cmd,arms['right'].cmd]
 for _ in wait(2.5):yield np.r_[arms['left'].cmd,arms['right'].cmd]
