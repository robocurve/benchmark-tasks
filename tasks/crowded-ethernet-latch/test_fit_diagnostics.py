"""Candidate fit geometry and unforced measurement checks, not native evidence."""
import math
import unittest
import numpy as np
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from yam_arena import ik
from geometry import socket_boxes,object_obstacles,wire_local_frame,GUIDE_X_MIN,GUIDE_X_MAX,GUIDE_Y,GUIDE_WIDTH,GUIDE_BOTTOM,GUIDE_TOP
from diagnostics import connector_fit,fit_summary,bench_command,offcenter_clip_wrench,clip_constraint_summary
from config import *

class CandidateFitTests(unittest.TestCase):
 def test_guide_channels_leave_nominal_shoulder_and_clip_clearance(self):
  guides=[b for b in socket_boxes() if b[0].startswith('guide_')]
  self.assertEqual(len(guides),16)
  for name,center,size,_ in guides:
   self.assertAlmostEqual(center[0]-size[0]/2,.3995)
   self.assertAlmostEqual(center[0]+size[0]/2,.4255)
   self.assertAlmostEqual(size[1],.0025)
   self.assertAlmostEqual(size[2],.0025)
  self.assertAlmostEqual(GUIDE_BOTTOM-.004,.0003)
  self.assertAlmostEqual(2*(GUIDE_Y-GUIDE_WIDTH/2),.007)
  self.assertGreater(GUIDE_Y-GUIDE_WIDTH/2-.0025,.0009)
  self.assertAlmostEqual(GUIDE_TOP,.0073-.001/2)

 def test_all_native_body_colliders_require_explicit_wire_scope(self):
  plugs={i:np.r_[port(i),0,0,0,1] for i in USED}
  clips={i:np.r_[np.array(port(i))+[0,0,CLIP_Z],0,0,0,1] for i in USED}
  with self.assertRaisesRegex(ValueError,'Actual wire poses required'):object_obstacles(plugs,clips)
  wires={(i,k):np.r_[(np.array(cable_points(i)[k])+cable_points(i)[k+1])/2,0,0,0,1] for i in USED for k in range(LINKS)}
  obstacles=object_obstacles(plugs,clips,wires)
  self.assertEqual(sum(owner.startswith('wire_') for owner,_,_ in obstacles),len(USED)*LINKS)
  self.assertEqual(sum(name.startswith('guide_') for _,name,_ in obstacles),16)
  self.assertEqual(sum(owner.startswith('clip_') and name=='tab' for owner,name,_ in obstacles),7)
  self.assertEqual(sum(name=='boot_bound' for _,name,_ in obstacles),7)
  frame,height=wire_local_frame(SERVICE,0);a,b=cable_points(SERVICE)[:2]
  np.testing.assert_allclose(frame[:,2],(np.array(b)-a)/np.linalg.norm(np.array(b)-a),atol=1e-12)
  self.assertAlmostEqual(height,np.linalg.norm(np.array(b)-a))
  self.assertGreater(np.linalg.norm(frame[:,2]-[0,0,1]),.5)

 def test_fit_distinguishes_task_limits_and_engineering_margin(self):
  values={}
  for i in USED:
   plug=np.r_[port(i),0,0,0,1];clip=plug.copy();clip[2]+=CLIP_Z
   values[str(i)]=connector_fit(plug,clip)
  sample={'time':2.,'connector_fit':values}
  result=fit_summary([sample])
  self.assertEqual(len(result['connectors']),7);self.assertTrue(result['pass'])
  bad=USED[-1];angle=math.radians(4.);rotation=axis_angle_matrix(np.array((0.,1.,0.)),angle)
  plug=np.r_[port(bad),ik.matrix_to_quat_xyzw(rotation)];clip=np.r_[plug[:3]+rotation@[0,0,CLIP_Z-.00025],plug[3:]]
  values[str(bad)]=connector_fit(plug,clip)
  result=fit_summary([sample]);self.assertTrue(result['task_fit_pass'])
  self.assertFalse(result['engineering_margin_pass'])
  self.assertTrue(result['connectors'][str(bad)]['task_fit_pass'])
  self.assertAlmostEqual(result['connectors'][str(bad)]['max_abs_pitch_deg'],4.)
  self.assertAlmostEqual(result['connectors'][str(bad)]['max_abs_clip_travel_m'],.00025)
  values[str(bad)]['clip_travel_m']=.00035
  self.assertFalse(fit_summary([sample])['task_fit_pass'])

 def test_only_explicit_bench_diagnostics_can_request_external_loads(self):
  for mode in ('fit-probe','grasp-probe','release-probe'):
   self.assertTrue(all(bench_command(step,mode)[1:]==(0.,0.) for step in range(180)))
  self.assertEqual(bench_command(45,'retention-probe'),('pull_engaged',-1.,0.))
  self.assertEqual(bench_command(90,'retention-probe'),('pull_pressed',-1.,-.70))
  self.assertEqual(bench_command(90,'clip-constraint-probe'),('pull_pressed',-1.,-.50))

 def test_offcenter_force_includes_actual_tab_moment(self):
  plug=np.r_[port(SERVICE),0,0,0,1];clip=plug.copy();clip[2]+=CLIP_Z
  force,torque=offcenter_clip_wrench(plug,clip,-.5)
  np.testing.assert_allclose(force,[0,0,-.5]);np.testing.assert_allclose(torque,[.00075,-.005,0])
  rotation=axis_angle_matrix(np.array((1.,0,0)),.15);clip[3:]=ik.matrix_to_quat_xyzw(rotation)
  force,torque=offcenter_clip_wrench(plug,clip,-.5)
  np.testing.assert_allclose(torque,np.cross(rotation@[-.010,-.0015,.0005],force))
  self.assertGreater(np.linalg.norm(torque-[.00075,-.005,0]),1e-5)

 def test_all_seven_slider_constraints_detect_rotation_xy_and_overtravel(self):
  plugs={str(i):np.r_[port(i),0,0,0,1] for i in USED}
  clips={str(i):np.r_[np.array(port(i))+[0,0,CLIP_Z],0,0,0,1] for i in USED}
  sample={'plug_poses':plugs,'clip_poses':clips}
  self.assertTrue(clip_constraint_summary([sample])['pass'])
  bad=str(USED[-1]);initial=clips[bad].copy()
  clips[bad][3:]=ik.matrix_to_quat_xyzw(axis_angle_matrix(np.array((1.,0,0)),math.radians(7.9)))
  result=clip_constraint_summary([sample]);self.assertFalse(result['pass'])
  self.assertAlmostEqual(result['connectors'][bad]['max_relative_angle_deg'],7.9)
  clips[bad]=initial.copy();clips[bad][0]+=.00011
  self.assertFalse(clip_constraint_summary([sample])['pass'])
  clips[bad]=initial.copy();clips[bad][2]-=CLIP_TRAVEL+.00002
  self.assertFalse(clip_constraint_summary([sample])['pass'])
