import unittest
import numpy as np
from feedback_geometry import blade_tcp_target,flap_grip_frame,TrackedProgress

class FeedbackTests(unittest.TestCase):
 def test_rotated_tool_offset_is_compensated_in_tcp_goal(self):
  turn=np.array([[0,-1,0],[1,0,0],[0,0,1]])
  offset=np.array((.062,0,-.026));goal=np.array((.2,-.05,.85))
  tcp,hand=blade_tcp_target(goal,turn,np.eye(3),offset)
  np.testing.assert_allclose(tcp+hand@offset,goal,atol=1e-12)
  self.assertGreater(np.linalg.norm((goal-offset)+hand@offset-goal),.08)
 def test_arbitrary_measured_grasp_rotation(self):
  tool_in_hand=np.array([[1,0,0],[0,0,-1],[0,1,0]])
  tool=np.array([[0,-1,0],[1,0,0],[0,0,1]])
  tip=np.array((.01,.04,.06));goal=np.array((.3,.1,.9))
  tcp,hand=blade_tcp_target(goal,tool,tool_in_hand,tip)
  np.testing.assert_allclose(hand@tool_in_hand,tool)
  np.testing.assert_allclose(tcp+hand@tip,goal)
 def test_fingers_close_across_flap_edge_with_valid_rotation(self):
  for angle in np.arange(0,360,45):
   a=np.radians(angle);outward=np.array((np.cos(a),np.sin(a),0.));approach=outward*.8-np.array((0.,0.,.6))
   for sign in (-1,1):
    r=flap_grip_frame(outward,approach,sign);tangent=np.cross((0,0,1),outward)
    self.assertAlmostEqual(abs(np.dot(r[:,1],tangent)),1.)
    self.assertAlmostEqual(np.dot(r[:,0],tangent),0.)
    np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
    self.assertAlmostEqual(np.linalg.det(r),1.)
 def test_wall_time_cannot_advance_a_lagging_cut(self):
  gate=TrackedProgress(.24,0,speed=.02)
  for _ in range(1000):self.assertFalse(gate.update(.05,.12,0,.12,0))
  self.assertEqual(gate.fraction,0.)
 def test_orientation_must_settle_before_progress(self):
  gate=TrackedProgress(.05,1.)
  for _ in range(100):self.assertFalse(gate.update(.05,0,.3,0,.3))
  self.assertEqual(gate.fraction,0.)
 def test_endpoint_needs_actual_pose_and_multiple_samples(self):
  gate=TrackedProgress(.005,0)
  for _ in range(10):self.assertFalse(gate.update(.05,0,0,.02,0))
  self.assertEqual(gate.fraction,1.)
  self.assertFalse(gate.update(.05,0,0,0,0));self.assertFalse(gate.update(.05,0,0,0,0))
  self.assertTrue(gate.update(.05,0,0,0,0))

class ActualToolFeedbackTests(unittest.TestCase):
 def test_actual_grasp_compensates_translation_and_torsional_slip(self):
  from feedback_geometry import measured_blade_grasp
  from yam_arena.urdf import axis_angle_matrix
  hand=axis_angle_matrix(np.array((0.,0.,1.)),.6)
  tcp=np.array((.13,.21,.78));tool=np.array((.135,.21,.775))
  tool_rotation=hand@axis_angle_matrix(np.array((1.,0.,0.)),.033)
  grasp,offset,tip=measured_blade_grasp(tcp,hand,tool,tool_rotation)
  goal=np.array((.3,0.,.85));desired=np.eye(3)
  target,rotation=blade_tcp_target(goal,desired,grasp,offset)
  np.testing.assert_allclose(rotation@grasp,desired,atol=1e-12)
  np.testing.assert_allclose(target+rotation@offset,goal,atol=1e-12)
  # An observed 3 mm change is compensated rather than kept as calibration.
  slipped_tool=tool+hand@np.array((0.,.001,.003))
  moved_grasp,moved_offset,_=measured_blade_grasp(tcp,hand,slipped_tool,tool_rotation)
  stale_error=target+rotation@moved_offset-goal
  self.assertGreater(np.linalg.norm(stale_error),.003)
  moved_target,moved_rotation=blade_tcp_target(goal,desired,moved_grasp,moved_offset)
  np.testing.assert_allclose(moved_target+moved_rotation@moved_offset,goal,atol=1e-12)

 def test_cut_progress_pauses_on_invalid_cross_track_or_depth(self):
  gate=TrackedProgress(.24,0.,speed=.012)
  for _ in range(100):gate.update(.05,.001,0.,.2,0.,tracking_tolerance=.0025,advance_allowed=False)
  self.assertEqual(gate.fraction,0.)
  gate.update(.05,.001,0.,.2,0.,tracking_tolerance=.0025,advance_allowed=True)
  self.assertGreater(gate.fraction,0.)

 def test_native_missing_cells_receive_only_physical_retraces(self):
  from feedback_geometry import missing_cut_spans
  from model import BladeCoverage,FILM_Z
  coverage=BladeCoverage()
  def sweep(start,end):
   start=np.array(start,dtype=float);end=np.array(end,dtype=float)
   coverage.update((*start[:2],-.020),1.,True,.05)
   for fraction in np.linspace(0,1,max(2,int(np.linalg.norm(end-start)/.0005)+1)):
    coverage.update(start+fraction*(end-start),1.,True,.05)
  depth=(FILM_Z+.001)/2
  sweep((-.12,0,depth),(-.02,0,depth))
  sweep((-.11,-.034,-.001),(-.11,.034,-.001))
  sweep((.11,-.034,-.001),(.11,.034,-.001))
  # The two end-fold traversals also cross endpoint seam cells.
  self.assertEqual(coverage.groups()['seam']['cut'],47)
  before=frozenset(coverage.cut)
  spans=missing_cut_spans(tuple(coverage.cells),before)
  self.assertEqual(len(spans),1)
  self.assertEqual(spans[0]['group'],'seam')
  self.assertEqual(coverage.cut,set(before))
  self.assertFalse(coverage.complete)
  self.assertGreater(spans[0]['start'][0],spans[0]['end'][0])
  start=spans[0]['start'].copy();end=spans[0]['end'].copy();start[2]=end[2]=depth
  sweep(start,end)
  self.assertTrue(coverage.complete)
  self.assertEqual(missing_cut_spans(coverage.cells,frozenset(coverage.cut)),[])
