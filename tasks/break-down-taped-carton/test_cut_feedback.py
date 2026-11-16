"""Reviewed production cutter feedback and actual command continuity regressions."""
import unittest
from types import SimpleNamespace
import numpy as np
from yam_arena import ik
from yam_arena.urdf import quat_xyzw_to_matrix
from feedback_geometry import matching_hover_lower,bounded_cut_integral,lower_lateral_advance,TrackedProgress
from robot import CartonCutProbe
from test_cut_controller import Array
# Coherent native0112.60s hand/cutter/north input; no restored-start code.
# Trace SHA256 a476a1902ee42d3453331487597a56fad4265a006347357c33bc27eb9dc2a110.
ROW={'action_14d': [-0.4810550808906555,
                2.23368239402771,
                2.266294240951538,
                -1.5814279317855835,
                -0.06828521937131882,
                1.8761454820632935,
                0.23429180681705475,
                0.46344178915023804,
                1.1025381088256836,
                1.1107721328735352,
                -1.579027771949768,
                -3.626863326644525e-05,
                0.46343401074409485,
                1.0],
 'control': {'bias_m': [-0.0007994953951518293, -0.001960770820719276, 0.0009928921580175888],
             'tip_in_hand': [2.4401950505657312e-05, -0.062149164706901544, 0.023482071299720946]},
 'cutter_pose': [0.3869021236896515,
                 0.11649151891469955,
                 0.8873984813690186,
                 -0.0015479420544579625,
                 0.0030999579466879368,
                 -0.924644947052002,
                 0.3808145225048065],
 'native_hands': [{'finger_joints': [-0.012431425973773003, -0.012465653009712696],
                   'joints': [-0.4846770763397217,
                              2.2354443073272705,
                              2.264643669128418,
                              -1.5845352411270142,
                              -0.06981690227985382,
                              1.876531720161438]}],
 'poses': {'north': [0.33363962173461914,
                     -0.0636395588517189,
                     0.7974998354911804,
                     0.9238795042037964,
                     0.3826834559440613,
                     -5.398139251155953e-07,
                     1.0599264754773685e-07]}}

class CutFeedbackTests(unittest.TestCase):
 def test_recorded011_handoff_keeps_exact_14d_command_nominal_start_and_world_bias(self):
  hand=ROW['native_hands'][0];native=Array([[*hand['joints'],*hand['finger_joints']]])
  env=SimpleNamespace(_ids={'left_robot':(list(range(6)),[6,7])},_base=SimpleNamespace(scene={'left_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=native))}),rig=SimpleNamespace(bases={'left':(0,.305,.75)}),_low=np.full(14,-10.),_high=np.full(14,10.))
  p=CartonCutProbe.__new__(CartonCutProbe);p.env=env;p.previous=np.array(ROW['action_14d']);p.failure=None;p.done=False;p.index=1;p.motion=None;p.bias=np.array(ROW['control']['bias_m']);before=p.bias.copy();p.last_time=2.6;p.calibrated=True;p.initial_tip_in_hand=np.array(ROW['control']['tip_in_hand']);p.initial_tool_rotation=np.eye(3);p.rest_tool=None;p.rest_time=None;p.coverage_report=None;p.repass_round=0;p.cuts_verified=False
  R=np.diag([-1.,1.,-1.]);start=np.array([.1065,0,-.020]);p.phases=[('seam_hover_reposition',start,np.pi),('seam_lower',(.1065,0,-.001),np.pi)];p.completed_motion=dict(phase='seam_hover_reposition',local_endpoint=start,local_rotation=R)
  p.park=SimpleNamespace(duration=0.,action7=lambda t:(p.previous[7:].copy(),None));p.pickup=SimpleNamespace(duration=0.)
  poses={**ROW['poses'],'cutter':ROW['cutter_pose']};p.pose=lambda name:(np.array(poses[name][:3]),quat_xyzw_to_matrix(poses[name][3:]))
  action,report=p.command(2.65);np.testing.assert_array_equal(action,ROW['action_14d']);np.testing.assert_array_equal(p.bias,before);np.testing.assert_allclose(p.motion['local_start'],start,atol=1e-15);np.testing.assert_allclose(p.motion['local_r0'],R,atol=1e-15);self.assertEqual(p.motion['progress'].fraction,0.);self.assertTrue(report[0]['hover_lower_handoff_hold'])
  oldP,oldR=ik.tcp_pose(action[:6],env.rig.bases['left']);np.testing.assert_array_equal(report[0]['tcp_target'],oldP);self.assertEqual(p.motion['progress'].settled,0)
  self.assertAlmostEqual(p.motion['progress'].speed,.010/1.5)
  p.command(2.7);self.assertFalse(np.array_equal(p.bias,before)) # resumes bounded feedback after the single hold

 def test_handoff_identity_rejects_mismatched_frame_rotation_or_unrelated_phase(self):
  previous=dict(phase='seam_hover_reposition',local_endpoint=[.1065,0,-.020],local_rotation=np.eye(3))
  self.assertTrue(matching_hover_lower(previous,'seam_lower',[.1065,0,-.001],np.eye(3)))
  self.assertFalse(matching_hover_lower(previous,'return_lower',[.1065,0,-.001],np.eye(3)))
  self.assertFalse(matching_hover_lower(previous,'front_lower',[.1065,0,-.001],np.eye(3)))
  self.assertFalse(matching_hover_lower(previous,'seam_lower',[.1065,.002,-.001],np.eye(3)))
  self.assertFalse(matching_hover_lower(previous,'seam_lower',[.1065,0,-.001],np.diag([-1,-1,1])))

 def test_wrong_sign_along_bias_unwinds_full_vector_with_norm_cap_and_total_cap(self):
  b=np.array([.001629,0,0]);e=np.array([-.002539,0,0]);new=bounded_cut_integral(b,e,.05);self.assertLess(new[0],b[0]);self.assertAlmostEqual(new[0]-b[0],.5*.05*e[0])
  for error in ([.006,.006,.006],[-.006,-.006,-.006],[.02,0,0]):
   new=bounded_cut_integral(np.array([.0029,-.0029,.0029]),error,.05);self.assertLessEqual(np.linalg.norm(new-np.array([.0029,-.0029,.0029])),75e-6+1e-15);self.assertTrue(np.all(abs(new)<=.003))
  for _ in range(100):b=bounded_cut_integral(b,e,.05)
  self.assertLess(b[0],0);self.assertGreaterEqual(b[0],-.003)

 def test_absolute_seam_lateral_gate_freezes_clock_while_feedback_can_update(self):
  tip=np.array([.1065,-.0036,-.001]);goal=np.array([.1065,0,-.001]);self.assertFalse(lower_lateral_advance(tip,goal));p=TrackedProgress(.019,0,speed=.010/1.5)
  p.update(.05,.0036,0,.019,0,advance_allowed=lower_lateral_advance(tip,goal));self.assertEqual(p.fraction,0.);self.assertGreater(bounded_cut_integral(np.zeros(3),goal-tip,.05)[1],0)
  tip[1]=.00065;self.assertTrue(lower_lateral_advance(tip,goal));tip[1]=.000650001;self.assertFalse(lower_lateral_advance(tip,goal))

 def test_fold_gate_uses_X_cross_track_and_ignores_Y_along_cut_error(self):
  endpoint=np.array([-.110,-.034,-.001]);tip=endpoint+np.array([.0004,.003,0]);self.assertTrue(lower_lateral_advance(tip,endpoint,[0,1,0]));tip[0]-=.002;self.assertFalse(lower_lateral_advance(tip,endpoint,[0,1,0]))

 def test_lower_smoothstep_true_peak_is_at_most_10mm_s(self):
  length=.019;speed=.010/1.5;duration=length/speed;t=np.linspace(0,duration,1001);f=t/duration;p=length*(3*f*f-2*f*f*f);self.assertLessEqual(max(np.diff(p)/np.diff(t)),.010+1e-12);self.assertAlmostEqual(p[-1],length)

class ProductionRouteTests(unittest.TestCase):
 def probe(self):
  from yam_arena.constants import READY
  env=SimpleNamespace(_ready=Array(np.r_[READY,1.,READY,1.]),rig=SimpleNamespace(bases={'left':(0,.305,.75),'right':(0,-.305,.75)}))
  return CartonCutProbe(env)

 def test_reviewed_initial_route_positions_and_explicit_tool_yaw(self):
  from yam_arena.urdf import axis_angle_matrix
  p=self.probe();phases={name:(point,yaw) for name,point,yaw in p.phases}
  for name,point in (('seam_hover',(.1065,0,-.020)),('seam_lower',(.1065,0,-.001)),('seam_cut',(-.107,0,-.001)),('seam_lift',(-.107,0,-.020))):
   np.testing.assert_array_equal(phases[name][0],point)
   np.testing.assert_allclose(np.diag([1.,-1.,-1.])@axis_angle_matrix(np.array([0.,0.,1.]),phases[name][1]),np.diag([-1.,1.,-1.]),atol=1e-15)
  self.assertEqual(phases['front_orient'],(None,-np.pi/2))
  self.assertFalse(hasattr(p,'coupon_rotations'))

 def test_seam_repass_stages_centrally_with_incoming_yaw_before_turning(self):
  import sys
  from unittest.mock import patch
  from model import BladeCoverage
  p=self.probe();p.phases=[('back_lift',(.11,.034,-.020),-np.pi/2),('return_hover',None,None)];p.index=1
  c=BladeCoverage();c.cut=set(range(len(c.cells)))-{next(i for i,(name,v) in enumerate(c.cells) if name=='seam' and abs(v[0]+.051)<1e-8)}
  before=c.cut.copy()
  with patch.dict(sys.modules,{'task':SimpleNamespace(_tracker=SimpleNamespace(coverage=c))}):p.schedule_remaining_cuts()
  staging,orient=p.phases[1:3]
  self.assertEqual(staging,('repass_1_seam_staging',(0.,-.05,-.04),-np.pi/2))
  self.assertEqual(orient,('repass_1_seam_orient',(0.,-.05,-.04),np.pi))
  self.assertEqual(c.cut,before);self.assertFalse(p.cuts_verified)
  # The803 suffix certificate uses the original internal-repass film depth.
  self.assertAlmostEqual(next(x[1][2] for x in p.phases if x[0]=='repass_1_seam_lower'),-.000275)
  # A second seam-only retry preserves the previous seam yaw explicitly;
  # passing None here would silently command initial pickup tool rotation.
  p.index=next(i for i,x in enumerate(p.phases) if x[0]=='return_hover')
  with patch.dict(sys.modules,{'task':SimpleNamespace(_tracker=SimpleNamespace(coverage=c))}):p.schedule_remaining_cuts()
  self.assertEqual(p.phases[p.index],('repass_2_seam_staging',(0.,-.05,-.04),np.pi))

 def test_generic_front_and_back_lower_identity_and_cross_track(self):
  for stem in ('front','back','repass_1_front_fold','repass_2_back_fold'):
   previous=dict(phase=stem+'_hover',local_endpoint=[-.11,-.034,-.020],local_rotation=np.eye(3))
   goal=np.array([-.11,-.034,-.001])
   self.assertTrue(matching_hover_lower(previous,stem+'_lower',goal,np.eye(3)))
   self.assertFalse(lower_lateral_advance(goal+[.000651,0,0],goal,[0,.068,0]))
   self.assertTrue(lower_lateral_advance(goal+[.00065,.004,0],goal,[0,.068,0]))

class RepassBoundarySafetyTests(unittest.TestCase):
 def probe(self):
  p=CartonCutProbe.__new__(CartonCutProbe);p.index=1
  p.phases=[('back_lift',(.11,.034,-.020),-np.pi/2),('return_hover',None,None)]
  p.repass_round=0;p.cuts_verified=False;p.failure=None
  return p

 def test_boundary_missing_cells_reject_requested_margins_without_partial_credit(self):
  import sys
  from unittest.mock import patch
  from model import BladeCoverage
  for x in (-.109,.109):
   with self.subTest(missing_cell=x):
    c=BladeCoverage();missing=next(i for i,(g,v) in enumerate(c.cells) if g=='seam' and abs(v[0]-x)<1e-8)
    c.cut=set(range(len(c.cells)))-{missing};before=c.cut.copy();p=self.probe();phases=p.phases.copy()
    with patch.dict(sys.modules,{'task':SimpleNamespace(_tracker=SimpleNamespace(coverage=c))}):p.schedule_remaining_cuts()
    self.assertIn('unreviewed seam repass',p.failure);self.assertIn('0.113',p.failure)
    self.assertEqual(p.phases,phases);self.assertEqual(p.repass_round,0);self.assertFalse(p.cuts_verified);self.assertEqual(c.cut,before)

 def test_mixed_remaining_spans_are_atomic_and_nonfinite_request_fails_closed(self):
  import sys
  from unittest.mock import patch
  from model import BladeCoverage
  for xs in ((.113,.105),(float('nan'),-.055)):
   with self.subTest(span=xs):
    p=self.probe();phases=p.phases.copy();c=BladeCoverage();before=c.cut.copy()
    # Even a safe fold listed before the invalid seam must not be inserted.
    spans=[dict(group='front_fold',start=np.array([-.11,-.01,0]),end=np.array([-.11,.01,0]),missing_cells=2),
           dict(group='seam',start=np.array([xs[0],0,0]),end=np.array([xs[1],0,0]),missing_cells=1)]
    with patch.dict(sys.modules,{'task':SimpleNamespace(_tracker=SimpleNamespace(coverage=c))}),patch('feedback_geometry.missing_cut_spans',return_value=spans):p.schedule_remaining_cuts()
    self.assertIn('unreviewed seam repass',p.failure);self.assertEqual(p.phases,phases);self.assertEqual(p.repass_round,0);self.assertFalse(p.cuts_verified);self.assertEqual(c.cut,before)

class FoldCandidateTests(unittest.TestCase):
 def test_first_pass_and_cleanup_share_shifted_fold_planes_but_original_cells(self):
  import sys
  from unittest.mock import patch
  from model import BladeCoverage
  p=ProductionRouteTests().probe();first={n:point for n,point,_ in p.phases};c=BladeCoverage()
  self.assertEqual({v[0]for g,v in c.cells if g=='front_fold'},{-.110})
  self.assertEqual({v[0]for g,v in c.cells if g=='back_fold'},{.110})
  c.cut={i for i,(g,_)in enumerate(c.cells)if g=='seam'};before=c.cut.copy()
  p.index=next(i for i,x in enumerate(p.phases)if x[0]=='return_hover')
  with patch.dict(sys.modules,{'task':SimpleNamespace(_tracker=SimpleNamespace(coverage=c))}):p.schedule_remaining_cuts()
  for stem,group,x in (('front','front_fold',-.1095),('back','back_fold',.1095)):
   for leg in ('hover','lower','cut','lift'):
    original=first[stem+'_'+leg];cleanup=next(point for n,point,_ in p.phases if n=='repass_1_'+group+'_'+leg)
    self.assertEqual(original[0],x);self.assertEqual(cleanup[0],x)
    z=-.00075 if leg in ('lower','cut')else-.020
    self.assertEqual(original[2],z);self.assertEqual(cleanup[2],z)
  self.assertEqual(c.cut,before);self.assertFalse(p.cuts_verified)

 def test_actual010_four_holes_counterfactual_fills158_without_native_credit(self):
  from model import BladeCoverage
  # Numeric POSTSTEP010 crossing windows. This TEST seeds its measured154
  # cells; production never writes coverage. The transform assumes identical
  # residual tracking after shifting commands: it is not a native result.
  front=[[-.11031106369345756,-.0006846181063866406,-.000601355859942553],[-.11031803229342867,.0005206137195712262,-.0006050513774008704]]
  back=[[[.11057787222828341,-.014373300950171192,-.0013563464608533273],[.11030826937960726,-.013677659031247097,-.0015619358608767345],[.10980906349905997,-.012169580968144156,-.0016996776915104974]],
        [[.10951484530517772,-.010748772947253248,-.0012572247859558173],[.10982509576729281,-.00954527360100748,-.0015704666839104933],[.1101764026879095,-.008205529893015175,-.0017279053530189024]],
        [[.1095012400502273,-.0002003519074723616,-.0013762356969846024],[.10981696750534131,.0010642463754467822,-.0016170292494970785],[.11016918504735779,.002450573897488277,-.0016892438285281413]]]
  holes={0,139,141,146}
  for shifted,expected in ((False,154),(True,158)):
   c=BladeCoverage();c.cut=set(range(len(c.cells)))-holes
   for group,window in [('front',front)]+[('back',w)for w in back]:
    c.previous=None
    for xyz in window:
     point=np.array(xyz)
     if shifted:point+=np.array([.0005 if group=='front'else-.0005,0,.00025])
     c.update(point,1.,True,.05)
   self.assertEqual(len(c.cut),expected);self.assertEqual(c.complete,shifted)

 def test_cleanup_cut_speed_8mm_s_and_first_pass_speeds_unchanged(self):
  from unittest.mock import patch
  for phase,round_,speed in (('front_cut',0,.016),('seam_cut',0,.012),('repass_1_front_fold_cut',1,.008),('repass_1_seam_cut',1,.008)):
   hand=ROW['native_hands'][0];native=Array([[*hand['joints'],*hand['finger_joints']]])
   env=SimpleNamespace(_ids={'left_robot':(list(range(6)),[6,7])},_base=SimpleNamespace(scene={'left_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=native))}),rig=SimpleNamespace(bases={'left':(0,.305,.75)}),_low=np.full(14,-10.),_high=np.full(14,10.))
   p=CartonCutProbe.__new__(CartonCutProbe);p.env=env;p.previous=np.array(ROW['action_14d']);p.failure=None;p.done=False;p.index=1;p.motion=None;p.bias=np.zeros(3);p.last_time=2.6;p.calibrated=True;p.initial_tip_in_hand=np.array(ROW['control']['tip_in_hand']);p.initial_tool_rotation=np.eye(3);p.rest_tool=None;p.rest_time=None;p.coverage_report=None;p.repass_round=round_;p.cuts_verified=False;p.completed_motion=None
   p.phases=[(phase[:-4]+'_lower',(.1065,0,-.001),np.pi),(phase,(.0965,0,-.001),np.pi)]
   p.park=SimpleNamespace(duration=0.,action7=lambda t:(p.previous[7:].copy(),None));p.pickup=SimpleNamespace(duration=0.)
   poses={**ROW['poses'],'cutter':ROW['cutter_pose']};p.pose=lambda name:(np.array(poses[name][:3]),quat_xyzw_to_matrix(poses[name][3:]))
   with patch('yam_arena.ik.solve',return_value=(np.array(hand['joints']),0.,0.)):p.command(2.65)
   self.assertAlmostEqual(p.motion['progress'].speed,speed)
