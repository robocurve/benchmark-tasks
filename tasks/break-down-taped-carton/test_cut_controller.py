"""Read-only coverage planning and released-tool hand-retreat regressions."""
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from yam_arena import ik
from yam_arena.demo import grip_rotation
from yam_arena.constants import READY,FINGER_OPEN
from robot import CartonCutProbe
from model import BladeCoverage,WIDTH,HEIGHT
from config import TOP

class Array:
 def __init__(self,value):self.value=np.array(value,dtype=float)
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):return self.value

class CutControllerTests(unittest.TestCase):
 def test_missing_coverage_only_inserts_physical_actions_and_is_bounded(self):
  probe=CartonCutProbe.__new__(CartonCutProbe)
  probe.index=0;probe.phases=[('return_hover',None,None)];probe.repass_round=0
  probe.cuts_verified=False;probe.failure=None;probe.seam_depth=-.000275
  coverage=BladeCoverage()
  # Boundary retraces now fail closed; exercise bounded retries with only
  # reviewed interior seam cells missing plus the two ordinary end folds.
  coverage.cut={i for i,(group,p) in enumerate(coverage.cells) if group=='seam' and abs(p[0])>.100}
  before=coverage.cut.copy();task=SimpleNamespace(_tracker=SimpleNamespace(coverage=coverage))
  with patch.dict(sys.modules,{'task':task}):
   for repeat in range(3):
    probe.index=next(i for i,p in enumerate(probe.phases) if p[0]=='return_hover')
    probe.schedule_remaining_cuts()
    self.assertEqual(probe.repass_round,repeat+1)
    self.assertTrue(probe.phases[probe.index][0].startswith('repass_'))
    self.assertEqual(coverage.cut,before)
    self.assertFalse(probe.cuts_verified)
   probe.schedule_remaining_cuts()
  self.assertIn('three measured physical retraces',probe.failure)
  self.assertEqual(coverage.cut,before)

 def test_empty_kernel_is_not_mistaken_for_success(self):
  probe=CartonCutProbe.__new__(CartonCutProbe);probe.failure=None
  with patch.dict(sys.modules,{'task':SimpleNamespace(_tracker=None)}):probe.schedule_remaining_cuts()
  self.assertIn('coverage unavailable',probe.failure)

 def test_released_cutter_stays_parked_while_hand_retracts(self):
  base=(0,.305,.75)
  q,_,_=ik.solve(np.array((.13,.21,TOP+.026)),ik.matrix_to_quat_xyzw(grip_rotation()),np.array(READY),base_pos=base,iters=300)
  native=Array([[*q,FINGER_OPEN,FINGER_OPEN]])
  ready=np.r_[q,1.,READY,1.]
  env=SimpleNamespace(_ready=Array(ready),_ids={'left_robot':(list(range(6)),[6,7])},
                      _base=SimpleNamespace(scene={'left_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=native))}),
                      rig=SimpleNamespace(bases={'left':base}),_low=np.full(14,-10.),_high=np.full(14,10.))
  probe=CartonCutProbe.__new__(CartonCutProbe)
  probe.env=env;probe.previous=ready.copy();probe.failure=None;probe.done=False
  probe.index=0;probe.phases=[('retreat',None,None)];probe.motion=None;probe.bias=np.zeros(3)
  probe.last_time=0.;probe.calibrated=True;probe.initial_tip_in_hand=np.zeros(3);probe.rest_time=1.95
  probe.coverage_report={};probe.repass_round=0;probe.cuts_verified=True;probe.rest_tool=None
  probe.park=SimpleNamespace(duration=0.,action7=lambda t:(np.r_[READY,1.],None))
  probe.pickup=SimpleNamespace(duration=0.)
  parked=np.array((.13,.21,TOP+.022));identity=np.eye(3)
  probe.pose=lambda name:(parked.copy(),identity.copy()) if name=='cutter' else (np.array((.27,WIDTH/2,TOP+.1+HEIGHT/2)),identity.copy())
  start=ik.tcp_pose(q,base)[0]
  for step in range(300):
   action,report=probe.command(step*.05)
   native.value[0,:6]=action[:6]
   if probe.done or probe.failure:break
  self.assertIsNone(probe.failure)
  self.assertTrue(probe.done)
  self.assertTrue(report[0]['hand_retreat'])
  self.assertGreater(ik.tcp_pose(native.value[0,:6],base)[0][2]-start[2],.075)
  np.testing.assert_allclose(probe.pose('cutter')[0],parked)

class CutHandoffRegressionTests(unittest.TestCase):
 def test_frozen007_lower_to_cut_preserves_world_bias_target_rotation_and_same_seed_ik(self):
  from feedback_geometry import blade_tcp_target,cut_entry_bias
  # Native007 lower endpoint28.35s, captured before its subsequent bias reset.
  point=np.array((.3552039120161092,.08439563500842412,.8477760605317747))
  bias=np.array((-.003,.0022331102096589265,-.00041211273765918153))
  grasp=np.array(((-.0010000767014928988,-.9999996926373426,.0000070758653124889615),
                  (-.9881483605769099,.0009892715895942026,.1534998302638446),
                  (-.15349979310875858,.00014651609364036604,-.9881486677410051)))
  offset=np.array((-.00006471925043257483,-.06386046555153711,.017945017246602635))
  rotation=np.array(((-.7088817678785438,.7053273275770655,.000011932531551042828),
                     (-.7053273276770948,-.708881767803308,-.000010389646893103713),
                     (.0000011306521826897853,-.00001578137184853684,.9999999998748348)))
  seed=np.array((-.477059543132782,2.1774799823760986,2.083102226257324,-1.414480209350586,-.14799317717552185,1.8793575763702393))
  local=np.array((.12,0,-.000275));localR=np.diag((-1.,1.,-1.))
  previous=dict(phase='seam_lower',local_endpoint=local.copy(),local_rotation=localR.copy())
  retained=cut_entry_bias(bias,previous,'seam_cut',local,localR)
  np.testing.assert_array_equal(retained,bias);self.assertFalse(np.shares_memory(retained,bias))
  old_target,oldR=blade_tcp_target(point+bias,rotation,grasp,offset)
  target,R=blade_tcp_target(point+retained,rotation,grasp,offset)
  np.testing.assert_array_equal(target,old_target);np.testing.assert_array_equal(R,oldR)
  np.testing.assert_allclose(target,(.3949816947273354,.1291973133992306,.8748987948985376),atol=2e-8,rtol=0.)
  oldq,_,_=ik.solve(old_target,ik.matrix_to_quat_xyzw(oldR),seed,base_pos=(0,.305,.75),iters=140)
  q,_,_=ik.solve(target,ik.matrix_to_quat_xyzw(R),seed,base_pos=(0,.305,.75),iters=140)
  np.testing.assert_array_equal(q,oldq)
  reset_target,_=blade_tcp_target(point,rotation,grasp,offset)
  self.assertGreater(np.linalg.norm(reset_target-target),.0037)

 def test_bias_carry_requires_same_lower_identity_local_endpoint_and_rotation(self):
  from feedback_geometry import cut_entry_bias
  bias=np.array((-.003,.002,-.0004));start=np.array((.12,0,-.000275));R=np.eye(3)
  previous=dict(phase='seam_lower',local_endpoint=start,local_rotation=R)
  for phase,endpoint,rotation in (('front_cut',start,R),('seam_cut',start+[.001,0,0],R),('seam_cut',start,np.diag((-1,-1,1)))):
   np.testing.assert_array_equal(cut_entry_bias(bias,previous,phase,endpoint,rotation),np.zeros(3))
  repass={**previous,'phase':'repass_1_seam_lower'}
  np.testing.assert_array_equal(cut_entry_bias(bias,repass,'repass_1_seam_cut',start,R),bias)
  with self.assertRaises(ValueError):cut_entry_bias([.0031,0,0],previous,'seam_cut',start,R)

 def test_cut_watchdogs_reset_only_idle_and_preserve_hard_wall_cap(self):
  from feedback_geometry import cut_watchdog
  self.assertIsNone(cut_watchdog(80.,20.))
  self.assertIn('no accepted advance',cut_watchdog(80.,20.01))
  self.assertIsNone(cut_watchdog(80.,0.)) # Actual cursor increment resets idle.
  self.assertIsNone(cut_watchdog(100.,0.))
  self.assertIn('hard cap',cut_watchdog(100.01,0.))

class ShortSeamRouteTests(unittest.TestCase):
 def probe(self):
  env=SimpleNamespace(_ready=Array(np.r_[READY,1.,READY,1.]),
                      rig=SimpleNamespace(bases={'left':(0,.305,.75),'right':(0,-.305,.75)}))
  return CartonCutProbe(env)

 def test_initial_seam_lifts_vertically_at_same_short_endpoint(self):
  p=self.probe();phases={name:(np.array(goal) if goal is not None else None,yaw) for name,goal,yaw in p.phases}
  end,R=phases['seam_cut'];lift,liftR=phases['seam_lift'];front,frontR=phases['front_hover']
  np.testing.assert_array_equal(end[:2],(-.107,0.));np.testing.assert_array_equal(lift[:2],end[:2])
  self.assertEqual(end[2],p.seam_depth);self.assertEqual(lift[2],-.020);self.assertEqual(R,liftR)
  self.assertEqual(front[2],lift[2]);self.assertEqual(phases['front_cut'][0][0],-.1095)
  self.assertEqual(phases['front_cut'][1],-np.pi/2)

 def test_recorded008_five_frames_meet_unchanged_short_endpoint_gate(self):
  from feedback_geometry import cut_endpoint_ready
  from model import FILM_Z
  # Actual008 samples71.60–71.80s; this is feasibility evidence, not new native success.
  samples=[(-.10751897404395508,.0005295424392580729,-.000046565355226453855,.012950603844897973),
           (-.10752842517038806,.0005004241179973961,-.00004603767058127422,.01295280643834029),
           (-.10753256205256592,.0004901967226581529,-.000046788576400718266,.013058056717491686),
           (-.1075376958436561,.00045787665839316417,-.000046424526043669265,.013131538649596097),
           (-.10753872897285309,.0004297535118023698,-.00003199999408801004,.013169223181254545)]
  end=np.array((-.107,0.,-.000275));frames=0
  for x,y,z,angle in samples:
   ready=cut_endpoint_ready(1.,FILM_Z<=z<=.001,np.array((x,y,z)),end,angle)
   frames=frames+1 if ready else 0
  self.assertEqual(frames,5)
  self.assertFalse(cut_endpoint_ready(.99,True,end,end,0.))
  self.assertFalse(cut_endpoint_ready(1.,False,end,end,0.))
  self.assertFalse(cut_endpoint_ready(1.,True,end+[0,.000751,0],end,0.))
  self.assertFalse(cut_endpoint_ready(1.,True,end,end,.06001))

 def test_transverse_endfolds_physically_connect_both_boundary_seam_cells(self):
  p=self.probe();phases={name:np.array(goal) for name,goal,yaw in p.phases if goal is not None};coverage=BladeCoverage()
  def sweep(a,b):
   coverage.update(a+[0,0,-.02],1.,True,.05)
   for f in np.linspace(0,1,int(np.linalg.norm(b-a)/.0005)+2):coverage.update(a+f*(b-a),1.,True,.05)
  sweep(phases['seam_lower'],phases['seam_cut'])
  # The reviewed+106.5→−107mm seam leaves BOTH boundary cells for the
  # physical transverse end folds; their depths/coverage kernel are unchanged.
  self.assertEqual(coverage.groups()['seam']['cut'],108)
  sweep(phases['front_lower'],phases['front_cut'])
  self.assertEqual(coverage.groups()['seam']['cut'],109)
  sweep(phases['back_lower'],phases['back_cut'])
  self.assertEqual(coverage.groups()['seam']['cut'],110)
  self.assertTrue(coverage.complete);self.assertEqual(len(coverage.cut),158)
