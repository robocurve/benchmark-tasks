"""Ordinary-motion raster invariants; no native pickup or full-drying claim."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY
from config import HEAD,HEAD_QUAT,TOP,LOCKS,SEGMENTS,segment,segment_quat
from yam_arena.urdf import quat_xyzw_to_matrix
from dryer_sweep import DryerSweepPlan,MAX_SWEEP_SPEED,bounds_gap,tool_and_hand_bounds
from robot import DryerProbe,DryerSweepProbe,LEFT_R,HANDLE_GRASP_LOCAL
from switch_geometry import FingerGeometry
from airflow import HairThermal


class DryerSweepTests(unittest.TestCase):
 def fixture(self):
  dryer=(np.array((.23,0.,1.0575)),np.eye(3))
  hand=(dryer[0]+HANDLE_GRASP_LOCAL,LEFT_R)
  return DryerSweepPlan(hand,hand,dryer),hand,dryer

 def test_command_anchor_exact_and_live_grasp_transform(self):
  plan,hand,dryer=self.fixture();command=(hand[0]+[.002,-.001,-.003],hand[1])
  plan=DryerSweepPlan(command,hand,dryer)
  for a,b in zip(plan.target(0.),command):np.testing.assert_array_equal(a,b)
  self.assertEqual(plan.grasp_error(hand,dryer),(0.,0.))
  shifted=(dryer[0]+[.006,0,0],dryer[1]);self.assertGreater(plan.grasp_error(hand,shifted)[0],.005)
  for bad in (([np.nan,0,0],np.eye(3)),([0,0,0],np.zeros((3,3)))):
   with self.assertRaises(ValueError):DryerSweepPlan(hand,hand,bad)

 def test_continuous_nominal_path_ik_rates_geometry_and_actual_airflow(self):
  plan,hand,dryer=self.fixture();seed=np.asarray(READY[:6]);geometry=FingerGeometry();pieces=geometry.pieces([-.015,-.015])
  scalp=np.asarray(HEAD);segments=[{'pos':segment(k,j),'rotation':quat_xyzw_to_matrix(segment_quat(k,j))} for k in range(LOCKS) for j in range(SEGMENTS)]
  low=[scalp-.065];high=[scalp+.065]
  for s in segments:
   axis=s['rotation'][:,2]*(.025-.007);low.append(s['pos']-abs(axis)-.007);high.append(s['pos']+abs(axis)+.007)
  old=None;exposed=0
  for clock in np.linspace(0,plan.duration,121):
   p,r=plan.target(float(clock));q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),seed,base_pos=(0,.305,.75),iters=220,tol=1e-6)
   self.assertLess(pe,1e-6);self.assertLess(re,.001);seed=q
   dp=p-r@plan.offset;dr=r@plan.rotation.T
   bounds=tool_and_hand_bounds([t@r.T+p for _,t in pieces],(dp,dr))
   self.assertGreater(bounds_gap(bounds,(low,high)),.003)
   self.assertGreater(bounds[0][:,2].min()-TOP,.003)
   if old is not None:self.assertLessEqual(np.linalg.norm(p-old),MAX_SWEEP_SPEED*plan.duration/120+1e-10)
   old=p
   if int(clock*120/plan.duration)%15==0:
    result=HairThermal().step(.05,segments,(dp+dr@[.088,0,0]).tolist(),dr[:,0].tolist(),1.)
    exposed=max(exposed,result['exposed_patches']);self.assertLessEqual(result['max_temperature_C'],45.)
  self.assertGreater(exposed,0)

 def test_pickup_handoff_reuses_exact14d_and_has_no_teacher_fallback(self):
  plan,hand,dryer=self.fixture();probe=DryerSweepProbe.__new__(DryerSweepProbe)
  q,pe,re=ik.solve(hand[0],ik.matrix_to_quat_xyzw(hand[1]),np.array(READY[:6]),base_pos=(0,.305,.75),iters=220,tol=1e-6)
  self.assertLess(pe,1e-6);issued=np.r_[q,.1,np.array(READY[:6]),1.].astype(np.float32)
  probe.sweep=None;probe.failure=None;probe.env=SimpleNamespace(rig=SimpleNamespace(bases={'left':(0,.305,.75)}))
  probe.read_arm=lambda side:(q,np.array((-.015,-.015)),*hand)
  probe.pose=lambda name:np.r_[dryer[0],0,0,0,1]
  with patch.object(DryerProbe,'command',return_value=(issued,{'complete':True,'failure':None})):
   out,control=probe.command(40.)
  np.testing.assert_array_equal(out,issued);self.assertIsNotNone(probe.sweep)
  self.assertFalse(control['complete']);self.assertEqual(control['phase'],'sweep_entry')
  np.testing.assert_array_equal(probe.sweep_right,issued[7:])


 def test_payload_handoff_anchor_does_not_add_feedforward_twice(self):
  _,hand,dryer=self.fixture();probe=DryerSweepProbe.__new__(DryerSweepProbe)
  q,pe,re=ik.solve(hand[0],ik.matrix_to_quat_xyzw(hand[1]),np.array(READY[:6]),base_pos=(0,.305,.75),iters=220,tol=1e-6)
  offset=np.array([.005,-.01,.02,.15,.01,0.]);issued=np.r_[q+offset,.1,np.array(READY[:6]),1.]
  probe.sweep=None;probe.failure=None;probe.payload_bias=offset;probe.env=SimpleNamespace(rig=SimpleNamespace(bases={'left':(0,.305,.75)}));probe.read_arm=lambda side:(q,np.array((-.015,-.015)),*hand);probe.pose=lambda name:np.r_[dryer[0],0,0,0,1]
  with patch.object(DryerProbe,'command',return_value=(issued,{'complete':True,'failure':None})):out,control=probe.command(40.)
  np.testing.assert_array_equal(out,issued);p,R=probe.sweep.target(0.);expected=ik.tcp_pose(issued[:6]-offset,(0,.305,.75));np.testing.assert_allclose(p,expected[0],atol=1e-12);np.testing.assert_allclose(R,expected[1],atol=1e-12)
 def test_original_pre_lift_grasp_is_not_rebased_after_lift(self):
  _,hand,dryer=self.fixture();probe=DryerProbe.__new__(DryerProbe);probe.failure=None;p,R=hand;probe.original_payload_grasp=(R.T@(dryer[0]-p),R.T@dryer[1]);body=np.r_[dryer[0]+[.006,0,0],0,0,0,1]
  self.assertFalse(probe.check_payload_grasp((None,None,p,R),body));self.assertIn('original pre-lift',probe.failure)

 def test_live_sweep_guard_reads_native_hair_and_freezes_bad_grasp(self):
  plan,hand,dryer=self.fixture();probe=DryerSweepProbe.__new__(DryerSweepProbe)
  q,pe,re=ik.solve(hand[0],ik.matrix_to_quat_xyzw(hand[1]),np.array(READY[:6]),base_pos=(0,.305,.75),iters=220,tol=1e-6)
  issued=np.r_[q,.1,np.array(READY[:6]),1.]
  probe.sweep=plan;probe.sweep_previous=issued.copy();probe.sweep_right=issued[7:].copy();probe.sweep_last_t=40.;probe.sweep_start_t=40.;probe.sweep_clock=0.;probe.sweep_hold=0;probe.failure=None
  probe.env=SimpleNamespace(rig=SimpleNamespace(bases={'left':(0,.305,.75)}),_low=np.full(14,-10.),_high=np.full(14,10.))
  probe.geometry=FingerGeometry()
  rq=np.array(READY[:6]);rp,rr=ik.tcp_pose(rq,(0,-.305,.75))
  left=(q,np.array((-.015,-.015)),*hand);right=(rq,np.array((-.04695,-.04695)),rp,rr)
  probe.read_arm=lambda side:left if side=='left' else right
  # Same native ProxyArray->torch->detach->cpu->numpy conversion contract.
  names=['head']+[f'hair_{k}_{j}' for k in range(LOCKS) for j in range(SEGMENTS)]
  values=np.array([[*HEAD,*HEAD_QUAT]]+[[*segment(k,j),*segment_quat(k,j)] for k in range(LOCKS) for j in range(SEGMENTS)])
  class Tensor:
   def detach(self):return self
   def cpu(self):return self
   def numpy(self):return values[None,:]
  proxy=SimpleNamespace(torch=Tensor());hair=SimpleNamespace(body_names=names,data=SimpleNamespace(body_link_pose_w=proxy))
  probe.base=SimpleNamespace(scene={'hair_model':hair})
  probe.pose=lambda name:np.r_[dryer[0] if name=='dryer' else dryer[0]+[-.037,0,-.041],0,0,0,1]
  out,c=probe.command(40.05)
  self.assertIsNone(c['failure']);self.assertGreater(c['clearance_lower_bound_m'],.003)
  self.assertGreater(c['path_clock_s'],0.);np.testing.assert_array_equal(out[7:],issued[7:])
  # The original calibrated grasp remains the reference; no rebase to hide slip.
  probe.pose=lambda name:np.r_[dryer[0]+[.006,0,0] if name=='dryer' else dryer[0]+[.006-.037,0,-.041],0,0,0,1]
  previous=out.copy();out,c=probe.command(40.10)
  self.assertIn('measured grasp',c['failure']);np.testing.assert_array_equal(out,previous)

if __name__=='__main__':unittest.main()
