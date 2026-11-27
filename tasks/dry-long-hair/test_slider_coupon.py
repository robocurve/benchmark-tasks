"""CPU checks for unscored bench schedule/math/isolation, not native evidence."""
import ast
import math
from pathlib import Path
import unittest
from config import DRYER_START,rotate
from slider_coupon import coupon_command,attachment_sample,inferred_joint_reaction,CouponTravel,COUPON_DURATION,COUPON_FORCE
from switch_detent import DetentLaw


class SliderCouponTests(unittest.TestCase):
 def test_force_is_removed_for_both_two_second_holds(self):
  self.assertEqual(COUPON_DURATION,7.7)
  self.assertEqual(coupon_command(1.),('push_on',COUPON_FORCE))
  self.assertEqual(coupon_command(5.2),('push_off',-COUPON_FORCE))
  for t in (0.,.99,1.5,2.,3.49,4.2,5.,5.7,6.,7.7):self.assertEqual(coupon_command(t)[1],0.)
  for t in (-1.,float('nan')):
   with self.assertRaises(ValueError):coupon_command(t)

 def test_reverse_disturbance_is_gently_ramped_at_native_substep_rate(self):
  for time_s,expected in ((3.5,0.),(3.6,-.05),(3.7,-.1),(3.9,-.1),(4.,-.1),(4.1,-.05),(4.2,0.)):
   self.assertAlmostEqual(coupon_command(time_s)[1],expected)
  samples=[coupon_command(3.5+i/480)[1] for i in range(337)]
  self.assertLessEqual(max(abs(b-a) for a,b in zip(samples,samples[1:])),.5/480+1e-12)

 def test_native_extrema_catch_failures_between_twenty_hz_trace_frames(self):
  travel=CouponTravel()
  self.assertFalse(any(travel.checks().values()))
  for t,phase,q in ((1.4,'push_on',.0039),(3.,'on_force_removed_hold',.0037),
                    (3.8,'reverse_load_hold',.0027),(5.1,'on_zero_force_recovery',.0037),
                    (5.6,'push_off',.0001),(7.,'off_force_removed_hold',0.)):
   travel.observe(t,phase,q)
  self.assertTrue(all(travel.checks().values()))
  # A single 480 Hz failure between acceptable 20 Hz samples must fail closed.
  travel.observe(3.+1/480,'on_force_removed_hold',.003)
  travel.observe(3.8+1/480,'reverse_load_hold',.0019)
  travel.observe(5.1+1/480,'on_zero_force_recovery',.003)
  travel.observe(7.+1/480,'off_force_removed_hold',.0005)
  for name in ('on_retained_after_force_removed','on_well_retained_during_reverse_load',
               'on_recovered_after_reverse_force_removed','off_retained_after_force_removed'):
   self.assertFalse(travel.checks()[name])
  self.assertEqual(travel.windows['on_hold']['samples'],2)

 def test_implicit_cpu_constitutive_prediction_survives_reverse_ramp_and_recovers(self):
  # A model/schedule regression, explicitly not native drive-consumption proof.
  law=DetentLaw();mass=.003;dt=1/480;q=0.;v=0.;travel=CouponTravel()
  for i in range(round(COUPON_DURATION/dt)):
   phase,load=coupon_command(i*dt)
   v=(mass*v+dt*(load-mass*9.81-law.stiffness*(q-law.target(q))))/(mass+dt*law.damping+dt*dt*law.stiffness)
   q+=dt*v
   if q<0.:q=0.;v=max(0.,v)
   if q>law.travel:q=law.travel;v=min(0.,v)
   travel.observe((i+1)*dt,phase,q)
  self.assertTrue(all(travel.checks().values()),travel.windows)

 def test_attachment_measurement_tracks_real_relative_pose(self):
  angle=.7;quat=(math.sin(angle/2),0.,0.,math.cos(angle/2))
  delta=rotate(quat,(-.037,0.,-.041));switch=tuple(a+b for a,b in zip(DRYER_START,delta))
  state=attachment_sample((*DRYER_START,*quat),(*switch,*quat))
  self.assertAlmostEqual(state['q_m'],.004);self.assertAlmostEqual(state['lateral_anchor_error_m'],0.)
  self.assertAlmostEqual(state['relative_angle_deg'],0.);self.assertEqual(state['fixture_translation_error_m'],0.)
  self.assertAlmostEqual(state['fixture_angle_error_deg'],math.degrees(angle))
  shifted=(switch[0]+.001,*switch[1:])
  self.assertGreater(attachment_sample((*DRYER_START,*quat),(*shifted,0.,0.,0.,1.))['lateral_anchor_error_m'],.0005)
  self.assertAlmostEqual(attachment_sample((*DRYER_START,*quat),(*switch,0.,0.,0.,1.))['relative_angle_deg'],math.degrees(angle))

 def test_inferred_reaction_balances_known_gravity_force_and_acceleration(self):
  still=[0.]*6;mass=.003
  self.assertAlmostEqual(inferred_joint_reaction(still,still,.01,mass,0.)[2],mass*9.81)
  freefall=[0.,0.,-.0981,0.,0.,0.]
  self.assertAlmostEqual(inferred_joint_reaction(still,freefall,.01,mass,0.)[2],0.)
  # A clamped slider under an upward pulse reacts against that load plus gravity.
  self.assertAlmostEqual(inferred_joint_reaction(still,still,.01,mass,.35)[2],mass*9.81-.35)
  with self.assertRaises(ValueError):inferred_joint_reaction([],[],.01,mass,0.)

 def test_external_coupon_is_explicit_mode_and_returns_before_robot_harness(self):
  path=Path(__file__).parent
  root=ast.parse((path/'run.py').read_text());main=next(n for n in root.body if isinstance(n,ast.FunctionDef) and n.name=='main')
  branch=next(n for n in main.body if isinstance(n,ast.If) and ast.unparse(n.test)=="a.mode == 'slider-coupon'")
  self.assertIsInstance(branch.body[-1],ast.Return)
  calls=[n for n in ast.walk(root) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='run_coupon']
  self.assertEqual(len(calls),1);self.assertIn(calls[0],list(ast.walk(branch)))
  coupon=(path/'slider_coupon.py').read_text()
  self.assertNotIn('write_root_pose',coupon);self.assertNotIn('write_root_state',coupon)
  self.assertNotIn('HairScorer',coupon);self.assertNotIn('from robot import',coupon)
  self.assertIn("'task_score':None",coupon);self.assertIn("'external_bench_forces':True",coupon)


if __name__=='__main__':unittest.main()
