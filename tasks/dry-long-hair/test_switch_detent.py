"""Constitutive and callback-contract tests, not native retention evidence."""
import math
import json
import unittest
from switch_detent import DetentLaw,SubstepDetent,slider_coordinate,CrossingTrace
from config import rotate


class DetentConstitutiveTests(unittest.TestCase):
 def test_two_equal_wells_barrier_and_restoring_force_symmetry(self):
  law=DetentLaw()
  self.assertEqual(law.potential(0.),0.);self.assertEqual(law.potential(.004),0.)
  self.assertAlmostEqual(law.barrier,.0002);self.assertAlmostEqual(law.force(.002),-.2)
  for q in (.0001,.0007,.0015,.0025,.0033,.0039):
   self.assertAlmostEqual(law.potential(q),law.potential(.004-q))
   self.assertAlmostEqual(law.force(q),-law.force(.004-q))
   gradient=(law.potential(q+1e-8)-law.potential(q-1e-8))/2e-8
   self.assertAlmostEqual(law.force(q),-gradient,places=7)

 def test_damping_removes_energy_and_nearest_well_selection_cannot_add_it(self):
  law=DetentLaw()
  for q in (-.001,.0005,.0019,.002,.0021,.0035,.005):
   for v in (-.04,0.,.04):
    self.assertAlmostEqual((law.force(q,v)-law.force(q))*v,-law.damping*v*v)
   for old_target in (0.,law.travel):
    jump=.5*law.stiffness*((q-law.target(q))**2-(q-old_target)**2)
    self.assertLessEqual(jump,1e-15)

 def test_implicit_substep_model_is_passive_and_does_not_force_a_state(self):
  # This is a CPU constitutive integrator check, not a PhysX calibration.
  law=DetentLaw();mass=.003;h=1/480
  for initial in (0.,law.travel):
   q=initial;v=.2 if initial==0. else -.2
   energy=.5*mass*v*v+law.potential(q)
   for _ in range(960):
    target=law.target(q)
    v=(mass*v-h*law.stiffness*(q-target))/(mass+h*law.damping+h*h*law.stiffness)
    q+=h*v;after=.5*mass*v*v+law.potential(q)
    self.assertLessEqual(after,energy+1e-14);energy=after
    self.assertEqual(law.target(q),initial)
   self.assertAlmostEqual(q,initial,places=6)
  # Ordinary external force exceeding the detent barrier force can cross it.
  q=0.;v=0.
  for _ in range(120):
   v=(mass*v+h*(.3-law.stiffness*(q-law.target(q))))/(mass+h*law.damping+h*h*law.stiffness);q+=h*v
   if q>law.travel/2:break
  self.assertGreater(q,law.travel/2)

 def test_measured_coordinate_is_invariant_under_dryer_tilt_and_translation(self):
  angle=.7;quat=(math.sin(angle/2),0.,0.,math.cos(angle/2));p=(.3,-.1,.9)
  for q in (0.,.001,.004):
   offset=rotate(quat,(-.037,0.,-.045+q));switch=tuple(a+b for a,b in zip(p,offset))
   self.assertAlmostEqual(slider_coordinate((*p,*quat),(*switch,*quat)),q)


class DetentCallbackTests(unittest.TestCase):
 def test_measured_coordinate_alone_selects_well_and_switching_energy_never_increases(self):
  coordinate=[0.];writes=[]
  detent=SubstepDetent(lambda:coordinate[0],lambda target:writes.append(target),1/480)
  for q in (0.,.0019,.0021,.0039,.004,.0021,.0019,0.):
   coordinate[0]=q;detent.on_physics_step(1/480)
  self.assertEqual(writes,[.004,0.]);self.assertLess(detent.retarget_energy,0.)
  self.assertEqual(detent.max_retarget_increase,0.)

 def test_requires_native_substep_cadence_and_rejects_a_policy_rate_fallback(self):
  detent=SubstepDetent(lambda:0.,lambda target:True,1/480)
  before=(detent.steps,detent.sim_time)
  for _ in range(24):detent.on_physics_step(1/480)
  detent.check_interval(*before,.05)
  with self.assertRaisesRegex(RuntimeError,'physics timestep'):detent.on_physics_step(.05)
  with self.assertRaisesRegex(RuntimeError,'one callback'):detent.check_interval(0,0.,.1)

 def test_nonfinite_measurement_and_failed_native_write_fail_closed(self):
  with self.assertRaises(ValueError):SubstepDetent(lambda:float('nan'),lambda target:True,1/480).on_physics_step(1/480)
  with self.assertRaisesRegex(RuntimeError,'native slider target'):SubstepDetent(lambda:.003,lambda target:False,1/480).on_physics_step(1/480)

 def test_event_records_authored_target_native_response_and_energy_without_claiming_latency(self):
  q=[0.];target=[0.]
  def write(value):target[0]=value
  detent=SubstepDetent(lambda:q[0],write,1/480,
    read_sample=lambda:{'switch_pose':[0.,0.,q[0],0.,0.,0.,1.],'switch_velocity_com_w':[0.,0.,.01,0.,0.,0.]},read_target=lambda:target[0])
  for value in [0.]*8+[.0021,.0025,.003]+[.004]*25:
   q[0]=value;detent.on_physics_step(1/480)
  trace=json.loads(json.dumps(detent.trace.export(),allow_nan=False));event=trace['events'][0]
  self.assertFalse(trace['native_target_application_latency_verified'])
  self.assertEqual(event['crossing_callback_index'],9)
  crossing=event['samples'][8];response=event['samples'][9]
  self.assertEqual(crossing['selected_well_m'],.004);self.assertEqual(crossing['authored_target_m'],.004)
  self.assertLess(crossing['retarget_energy_change_J'],0.)
  self.assertEqual(response['callback_index'],10);self.assertEqual(response['switch_pose'][2],.0025)
  self.assertEqual(len(response['switch_velocity_com_w']),6)
  self.assertAlmostEqual(response['potential_change_since_previous_callback_J'],response['elastic_potential_J']-crossing['elastic_potential_J'])
  self.assertEqual(event['post_remaining'],0)
  with self.assertRaisesRegex(RuntimeError,'attribute readback'):
   SubstepDetent(lambda:.003,lambda value:True,1/480,read_target=lambda:0.).on_physics_step(1/480)


class CrossingTraceTests(unittest.TestCase):
 def test_window_has_exact_pre_crossing_and_subsequent_samples(self):
  trace=CrossingTrace(pre_steps=3,post_steps=4,max_events=2)
  for i in range(1,25):trace.record({'callback_index':i},crossing=i==10)
  event=trace.export()['events'][0]
  self.assertEqual([sample['callback_index'] for sample in event['samples']],list(range(7,15)))
  self.assertEqual(event['post_remaining'],0);self.assertEqual(len(trace.history),3)

 def test_long_or_chattering_runs_stay_bounded_and_report_eviction(self):
  trace=CrossingTrace()
  for i in range(10000):trace.record({'callback_index':i},crossing=i<40)
  self.assertEqual(len(trace.events),16);self.assertEqual(trace.dropped,24)
  self.assertEqual(len(trace.history),8)
  self.assertTrue(all(len(event['samples'])<=33 and event['post_remaining']==0 for event in trace.events))


if __name__=='__main__':unittest.main()
