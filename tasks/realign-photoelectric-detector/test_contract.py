"""Adversarial optical/scoring checks; no simulator or paid API required."""
import unittest
import math
from dataclasses import replace
import numpy as np
from scorer import Evidence,Scorer
import optics,spec,clamp


GOOD=Evidence(True,False,True,True,True,True,0.,0.,True,True,False,True,3.2,3.2,-math.degrees(3.2/spec.DRIVER_SPRING_NM_RAD),0.,-35.,False)


class FrictionTests(unittest.TestCase):
    def test_subyield_deflection_does_not_move_rest(self):
        c=clamp.FrictionContact(1000.,0.)
        self.assertAlmostEqual(c.update(.0001,.245),0.)
        self.assertAlmostEqual(c.update(0.,.245),0.)

    def test_sliding_updates_rest_without_target_snapping(self):
        c=clamp.FrictionContact(1000.,0.)
        self.assertAlmostEqual(c.update(.1,3.2),.0968)
        self.assertAlmostEqual(c.update(.099,3.2),.0968)
        self.assertAlmostEqual(c.update(.09,3.2),.0932)

    def test_unloaded_contact_has_no_remembered_holding_force(self):
        c=clamp.FrictionContact(1000.,0.)
        self.assertEqual(c.update(.7,0.),.7)


class ScoreTests(unittest.TestCase):
    def sequence(self,s,e,n=10):
        return [s.update(e) for _ in range(n)]

    def confirmed(self):
        s=Scorer()
        self.sequence(s,replace(GOOD,tool_engaged=True,hands_clear=False,tool_clear=False))
        self.assertTrue(s.torque_confirmed)
        return s

    def test_full_score_requires_torque_and_optical_dwells(self):
        s=self.confirmed()
        self.assertEqual(s.update(replace(GOOD,clamp_closed=False)),.25)
        self.assertEqual(self.sequence(s,GOOD),[.5]*10)
        blocked=replace(GOOD,beam_on=False,card_blocks=True,card_parked=False)
        self.assertEqual(self.sequence(s,blocked),[0.]*10)
        self.assertEqual(self.sequence(s,GOOD,9),[.5]*9)
        self.assertEqual(s.update(GOOD),1.)
        self.assertEqual(s.update(replace(GOOD,card_parked=False)),.5)
        self.assertEqual(s.update(replace(GOOD,card_released=False)),.5)

    def test_interrupted_alignment_cannot_authorize_block_test(self):
        s=self.confirmed();self.sequence(s,GOOD,9)
        s.update(replace(GOOD,beam_on=False))
        s.update(GOOD)
        self.sequence(s,replace(GOOD,beam_on=False,card_blocks=True),10)
        self.assertEqual(self.sequence(s,GOOD,10),[.5]*10)

    def test_blocking_with_hand_or_tool_is_not_proof(self):
        for change in [{'hands_clear':False},{'tool_clear':False},{'card_blocks':False}]:
            with self.subTest(change=change):
                s=self.confirmed();self.sequence(s,GOOD)
                state=replace(GOOD,beam_on=False,card_blocks=True)
                self.sequence(s,replace(state,**change))
                self.assertEqual(self.sequence(s,GOOD),[.5]*10)

    def test_setting_change_clears_completed_proof(self):
        s=self.confirmed();self.sequence(s,GOOD)
        self.sequence(s,replace(GOOD,beam_on=False,card_blocks=True))
        self.assertEqual(self.sequence(s,replace(GOOD,sensor_yaw_deg=1.1)),[.5]*10)

    def test_failures_zero_partial_credit_and_latch(self):
        for change in [{'inside_workspace':False},{'intact':False},{'shortcut':True},
                       {'chassis_tilt_deg':15.01},{'sensor_yaw_deg':float('nan')}]:
            with self.subTest(change=change):
                s=self.confirmed();s.update(GOOD)
                self.assertEqual(s.update(replace(GOOD,**change)),0.)
                self.assertEqual(s.update(GOOD),0.)
                self.assertEqual(Scorer().update(GOOD),.25)

    def test_tilt_boundary_is_inclusive(self):
        self.assertEqual(Scorer().update(replace(GOOD,chassis_tilt_deg=15.)),.25)

    def test_angle_or_hidden_load_alone_never_proves_tightening(self):
        for angle in [-5.,-35.,-95.]:
            s=Scorer()
            self.assertEqual(self.sequence(s,replace(GOOD,nut_angle_deg=angle),60),[.25]*60)
            self.assertFalse(s.torque_confirmed)

    def test_torque_witness_rejects_underload_disengagement_and_bracing(self):
        witness=replace(GOOD,tool_engaged=True,hands_clear=False,tool_clear=False)
        for change in [{'tool_deflection_deg':-math.degrees(2.99/10.)},
                       {'tool_engaged':False},{'torque_path_clear':False},
                       {'tool_deflection_deg':math.degrees(3.2/10.)},
                       {'beam_on':False}]:
            s=Scorer();self.sequence(s,replace(witness,**change),20)
            self.assertFalse(s.torque_confirmed,change)

    def test_turning_output_is_not_a_secure_fastening(self):
        s=Scorer()
        for i in range(30):
            s.update(replace(GOOD,tool_engaged=True,nut_angle_deg=-35.-.2*i))
        self.assertFalse(s.torque_confirmed)

    def test_torque_witness_requires_uninterrupted_half_second(self):
        s=Scorer();w=replace(GOOD,tool_engaged=True)
        self.sequence(s,w,9);self.assertFalse(s.torque_confirmed)
        s.update(GOOD);self.sequence(s,w,9);self.assertFalse(s.torque_confirmed)
        s.update(w);self.assertTrue(s.torque_confirmed)

    def test_loosening_clears_torque_proof_and_overtorque_latches_failure(self):
        s=self.confirmed();self.sequence(s,GOOD)
        self.assertEqual(s.update(replace(GOOD,nut_angle_deg=-33.9)),.25)
        self.assertFalse(s.torque_confirmed)
        s=self.confirmed()
        self.assertEqual(s.update(replace(GOOD,peak_tightening_torque_nm=3.5001)),0.)
        self.assertEqual(s.update(GOOD),0.)

    def test_reset_preload_and_five_degree_turn_do_not_meet_requirement(self):
        self.assertAlmostEqual(clamp.load(0.)[1],spec.RESET_TORQUE)
        self.assertLess(clamp.load(5.)[1],spec.TORQUE_MIN)
        # The former 95-degree endpoint would grossly over-tighten this reset.
        self.assertGreater(clamp.load(95.)[1],spec.TORQUE_MAX)
        self.assertAlmostEqual(clamp.load(34.875)[1],3.20)


class OpticsTests(unittest.TestCase):
    def test_initial_misalignment_and_clear_alignment(self):
        from demo import rz
        for a,expected in [(spec.INITIAL_YAW,False),(0.,True)]:
            r=rz(a);origin=np.array(spec.PIVOT)+r@np.array(spec.LENS_LOCAL)
            self.assertEqual(optics.response(origin,r,spec.REFLECTOR,[])['on'],expected)

    def test_opaque_block_suppresses_return(self):
        origin=np.array(spec.PIVOT)+np.array(spec.LENS_LOCAL)
        result=optics.response(origin,np.eye(3),spec.REFLECTOR,[('card',(.57,-.02,.81),(.61,.02,.87))])
        self.assertFalse(result['on']);self.assertEqual(result['geometric_hits'],9)
        self.assertEqual(result['blocked_by'],['card'])

    def test_objects_behind_sensor_or_reflector_do_not_block(self):
        origin=np.array(spec.PIVOT)+np.array(spec.LENS_LOCAL)
        obstacles=[('behind',(.1,-1,0),(.2,1,2)),('beyond',(1.,-1,0),(1.1,1,2))]
        self.assertTrue(optics.response(origin,np.eye(3),spec.REFLECTOR,obstacles)['on'])

    def test_parallel_ray_outside_box(self):
        self.assertFalse(optics.ray_box((0,0,0),(1,0,0),(.2,.1,-1),(.4,.3,1),1.))
        self.assertTrue(optics.ray_box((0,0,0),(1,0,0),(.2,-.1,-1),(.4,.1,1),1.))

if __name__=='__main__':unittest.main()
