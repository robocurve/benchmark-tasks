import unittest
from dataclasses import replace
from scorer import Evidence,Scorer

GOOD=Evidence(.08,.15,.360,0.,0.,0.,0.,True,True,True,True,True,True,0.)
class Contract(unittest.TestCase):
    def test_dwell_interruptions(self):
        s=Scorer()
        self.assertEqual([s.update(GOOD) for _ in range(9)],[.5]*9)
        self.assertEqual(s.update(replace(GOOD,hands_clear=False)),.5)
        self.assertEqual([s.update(GOOD) for _ in range(9)],[.5]*9)
        self.assertEqual(s.update(GOOD),1.)
    def test_partial_credit_current_state(self):
        s=Scorer()
        self.assertEqual(s.update(replace(GOOD,axle_x=.455,wheels_max_x=.5185,cart_max_x=.7344)),0.)
        self.assertEqual(s.update(replace(GOOD,axle_x=.415,wheels_max_x=.4785,cart_max_x=.6944)),.25)
        self.assertEqual(s.update(replace(GOOD,axle_x=.455,wheels_max_x=.5185,cart_max_x=.7344)),0.)
    def test_invalid_outcomes_latch(self):
        for change in [{'case_on_plate':False},{'inside_workspace':False},{'intact':False},{'pitch_deg':35.01},{'roll_deg':15.01},{'axle_x':float('nan')},{'wheel_support_gap_m':.00301}]:
            s=Scorer();self.assertEqual(s.update(replace(GOOD,**change)),0.);self.assertEqual(s.update(GOOD),0.)
    def test_carrying_zeroes_every_tier_and_latches(self):
        for state in [replace(GOOD,axle_x=.414,wheels_max_x=.48),replace(GOOD,cart_max_x=.5),GOOD]:
            s=Scorer()
            for _ in range(12):self.assertEqual(s.update(replace(state,wheel_support_gap_m=.020)),0.)
            self.assertEqual(s.update(GOOD),0.)
    def test_wheels_only_cannot_complete(self):
        for change in [{'cart_max_x':.366},{'stable':False},{'plate_floor_gap_m':.003},{'yaw_deg':6.}]:
            s=Scorer();self.assertEqual([s.update(replace(GOOD,**change)) for _ in range(20)],[.5]*20)
    def test_retained_case_need_not_preserve_exact_reset_alignment(self):
        s=Scorer()
        for _ in range(10):score=s.update(replace(GOOD,case_aligned=False))
        self.assertEqual(score,1.)
if __name__=='__main__':unittest.main()
