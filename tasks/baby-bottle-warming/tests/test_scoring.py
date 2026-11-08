import unittest
from scoring import TaskScorer


class ScoreTests(unittest.TestCase):
    def setUp(self):
        self.s=TaskScorer();self.state=dict(door_angle_deg=35,button_travel_m=0,bottle_in_bath=False,
            bottle_supported=False,released=False,lifted=False,neck_submerged=False,stable=True,valid=True)
    def step(self,n=1,**changes):
        self.state.update(changes)
        for _ in range(n):result=self.s.update(**self.state)
        return result
    def test_initial_zero(self):self.assertEqual(self.step(50)['score'],0)
    def test_lift_and_held_partial(self):
        self.assertEqual(self.step(6,lifted=True)['score'],.25)
        self.assertEqual(self.step(10,bottle_in_bath=True)['score'],.5)
    def test_full_requires_supported_released_stable_dwell(self):
        self.step(20,bottle_in_bath=True,bottle_supported=True)
        self.assertEqual(self.step(20,released=True,stable=False)['score'],.5)
        self.assertFalse(self.step(19,stable=True)['success'])
        self.assertTrue(self.step()['success'])
    def test_leaving_bath_removes_full(self):
        self.step(20,bottle_in_bath=True,bottle_supported=True,released=True)
        self.assertEqual(self.step(bottle_in_bath=False)['score'],0)
    def test_microwave_activation_permanent_zero(self):
        self.step(10,door_angle_deg=0);self.step(3,button_travel_m=.005)
        r=self.step(30,button_travel_m=0,bottle_in_bath=True,bottle_supported=True,released=True)
        self.assertTrue(r['microwave_started']);self.assertEqual(r['score'],0)
    def test_open_door_interlock(self):
        r=self.step(30,button_travel_m=.007,bottle_in_bath=True,bottle_supported=True,released=True)
        self.assertFalse(r['microwave_started']);self.assertEqual(r['score'],1)
    def test_neck_submersion_persists(self):
        self.step(neck_submerged=True)
        self.assertEqual(self.step(30,neck_submerged=False,bottle_in_bath=True,bottle_supported=True,released=True)['score'],0)
    def test_invalid_and_reset(self):
        self.step(valid=False);self.assertEqual(self.step(30,valid=True,bottle_in_bath=True,bottle_supported=True,released=True)['score'],0)
        self.s.reset();self.assertEqual(self.step(20)['score'],1)


if __name__=='__main__':unittest.main()
