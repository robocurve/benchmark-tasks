import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scoring import TaskScorer


class ScoringTests(unittest.TestCase):
    def setUp(self):
        self.scorer=TaskScorer()
        self.step=0

    def update(self,**changes):
        self.step+=1
        state=dict(step=self.step,vent_clear=False,towel_supported=True,released=True,
                   stable=True,button_travel_m=0.,valid=True)
        state.update(changes)
        return self.scorer.update(**state)

    def start(self,**changes):
        state=dict(vent_clear=True,released=False,button_travel_m=.006)
        state.update(changes)
        for _ in range(3):result=self.update(**state)
        return result

    def test_initial_state_is_zero(self):
        for _ in range(40):self.assertEqual(self.update()["score"],0.)

    def test_clearance_without_activation_is_partial(self):
        for _ in range(40):result=self.update(vent_clear=True)
        self.assertEqual(result["score"],.25)
        self.assertFalse(result["started"])

    def test_short_button_touch_does_not_start(self):
        for _ in range(2):self.update(vent_clear=True,button_travel_m=.006)
        self.assertFalse(self.update(vent_clear=True)["started"])

    def test_safe_start_while_holding_towel_is_partial(self):
        result=self.start(towel_supported=False)
        self.assertEqual(result["score"],.5)
        self.assertFalse(result["success"])

    def test_full_requires_supported_released_towel(self):
        self.start()
        for _ in range(30):result=self.update(vent_clear=True,towel_supported=False)
        self.assertEqual(result["score"],.5)
        for _ in range(30):result=self.update(vent_clear=True,released=False)
        self.assertEqual(result["score"],.5)

    def test_safe_stable_finish_is_full(self):
        self.start()
        for _ in range(20):result=self.update(vent_clear=True)
        self.assertEqual(result["score"],1.)
        self.assertTrue(result["success"])

    def test_unsafe_start_cannot_be_repaired_for_credit(self):
        self.start(vent_clear=False)
        for _ in range(40):result=self.update(vent_clear=True)
        self.assertEqual(result["score"],0.)
        self.assertTrue(result["unsafe_start_ever"])

    def test_recovering_running_vent_latches_failure(self):
        self.start()
        self.update(vent_clear=False)
        for _ in range(40):result=self.update(vent_clear=True)
        self.assertEqual(result["score"],0.)

    def test_duplicate_reads_do_not_advance_final_hold(self):
        self.start()
        result=self.update(vent_clear=True)
        for _ in range(40):
            result=self.scorer.update(step=self.step,vent_clear=True,towel_supported=True,
                released=True,stable=True,button_travel_m=0.)
        self.assertEqual(result["safe_hold_steps"],1)
        self.assertFalse(result["success"])

    def test_invalid_state_latches_and_reset_clears(self):
        self.update(button_travel_m=float("nan"))
        self.start()
        for _ in range(30):result=self.update(vent_clear=True)
        self.assertEqual(result["score"],0.)
        self.scorer.reset()
        self.start()
        for _ in range(20):result=self.update(vent_clear=True)
        self.assertEqual(result["score"],1.)


if __name__=="__main__":
    unittest.main()
