import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scoring import TaskScorer


class ScoreTests(unittest.TestCase):
    def setUp(self):
        self.scorer = TaskScorer()
        self.step = 0

    def update(self, **changes):
        self.step += 1
        state = dict(step=self.step, egg_clear=False, egg_supported=True,
                     pot_on_trivet=False, released=True, stable=True,
                     pot_egg_force_n=0., valid=True)
        state.update(changes)
        return self.scorer.update(**state)

    def test_initial_state_has_no_reward(self):
        for _ in range(60):
            self.assertEqual(self.update()["score"], 0.)

    def test_safe_clearance_partial(self):
        for _ in range(6):
            result = self.update(egg_clear=True)
        self.assertEqual(result["score"], .25)

    def test_supported_pot_while_still_held_is_partial(self):
        for _ in range(30):
            result = self.update(egg_clear=True, pot_on_trivet=True, released=False)
        self.assertEqual(result["score"], .5)
        self.assertFalse(result["success"])

    def test_full_score_requires_supported_egg(self):
        for _ in range(40):
            result = self.update(egg_clear=True, pot_on_trivet=True, egg_supported=False)
        self.assertEqual(result["score"], .5)

    def test_released_stable_hold_reaches_full(self):
        for _ in range(20):
            result = self.update(egg_clear=True, pot_on_trivet=True)
        self.assertEqual(result["score"], 1.)
        self.assertTrue(result["success"])

    def test_duplicate_samples_cannot_complete_dwell(self):
        self.update(egg_clear=True, pot_on_trivet=True)
        for _ in range(50):
            result = self.scorer.update(step=self.step, egg_clear=True, egg_supported=True,
                pot_on_trivet=True, released=True, stable=True, pot_egg_force_n=0.)
        self.assertFalse(result["success"])
        self.assertEqual(result["safe_hold_steps"], 1)

    def test_pot_contact_is_irreversible(self):
        self.update(pot_egg_force_n=3.)
        for _ in range(40):
            result = self.update(egg_clear=True, pot_on_trivet=True)
        self.assertEqual(result["score"], 0.)
        self.assertTrue(result["pot_contact_ever"])

    def test_invalid_state_latches_and_reset_clears(self):
        self.update(pot_egg_force_n=float("nan"))
        self.assertEqual(self.update(egg_clear=True, pot_on_trivet=True)["score"], 0.)
        self.scorer.reset()
        for _ in range(20):
            result = self.update(egg_clear=True, pot_on_trivet=True)
        self.assertEqual(result["score"], 1.)


if __name__ == "__main__":
    unittest.main()
