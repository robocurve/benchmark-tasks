import unittest
from scoring import TaskScorer


class ScoringTests(unittest.TestCase):
    def test_idle_and_near_miss(self):
        scorer=TaskScorer()
        for _ in range(100): result=scorer.update(.0079,[19.9,-19.9])
        self.assertEqual(result['score'],0)

    def test_safe_shutoff(self):
        scorer=TaskScorer()
        for _ in range(22): result=scorer.update(.008,[0,0])
        self.assertEqual(result['score'],1)
        self.assertTrue(result['success'])

    def test_short_contact_does_not_latch(self):
        scorer=TaskScorer()
        for _ in range(2): scorer.update(.012,[0,0])
        for _ in range(40): result=scorer.update(0,[0,0])
        self.assertEqual(result['score'],0)

    def test_either_knob_history_caps_score(self):
        for angles in ([20,0],[0,-20]):
            scorer=TaskScorer()
            for _ in range(3): result=scorer.update(0,angles)
            self.assertEqual(result['score'],.2)
            for _ in range(40): result=scorer.update(.012,[0,0])
            self.assertEqual(result['score'],.2)
            self.assertFalse(result['success'])

    def test_invalid_geometry_prevents_credit(self):
        scorer=TaskScorer()
        scorer.update(0,[30,0],valid=False)
        for _ in range(30): result=scorer.update(.012,[30,0])
        self.assertEqual(result['score'],0)

    def test_reset_clears_history(self):
        scorer=TaskScorer()
        for _ in range(3): scorer.update(0,[20,0])
        scorer.reset()
        for _ in range(22): result=scorer.update(.012,[0,0])
        self.assertEqual(result['score'],1)


if __name__=='__main__': unittest.main()
