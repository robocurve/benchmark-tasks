"""The live per-step judge must equal batch re-scoring at every recorded step."""
import unittest
from drawstring_score import DrawstringJudge, evaluate
from test_drawstring_score import episode


class StreamingScoringTests(unittest.TestCase):
    def test_streaming_matches_batch_at_each_recorded_state(self):
        trace, states, p = episode()
        judge = DrawstringJudge(p)
        for i, s in enumerate(trace):
            self.assertEqual(judge.update(s, states[s['step']]), evaluate(trace[:i + 1], states, p))


if __name__ == '__main__':
    unittest.main()
