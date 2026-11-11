"""Behavioral checks for credit, timing, irreversible failure and geometry."""
import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scoring import TaskScorer, CRACKERS

class ScoreTests(unittest.TestCase):
    def advance(self,scorer,n,*,packet=False,start=0,steps=20):
        eligible={name:i<n for i,name in enumerate(CRACKERS)}
        for step in range(start,start+steps):
            result=scorer.update(step=step,eligible=eligible,packet_in_basket=packet)
        return result
    def test_every_count(self):
        for n in range(7):
            with self.subTest(n=n):
                r=self.advance(TaskScorer(),n)
                self.assertAlmostEqual(r['score'],n/6)
                self.assertEqual(r['success'],n==6)
    def test_packet_overrides_every_count(self):
        for n in range(7):
            self.assertEqual(self.advance(TaskScorer(),n,packet=True)['score'],0)
    def test_packet_removal_does_not_restore_credit(self):
        s=TaskScorer();self.advance(s,0,packet=True,steps=1)
        self.assertEqual(self.advance(s,6,start=1)['score'],0)
        self.assertTrue(s.packet_entered_ever)
    def test_trial_reset_clears_packet_failure(self):
        s=TaskScorer();self.advance(s,6,packet=True);s.reset()
        self.assertEqual(self.advance(s,6)['score'],1)
    def test_lost_cracker_only_loses_own_credit(self):
        s=TaskScorer();self.advance(s,5)
        self.assertAlmostEqual(self.advance(s,4,start=20,steps=1)['score'],4/6)
    def test_one_second_dwell(self):
        s=TaskScorer();self.assertEqual(self.advance(s,6,steps=19)['score'],0)
        self.assertEqual(self.advance(s,6,start=19,steps=1)['score'],1)
    def test_repeat_reads_do_not_advance_dwell(self):
        s=TaskScorer()
        for _ in range(100):r=self.advance(s,6,steps=1)
        self.assertEqual(r['score'],0);self.assertEqual(r['delivery_dwell_steps']['cracker_1'],1)
    def test_skipped_steps_reset_dwell(self):
        s=TaskScorer();self.advance(s,6,steps=19)
        self.assertEqual(self.advance(s,6,start=25,steps=1)['score'],0)
    def test_invalid_state(self):
        s=TaskScorer();self.advance(s,6)
        r=s.update(step=20,eligible=dict.fromkeys(CRACKERS,True),packet_in_basket=False,valid=False)
        self.assertEqual(r['score'],0)
        self.assertEqual(self.advance(s,6,start=21)['score'],0)

if __name__=='__main__':unittest.main()
