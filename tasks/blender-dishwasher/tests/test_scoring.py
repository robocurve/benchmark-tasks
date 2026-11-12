"""Behavior checks for delivery credit and the irreversible electrical-base veto."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scoring import TaskScorer,PARTS

class ScoringTests(unittest.TestCase):
    def test_empty_partial_full_and_loss(self):
        s=TaskScorer()
        for step in range(20):r=s.update(step=step,eligible=dict.fromkeys(PARTS,False),motor_in_dishwasher=False)
        self.assertEqual(r['score'],0)
        for step in range(20,39):r=s.update(step=step,eligible=dict(zip(PARTS,[True,False])),motor_in_dishwasher=False)
        self.assertEqual(r['score'],0)
        r=s.update(step=39,eligible=dict(zip(PARTS,[True,False])),motor_in_dishwasher=False);self.assertEqual(r['score'],.5)
        for step in range(40,60):r=s.update(step=step,eligible=dict.fromkeys(PARTS,True),motor_in_dishwasher=False)
        self.assertEqual(r['score'],1)
        r=s.update(step=60,eligible=dict.fromkeys(PARTS,False),motor_in_dishwasher=False);self.assertEqual(r['score'],0)
    def test_motor_entry_is_permanent_until_reset(self):
        s=TaskScorer()
        for step in range(60):r=s.update(step=step,eligible=dict.fromkeys(PARTS,True),motor_in_dishwasher=step==25)
        self.assertEqual(r['score'],0);self.assertTrue(r['motor_entered_ever'])
        s.reset()
        for step in range(20):r=s.update(step=step,eligible=dict.fromkeys(PARTS,True),motor_in_dishwasher=False)
        self.assertEqual(r['score'],1)
    def test_duplicate_frames_do_not_inflate_dwell(self):
        s=TaskScorer()
        for _ in range(30):r=s.update(step=0,eligible=dict.fromkeys(PARTS,True),motor_in_dishwasher=False)
        self.assertEqual(r['score'],0);self.assertEqual(r['delivery_dwell_steps'][PARTS[0]],1)
    def test_invalid_state_cannot_receive_credit(self):
        s=TaskScorer()
        for step in range(30):r=s.update(step=step,eligible=dict.fromkeys(PARTS,True),motor_in_dishwasher=False,valid=step!=5)
        self.assertEqual(r['score'],0);self.assertTrue(r['invalid_state_ever'])

if __name__=='__main__':unittest.main()
