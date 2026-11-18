"""The revised task cannot be passed by its old straight-only demo."""
import unittest
from pathlib import Path
import numpy as np
from scoring import assess,assess_segment

class CornerChecks(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  with np.load(Path(__file__).parent/'evidence/straight-negative-control.npz',allow_pickle=False) as a:
   cls.s={k:a[k].copy() for k in a.files}
 def test_old_success_is_incomplete(self):
  s=self.s;r=assess(s['x'],s['y'],s['volume'],dx=float(s['dx']),emitted_m3=float(s['emitted_m3']),completed=True,tools_clear=True)
  self.assertLess(r['score'],1.);self.assertFalse(r['corner_connected']);self.assertEqual(r['section_lengths_mm'][1],0.)
 def test_shifted_border_fails_straightness(self):
  s=self.s;v=np.roll(s['volume'],10,axis=0)
  r=assess_segment(s['x'],s['y'],v,dx=float(s['dx']),emitted_m3=float(v.sum()),completed=True,tools_clear=True)
  self.assertEqual(r['score'],0.)
 def test_blank_corner_cannot_score(self):
  s=self.s;r=assess(s['x'],s['y'],s['volume']*0,dx=float(s['dx']),emitted_m3=0.,completed=True,tools_clear=True)
  self.assertEqual(r['score'],0.)
if __name__=='__main__':unittest.main()
