"""Material geometry and scoring counterexamples; CPU checks, not robot proof."""
import unittest
import numpy as np
from stock import Stock


class CuttingGeometry(unittest.TestCase):
    def test_partial_width_keeps_one_piece(self):
        wood=Stock()
        ix=round(.150/wood.dx)
        wood.cleared[ix:ix+4,:wood.ny//2]=wood.nz
        self.assertEqual(len(wood.components()),1)
        self.assertEqual(wood.score()['value'],0.)

    def test_wrong_length_is_failure_after_real_disconnection(self):
        wood=Stock()
        ix=round(.155/wood.dx)
        wood.cleared[ix:ix+4,:]=wood.nz
        self.assertEqual(len(wood.components()),2)
        self.assertEqual(wood.score()['reason'],'wrong_length')
        self.assertEqual(wood.score(target=.155)['value'],1.)

    def test_full_width_correct_length_passes(self):
        wood=Stock()
        ix=round(.150/wood.dx)
        wood.cleared[ix:ix+4,:]=wood.nz
        result=wood.score()
        self.assertEqual(result['value'],1.)
        self.assertEqual(result['pieces'],2)
        self.assertLessEqual(result['max_error_m'],.001)
        left=wood.components()[0]
        boxes=wood.boxes(left)
        self.assertAlmostEqual(max(b[0]+b[3]/2 for b in boxes),.150)

    def test_actual_tooth_line_selects_material_and_requires_motion(self):
        a=Stock();b=Stock()
        a.remove((.1455,-.02,.02),(.1455,.02,.02),.02,5)
        b.remove((.1555,-.02,.02),(.1555,.02,.02),.02,5)
        self.assertLess(max(np.where(a.progress>0)[0]),min(np.where(b.progress>0)[0]))
        idle=Stock();idle.remove((.15,-.02,.02),(.15,.02,.02),0,5)
        idle.remove((.15,-.02,.02),(.15,.02,.02),.02,0)
        self.assertFalse(np.any(idle.progress))
        tiny=Stock()
        self.assertFalse(tiny.remove((.15,-.02,.02),(.15,.02,.02),.0001,5))
        self.assertTrue(tiny.removed_last_step)
        self.assertGreater(float(tiny.progress.max()),0.)

    def test_more_normal_force_removes_more_at_equal_stroke(self):
        low=Stock();high=Stock()
        tooth0=(.15,-.02,.02);tooth1=(.15,.02,.02)
        low.remove(tooth0,tooth1,.001,1.)
        high.remove(tooth0,tooth1,.001,10.)
        self.assertGreater(float(high.progress.sum()),float(low.progress.sum()))


if __name__=='__main__':unittest.main()
