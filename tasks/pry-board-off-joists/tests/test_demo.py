"""Regression for visible nail clearance followed by the loose board settling."""
import pathlib
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import demo as D


class Demo(unittest.TestCase):
    def test_does_not_pry_again_after_cleared_end_settles(self):
        observed_gap = [0.0]
        strokes = []

        def lever(_ctx, _arm, _side, want):
            strokes.append(want)
            observed_gap[0] = .0041 if len(strokes) == 1 else .0175
            yield

        class Arm:
            side = "left"

            def at(self):
                return np.array([.15, .20, D.L.TOP + .20])

            def move(self, *args, **kwargs):
                return iter(())

            def arc(self, *args, **kwargs):
                return iter(())

            def wait(self, _seconds):
                if observed_gap[0] >= D.GAP_DONE:
                    observed_gap[0] = 0.0  # the loose board falls back onto the joist
                yield

        def no_motion(*args, **kwargs):
            return iter(())

        with patch.multiple(D, lever=lever, gap=lambda *_: observed_gap[0],
                            level_blade=no_motion, bring_tip=no_motion, relax=no_motion,
                            report=lambda *_: None,
                            bar_world=lambda *_: np.array([.36, .20, D.L.JOIST_TOP])):
            list(D.pry(None, Arm(), "left", withdraw=False))
        self.assertEqual(len(strokes), 2)
        self.assertEqual(observed_gap[0], 0.0)


if __name__ == "__main__":
    unittest.main()
