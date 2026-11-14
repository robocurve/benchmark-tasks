"""The scoring rules, driven through task._STATE (the monitor's state), without the simulator: the start state scores 0,
each ladder step, undoing a step lowers the score again, every latched violation caps it, and success needs everything."""

import importlib
import math
import os
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def load_task(variant="A"):
    os.environ["TASK_VARIANT"] = variant
    for m in ("task", "geometry"):
        sys.modules.pop(m, None)
    return importlib.import_module("task")


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.t = load_task("A")
        self.g = self.t.g
        self.S = self.t._STATE
        self.t._reset_state()

    def put(self, **kw):
        self.S.update(kw)

    def right_way_shut(self):
        self.put(shut=True, raised=0.0, err=math.radians(10), moved=0.0, yaw=0.0, tilt=0.0, clear=True)

    def test_start_state_scores_zero(self):
        self.assertEqual(self.t.score(None), 0.0)
        self.assertFalse(self.t.success(None))
        self.assertGreaterEqual(self.S["err"], math.radians(170))     # layout A: 180 degrees to turn

    def test_ladder_steps(self):
        self.put(shut=False, raised=math.radians(80))
        self.assertEqual(self.t.score(None), 0.25)                     # the lever has been raised
        self.put(err=math.radians(80))
        self.assertEqual(self.t.score(None), 0.50)                     # the guard within 90 degrees
        self.put(shut=True, raised=0.0, err=math.radians(40))
        self.assertEqual(self.t.score(None), 0.75)                     # shut and within 45 degrees
        self.put(err=math.radians(10), clear=True)
        self.assertEqual(self.t.score(None), 1.0)
        self.assertTrue(self.t.success(None))

    def test_undoing_a_step_lowers_the_score(self):
        self.right_way_shut()
        self.assertEqual(self.t.score(None), 1.0)
        self.put(shut=False, raised=math.radians(70))                  # lever raised again
        self.assertEqual(self.t.score(None), 0.50)                     # no longer 0.75 or 1.0
        self.put(shut=True, raised=0.0, err=math.radians(100))         # the guard turned back the wrong way, clamped
        self.assertEqual(self.t.score(None), 0.0)

    def test_the_score_is_the_state_now_never_the_best_so_far(self):
        self.put(shut=False, raised=math.radians(80), err=math.radians(180))
        self.assertEqual(self.t.score(None), 0.25)                     # the lever is up
        self.put(shut=True, raised=0.0)                                # shut it again without turning the guard
        self.assertEqual(self.t.score(None), 0.0)
        self.put(shut=False, raised=math.radians(80), err=math.radians(40))
        self.assertEqual(self.t.score(None), 0.50)
        self.put(shut=False, raised=math.radians(80), err=math.radians(150))   # turned back
        self.assertEqual(self.t.score(None), 0.25)

    def test_a_lever_between_8_and_40_degrees_from_flat_cannot_score_full(self):
        """The clamp's snap state says shut below 40 degrees, but full credit needs the measured lever within 8 degrees."""
        for deg in (8.5, 15, 25, 39.9):
            self.right_way_shut()
            self.put(shut=True, raised=math.radians(deg), lever_open=False)   # even with the snap state "shut"
            self.assertFalse(self.t.success(None), deg)
            self.assertLess(self.t.score(None), 1.0, deg)
            self.assertLess(self.t.score(None), 0.75, deg)                   # nor the lever-flat 0.75 rung
        for deg in (0.0, 4.0, 8.0):
            self.right_way_shut()
            self.put(raised=math.radians(deg), lever_open=False)
            self.assertTrue(self.t.success(None), deg)
            self.assertEqual(self.t.score(None), 1.0, deg)

    def test_the_lever_shut_with_the_guard_wrong_is_not_075(self):
        self.put(shut=True, raised=0.0, err=math.radians(60))
        self.assertEqual(self.t.score(None), 0.50)

    def test_success_needs_every_condition(self):
        for key, bad in (("raised", math.radians(8.5)), ("lever_open", True), ("err", math.radians(31)), ("moved", 0.021), ("yaw", math.radians(11)),
                         ("tilt", math.radians(11)), ("clear", False)):
            self.right_way_shut()
            self.put(**{key: bad})
            self.assertFalse(self.t.success(None), key)
            self.assertLess(self.t.score(None), 1.0, key)

    def test_each_violation_latches_and_caps_at_025(self):
        for kind in ("disc_touch", "forced", "shoved"):
            self.t._reset_state()
            self.right_way_shut()
            self.assertEqual(self.t.score(None), 1.0)
            self.t._flag(kind, f"{kind} test")
            self.assertEqual(self.t.score(None), 0.25, kind)
            self.assertFalse(self.t.success(None), kind)
            self.right_way_shut()                                      # setting the state right again does not clear it
            self.assertEqual(self.t.score(None), 0.25, kind)

    def test_a_reset_clears_everything(self):
        self.t._flag("disc_touch", "x")
        self.right_way_shut()
        self.t._reset_state()
        self.assertEqual(self.t.score(None), 0.0)
        self.assertEqual(self.S["why"], [])

    def test_the_guard_alone_turned_by_shoving_the_grinder_does_not_count(self):
        # turning the whole grinder round (rather than the guard) is a shove: it caps the score at 0.25
        self.put(shut=True, err=math.radians(5), moved=0.05, yaw=math.radians(170))
        self.t._flag("shoved", "shoved")
        self.assertEqual(self.t.score(None), 0.25)

    def test_layout_b_has_its_own_start_and_target(self):
        t = load_task("B")
        t._reset_state()
        g = t.g
        self.assertAlmostEqual(math.degrees(t._STATE["err"]), 120.0, places=3)
        self.assertEqual(t.score(None), 0.0)
        self.assertAlmostEqual(math.degrees(g.layout("B")["operator_bearing"]), 270.0)


if __name__ == "__main__":
    unittest.main()
