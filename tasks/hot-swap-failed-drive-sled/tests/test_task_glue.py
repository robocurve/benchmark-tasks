"""Unit tests for task glue in hot-swap-failed-drive-sled."""
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import task as T


class TestTaskGlue(unittest.TestCase):
    def test_task_attributes(self):
        self.assertEqual(T.SCENE, "station")
        self.assertEqual(T.SCORE_OVER, "end")
        self.assertEqual(T.SUCCESS_HOLD_S, 1.0)
        self.assertGreater(len(T.INSTRUCTION), 20)
        self.assertIn("key tab", T.INSTRUCTION)
        self.assertIn("discard pad", T.INSTRUCTION)
        self.assertNotIn("latch", T.INSTRUCTION)  # goal only: the method is not given away

    def test_objects_list(self):
        self.assertIn(T.RACK_FIXTURE, T.OBJECTS)
        self.assertIn(T.SPARE_STAND, T.OBJECTS)
        self.assertIn(T.DISCARD_PAD, T.OBJECTS)
        for bay in ("bay_0", "bay_1", "bay_2"):
            self.assertIn(T.STATUS_LAMPS[bay], T.OBJECTS)
            self.assertIn(T.SLEDS[bay], T.OBJECTS)
        self.assertIn(T.SPARES["spare_blue"], T.OBJECTS)
        self.assertIn(T.SPARES["spare_orange"], T.OBJECTS)

    def test_sliders_registered(self):
        slider_names = {s.name for s in T.RACK_FIXTURE.sliders}
        self.assertIn("latch_bay_0", slider_names)
        self.assertIn("latch_bay_1", slider_names)
        self.assertIn("latch_bay_2", slider_names)


if __name__ == "__main__":
    unittest.main()

