"""Unit tests for task glue in weigh-and-reject-heavy-block."""
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
        self.assertIn("Weigh each grey block", T.INSTRUCTION)

    def test_objects_list(self):
        self.assertIn(T.SCALE_FIXTURE, T.OBJECTS)
        self.assertIn(T.SUPPLY_RACK, T.OBJECTS)
        self.assertIn(T.SHELF, T.OBJECTS)
        self.assertIn(T.TRAY, T.OBJECTS)
        self.assertIn(T.METER_FIXTURE, T.OBJECTS)
        self.assertIn(T.NEEDLE_MARKER, T.OBJECTS)
        self.assertEqual(len(T.OBJECTS), 6 + 3)  # 6 fixtures/markers + 3 blocks

    def test_scale_slider_registered(self):
        slider_names = {s.name for s in T.SCALE_FIXTURE.sliders}
        self.assertIn("scale_platform", slider_names)


if __name__ == "__main__":
    unittest.main()
