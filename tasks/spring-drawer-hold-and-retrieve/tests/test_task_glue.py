"""Unit tests for task glue in spring-drawer-hold-and-retrieve."""
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
        self.assertIn("Pull the drawer fully open", T.INSTRUCTION)

    def test_objects_list(self):
        self.assertIn(T.CABINET_FIXTURE, T.OBJECTS)
        self.assertIn(T.SHELF, T.OBJECTS)
        self.assertIn(T.CYAN_LAMP, T.OBJECTS)
        self.assertIn(T.WHITE_BLOCK, T.OBJECTS)

    def test_slider_registered(self):
        slider_names = {s.name for s in T.CABINET_FIXTURE.sliders}
        self.assertIn("drawer_slider", slider_names)


if __name__ == "__main__":
    unittest.main()

