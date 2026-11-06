"""Unit tests verifying task.py exports and integration glue."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import task


class TestTwoKeyTaskGlue(unittest.TestCase):
    def test_task_attributes(self):
        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertEqual(task.SUCCESS_HOLD_S, 1.0)
        self.assertGreater(len(task.INSTRUCTION), 20)
        self.assertIn("Slide both red guards aside", task.INSTRUCTION)

    def test_objects_list(self):
        self.assertIn(task.FIXTURE, task.OBJECTS)
        self.assertIn(task.ARMED_LAMP, task.OBJECTS)
        self.assertIn(task.FAULT_LAMP, task.OBJECTS)
        for lamp in task.PROGRESS_LAMPS:
            self.assertIn(lamp, task.OBJECTS)
        self.assertEqual(len(task.OBJECTS), 1 + 1 + 1 + len(g.PROGRESS_LAMP_Y))

    def test_sliders_registered(self):
        slider_names = {s.name for s in task.FIXTURE.sliders}
        expected = {
            "btn_key_left",
            "btn_key_right",
            "btn_confirm",
            "slider_guard_left",
            "slider_guard_right",
        }
        self.assertEqual(slider_names, expected)


if __name__ == "__main__":
    unittest.main()
