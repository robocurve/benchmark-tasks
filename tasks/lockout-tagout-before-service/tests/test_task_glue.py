"""Tests for task.py glue and interface exports."""

import unittest
from pathlib import Path


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("console_fixture", obj_names)
        self.assertIn("red_lock_pin", obj_names)
        self.assertIn("live_lamp", obj_names)
        self.assertIn("dead_lamp", obj_names)

        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))
        self.assertTrue(callable(task.plans))


if __name__ == "__main__":
    unittest.main()

