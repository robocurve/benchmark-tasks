"""Unit tests for task.py glue exports and interface compliance."""

import unittest
from pathlib import Path


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertIn("Press the red E-STOP", task.INSTRUCTION)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("console_fixture", obj_names)
        self.assertIn("lane_fixture", obj_names)
        self.assertIn("green_tray", obj_names)
        self.assertIn("parked_pad", obj_names)
        self.assertIn("orange_block", obj_names)
        self.assertIn("pusher", obj_names)
        self.assertIn("run_lamp", obj_names)
        self.assertIn("stop_lamp", obj_names)
        self.assertIn("fault_lamp", obj_names)

        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))
        self.assertTrue(callable(task.plans))


if __name__ == "__main__":
    unittest.main()
