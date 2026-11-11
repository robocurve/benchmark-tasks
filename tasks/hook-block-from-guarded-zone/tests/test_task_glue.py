"""Tests for task.py glue and interface exports in hook-block-from-guarded-zone."""

import unittest


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("yellow_line", obj_names)
        self.assertIn("green_tray", obj_names)
        self.assertIn("hooked_rod", obj_names)
        self.assertIn("red_block", obj_names)

        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))
        self.assertTrue(callable(task.plans))
        self.assertTrue(callable(task.reset))

    def test_demo_returns_plans(self):
        import task
        p = task.demo()
        self.assertIn("left", p)
        self.assertIn("right", p)


if __name__ == "__main__":
    unittest.main()
