"""Tests for task.py glue and interface exports in bridge-plate-across-trench."""

import unittest


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("platform_near", obj_names)
        self.assertIn("platform_far", obj_names)
        self.assertIn("target_mark", obj_names)
        self.assertIn("bridge_plate", obj_names)
        self.assertIn("block", obj_names)

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
