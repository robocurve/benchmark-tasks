"""Tests for task.py glue and interface exports in tower-transfer-no-big-on-small."""

import unittest


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("mark_left", obj_names)
        self.assertIn("mark_mid", obj_names)
        self.assertIn("mark_right", obj_names)
        self.assertIn("block_large", obj_names)
        self.assertIn("block_medium", obj_names)
        self.assertIn("block_small", obj_names)

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

