"""Tests for task.py glue and interface exports."""

import unittest


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("pallet_mark", obj_names)
        self.assertIn("crate_large", obj_names)
        self.assertIn("crate_medium", obj_names)
        self.assertIn("crate_small", obj_names)

        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))
        self.assertTrue(callable(task.plans))
        self.assertTrue(callable(task.reset))


if __name__ == "__main__":
    unittest.main()

