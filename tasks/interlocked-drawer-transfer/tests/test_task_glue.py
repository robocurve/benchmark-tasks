"""Tests for task.py glue and interface exports in interlocked-drawer-transfer."""

import unittest


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("cabinet_fixture", obj_names)
        self.assertIn("lamp_ok", obj_names)
        self.assertIn("lamp_tip", obj_names)
        self.assertIn("white_block", obj_names)

        fixture = next(obj for obj in task.OBJECTS if obj.name == "cabinet_fixture")
        slider_names = [s.name for s in fixture.sliders]
        self.assertIn("drawer_left", slider_names)
        self.assertIn("drawer_right", slider_names)

        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))
        self.assertTrue(callable(task.plans))


if __name__ == "__main__":
    unittest.main()
