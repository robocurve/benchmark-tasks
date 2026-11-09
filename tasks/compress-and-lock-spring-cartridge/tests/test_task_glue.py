"""Tests for task.py interface glue and exports in compress-and-lock-spring-cartridge."""

import unittest


class TestTaskGlue(unittest.TestCase):
    def test_task_interface(self):
        import task

        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 10)

        obj_names = [obj.name for obj in task.OBJECTS]
        self.assertIn("cartridge_fixture", obj_names)
        self.assertIn("status_lamp", obj_names)

        fixture = next(obj for obj in task.OBJECTS if obj.name == "cartridge_fixture")
        slider_names = [s.name for s in fixture.sliders]
        self.assertIn("plunger_slider", slider_names)
        self.assertIn("lock_bar_slider", slider_names)

        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))
        self.assertTrue(callable(task.plans))
        self.assertTrue(callable(task.reset))


if __name__ == "__main__":
    unittest.main()
