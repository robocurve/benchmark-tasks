"""Unit tests for task.py glue interfaces and exports."""

import unittest

import task


class TestTaskGlue(unittest.TestCase):
    def test_station_scene(self):
        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")

    def test_objects_list(self):
        self.assertGreaterEqual(len(task.OBJECTS), 5)
        names = [o.name for o in task.OBJECTS]
        self.assertIn("frame", names)
        self.assertIn("card", names)
        self.assertIn("tile_blue", names)
        self.assertIn("tile_orange", names)
        self.assertIn("tile_yellow", names)

    def test_demo_returns_plans(self):
        p = task.demo()
        self.assertIn("left", p)
        self.assertIn("right", p)


if __name__ == "__main__":
    unittest.main()

