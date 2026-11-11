"""Unit tests for task.py glue interfaces."""

import unittest

import task


class TestTaskGlue(unittest.TestCase):
    def test_station_scene(self):
        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")

    def test_objects_list(self):
        self.assertGreaterEqual(len(task.OBJECTS), 6)
        names = [o.name for o in task.OBJECTS]
        self.assertIn("strip", names)
        self.assertIn("slab_1", names)
        self.assertIn("slab_2", names)
        self.assertIn("slab_3", names)
        self.assertIn("slab_4", names)
        self.assertIn("slab_5", names)

    def test_demo_returns_plans(self):
        p = task.demo()
        self.assertIn("left", p)
        self.assertIn("right", p)


if __name__ == "__main__":
    unittest.main()

