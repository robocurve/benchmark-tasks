"""Unit tests for task glue and interface definitions in clear-lane-then-push-block-to-dock."""

import os
from pathlib import Path
import unittest
import task
from demo_plan import MODES

HERE = Path(__file__).resolve().parent.parent


class TestTaskGlue(unittest.TestCase):
    def test_scene_definition(self):
        self.assertEqual(task.SCENE, "station")

    def test_demo_interface(self):
        d = task.demo()
        self.assertIn("left", d)
        self.assertIn("right", d)
        self.assertGreater(len(d["left"]), 0)
        self.assertGreater(len(d["right"]), 0)

        p = task.plans()
        self.assertEqual(len(p["left"]), len(d["left"]))

    def test_objects_list(self):
        names = [o.name for o in task.OBJECTS]
        expected = [
            "rail_left",
            "rail_right",
            "stop_plate",
            "dock_mark",
            "red_post",
            "blue_block",
            "yellow_block",
            "cargo_block",
        ]
        for exp in expected:
            self.assertIn(exp, names)

    def test_run_sh_structure(self):
        run_sh = (HERE / "run.sh").read_text()
        self.assertIn("inspect-robots run", run_sh)
        for mode in MODES:
            self.assertIn(f"{mode})", run_sh)


if __name__ == "__main__":
    unittest.main()
