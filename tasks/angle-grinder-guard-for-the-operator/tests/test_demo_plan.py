"""The scripted demo's plan, checked offline with the real IK (no simulator): every mode in both layouts is reachable
within 3 mm and 3 degrees with every joint at least 0.02 rad from its limit, finishes well inside the run limit, never
reads the scene (open loop), and the modes match run.sh and the README."""

import re
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import check_plan  # noqa: E402
import demo  # noqa: E402


class TestDemoPlan(unittest.TestCase):
    def test_every_mode_is_reachable_in_both_layouts(self):
        for v in "AB":
            for mode in demo.MODES:
                r = check_plan.drain(mode, v)
                self.assertLess(r["steps"], 2400 - 100, (v, mode))
                for side, w in r["worst"].items():
                    self.assertLessEqual(w["pe"], check_plan.POS_TOL_MM, (v, mode, side))
                    self.assertLessEqual(w["re"], check_plan.ROT_TOL_DEG, (v, mode, side))
                    self.assertGreaterEqual(w["margin"], check_plan.MARGIN_RAD, (v, mode, side))

    def test_the_demo_is_open_loop(self):
        text = (HERE / "demo.py").read_text()
        for needle in ("ctx.states", "object_states", "joints[", "_STATE["):
            code = [ln for ln in text.splitlines() if needle in ln and not ln.strip().startswith(("#", '"'))]
            code = [ln for ln in code if "def log" not in ln and "S = ctx.task" not in ln and "S['" not in ln]
            self.assertEqual(code, [], f"the demo reads the scene: {needle}")

    def test_modes_are_documented(self):
        run_sh = (HERE / "run.sh").read_text()
        readme = (HERE / "README.md").read_text()
        for mode in demo.MODES:
            if mode == "demo":
                continue
            self.assertIn(mode, run_sh, mode)
            self.assertIn(f"`{mode}`", readme, mode)

    def test_full_demo_fits_the_run_limit_with_room(self):
        for v in "AB":
            self.assertLess(check_plan.drain("demo", v)["steps"] * demo.DEMO_TIME_SCALE, 2400 - 150)

    def test_plans_exist_for_every_mode_and_layout(self):
        from yam_arena.demo import Waypoint

        for v in "AB":
            for mode in demo.MODES:
                plan = demo.plans(mode, v)
                self.assertEqual(sorted(plan), ["left", "right"], (v, mode))
                for side, wps in plan.items():
                    self.assertTrue(wps, (v, mode, side))
                    self.assertTrue(all(isinstance(w, Waypoint) and w.duration > 0 for w in wps), (v, mode, side))
                steps = max(sum(w.duration for w in wps) for wps in plan.values()) * 20
                self.assertLess(steps, 2400 - 100, (v, mode))

    def test_the_plan_is_the_script_stretched(self):
        a, b = demo.plans("demo", "A", 1.0), demo.plans("demo", "A", 1.8)
        for side in a:
            self.assertAlmostEqual(sum(w.duration for w in b[side]), 1.8 * sum(w.duration for w in a[side]), places=6)

    def test_run_sh_plays_every_mode_with_inspect_robots(self):
        run_sh = (HERE / "run.sh").read_text()
        self.assertIn("inspect-robots run", run_sh)
        self.assertIn("yam_scripted", run_sh)
        self.assertNotIn("python demo.py", run_sh.replace("# ", ""))
        case = re.search(r"demo\|[a-z_|]+\)", run_sh)
        self.assertIsNotNone(case)
        self.assertEqual(sorted(case.group(0)[:-1].split("|")), sorted(m for m in demo.MODES))

    def test_the_task_demo_returns_the_plans_dict(self):
        import os

        import task

        os.environ["DEMO_MODE"] = "forget_lock"
        try:
            self.assertEqual(sorted(task.demo()), ["left", "right"])
            os.environ["DEMO_MODE"] = "nonsense"
            with self.assertRaises(ValueError):
                demo.mode_from_env()
        finally:
            os.environ.pop("DEMO_MODE", None)


if __name__ == "__main__":
    unittest.main()
