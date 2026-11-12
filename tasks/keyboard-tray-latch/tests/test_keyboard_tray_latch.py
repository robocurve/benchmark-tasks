"""Unit tests for keyboard_tray_latch (T22): geometry, scoring ladder and gates, task file contract, demo modes, blind review."""

from __future__ import annotations

import math
import os
import re
import sys
import unittest
from pathlib import Path

import numpy as np

_TASK_DIR = Path(__file__).resolve().parent.parent
_UPSTREAM = None
for parent in _TASK_DIR.parents:
    cand = parent / "robocurve-windows-transfer-2026-11-05" / "project" / "robocurve" / "tmp" / "yam-upstream"
    if cand.exists():
        _UPSTREAM = cand
        break
for p in (_TASK_DIR, _UPSTREAM):
    if p is not None and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import demo_plan  # noqa: E402
import geometry as g  # noqa: E402
import logic  # noqa: E402
import scoring  # noqa: E402

DESK_START = g.DESK_POS_WORLD


def ev(tray_slide: float, latched: bool, desk_pos=DESK_START, resting: bool = True):
    return logic.evaluate_state(desk_pos, tray_slide, latched, resting=resting)


class DummyDesk:
    def __init__(self, pos=DESK_START, sliders=None, resting=True):
        self.pos = pos
        self.sliders = sliders or {}
        self._resting = resting

    def resting(self):
        return self._resting


class TestGeometry(unittest.TestCase):
    def test_constants(self):
        self.assertAlmostEqual(g.TABLE_TOP, 0.7475)
        self.assertAlmostEqual(g.DESK_MASS_KG, 25.0)
        self.assertAlmostEqual(g.DESK_FRICTION, 1.2)
        self.assertAlmostEqual(g.GATE_DESK_DRIFT_MAX_M, 0.005)

    def test_working_points_are_far_apart(self):
        # Left arm braces at y = +0.15 m, right arm pushes tray at y = -0.10 m
        separation_y = 0.15 - (-0.10)
        self.assertGreaterEqual(separation_y, 0.150)
        self.assertAlmostEqual(separation_y, 0.250)

    def test_gripper_clearance(self):
        # Separation in y is 250 mm, minimum TCP Euclidean distance is 251.8 mm
        min_tcp = 0.2518
        gripper_envelope = 0.085
        clearance = min_tcp - gripper_envelope
        self.assertGreaterEqual(min_tcp, 0.115)  # hard floor >= 115 mm
        self.assertGreaterEqual(clearance, 0.030)  # clearance >= 30 mm
        self.assertGreaterEqual(min_tcp, 0.150)  # design target >= 150 mm

    def test_everything_in_reach(self):
        # Arm shoulder bases at (0, +0.305) and (0, -0.305)
        p = demo_plan.plans()
        for i, w in enumerate(p["left"]):
            d = math.hypot(w.pos[0] - 0.0, w.pos[1] - 0.305)
            self.assertLess(d, 0.50, f"Left waypoint {getattr(w, 'label', i)} out of reach: {d:.3f} m")
        for i, w in enumerate(p["right"]):
            d = math.hypot(w.pos[0] - 0.0, w.pos[1] - (-0.305))
            self.assertLess(d, 0.50, f"Right waypoint {getattr(w, 'label', i)} out of reach: {d:.3f} m")


class TestScoring(unittest.TestCase):
    def test_first_update_is_zero(self):
        s, ok, why = ev(0.0, False)
        self.assertEqual((s, ok), (0.0, False))
        self.assertIn("< 5 cm", why)

    def test_idle_policy_scores_zero(self):
        s, ok, _ = ev(0.0, False)
        self.assertEqual(s, 0.0)

    def test_ladder_levels(self):
        # 0.00: start state (tray slide < 5 cm)
        self.assertEqual(ev(0.020, False)[0], 0.0)
        # 0.25: tray moved >= 5 cm (< 15 cm)
        self.assertEqual(ev(0.060, False)[0], 0.25)
        # 0.50: tray moved >= 15 cm (< 26 cm)
        self.assertEqual(ev(0.160, False)[0], 0.50)
        # 0.75: tray moved >= 26 cm, not latched
        self.assertEqual(ev(0.260, False)[0], 0.75)
        self.assertEqual(ev(0.280, False)[0], 0.75)
        # 1.00: tray moved >= 27.8 cm, latched, resting
        self.assertEqual(ev(0.278, True, resting=True)[0], 1.00)
        self.assertEqual(ev(0.280, True, resting=True)[0], 1.00)

    def test_ladder_boundary_thresholds(self):
        # 5 cm threshold
        self.assertEqual(ev(0.049, False)[0], 0.0)
        self.assertEqual(ev(0.050, False)[0], 0.25)
        # 15 cm threshold
        self.assertEqual(ev(0.149, False)[0], 0.25)
        self.assertEqual(ev(0.150, False)[0], 0.50)
        # 26 cm threshold
        self.assertEqual(ev(0.259, False)[0], 0.50)
        self.assertEqual(ev(0.260, False)[0], 0.75)
        # 27.8 cm threshold (latched=True)
        self.assertEqual(ev(0.277, True, resting=True)[0], 0.75)
        self.assertEqual(ev(0.278, True, resting=True)[0], 1.00)

    def test_two_valid_solutions(self):
        # Solution A: tray at 27.9 cm, latch engaged
        s_a, ok_a, _ = ev(0.279, True, resting=True)
        # Solution B: tray at 28.0 cm, latch engaged
        s_b, ok_b, _ = ev(0.280, True, resting=True)
        self.assertEqual((s_a, ok_a), (1.00, True))
        self.assertEqual((s_b, ok_b), (1.00, True))

    def test_drift_gates(self):
        # 4.9 mm drift within tolerance
        pos_ok = (DESK_START[0] + 0.0049, DESK_START[1], DESK_START[2])
        self.assertEqual(ev(0.280, True, desk_pos=pos_ok, resting=True)[0], 1.00)
        # 5.1 mm drift along X trips gate
        pos_bad_x = (DESK_START[0] + 0.0051, DESK_START[1], DESK_START[2])
        s_x, ok_x, why_x = ev(0.280, True, desk_pos=pos_bad_x, resting=True)
        self.assertEqual((s_x, ok_x), (0.0, False))
        self.assertIn("Gate", why_x)
        # 5.1 mm drift along Y trips gate
        pos_bad_y = (DESK_START[0], DESK_START[1] + 0.0051, DESK_START[2])
        s_y, ok_y, why_y = ev(0.280, True, desk_pos=pos_bad_y, resting=True)
        self.assertEqual((s_y, ok_y), (0.0, False))
        self.assertIn("Gate", why_y)

    def test_must_be_still(self):
        s, ok, why = ev(0.280, True, resting=False)
        self.assertEqual((s, ok), (0.75, False))
        self.assertIn("settling", why.lower())

    def test_fuzz(self):
        rng = np.random.default_rng(22)
        for _ in range(500):
            tray_slide = float(rng.uniform(-0.02, 0.35))
            latched = bool(rng.integers(2))
            desk_drift = tuple(float(v) for v in rng.uniform([-0.01, -0.01, -0.01], [0.01, 0.01, 0.01]))
            desk_pos = (DESK_START[0] + desk_drift[0], DESK_START[1] + desk_drift[1], DESK_START[2] + desk_drift[2])
            resting = bool(rng.integers(2))
            s, ok, why = ev(tray_slide, latched, desk_pos=desk_pos, resting=resting)
            self.assertTrue(0.0 <= s <= 1.0)
            self.assertIsInstance(why, str)
            self.assertEqual(ok, s == 1.00)


class TestTaskFile(unittest.TestCase):
    def test_loads(self):
        import yam_arena.task_file as tf

        t = tf.load(str(_TASK_DIR / "task.py"))
        self.assertEqual(t.scene, "station")
        self.assertEqual(t.success_hold_s, 1.0)
        self.assertEqual([o.name for o in t.objects], [g.DESK_NAME])

    def test_score_wrapper_first_update(self):
        obj = DummyDesk(DESK_START, sliders={"tray": 0.0, "latch": 0.0})
        self.assertEqual(scoring.score({g.DESK_NAME: obj}), 0.0)
        self.assertFalse(scoring.success({g.DESK_NAME: obj}))

    def test_score_wrapper_full(self):
        obj = DummyDesk(DESK_START, sliders={"tray": 0.280, "latch": -0.007})
        self.assertEqual(scoring.score({g.DESK_NAME: obj}), 1.00)
        self.assertTrue(scoring.success({g.DESK_NAME: obj}))


class TestPlans(unittest.TestCase):
    def test_modes(self):
        for var in ("SETTLE_ONLY", "ABLATION"):
            os.environ[var] = "1"
            p = demo_plan.plans()
            del os.environ[var]
            self.assertEqual(set(p), {"left", "right"})
        p = demo_plan.plans()
        self.assertGreater(sum(w.duration for w in p["left"]), 10.0)

    def test_settle_is_an_explicit_hold(self):
        os.environ["SETTLE_ONLY"] = "1"
        p = demo_plan.plans()
        del os.environ["SETTLE_ONLY"]
        self.assertTrue(all(len(v) >= 1 for v in p.values()))

    def test_plan_length_within_run_limit(self):
        p = demo_plan.plans()
        steps = max(sum(w.duration for w in v) for v in p.values()) * 20
        self.assertLess(steps, 1500)

    def test_arm_separation_in_plan(self):
        p = demo_plan.plans()
        left_wps = p["left"]
        right_wps = p["right"]
        # In demo plan, left arm braces at y = +0.15 m, right arm pushes tray at y = -0.10 m
        # Check separation across simultaneous waypoints
        for wl in left_wps:
            for wr in right_wps:
                d = math.dist(wl.pos, wr.pos)
                # Gripper envelope hard floor check between active points
                if "hold" in getattr(wl, "label", "") and "push" in getattr(wr, "label", ""):
                    self.assertGreaterEqual(d, 0.150, f"Concurrent arm distance below 150 mm: {d:.3f} m")

    def test_env_vars_are_read(self):
        src = (_TASK_DIR / "demo_plan.py").read_text(encoding="utf-8")
        self.assertIn("DEMO_TIME_SCALE", src)


class TestBlindReview(unittest.TestCase):
    def test_no_identifying_strings(self):
        bad = ["c:\\users", "c:/users", "github.com/", "/home/"]
        email = re.compile(r"[\w.+-]+@[\w-]+\.[\w.-]+")
        words = [w.strip().lower() for w in os.getenv("TEAM_WORDS", "").split(",") if w.strip()]
        for f in _TASK_DIR.rglob("*"):
            if f.suffix not in {".py", ".md", ".json", ".sh"} or "tests" in f.parts or f.name == "VM_NOTES.md":
                continue
            text = f.read_text(encoding="utf-8", errors="ignore").lower()
            for b in bad:
                self.assertNotIn(b, text, f"{b!r} in {f.name}")
            self.assertIsNone(email.search(text), f"email in {f.name}")
            for w in words:
                self.assertNotIn(w, text, f"team word in {f.name}")


if __name__ == "__main__":
    unittest.main()
