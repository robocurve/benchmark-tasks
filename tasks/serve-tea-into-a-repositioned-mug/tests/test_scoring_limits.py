import unittest
from unittest.mock import patch

import numpy as np

from tea.model import Kitchen, WET_LIMIT


class SceneBoundaryTests(unittest.TestCase):
    def test_registration_trip_persists_after_unloading_each_axis_and_direction(self):
        for idx in range(2):
            for direction in (-1, 1):
                with self.subTest(idx=idx, direction=direction):
                    env = Kitchen()
                    q = env.witness_q[idx]
                    env.d.qpos[q] = direction * .0029999
                    env.update_flag()
                    self.assertIsNone(env.flag_source)
                    env.d.qpos[q] = direction * .003
                    env.update_flag()
                    source = env.flag_source
                    self.assertEqual(source, "xy"[idx] + ("+" if direction > 0 else "-"))
                    env.d.qpos[q] = 0.0
                    env.update_flag()
                    self.assertEqual(env.report()["displacement_flag"], source)
                    self.assertEqual(env.first_fault[0], "receiver_displacement")
                    self.assertGreater(env.m.geom_rgba[env.flag_geom][0], 0.9)

    def test_deposit_threshold_is_strict_and_persists(self):
        env = Kitchen()
        hand = int(np.flatnonzero(env.groups == 3)[0])
        def hit(*args):
            args[-1][0] = hand
            return 0.00001
        with patch("tea.model.mj.mj_ray", side_effect=hit):
            for dose in (1, 1, .001):
                env.remaining -= dose
                env.packets.append((np.array([.5, .1, .98]), np.array([0., 0., -.1]), dose))
                env.flow()
                self.assertEqual(env.first_fault is not None, env.skin > WET_LIMIT)
        env.cup = 120.0   # Later completion cannot clear the wet-pad event.
        env.remaining -= 120.0
        self.assertEqual((env.report()["score"], env.first_fault[0]), (0.0, "liquid_on_resident"))

    def test_pose_span_checks_inside_the_window_and_quaternion_sign(self):
        env = Kitchen()
        pose, q, _ = env.frames[0]
        for i in range(40):
            xyz = pose + [0.0035 if i == 19 else 0.0, 0, 0]
            env.frames.append((xyz, -q if i % 2 else q, (False, True, True, True, True, 0.0)))
        r = env.observations()
        self.assertEqual(r["window_s"], 2.0)
        self.assertAlmostEqual(r["position_span_mm"], 3.5)
        self.assertLess(r["orientation_span_deg"], 0.00001)

    def test_measure_carriage_when_its_driver_is_stuck_despite_trigger(self):
        env = Kitchen()
        env.cup = 60.0
        env.remaining -= env.cup
        # Hold the physical carriage marker at A; the ordinary trigger state alone
        # would claim it moved. Inspect the actual geometry through 1.3 seconds.
        with patch.object(env, "move_person", return_value=None):
            for _ in range(26):
                env.action(env.state())
        r = env.report()
        self.assertEqual((r["status"], r["score"]), ("invalid", None))
        self.assertEqual(r["stage_error"], "triggered carriage did not move")
        self.assertFalse(r["robot_interference_observed"])

    def test_registration_uses_signed_stroke_duration_and_observed_trigger_mass(self):
        from tea.scene import MUG
        env = Kitchen()
        env.cup = 48
        env.observe_stage(0, MUG)
        env.observe_stage(.1, MUG + [-.002, 0, 0])
        env.observe_stage(.4, MUG + [-.083, 0, 0])
        self.assertEqual(env.stage_error, "stage movement duration outside tolerance")
        env = Kitchen()
        env.cup = 40
        env.observe_stage(.0, MUG + [-.002, 0, 0])
        self.assertEqual(env.stage_error, "onset outside calibrated fill band")
        env = Kitchen()
        env.observe_stage(.0, MUG + [.004, 0, 0])
        self.assertEqual(env.stage_error, "carriage marker outside its calibrated path")

    def test_brief_unsupported_and_new_visible_discharge_interrupt_window(self):
        env = Kitchen()
        pose, q, _ = env.frames[0]
        env.cup = 120
        env.remaining -= env.cup
        env.motion_completed = True
        for i in range(40):
            env.frames.append((pose, q, (i == 10, i != 6, i != 6, True, True, 0.0)))
        r = env.report()
        self.assertTrue(r["primary_observations"]["visible_discharge"])
        self.assertFalse(r["primary_observations"]["supported_throughout"])
        self.assertEqual(r["score"], 0.0)

    def test_hidden_flow_rate_and_inflight_mass_are_diagnostic_only(self):
        env = Kitchen()
        pose, q, _ = env.frames[0]
        env.cup = 120
        env.remaining -= env.cup
        env.motion_completed = True
        for _ in range(40):
            env.frames.append((pose, q, (False, True, True, True, True, 0.0)))
        self.assertEqual(env.report()["score"], 1.0)
        # A diagnostic perturbation alone cannot change the common observed outcome.
        env.rate = 42
        env.packets.append((np.array([0., 0., .9]), np.zeros(3), .06))
        env.remaining -= .06
        self.assertEqual(env.report()["score"], 1.0)
