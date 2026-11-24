"""A policy can open the gripper on the actual 1200th action."""

import unittest

from tea.model import Kitchen
from tea.reference import Server


class TerminalSupportTests(unittest.TestCase):
    def test_opening_on_final_action_removes_partial_credit(self):
        env = Kitchen("reposition")
        controller = Server(env, "hold")
        for tick in range(1199):
            if tick < 500:
                command = controller.command()
            env.action(command)
        before = env.report()
        self.assertEqual((before["score"], before["first_fault"]), (0.5, None))
        self.assertEqual(before["primary_observations"]["window_s"], 2.0)
        # Drop opportunity at the deadline: even before impact the pitcher lacks support.
        release = command.copy()
        release[6] = 1.0
        env.action(release)
        after = env.report()
        self.assertFalse(env.robot_touches or env.supported)
        self.assertEqual(after["t_s"], 60)
        self.assertEqual(after["source_ml"], before["source_ml"])
        self.assertEqual(after["received_ml"], before["received_ml"])
        self.assertIsNone(after["first_fault"])
        self.assertFalse(after["primary_observations"]["supported_throughout"])
        self.assertEqual(after["score"], 0)
