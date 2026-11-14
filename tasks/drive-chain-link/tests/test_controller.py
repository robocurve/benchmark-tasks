"""The reference must run with no scene state, images, contacts or score input."""

import unittest
import numpy as np
from inspect_robots.guard import ClampGuard, DeltaLimitGuard
from inspect_robots.types import Action

from chain_link.controller import ScriptedPolicy


class ClockOnlyObservation:
    def __init__(self, time):
        self.state_time = time

    def __getattr__(self, name):
        raise AssertionError(f"Reference requested unavailable observation: {name}")


class ReferenceInputTests(unittest.TestCase):
    def test_standard_inspect_guards_preserve_every_taught_robot_command(self):
        policy = ScriptedPolicy()
        guards = [
            ClampGuard(policy.info.action_space),
            DeltaLimitGuard(policy.info.action_space),
        ]
        store = {}
        for command in policy.actions:
            action = Action(data=command)
            for guard in guards:
                action = guard.review(action, store)
            np.testing.assert_array_equal(action.data, command)

    def test_registered_policy_can_execute_the_whole_program_with_clock_only(self):
        policy = ScriptedPolicy()
        policy.reset(None)
        for step, expected in enumerate(policy.actions):
            chunk = policy.act(ClockOnlyObservation(step / 20))
            self.assertEqual(len(chunk.actions), 1)
            np.testing.assert_array_equal(chunk.actions[0].data, expected)

    def test_reset_repeats_robot_commands_without_scene_state(self):
        policy = ScriptedPolicy()
        first = policy.act(ClockOnlyObservation(0)).actions[0].data.copy()
        policy.act(ClockOnlyObservation(40))
        policy.reset(None)
        np.testing.assert_array_equal(
            first, policy.act(ClockOnlyObservation(0)).actions[0].data
        )

    def test_after_program_only_the_final_robot_command_is_held(self):
        policy = ScriptedPolicy()
        np.testing.assert_array_equal(
            policy.action_at(policy.duration), policy.actions[-1]
        )

    def test_unsupported_control_rate_fails_before_execution(self):
        class WrongRate:
            control_hz = 10

        with self.assertRaises(ValueError):
            ScriptedPolicy().bind(WrongRate())


if __name__ == "__main__":
    unittest.main()
