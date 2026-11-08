"""CPU checks for the held-tip guard; no grasp or knot feasibility is inferred."""
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch

import numpy as np

import table_policy as policy
import telemetry


class HeldTerminalGuardTests(unittest.TestCase):
    def setUp(self):
        saved_sample, saved_events = telemetry.latest_sample, telemetry.demo_events
        self.addCleanup(setattr, telemetry, 'latest_sample', saved_sample)
        self.addCleanup(setattr, telemetry, 'demo_events', saved_events)
        # Measured table-002 local_arch.working_tip_m; keep this small regression
        # fixture independent of the historical development evidence ledger.
        terminal = [0.166401751, 0.164726624, 0.752499981]
        # This measured tip is only a pickup-region fixture. The mocked motions
        # below do not model the rope, grasp, passage or endpoint orientation.
        self.point = np.asarray(terminal, float)
        telemetry.latest_sample = {'time': 0., 'cord': {'points': [terminal], 'end_tip': terminal},
            'hands': [{'tcp': [0., 0., .9], 'tcp_rotation': policy.DOWN.tolist()} for _ in range(2)],
            'metrics': {'invalid': None, 'score': 0., 'topology_status': 'unknot'}}
        telemetry.demo_events = []
        # The guard imports only this constant; no simulator adapter is needed.
        constants = ModuleType('yam_arena.constants')
        constants.FINGER_OPEN = -.04695
        self.enterContext(patch.dict(sys.modules, {'yam_arena.constants': constants}))
        self.left = Mock(spec=['grip', 'move_pose'])
        self.right = Mock(spec=['grip', 'move_pose'])
        self.left.grip.side_effect = lambda *args: iter([None])
        self.right.grip.side_effect = lambda *args: iter([None])
        self.right.move_pose.return_value = iter(())
        self.motion_labels = []

    def _run_thread(self, measured_tip, released_tip=None):
        def command(arm, side, point, rotation, seconds, label, **kwargs):
            self.motion_labels.append(label)
            return iter(())
        def measure(label):
            if released_tip is not None and label == 'table terminal after opening feeding hand':
                return released_tip
            return dict(measured_tip, axis_inside=measured_tip['inside'],
                        axis_boundary_distance_m=measured_tip['boundary_distance_m'])
        with patch.object(policy, '_move', side_effect=command), \
                patch.object(policy, '_point', return_value=self.point), \
                patch.object(policy, '_held'), \
                patch.object(policy, '_check_lowered_support'), \
                patch.object(policy, '_tip_location', side_effect=measure), \
                patch.object(policy, '_recover_dropped_terminal', return_value=iter(())), \
                patch('demo_policy.wait', return_value=iter(())):
            list(policy._thread_held_terminal((self.left, self.right)))

    def test_unclear_terminal_blocks_empty_hand_withdrawal(self):
        outcomes = [
            {'inside': False, 'boundary_distance_m': .20},
            {'inside': False, 'boundary_distance_m': 0.},
            {'inside': True, 'boundary_distance_m': 0.},
            {'inside': True, 'boundary_distance_m': .024999},
        ]
        for feed in outcomes:
            with self.subTest(feed=feed):
                self.motion_labels.clear()
                with self.assertRaisesRegex(policy._StopAttempt, 'clear loop interior'):
                    self._run_thread(feed)
                self.assertIn('carry actual terminal under arch', self.motion_labels)
                self.assertNotIn('withdraw empty feeding hand along the entry path',
                                 self.motion_labels)
                self.assertEqual(self.right.mock_calls, [])

    def test_clearance_boundary_allows_handoff_without_certifying_a_knot(self):
        self._run_thread({'inside': True, 'boundary_distance_m': .025},
                         {'axis_inside': True, 'axis_boundary_distance_m': .012})
        self.assertIn('withdraw empty feeding hand along the entry path', self.motion_labels)
        event = telemetry.demo_events[-1]
        self.assertEqual(event['label'], 'table held-terminal passage measured')
        self.assertIs(event['completed_knot_claimed'], False)
        self.assertEqual(event['metrics']['topology_status'], 'unknot')
        self.assertEqual(event['metrics']['score'], 0.)

    def test_tip_motion_after_release_prevents_empty_hand_withdrawal(self):
        for released in ({'axis_inside': False, 'axis_boundary_distance_m': .03},
                         {'axis_inside': True, 'axis_boundary_distance_m': .011999}):
            with self.subTest(released=released):
                self.motion_labels.clear()
                with self.assertRaisesRegex(policy._StopAttempt, 'lacks loop clearance'):
                    self._run_thread({'inside': True, 'boundary_distance_m': .025}, released)
                self.assertNotIn('withdraw empty feeding hand along the entry path', self.motion_labels)
                self.assertEqual(self.right.mock_calls, [])


class TerminalAxisClearanceTests(unittest.TestCase):
    def test_interior_axis_uses_nearest_boundary(self):
        boundary = np.array([[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1], [-.1, -.1]])
        inside, distance = policy._axis_loop_clearance(np.array([[-.04, 0.], [.04, 0.]]), boundary)
        self.assertTrue(inside)
        self.assertAlmostEqual(distance, .06)

    def test_endpoints_inside_do_not_allow_axis_to_cross_concave_gap(self):
        boundary = np.array([[0., 0.], [.4, 0.], [.4, .4], [.3, .4], [.3, .1],
                             [.1, .1], [.1, .4], [0., .4], [0., 0.]])
        inside, distance = policy._axis_loop_clearance(np.array([[.05, .25], [.35, .25]]), boundary)
        self.assertFalse(inside)
        self.assertAlmostEqual(distance, 0.)

    def test_vertical_axis_projection_is_finite(self):
        boundary = np.array([[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1], [-.1, -.1]])
        inside, distance = policy._axis_loop_clearance(np.zeros((2, 2)), boundary)
        self.assertTrue(inside)
        self.assertAlmostEqual(distance, .1)


class SupportLoweringTests(unittest.TestCase):
    def test_release_uses_measured_height_and_horizontal_position(self):
        goal = np.array([.3531498203, .0489738349, .7689901818])
        tcp = np.array([.3531808699, .0544705113, .7817408304])
        # Trial015: retained support surface17.35mm above the table, despite
        # a13.89mm TCP target error. No success or grasp physics is mocked in.
        def sample(low, high):
            base = policy.CONFIG['table_top']+policy.CONFIG['radius']
            return {'cord': {'endpoints': [[[0, 0, base+low], [0, 0, base+high]]]*24}}
        with patch.object(policy, '_held') as held, patch.object(policy, '_tcp', return_value=tcp), \
                patch.object(policy, '_sample', return_value=sample(.013643, .021058)), patch.object(policy, '_event'):
            policy._check_lowered_support(goal)
            held.assert_called_once_with(1, 23)
        for offset, height in ((.013, .02), (0., .025001), (0., -.000001)):
            with self.subTest(offset=offset, height=height), \
                    patch.object(policy, '_held'), \
                    patch.object(policy, '_tcp', return_value=goal+[offset, 0, 0]), \
                    patch.object(policy, '_sample', return_value=sample(height, height)), \
                    patch.object(policy, '_event'), patch('bimanual_policy._event'), \
                    self.assertRaises(policy._StopAttempt):
                policy._check_lowered_support(goal)


if __name__ == '__main__':
    unittest.main()
