"""Unit checks for loopholes; synthetic states are not native feasibility evidence."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

FOLDER = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('lp_task', FOLDER / 'task.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class ScoringTests(unittest.TestCase):
    def setUp(self):
        self.task = module.LPTask(FOLDER)
        self.task._start_radius = .142
        self.state = {
            'record': (np.array([.38, -.165, .8764]), np.eye(3), np.zeros(6)),
            'player/tonearm': (np.array([.38 + .2225, -.307, .9015]), np.eye(3), np.zeros(6)),
            'player/platter': (np.array([.38, -.165, .8645]), np.eye(3), np.zeros(6)),
            'player/button': (np.array([.530, -.043, .8645]), np.eye(3), np.zeros(6)),
            'sleeve': (np.array([.30, .30, .9397]), np.eye(3), np.zeros(6)),
        }
        self.hands = [[0, .5, 1.1], [0, -.5, 1.1]]

    def score(self):
        return self.task.score_states(self.state, self.hands)

    def test_seated_and_released(self):
        self.assertEqual(self.score()['score'], .75)

    def test_sleeve_location_is_not_the_loading_objective(self):
        self.state['sleeve'][0][0] += .08
        self.assertEqual(self.score()['score'], .75)
        self.assertFalse(self.score()['sleeve_home'])

    def test_spindle_tip_does_not_count_as_seated(self):
        self.state['record'][0][2] = .8884
        self.assertEqual(self.score()['score'], .25)
        self.assertFalse(self.score()['record_seated'])

    def test_off_center_does_not_count_as_seated(self):
        self.state['record'][0][0] += .002
        self.assertFalse(self.score()['record_seated'])

    def test_held_record_is_not_full_score(self):
        self.task._motor = self.task._qualified_start = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.assertEqual(self.score()['score'], 1)
        self.hands[0] = [.38, -.30, .88]
        self.assertLess(self.score()['score'], 1)

    def test_bench_contact_is_latched(self):
        self.state['record'][0][2] = .7484
        self.assertEqual(self.score()['score'], 0)
        self.state['record'][0][2] = .8764
        self.assertEqual(self.score()['score'], 0)

    def test_button_without_record_rotation_is_not_playback(self):
        self.task._motor = self.task._qualified_start = True
        self.assertFalse(self.score()['music_playing'])
        self.assertLess(self.score()['score'], 1)

    def test_rotation_without_stylus_contact_is_not_playback(self):
        self.task._motor = self.task._qualified_start = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.state['player/tonearm'][0][2] += .01
        self.assertFalse(self.score()['music_playing'])

    def test_playback_requires_motor_order_and_measured_rotation(self):
        self.task._motor = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.assertLess(self.score()['score'], 1)
        self.task._qualified_start = True
        self.assertEqual(self.score()['score'], 1)
        self.assertTrue(self.score()['music_playing'])

    def test_skating_cannot_count_as_stable_playback(self):
        self.task._motor = self.task._qualified_start = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.state['player/tonearm'][0][1] += .01
        self.assertLess(self.score()['score'], 1)
        self.assertFalse(self.score()['tracking_stable'])
        self.state['player/tonearm'][0][1] -= .01
        metrics = self.task.score_states(self.state, self.hands, (100/3, 100/3, .02))
        self.assertFalse(metrics['music_playing'])
        self.assertLess(metrics['score'], 1)

    def test_hovering_stylus_does_not_play(self):
        self.state['player/tonearm'][0][2] += .0005
        self.assertFalse(self.score()['stylus_on_record'])

    def test_holding_rear_arm_or_start_cap_prevents_full_score(self):
        self.task._motor = self.task._qualified_start = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.hands[0] = [.60, -.31, .95]
        self.assertLess(self.score()['score'], 1)
        self.hands[0] = [0, .5, 1.1]
        self.state['player/button'][0][2] -= .0031
        self.assertLess(self.score()['score'], 1)

    def test_stylus_drag_latches_failure(self):
        self.state['player/tonearm'][2][0] = .04
        self.assertEqual(self.score()['score'], 0)
        self.state['player/tonearm'][2][0] = 0
        self.assertEqual(self.score()['score'], 0)

    def test_drag_is_detected_before_record_is_seated(self):
        self.state['record'][0][0] += .002
        self.state['player/tonearm'][2][0] = .04
        self.assertEqual(self.score()['score'], 0)

    def test_moving_record_under_stationary_stylus_is_drag(self):
        self.state['record'][2][0] = .04
        self.assertEqual(self.score()['score'], 0)

    def test_abandoned_sleeve_latches_failure(self):
        self.state['sleeve'][0][2] = .70
        self.assertEqual(self.score()['score'], 0)
        self.state['sleeve'][0][2] = .9397
        self.assertEqual(self.score()['score'], 0)

    def test_airborne_or_overhanging_sleeve_cannot_finish(self):
        self.task._motor = self.task._qualified_start = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.state['sleeve'][2][2] = -.05
        self.assertLess(self.score()['score'], 1)
        self.state['sleeve'][2][2] = 0
        self.state['sleeve'][0][1] = .50
        self.assertLess(self.score()['score'], 1)

    def test_wide_open_finger_support_cannot_count_as_release(self):
        self.task._motor = self.task._qualified_start = True
        self.state['record'][2][5] = self.state['player/platter'][2][5] = -100 / 3 * 2 * np.pi / 60
        self.hands[0] = [.30, .30, .9897]
        self.assertLess(self.score()['score'], 1)

    def test_label_contact_latches_failure_even_with_motor_on(self):
        self.task._motor = True
        self.state['player/tonearm'][0][1] = -.165
        self.assertEqual(self.score()['score'], 0)
        self.state['player/tonearm'][0][1] = -.307
        self.assertEqual(self.score()['score'], 0)

    def test_stylus_penetration_latches_failure(self):
        self.state['player/tonearm'][0][2] -= .0015
        self.assertEqual(self.score()['score'], 0)
        self.state['player/tonearm'][0][2] += .0015
        self.assertEqual(self.score()['score'], 0)

    def test_nonfinite_native_state_aborts(self):
        self.state['record'][0][0] = np.nan
        with self.assertRaisesRegex(RuntimeError, 'Non-finite native state'):
            self.score()

    def test_encoder_handles_wrap_and_stationary_pose(self):
        def yaw(a):
            return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        self.assertAlmostEqual(module.rotation_speed(yaw(3.1), yaw(-3.1), .05), (2*np.pi-6.2)/.05)
        self.assertEqual(module.rotation_speed(np.eye(3), np.eye(3), .05), 0.)

    def test_continuous_collider_preserves_hole(self):
        from collections import Counter
        shapes = self.task.descriptions['record']['bodies'][0]['shapes']
        colliders = [s for s in shapes if s.get('collision')]
        self.assertEqual(len(colliders), 1)
        self.assertEqual(colliders[0]['collision_approximation'], 'sdf')
        lines = (self.task.folder / colliders[0]['mesh_file']).read_text().splitlines()
        points = np.array([[float(v) for v in line.split()[1:]] for line in lines if line.startswith('v ')])
        faces = [[int(v)-1 for v in line.split()[1:]] for line in lines if line.startswith('f ')]
        self.assertGreater(np.linalg.norm(points[:, :2], axis=1).min(), .00364)
        self.assertAlmostEqual(np.linalg.norm(points[:, :2], axis=1).max(), .15)
        self.assertAlmostEqual(np.ptp(points[:, 2]), .0018)
        self.assertTrue(all(np.linalg.norm(points[f, :2].mean(axis=0)) > .00364 for f in faces))
        edges = Counter(tuple(sorted((a,b))) for f in faces for a,b in zip(f, f[1:]+f[:1]))
        self.assertTrue(all(count == 2 for count in edges.values()))
        volume = sum(np.dot(points[f[0]], np.cross(points[f[i]], points[f[i+1]]))/6
                     for f in faces for i in range(1, len(f)-1))
        self.assertAlmostEqual(volume / (np.pi*(.15**2-.00365**2)*.0018), 1., places=3)

if __name__ == '__main__':
    unittest.main()
