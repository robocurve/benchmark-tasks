"""v4 timing and measurement tests; injected knots are not feasibility evidence."""
from copy import deepcopy
import unittest
from unittest.mock import patch

import numpy as np

from bimanual_scoring import BimanualKnotScorer, _span_metrics, _uncrossed_top_view, certify_result
from test_scoring import straight_sample


CONFIG = dict(radius=.004, segment=.01, length=.64, table_top=0.)
UNKNOT = dict(status='unknot', is_overhand=False, reason='test-only unknot')
OVERHAND = dict(status='overhand_candidate', is_overhand=True, reason='test-only knot certificate')


def sample(time, *, elevated=False, touching=False):
    value = straight_sample(time)
    shift = np.array([0., 0., .104 if elevated else .004])
    for field in ('points', 'endpoints', 'polyline'):
        value['cord'][field] = (np.asarray(value['cord'][field])+shift).tolist()
    if touching:
        for box in value['hands'][0]['finger_boxes']:
            if box['link'] == 'tip_left':
                box['center'] = [0., 0., shift[2]]
    return value


def arc_sample(time, outer_span):
    """An unknotted circular arc with exact link lengths and selected tip span."""
    n, seg, radius = 64, .01, .004
    def geometry(angle):
        theta = (np.arange(n)+.5)*angle/n-angle/2
        axes = np.column_stack((np.cos(theta), np.sin(theta), np.zeros(n)))
        nodes = np.vstack((np.zeros(3), np.cumsum(axes*seg, axis=0))) + [0, 0, .104]
        distance = np.linalg.norm(nodes[-1]+radius*axes[-1]-(nodes[0]-radius*axes[0]))
        return nodes, theta, distance
    low, high = 0., np.pi
    for _ in range(55):
        middle = (low+high)/2
        if geometry(middle)[2] > outer_span:
            low = middle
        else:
            high = middle
    nodes, angles, _ = geometry((low+high)/2)
    result = sample(time, elevated=True, touching=True)
    result['cord'].update(points=((nodes[:-1]+nodes[1:])/2).tolist(),
        endpoints=np.stack((nodes[:-1], nodes[1:]), axis=1).tolist(), polyline=nodes.tolist(),
        quats_xyzw=np.column_stack((np.zeros(n), np.zeros(n), np.sin(angles/2), np.cos(angles/2))).tolist())
    return result


class BimanualScorerTests(unittest.TestCase):
    def setUp(self):
        self.topology = UNKNOT
        context = patch('bimanual_scoring.analyze_open_rope', side_effect=lambda *a: dict(self.topology))
        context.start()
        self.addCleanup(context.stop)
        self.scorer = BimanualKnotScorer(CONFIG)
        self.tick = 0
        self.scorer.update(sample(0.))
        self.assertTrue(self.scorer.initial_validated)

    def advance(self, count, *, elevated=False, touching=False):
        for _ in range(count):
            self.tick += 1
            self.scorer.update(sample(self.tick/20, elevated=elevated, touching=touching))

    def stretch(self):
        self.topology = OVERHAND
        self.advance(21, elevated=True, touching=True)

    def complete(self):
        self.stretch()
        self.advance(101)

    def test_full_robot_span_then_five_seconds_of_release(self):
        self.stretch()
        self.assertIsNotNone(self.scorer.span_certificate)
        self.assertAlmostEqual(self.scorer.span_certificate['seconds'], 1.)
        self.advance(100)
        self.assertEqual(self.scorer.value, 0.)  # First released sample starts the timer.
        self.advance(1)
        result = self.scorer.summary(final=True)
        self.assertEqual(result['score'], 1.)
        self.assertAlmostEqual(result['released_after_span_s'], 5.)
        self.assertEqual(result['score_version'], 'overhand-bimanual-v4')

    def test_retained_loose_knot_gets_half_without_span_proof(self):
        self.topology = OVERHAND
        self.advance(101)
        self.assertEqual(self.scorer.summary(final=True)['score'], .5)
        self.assertIsNone(self.scorer.span_certificate)

    def test_short_span_window_does_not_accumulate_across_interruptions(self):
        self.topology = OVERHAND
        self.advance(20, elevated=True, touching=True)  # 0.95 seconds.
        self.advance(1, touching=True)  # Cord on table interrupts the span window.
        self.advance(20, elevated=True, touching=True)
        self.advance(101)
        self.assertEqual(self.scorer.value, .5)
        self.assertIsNone(self.scorer.span_certificate)

    def test_release_and_span_windows_do_not_overlap_for_full_credit(self):
        self.topology = OVERHAND
        self.advance(101, elevated=True)  # Five released seconds, only four after span1s.
        self.assertEqual(self.scorer.value, .5)
        self.advance(20, elevated=True)
        self.assertEqual(self.scorer.value, 1.)

    def test_regrasp_restarts_retention_but_does_not_ban_a_valid_strategy(self):
        self.complete()
        self.advance(1, touching=True)
        self.assertEqual(self.scorer.value, 0.)
        self.assertIsNotNone(self.scorer.span_certificate)
        self.advance(101)
        self.assertEqual(self.scorer.value, 1.)

    def test_untie_then_retie_cannot_reuse_an_old_span_certificate(self):
        self.complete()
        self.topology = UNKNOT
        self.advance(1)
        self.assertEqual(self.scorer.value, 0.)
        self.assertIsNone(self.scorer.span_certificate)
        self.topology = OVERHAND
        self.advance(101)
        self.assertEqual(self.scorer.value, .5)

    def test_ambiguity_is_not_invalid_physics_but_breaks_continuous_evidence(self):
        self.complete()
        self.topology = dict(status='ambiguous', is_overhand=None, reason='test-only uncertainty')
        self.advance(1)
        self.assertIsNone(self.scorer.invalid)
        self.assertEqual(self.scorer.value, 0.)
        self.assertIn('uncertain', self.scorer.status)
        self.assertIsNone(self.scorer.span_certificate)

    def test_final_topology_is_rechecked_and_can_erase_earlier_credit(self):
        self.complete()
        self.topology = UNKNOT
        self.assertEqual(self.scorer.summary(final=True)['score'], 0.)

    def test_preformed_or_crossed_initial_layout_is_rejected(self):
        self.topology = OVERHAND
        scorer = BimanualKnotScorer(CONFIG)
        scorer.update(sample(0.))
        self.assertIn('initial cord', scorer.invalid)
        self.assertEqual(scorer.value, 0.)
        self.assertFalse(_uncrossed_top_view(np.array([[-1,-1,0], [1,1,0], [-1,1,.1], [1,-1,.1]])))
        self.assertTrue(_uncrossed_top_view(np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0]])))
        self.assertFalse(_uncrossed_top_view(np.array([[0,0,0], [1,0,0], [2,0,0], [.5,0,0]])))

    def test_fixture_or_external_assessor_cannot_create_credit(self):
        for extra in ({'anchor_start': True}, {'development_jig': 'three-peg'}):
            scorer = BimanualKnotScorer({**CONFIG, **extra})
            self.assertEqual(scorer.value, 0.)
            self.assertIn('unclamped', scorer.invalid)
        self.stretch()
        value = sample((self.tick+1)/20)
        value.update(phase='proof', proof={'active': True, 'applied_force_n': .981})
        self.scorer.update(value)
        self.assertEqual(self.scorer.value, 0.)
        self.assertIn('external-force', self.scorer.invalid)

    def test_missing_frames_and_later_broken_geometry_erase_credit(self):
        self.complete()
        self.scorer.update(sample((self.tick+2)/20))
        self.assertEqual(self.scorer.value, 0.)
        self.assertIn('capture interval', self.scorer.invalid)
        scorer = BimanualKnotScorer(CONFIG)
        self.topology = UNKNOT
        scorer.update(sample(0.))
        value = sample(.05)
        value['cord']['joint_gaps'][0] = .003
        scorer.update(value)
        self.assertEqual(scorer.value, 0.)
        self.assertIn('joint gaps', scorer.invalid)

    def test_diagnostic_flag_is_sticky_and_never_earns_task_credit(self):
        self.stretch()
        value = sample((self.tick+1)/20)
        value['diagnostic_only'] = True
        self.tick += 1
        self.scorer.update(value)
        self.advance(100)
        result = self.scorer.summary(final=True)
        self.assertEqual(result['candidate_score'], 1.)
        self.assertEqual(result['score'], 0.)
        self.assertEqual(result['status'], 'diagnostic_only')

    def test_span_uses_physical_outer_tips_and_excludes_gap_extension(self):
        self.topology = OVERHAND
        self.advance(1, elevated=True, touching=True)
        metrics = self.scorer.span_metrics
        self.assertAlmostEqual(metrics['physical_free_length_m'], .648)
        self.assertAlmostEqual(metrics['required_span_m'], .536)
        self.assertAlmostEqual(metrics['outer_tip_span_m'], .648)
        cord = deepcopy(self.scorer._last_cord)
        cord['endpoints'][1:] += [.001, 0., 0.]
        shifted = _span_metrics(cord, self.scorer.config)
        self.assertAlmostEqual(shifted['outer_tip_span_m'], .649)
        self.assertAlmostEqual(shifted['span_lower_bound_m'], .648)
        self.assertAlmostEqual(shifted['total_joint_gap_m'], .001)

    def test_span_fails_below_table_clearance_boundary(self):
        self.topology = OVERHAND
        value = sample(.05)
        for field in ('points', 'endpoints', 'polyline'):
            value['cord'][field] = (np.asarray(value['cord'][field])+[0, 0, .0099]).tolist()
        self.scorer.update(value)
        self.assertFalse(self.scorer.span_condition)
        value['time'] = .1
        for field in ('points', 'endpoints', 'polyline'):
            value['cord'][field] = (np.asarray(value['cord'][field])+[0, 0, .0001]).tolist()
        self.scorer.update(value)
        self.assertTrue(self.scorer.span_condition)

    def test_physical_tip_span_boundary_decides_full_credit(self):
        for distance, expected in ((.536-.00001, .5), (.536+.00001, 1.)):
            self.topology = UNKNOT
            scorer = BimanualKnotScorer(CONFIG)
            scorer.update(sample(0.))
            self.topology = OVERHAND
            for tick in range(1, 22):
                scorer.update(arc_sample(tick/20, distance))
            for tick in range(22, 123):
                scorer.update(sample(tick/20))
            self.assertIsNone(scorer.invalid)
            self.assertEqual(scorer.summary(final=True)['score'], expected)

    def test_incomplete_finalization_preserves_geometry_but_not_full_credit(self):
        self.complete()
        result = self.scorer.summary(final=True)
        good = certify_result(result, finalization_complete=True)
        bad = certify_result(result, finalization_complete=False)
        self.assertEqual(good['score'], 1.)
        self.assertEqual(bad['score'], .5)
        self.assertEqual(bad['geometric_score'], 1.)
        self.assertFalse(bad['assay_certification']['required'])
        self.assertFalse(bad['finalization_certification']['complete'])


if __name__ == '__main__':
    unittest.main()
