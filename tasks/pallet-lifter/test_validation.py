"""CPU checks only; synthetic telemetry is not simulation evidence."""

from dataclasses import asdict
from types import SimpleNamespace
import unittest

import numpy as np
from yam_arena import station
from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import ArmPlan

from pallet_lifter.low_clearance import LowClearancePolicy, make_low_clearance_plan
from pallet_lifter.policy import make_plan
from pallet_lifter.scoring import History, evaluate
from pallet_lifter.telemetry import sample, summarize
from test_score import TOP, LIFTER, PALLET, objects, pallet


def row(o, h, step=0, contacts=(), **kwargs):
    for s in o.values():
        s.lin_vel = np.zeros(3)
        s.ang_vel = np.zeros(3)
    return sample(o, h, TOP, step=step, time_s=step / 20, contacts=contacts,
                  score=evaluate(o, h, TOP), **kwargs)


class ValidationChecks(unittest.TestCase):
    def test_plan_preserves_pickup_insertion_return_and_budget(self):
        baseline, low = make_plan(TOP), make_low_clearance_plan(TOP)
        self.assertEqual(sum(w.duration for w in low['left']), sum(w.duration for w in baseline['left']) + 2)
        self.assertLess((sum(w.duration for w in low['left']) + 3) * 20, 1800)
        self.assertEqual(low['right'][0].pos, baseline['right'][0].pos)
        self.assertEqual(low['right'][0].duration, sum(w.duration for w in low['left']))
        for label in ('approach', 'lower', 'grasp', 'lift tool', 'carry in front',
                      'lower outside pallet', 'insert tines', 'align under pallet',
                      'return', 'park', 'release', 'retreat'):
            old = next(w for w in baseline['left'] if w.label == label)
            new = next(w for w in low['left'] if w.label == label)
            self.assertEqual(asdict(old), asdict(new))

    def test_nominal_carry_clears_rails_and_withdrawal_clears_seated_feet(self):
        plan = make_low_clearance_plan(TOP)['left']
        for w in plan:
            if w.label in ('lift pallet', 'carry pallet'):
                # Level support geometry, not observed physics.
                nominal_bottom = w.pos[2] - TOP - .031
                self.assertGreater(nominal_bottom, .015)
                self.assertLess(nominal_bottom, .060)
            if w.label in ('carry pallet', 'lower pallet', 'settle pallet',
                           'lower fork clear', 'withdraw fork', 'lift clear'):
                self.assertAlmostEqual(w.pos[1], -.120)
            if w.label in ('lower fork clear', 'withdraw fork'):
                # Level tines top = TCP -11mm; seated feet bottom =15mm.
                tine_top = w.pos[2] - TOP - .011
                tine_bottom = tine_top - .005
                self.assertGreater(tine_bottom, 0.)
                self.assertGreaterEqual(.015 - tine_top, .0029)

    def test_stationary_low_hold_precedes_horizontal_withdrawal(self):
        waypoints = make_low_clearance_plan(TOP)['left']
        index = next(i for i, w in enumerate(waypoints) if w.label == 'withdraw fork')
        low, hold, withdraw = waypoints[index - 2:index + 1]
        self.assertEqual(low.label, 'lower fork clear')
        self.assertEqual(hold.label, 'lower fork clear')
        self.assertEqual(low.pos, hold.pos)
        self.assertGreaterEqual(hold.duration, 2.)
        self.assertEqual(hold.pos[1:], withdraw.pos[1:])
        self.assertLess(withdraw.pos[0], hold.pos[0])
        arm = ArmPlan(waypoints, station.BASES['left'])
        hold_start = sum(w.duration for w in waypoints[:index - 1])
        for t in np.linspace(hold_start, hold_start + hold.duration - .001, 8):
            pos, _, finger, _ = arm.target(t)
            np.testing.assert_allclose(pos, hold.pos, atol=1e-10)
            self.assertEqual(finger, FINGER_CLOSED)
        after, _, _, _ = arm.target(hold_start + hold.duration + .25)
        self.assertLess(after[0], hold.pos[0])
        np.testing.assert_allclose(after[1:], hold.pos[1:], atol=1e-10)

    def test_separate_policy_requires_expected_task(self):
        policy = LowClearancePolicy()
        with self.assertRaises(ValueError):
            policy.plan_for('')
        policy._task = SimpleNamespace(names=tuple(objects()) + ('rests', 'handle_support', 'tine_support'), table_top=TOP)
        self.assertEqual(policy.plan_for(''), make_low_clearance_plan(TOP))

    def test_synthetic_contact_transfer_and_release_telemetry(self):
        o, h = objects(), History()
        rows = [row(o, h)]
        h.contact(LIFTER, PALLET)
        o['pallet'] = pallet(bottom=.025, still=False)
        h.update(o, TOP)
        event = {'actors': [LIFTER, PALLET], 'contact_data_count': 2}
        rows.append(row(o, h, 1, [event]))
        self.assertEqual(rows[-1]['contacts_since_previous_sample'], [event])
        self.assertAlmostEqual(rows[-1]['rail_clearance_m'], .010)
        o['pallet'] = pallet()
        h.update(o, TOP)
        h.open_clear = True
        rows.append(row(o, h, 2, held_steps=20, terminated=True, termination_reason='success'))
        result = summarize(rows)
        self.assertTrue(result['contiguous_from_reset'])
        self.assertTrue(result['all_sampled_bottoms_below_60_mm'])
        self.assertTrue(result['scores_match_current_scorer'])
        self.assertTrue(result['runtime_success'])
        self.assertTrue(result['final_seated'])
        self.assertTrue(result['final_parked'])
        self.assertEqual(result['final_score'], 1.)
        # A sticky contact history must not invent new contact events.
        self.assertEqual(rows[-1]['contacts_since_previous_sample'], [])

    def test_peak_failure_and_incomplete_recording_cannot_be_hidden_by_seat(self):
        o, h = objects(), History(tool_contact=True)
        rows = [row(o, h)]
        o['pallet'] = pallet(bottom=.061, still=False)
        h.update(o, TOP)
        rows.append(row(o, h, 1))
        h.contact(PALLET, '/World/LeftArm/finger')
        o['pallet'] = pallet()
        h.update(o, TOP)
        h.open_clear = True
        rows.append(row(o, h, 3))
        result = summarize(rows)
        self.assertFalse(result['all_sampled_bottoms_below_60_mm'])
        self.assertTrue(result['invalid_ever'])
        self.assertEqual(result['final_score'], 0.)
        self.assertFalse(result['runtime_success'])
        self.assertFalse(result['contiguous_from_reset'])
        rows[-1]['objects']['pallet']['bottom_height_m'] = None
        self.assertFalse(summarize(rows)['all_sampled_bottoms_below_60_mm'])


if __name__ == '__main__':
    unittest.main()
