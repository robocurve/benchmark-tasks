"""Focused CPU integration-contract tests; no simulator compatibility claim."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from fixed_policy_physics import FixedPolicyPhysics, PolicyPhysicsError, SCHEDULE, validate_schedule


class FakeTransition:
    def __init__(self, base, hz, capture_time_s, *, diagnostic_only):
        self.base, self.hz = base, hz
        self.report = dict(status='not_started', error=None, intervals=[],
                           switch_capture_time_s=capture_time_s,
                           validated_control_intervals=0, callback_removed=False,
                           diagnostic_only=diagnostic_only)
        self.pending = None

    def start(self):
        self.base.constructed += 1
        if self.base.fail_start:
            raise ValueError('native start failure')
        self.base.dt, self.base.decimation = 1 / self.hz, self.hz // 20
        self.report['status'] = 'awaiting_native_validation'

    def before_step(self, t, frames):
        self.pending = (t, frames, self.base.native_steps)

    def after_step(self, t, frames):
        if self.base.fail_after:
            raise ValueError('native callback mismatch')
        previous, previous_frames, native = self.pending
        assert abs(t - previous - .05) < 1e-9 and frames == previous_frames + 1
        assert self.base.native_steps - native == 48
        self.pending = None
        self.report['intervals'].append(dict(validated=True, capture_time_s=t))
        self.report['validated_control_intervals'] += 1
        self.report['status'] = 'validated_intervals'

    def fail(self, exc):
        self.report.update(status='failed', error=str(exc))

    def close(self):
        self.report['callback_removed'] = True
        counters = dict(native=self.base.native_steps, environment=self.base.native_steps)
        self.report['closure'] = dict(schema='native-counter-closure-v1', complete=True,
                                     callback_error=None, expected_counters=counters,
                                     observed_counters=counters.copy())


def base():
    return SimpleNamespace(dt=1/240, decimation=12, native_steps=0,
                           physical_state={'joints': [1., 2.], 'rope': [3., 4.]},
                           constructed=0, fail_start=False, fail_after=False)


def advance(audit, count):
    for _ in range(count):
        step = audit.completed_actions
        audit.before_step(step / 20, step + 1)
        audit.base.native_steps += audit.base.decimation  # ordinary environment step
        audit.after_step((step + 1) / 20, step + 2)


class ScheduleTests(unittest.TestCase):
    def test_only_declared_schedule_and_initial_rate_are_allowed(self):
        self.assertIsNone(validate_schedule(None, 480))  # Historical unscheduled recordings.
        self.assertEqual(validate_schedule(SCHEDULE, 240), SCHEDULE)
        for hz in (480, 960):
            with self.assertRaisesRegex(ValueError, 'initial --physics-hz 240'):
                validate_schedule(SCHEDULE, hz)
        wrong = dict(SCHEDULE, switch_after_completed_actions=1559)
        with self.assertRaisesRegex(ValueError, 'Unsupported'):
            validate_schedule(wrong, 240)

    def test_same_exact_boundary_for_demo_and_dream_and_no_hidden_step(self):
        outcomes = []
        for policy in ('demo', 'dream'):
            b = base()
            audit = FixedPolicyPhysics(b, factory=FakeTransition)
            advance(audit, 1560)
            self.assertEqual(b.constructed, 0, policy)
            snapshot = deepcopy((b.physical_state, b.native_steps))
            audit.before_step(78., 1561)
            self.assertEqual(snapshot, (b.physical_state, b.native_steps))
            self.assertEqual((b.dt, b.decimation, b.constructed), (1/960, 48, 1))
            b.native_steps += b.decimation
            audit.after_step(78.05, 1562)
            advance(audit, 2)
            result = audit.close(78.15)
            self.assertEqual(result['expected_control_intervals'], 3)
            self.assertEqual(result['status'], 'complete')
            self.assertTrue(result['native_transition']['callback_removed'])
            self.assertEqual(audit.close(78.15), result)
            outcomes.append(result)
        self.assertEqual(outcomes[0], outcomes[1])

    def test_episode_ending_at_or_before_boundary_needs_no_transition(self):
        for steps in (0, 1559, 1560):
            audit = FixedPolicyPhysics(base(), factory=FakeTransition)
            advance(audit, steps)
            report = audit.close(steps / 20)
            self.assertEqual(report['status'], 'not_reached')
            self.assertIsNone(report['native_transition'])
            self.assertEqual(report['expected_control_intervals'], 0)

    def test_caught_start_failure_remains_sticky_and_cleans_up(self):
        audit = FixedPolicyPhysics(base(), factory=FakeTransition)
        advance(audit, 1560)
        audit.base.fail_start = True
        with self.assertRaisesRegex(ValueError, 'native start failure'):
            audit.before_step(78., 1561)
        with self.assertRaisesRegex(PolicyPhysicsError, 'native start failure'):
            audit.close(78.)
        self.assertTrue(audit.report['native_transition']['callback_removed'])
        with self.assertRaises(PolicyPhysicsError):
            audit.close(78.)

    def test_caught_native_interval_failure_remains_sticky(self):
        audit = FixedPolicyPhysics(base(), factory=FakeTransition)
        advance(audit, 1560)
        audit.base.fail_after = True
        audit.before_step(78., 1561)
        audit.base.native_steps += 48
        with self.assertRaisesRegex(ValueError, 'native callback mismatch'):
            audit.after_step(78.05, 1562)
        with self.assertRaises(PolicyPhysicsError):
            audit.close(78.05)
        self.assertTrue(audit.report['native_transition']['callback_removed'])
        self.assertEqual(audit.completed_actions, 1560)

    def test_ordinary_step_exception_and_missing_after_are_failures(self):
        for notify in (False, True):
            audit = FixedPolicyPhysics(base(), factory=FakeTransition)
            advance(audit, 1560)
            audit.before_step(78., 1561)
            if notify:
                audit.fail(RuntimeError('ordinary policy step failed'))
            with self.assertRaises(PolicyPhysicsError):
                audit.close(78.)
            self.assertTrue(audit.report['native_transition']['callback_removed'])

    def test_clock_frame_and_duplicate_before_mismatches_cannot_pass(self):
        for kind in ('wrong_before_time', 'wrong_before_frames', 'wrong_after_time', 'wrong_frames', 'fractional_frames', 'duplicate_before'):
            audit = FixedPolicyPhysics(base(), factory=FakeTransition)
            with self.assertRaises(ValueError):
                if kind == 'wrong_before_time':
                    audit.before_step(.05, 1)
                elif kind == 'wrong_before_frames':
                    audit.before_step(0., 2)
                else:
                    audit.before_step(0., 1)
                    if kind == 'duplicate_before':
                        audit.before_step(0., 1)
                    else:
                        audit.after_step(.10 if kind == 'wrong_after_time' else .05,
                                         1 if kind == 'wrong_frames' else 2.5 if kind == 'fractional_frames' else 2)
            with self.assertRaises(PolicyPhysicsError):
                audit.close(0.)

    def test_missing_native_row_or_count_fails_finalization(self):
        for mutation in ('row', 'count', 'clock'):
            audit = FixedPolicyPhysics(base(), factory=FakeTransition)
            advance(audit, 1562)
            if mutation == 'row':
                audit.transition.report['intervals'].pop()
            elif mutation == 'count':
                audit.transition.report['validated_control_intervals'] -= 1
            else:
                audit.transition.report['intervals'][0]['capture_time_s'] = 78.10
            with self.assertRaisesRegex(PolicyPhysicsError, 'certificate is incomplete'):
                audit.close(78.10)

    def test_v2_requires_native_shutdown_closure(self):
        audit = FixedPolicyPhysics(base(), factory=FakeTransition)
        advance(audit, 1561)
        audit.transition.close = lambda: audit.transition.report.update(callback_removed=True)
        with self.assertRaisesRegex(PolicyPhysicsError, 'final callback/counter closure'):
            audit.close(78.05)
        self.assertEqual(audit.report['schema'], 'fixed-policy-physics-v2')
        self.assertEqual(audit.report['status'], 'failed')

    def test_failed_report_can_be_saved_after_cleanup(self):
        audit = FixedPolicyPhysics(base(), factory=FakeTransition)
        audit.fail(RuntimeError('intentional interruption'))
        with self.assertRaises(PolicyPhysicsError):
            audit.close(0.)
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'policy-physics-transition.json'
            audit.save(path)
            report = json.loads(path.read_text())
            self.assertEqual(report['status'], 'failed')
            self.assertTrue(report['closed'])
            self.assertFalse(report['external_force_path_called'])

    def test_bad_final_clock_still_closes_callback_and_saves_failure(self):
        audit = FixedPolicyPhysics(base(), factory=FakeTransition)
        advance(audit, 1561)
        with self.assertRaises(PolicyPhysicsError):
            audit.close(float('nan'))
        self.assertTrue(audit.report['native_transition']['callback_removed'])
        self.assertIsNone(audit.report['final_capture_time_s'])
        with TemporaryDirectory() as directory:
            audit.save(Path(directory) / 'failed.json')


if __name__ == '__main__':
    unittest.main()
