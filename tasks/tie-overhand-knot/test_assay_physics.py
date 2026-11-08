"""CPU mocks verify transition safeguards; native compatibility needs GPU evidence."""
from copy import deepcopy
import hashlib
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

from assay_physics import AssayPhysicsSwitch
import assay_physics
import test_runner


class Camera:
    def __init__(self):
        self.frame = np.array([20], dtype=np.int64)
        self._timestamp = np.array([93.3], dtype=np.float32)
        self._timestamp_last_update = self._timestamp.copy()


class Backend:
    callback = None
    removed = False
    @classmethod
    def step(cls):
        pass
    @classmethod
    def handles_decimation(cls):
        return False
    @classmethod
    def set_decimation(cls, value):
        pass  # Installed PhysX implementation is a no-op.
    @classmethod
    def register_callback(cls, callback, event, **kwargs):
        assert event == 'POST_PHYSICS_STEP'
        assert kwargs == dict(order=0, name='rope_assay_timestep_audit', wrap_weak_ref=False)
        cls.callback, cls.removed = callback, False
        def remove():
            cls.callback, cls.removed = None, True
        return SimpleNamespace(deregister=remove)


class ResolvableString(str):
    def __new__(cls, value, target=Backend):
        result = super().__new__(cls, value)
        result.target = target
        return result
    def __getattr__(self, name):
        return getattr(self.target, name)


class Base:
    def __init__(self):
        self.cfg = SimpleNamespace(sim=SimpleNamespace(dt=1/240, render_interval=12), decimation=12)
        self._physics_handles_decimation = False
        self._sim_step_counter, self.native_counter = 2340, 2343
        self.state = np.array([.10, .20, .30, .001, .002, .003])
        self.scene = SimpleNamespace(sensors={'top_cam': Camera(), 'review_cam': Camera()})
        self.sim = SimpleNamespace(cfg=self.cfg.sim, physics_manager=Backend,
            get_physics_step_count=lambda: self.native_counter,
            get_physics_dt=lambda: self.cfg.sim.dt, get_rendering_dt=lambda: .05)
    @property
    def step_dt(self):
        return self.cfg.sim.dt*self.cfg.decimation
    def advance(self, *, count=None, callback_dt=None, fresh=True):
        for _ in range(self.cfg.decimation if count is None else count):
            self.native_counter += 1
            self._sim_step_counter += 1
            for camera in self.scene.sensors.values():
                camera._timestamp = np.float32(camera._timestamp + np.float32(self.cfg.sim.dt))
            Backend.callback(self.cfg.sim.dt if callback_dt is None else callback_dt)
        for camera in self.scene.sensors.values():
            camera.frame += 1
            if fresh:
                camera._timestamp_last_update = camera._timestamp.copy()


class SwitchTests(unittest.TestCase):
    def setUp(self):
        for target, replacement in (
            ('assay_physics._bindings', lambda: (Backend, 'POST_PHYSICS_STEP', Camera, ResolvableString)),
            ('assay_physics._source_provenance', lambda: {'mock': True}),
            ('assay_physics._state_digest', lambda base: hashlib.sha256(base.state.tobytes()).hexdigest()),
        ):
            context = patch(target, replacement)
            context.start()
            self.addCleanup(context.stop)
        self.base = Base()
        self.switch = AssayPhysicsSwitch(self.base, 960, 93.3)
        self.addCleanup(self.switch.close)

    def test_switch_preserves_state_counters_then_measures_native_48_substeps_and_cameras(self):
        state = self.base.state.copy()
        counters = (self.base.native_counter, self.base._sim_step_counter)
        self.switch.start()
        np.testing.assert_array_equal(state, self.base.state)
        self.assertEqual(counters, (self.base.native_counter, self.base._sim_step_counter))
        self.assertEqual(self.base.cfg.decimation, 48)
        self.assertEqual(self.base.step_dt, .05)
        for index in range(3):
            self.switch.before_step(93.3+index*.05, index)
            self.base.advance()
            self.switch.after_step(93.3+(index+1)*.05, index+1)
        self.assertEqual(self.switch.report['status'], 'validated_intervals')
        self.assertTrue(all(row['native_substeps'] == 48 for row in self.switch.report['intervals']))
        self.switch.close()
        self.assertTrue(Backend.removed)
        closure = self.switch.report['closure']
        self.assertTrue(closure['complete'])
        self.assertIsNone(closure['callback_error'])
        self.assertEqual(closure['expected_counters'], closure['observed_counters'])
        self.assertEqual(closure['observed_counters']['native'], counters[0]+3*48)
        self.assertFalse(self.switch.report['diagnostic_only'])

    def test_callback_after_last_interval_fails_closed(self):
        self.switch.start()
        self.switch.before_step(93.3, 0)
        self.base.advance()
        self.switch.after_step(93.35, 1)
        self.switch._callback(1/960)  # No following after_step() to catch this.
        self.switch.close()
        self.assertEqual(self.switch.report['status'], 'failed')
        self.assertIn('outside an audited interval', self.switch.report['closure']['callback_error'])
        self.assertFalse(self.switch.report['closure']['complete'])
        self.assertTrue(Backend.removed)

    def test_unobserved_counter_advance_after_last_interval_fails_closed(self):
        self.switch.start()
        self.switch.before_step(93.3, 0)
        self.base.advance()
        self.switch.after_step(93.35, 1)
        self.base.native_counter += 1
        self.base._sim_step_counter += 1
        self.switch.close()
        self.assertEqual(self.switch.report['status'], 'failed')
        closure = self.switch.report['closure']
        self.assertFalse(closure['complete'])
        self.assertEqual(closure['observed_counters']['native'] - closure['expected_counters']['native'], 1)
        self.assertTrue(Backend.removed)

    def test_counter_jump_between_intervals_cannot_hide_in_a_later_baseline(self):
        self.switch.start()
        self.switch.before_step(93.3, 0)
        self.base.advance()
        self.switch.after_step(93.35, 1)
        self.base.native_counter += 1
        self.base._sim_step_counter += 1
        self.switch.before_step(93.35, 1)
        self.base.advance()
        self.switch.after_step(93.4, 2)
        self.switch.close()
        self.assertEqual(self.switch.report['status'], 'failed')
        self.assertIn('outside completed audited intervals', self.switch.report['error'])

    def test_diagnostic_flag_is_explicit_and_does_not_relax_guards(self):
        base = Base()
        base.sim.physics_manager = object()
        switch = AssayPhysicsSwitch(base, 960, 0., diagnostic_only=True)
        with self.assertRaises(ValueError):
            switch.start()
        self.assertTrue(switch.report['diagnostic_only'])
        self.assertEqual(switch.report['status'], 'failed')

    def test_guard_backend_contact_sensor_config_identity_and_control_rate(self):
        mutations = [
            lambda b: setattr(b.sim, 'physics_manager', object()),
            lambda b: b.scene.sensors.update(contact_sensor=object()),
            lambda b: setattr(b.cfg, 'sim', deepcopy(b.sim.cfg)),
            lambda b: setattr(b.cfg, 'decimation', 24),
        ]
        for mutation in mutations:
            base = Base()
            mutation(base)
            switch = AssayPhysicsSwitch(base, 960, 0.)
            with self.assertRaises(ValueError):
                switch.start()
            self.assertEqual(switch.report['status'], 'failed')
            self.assertEqual(base.sim.cfg.dt, 1/240)
            self.assertIsNone(switch.handle)

    def test_exact_instance_and_lazy_proxy_are_supported_but_foreign_delegation_is_not(self):
        reference = Backend.__module__ + ':' + Backend.__qualname__
        for raw, kind in ((Backend(), 'instance'), (ResolvableString(reference), 'ResolvableString')):
            base = Base()
            base.sim.physics_manager = raw
            switch = AssayPhysicsSwitch(base, 960, 0.)
            switch.start()
            self.assertEqual(switch.report['physics_manager']['representation'], kind)
            self.assertEqual(switch.report['physics_manager']['callback_owner'], reference)
            switch.close()
        class OtherBackend(Backend):
            pass
        for raw in (ResolvableString(reference, OtherBackend), ResolvableString('wrong:Backend'), OtherBackend):
            base = Base()
            base.sim.physics_manager = raw
            switch = AssayPhysicsSwitch(base, 960, 0.)
            with self.assertRaises(ValueError):
                switch.start()
            self.assertEqual(base.cfg.sim.dt, 1/240)

    def test_wrong_native_count_or_dt_fails_even_when_requested_config_is_correct(self):
        for kwargs in ({'count': 47}, {'callback_dt': 1/240}):
            base = Base()
            switch = AssayPhysicsSwitch(base, 960, 0.)
            switch.start()
            switch.before_step(0., 0)
            base.advance(**kwargs)
            with self.assertRaisesRegex(ValueError, 'callback count or dt'):
                switch.after_step(.05, 1)
            self.assertEqual(switch.report['status'], 'failed')
            switch.close()
            self.assertTrue(Backend.removed)

    def test_stale_camera_or_capture_time_cannot_validate(self):
        for fresh, timestamp, frames in ((False, 93.35, 1), (True, 93.4, 1), (True, 93.35, 0)):
            base = Base()
            switch = AssayPhysicsSwitch(base, 960, 93.3)
            switch.start()
            switch.before_step(93.3, 0)
            base.advance(fresh=fresh)
            with self.assertRaises(ValueError):
                switch.after_step(timestamp, frames)
            self.assertEqual(switch.report['status'], 'failed')
            switch.close()

    def test_state_change_during_switch_fails_and_removes_callback_without_restoring_state(self):
        def bad_decimation(cls, value):
            self.base.state[0] += 1.
        with patch.object(Backend, 'set_decimation', classmethod(bad_decimation)):
            with self.assertRaisesRegex(ValueError, 'State or counters changed'):
                self.switch.start()
        self.assertEqual(self.base.cfg.sim.dt, 1/240)
        self.assertEqual(self.base.state[0], 1.1)  # Helper must never rewrite state to conceal failure.
        self.assertTrue(Backend.removed)

    def test_interrupted_interval_deregisters_without_inventing_validation(self):
        self.switch.start()
        self.switch.before_step(93.3, 0)
        self.switch.close()
        self.assertEqual(self.switch.report['status'], 'failed')
        self.assertTrue(Backend.removed)

    def test_actual_runner_switches_before_first_seating_and_cleans_callback(self):
        harness = test_runner.RunnerTests()
        harness.setUp()
        self.addCleanup(harness.doCleanups)
        runner = harness.embodiment()
        runner._base = self.base
        original_step = test_runner.FakeEmbodiment.step
        def measured_step(instance, action):
            instance._base.advance()
            return original_step(instance, action)
        with patch.dict(sys.modules, {'assay_physics': assay_physics}), \
                patch.object(test_runner.FakeEmbodiment, 'step', measured_step):
            runner.close()
        report = harness.report('assay-physics-transition.json')
        self.assertFalse(report['diagnostic_only'])
        self.assertEqual(report['status'], 'validated_intervals')
        self.assertEqual(report['validated_control_intervals'], 140)
        self.assertEqual(len(report['intervals']), 140)
        self.assertEqual(runner.seen[0], ('seating', .05, 0.))
        self.assertEqual(report['intervals'][0]['capture_time_s'], .05)
        self.assertAlmostEqual(report['intervals'][-1]['capture_time_s'], 7.)
        self.assertTrue(report['callback_removed'])
        self.assertEqual(harness.report()['score'], 1.)
        self.assertTrue(harness.report()['finalization']['complete'])
        self.assertEqual(harness.metadata['inspect_policy_score'], .5)

    def test_caught_step_failure_still_raises_at_close_and_marks_finalization_false(self):
        harness = test_runner.RunnerTests()
        harness.setUp()
        self.addCleanup(harness.doCleanups)
        runner = harness.embodiment()
        runner._base = self.base
        self.base.sim.physics_manager = object()  # Native failure001's guard path.
        with patch.dict(sys.modules, {'assay_physics': assay_physics}):
            with self.assertRaises(BaseExceptionGroup):
                runner._run_assay()
        # This call is deliberately outside exception handling, as when Inspect
        # records the step error then calls close with sys.exception() == None.
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        result = harness.report()
        self.assertEqual(result['score'], .5)
        self.assertFalse(result['finalization']['complete'])
        self.assertTrue(result['finalization']['errors'])
        self.assertEqual(result['assay_physics_transition']['status'], 'failed')
        self.assertTrue(result['assay_physics_transition']['callback_removed'])
        self.assertTrue(runner.base_closed)
        self.assertTrue(runner.audit.closed)
        self.assertEqual(len(runner.seen), 0)


if __name__ == '__main__':
    unittest.main()
