"""Local assessor/finalization checks; no simulator, encoder, or GPU required.

Compile the actual runner class from its AST because importing run.py requires
the simulator stack. Only its base embodiment, endpoint load, and video process
are replaced. The real scorer consumes synthetic telemetry from test_scoring;
topology positives are injected. These tests verify orchestration and cleanup,
not physical knot retention, scene startup, or the complete Inspect CLI.

Run: python -m unittest discover -s tasks/tie-overhand-knot -p test_runner.py -v
"""
import ast
import argparse
from copy import deepcopy
import io
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from scoring import KnotScorer
from test_scoring import candidate, proof_sample, straight_sample


class FakeEmbodiment:
    def step(self, action):
        if getattr(self, 'ordinary_step_error', False):
            raise RuntimeError('ordinary policy step failure')
        self._t += self._base.step_dt
        if self.endpoint_load is None:
            self.review_frames += 1
            self.seen.append(action)
            return 'ordinary-observation'
        self.endpoint_load.elapsed += self._base.step_dt
        elapsed, force = self.endpoint_load.elapsed, self.endpoint_load.magnitude
        self.task.module._tracker.update(proof_sample(self._t, elapsed, force, self.assay_phase))
        self.review_frames += 1
        self.seen.append((self.assay_phase, elapsed, force))

    def close(self):
        self.base_closed = True
        if getattr(self, "base_close_error", False):
            raise RuntimeError("base close failure")


class FakeLoad:
    fail_at = None

    def __init__(self, base, magnitude):
        self.magnitude, self.elapsed = magnitude, 0.
        self.active, self.calls = False, 0

    def apply(self, start, end):
        self.calls += 1
        self.active = True
        if self.calls == self.fail_at:
            raise RuntimeError("force apply failure")

    def clear(self):
        self.active = False


class FakeTransition:
    """Require one audit bracket for each control step without a native backend."""
    fail_start = False
    fail_after = None
    constructed = 0

    def __init__(self, base, target_hz, capture_time_s, *, diagnostic_only=False):
        type(self).constructed += 1
        self.target_hz = target_hz
        self.report = {'status': 'not_started', 'diagnostic_only': diagnostic_only,
                       'validated_control_intervals': 0, 'callback_removed': False,
                       'control_hz': 20, 'target_physics_hz': target_hz,
                       'requested_substeps': target_hz // 20, 'old_physics_hz': 240.,
                       'native_physics_dt_s': 1/target_hz, 'state_writes': False,
                       'counter_writes': False, 'state_sha256_before': 'test-only',
                       'state_sha256_after': 'test-only', 'counters_before': {'native': 1, 'environment': 1},
                       'counters_after': {'native': 1, 'environment': 1}, 'switch_capture_time_s': capture_time_s,
                       'intervals': []}
        self.pending = None

    def start(self):
        if self.fail_start:
            raise ValueError('transition guard failed')
        self.report['status'] = 'awaiting_intervals'

    def before_step(self, time, frames):
        if self.pending is not None:
            raise ValueError('unclosed interval')
        self.pending = (time, frames)

    def after_step(self, time, frames):
        previous, previous_frames = self.pending
        if not np.isclose(time - previous, .05) or frames != previous_frames + 1:
            raise ValueError('incorrect capture interval')
        if self.report['validated_control_intervals'] + 1 == self.fail_after:
            raise ValueError('native callback mismatch')
        self.pending = None
        substeps = self.target_hz // 20
        camera = dict(frame_delta=[1], timestamp_s=[time], last_update_s=[time],
                      float32_predicted_timestamp_s=[time])
        self.report['intervals'].append(dict(
            validated=True, capture_time_s=time, native_substeps=substeps,
            native_callback_dt_s=[1/self.target_hz]*substeps,
            counter_deltas={'native': substeps, 'environment': substeps},
            cameras={'review_cam': camera, 'top_cam': camera}))
        self.report['validated_control_intervals'] += 1
        self.report['status'] = 'validated_intervals'

    def fail(self, error):
        self.report.update(status='failed', error=str(error))

    def close(self):
        if self.pending is not None:
            self.fail(ValueError('unfinished interval'))
        self.report['callback_removed'] = True
        counters = {key: value + self.report['validated_control_intervals'] * (self.target_hz//20)
                    for key, value in self.report['counters_after'].items()}
        self.report['closure'] = dict(schema='native-counter-closure-v1', complete=True,
                                     callback_error=None, expected_counters=counters,
                                     observed_counters=counters.copy())

    def save(self, path):
        path.write_text(json.dumps(self.report))


class FakeEncoder:
    def __init__(self, timeout=False):
        self.stdin = io.BytesIO()
        self.timeout, self.killed, self.code = timeout, False, None

    def wait(self, timeout):
        if self.timeout and not self.killed:
            raise subprocess.TimeoutExpired("ffmpeg", timeout)
        self.code = -9 if self.killed else 0
        return self.code

    def poll(self):
        return self.code

    def kill(self):
        self.killed = True


class RunnerTests(unittest.TestCase):
    def test_preformed_demo_is_rejected_before_simulator_import_or_output_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'must-not-be-created'
            for policy in ('demo', 'dream'):
                with self.subTest(policy=policy):
                    result = subprocess.run([sys.executable, str(Path(__file__).with_name('run.py')),
                        '--policy', policy, '--diagnostic-layout', 'preformed',
                        '--log-dir', str(output)], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 2)
                    self.assertIn('v4 has only a free, uncrossed cord', result.stderr)
                    self.assertNotIn('ModuleNotFoundError', result.stderr)
                    self.assertFalse(output.exists())

    def test_dream_chunk_size_rejected_before_simulator_import(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'must-not-be-created'
            for value in ('0', '49'):
                result = subprocess.run([sys.executable, str(Path(__file__).with_name('run.py')),
                    '--policy', 'dream', '--dream-rows-per-request', value,
                    '--log-dir', str(output)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn('invalid choice', result.stderr)
                self.assertNotIn('ModuleNotFoundError', result.stderr)
                self.assertFalse(output.exists())

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name)
        source = Path(__file__).with_name("run.py")
        main = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == "main")
        cls = next(node for node in main.body
                   if isinstance(node, ast.ClassDef) and node.name == "AuditedRopeEmbodiment")
        self.metadata = {}
        namespace = dict(
            YamArenaEmbodiment=FakeEmbodiment, np=np, json=json, sys=sys,
            subprocess=subprocess, metadata=self.metadata, deepcopy=deepcopy,
            policy_physics_schedule=None,
            args=SimpleNamespace(skip_assay=False, diagnostic_layout=None, log_dir=self.folder,
                                 physics_hz=240, assay_physics_hz=960, proof_probe=False),
            CONFIG={"proof_seconds": 5., "proof_load_n": .981,
                    'physics_hz': 240, 'assay_physics_hz': 960},
            telemetry=SimpleNamespace(demo_events=[]),
        )
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), "exec"), namespace)
        self.runner_class = namespace[cls.name]
        modules = patch.dict(sys.modules, {
            "proof_load": SimpleNamespace(EndpointLoad=FakeLoad),
            "assay_physics": SimpleNamespace(AssayPhysicsSwitch=FakeTransition),
            "rope": SimpleNamespace(rope_state=lambda *args: SimpleNamespace(
                start_tip=np.zeros(3), end_tip=np.array([.64, 0., 0.]))),
        })
        modules.start()
        self.addCleanup(modules.stop)
        topology = patch("scoring.analyze_open_rope", return_value=candidate())
        topology.start()
        self.addCleanup(topology.stop)
        failure = patch.object(FakeLoad, "fail_at", None)
        failure.start()
        self.addCleanup(failure.stop)
        for name, value in (('fail_start', False), ('fail_after', None), ('constructed', 0)):
            context = patch.object(FakeTransition, name, value)
            context.start()
            self.addCleanup(context.stop)

    def embodiment(self):
        runner = self.runner_class.__new__(self.runner_class)
        runner._close_finished = False
        runner._base = SimpleNamespace(step_dt=.05)
        runner._t, runner.seen = 0., []
        runner.assay_attempted, runner.assay_phase = False, None
        runner.physics_switch, runner.assay_transition_required = None, False
        runner._transition_failure = None
        runner.policy_physics_audit, runner._policy_physics_failure = None, None
        runner.last_policy_action = SimpleNamespace(data=np.zeros(14))
        runner.policy_steps = 1
        scorer = KnotScorer()
        scorer.update(straight_sample())
        runner.task = SimpleNamespace(module=SimpleNamespace(_tracker=scorer))
        runner.audit = io.StringIO()
        runner.review_encoder, runner.review_stderr = FakeEncoder(), io.BytesIO()
        runner.review_frames, runner.endpoint_load = 1, None
        return runner

    def report(self, name="score-summary.json"):
        return json.loads((self.folder / name).read_text())

    def test_assay_phases_final_score_and_idempotent_close(self):
        runner = self.embodiment()
        runner.close()
        result = self.report()
        self.assertEqual(result["score"], 1.)
        self.assertEqual(self.metadata["inspect_policy_score"], .5)
        self.assertEqual(self.metadata["audited_final_score"], 1.)
        self.assertTrue(result["finalization"]["complete"])
        self.assertTrue(result['assay_physics_transition']['required'])
        self.assertEqual(result['assay_physics_transition']['validated_control_intervals'], 140)
        self.assertFalse(runner.physics_switch.report['diagnostic_only'])
        self.assertEqual(len(runner.seen), 140)
        self.assertEqual(runner.seen[0], ("seating", .05, 0.))
        self.assertEqual(runner.seen[40], ("proof", .05, .981))
        self.assertFalse(runner.endpoint_load.active)
        self.assertTrue(runner.audit.closed)
        self.assertTrue(runner.base_closed)
        runner.close()
        self.assertEqual(len(runner.seen), 140)

    def test_assay_failure_cleans_up_and_preserves_partial_score(self):
        FakeLoad.fail_at = 5
        runner = self.embodiment()
        encoder = runner.review_encoder
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        self.assertFalse(runner.endpoint_load.active)
        self.assertTrue(runner.audit.closed)
        self.assertTrue(encoder.stdin.closed)
        self.assertTrue(runner.review_stderr.closed)
        self.assertTrue(runner.base_closed)
        self.assertEqual(self.report()["score"], .5)
        self.assertFalse(self.report()["finalization"]["complete"])
        self.assertFalse(self.report("assay.json")["completed"])
        self.assertTrue(self.report("assay.json")["errors"])
        runner.close()  # Failure cleanup must also be idempotent.

    def test_encoder_timeout_kills_process_and_preserves_score(self):
        runner = self.embodiment()
        encoder = runner.review_encoder = FakeEncoder(timeout=True)
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        self.assertTrue(encoder.killed)
        self.assertTrue(runner.audit.closed)
        self.assertTrue(runner.base_closed)
        self.assertFalse(runner.endpoint_load.active)
        self.assertEqual(self.report()["score"], .5)
        self.assertEqual(self.report()['geometric_score'], 1.)
        self.assertFalse(self.report()["finalization"]["complete"])

    def test_existing_runner_failure_does_not_start_assay(self):
        runner = self.embodiment()
        try:
            raise RuntimeError("policy failed")
        except RuntimeError:
            runner.close()
        self.assertEqual(runner.seen, [])
        self.assertTrue(runner.base_closed)
        result = self.report()
        self.assertEqual(result["score"], .5)
        self.assertFalse(result["finalization"]["complete"])
        self.assertIn("policy failed", result["finalization"]["interrupted_by"])

    def test_base_close_failure_is_reported_in_preserved_summary(self):
        runner = self.embodiment()
        runner.base_close_error = True
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        finalization = self.report()["finalization"]
        self.assertFalse(finalization["complete"])
        self.assertIn("base close failure", " ".join(finalization["errors"]))

    def test_zero_policy_result_finishes_without_assessor_or_transition(self):
        runner = self.embodiment()
        runner.task.module._tracker = KnotScorer()
        with patch('scoring.analyze_open_rope', return_value={
                'status': 'unknot', 'is_overhand': False, 'reason': 'test-only negative'}):
            runner.task.module._tracker.update(straight_sample())
            runner.close()
        result = self.report()
        self.assertEqual(result['score'], 0.)
        self.assertTrue(result['finalization']['complete'])
        self.assertFalse(result['assay_physics_transition']['required'])
        self.assertEqual(result['assay_physics_transition']['status'], 'not_required')
        self.assertIsNone(runner.physics_switch)
        self.assertEqual(FakeTransition.constructed, 0)
        self.assertEqual(runner.seen, [])

    def test_required_switch_failure_cannot_be_swallowed_by_inspect(self):
        runner = self.embodiment()
        FakeTransition.fail_start = True
        with self.assertRaises(BaseExceptionGroup):
            runner._run_assay()
        # Inspect may catch this error before invoking close. The stored failure
        # must still prohibit a complete final result when sys.exception is None.
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        result = self.report()
        self.assertEqual(result['score'], .5)
        self.assertFalse(result['finalization']['complete'])
        self.assertEqual(result['assay_physics_transition']['status'], 'failed')
        self.assertTrue(result['assay_physics_transition']['callback_removed'])
        self.assertTrue(runner.base_closed)
        self.assertTrue(runner.audit.closed)
        self.assertEqual(runner.seen, [])

    def test_failed_last_native_interval_cannot_report_full_authoritative_credit(self):
        runner = self.embodiment()
        FakeTransition.fail_after = 140
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        result = self.report()
        self.assertEqual(result['geometric_score'], 1.)
        self.assertEqual(result['candidate_score'], 1.)
        self.assertEqual(result['score'], .5)
        self.assertEqual(self.metadata['audited_final_score'], .5)
        self.assertEqual(self.metadata['geometric_score'], 1.)
        self.assertEqual(result['assay_certification']['status'], 'incomplete')
        self.assertFalse(result['finalization']['complete'])
        self.assertEqual(result['assay_physics_transition']['validated_control_intervals'], 139)
        self.assertFalse(runner.endpoint_load.active)
        self.assertTrue(runner.base_closed)

    def bimanual_embodiment(self):
        runner = self.embodiment()
        self.runner_class.close.__globals__['CONFIG']['score_version'] = 'overhand-bimanual-v4'
        # This isolates runner finalization; scorer geometry/timing is exercised
        # independently in test_bimanual_scoring, not invented by this stub.
        runner.task.module._tracker = SimpleNamespace(summary=lambda final=False: dict(
            score=1., candidate_score=1., invalid=None, diagnostic_only=False,
            release_elapsed_s=5., release_required_s=5., status='qualified'))
        return runner

    def test_bimanual_close_never_adds_external_pull_or_simulation_steps(self):
        runner = self.bimanual_embodiment()
        runner.close()
        result = self.report()
        self.assertEqual(result['score'], 1.)
        self.assertTrue(result['finalization']['complete'])
        self.assertFalse(result['assay_certification']['required'])
        self.assertEqual(FakeTransition.constructed, 0)
        self.assertIsNone(runner.endpoint_load)
        self.assertEqual(runner.seen, [])
        self.assertTrue(runner.audit.closed)
        self.assertTrue(runner.base_closed)

    def test_bimanual_incomplete_recording_cannot_claim_full_credit(self):
        runner = self.bimanual_embodiment()
        runner.base_close_error = True
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        result = self.report()
        self.assertEqual(result['score'], .5)
        self.assertFalse(result['finalization']['complete'])
        self.assertFalse(result['finalization_certification']['complete'])
        self.assertEqual(FakeTransition.constructed, 0)
        self.assertEqual(runner.seen, [])

    def scheduled_embodiment(self):
        from fixed_policy_physics import FixedPolicyPhysics, SCHEDULE
        runner = self.bimanual_embodiment()
        self.runner_class.close.__globals__['policy_physics_schedule'] = deepcopy(SCHEDULE)
        runner.policy_physics_audit = FixedPolicyPhysics(runner._base, factory=FakeTransition)
        runner.policy_steps = 0
        return runner

    def advance_policy(self, runner, count):
        action = SimpleNamespace(data=np.zeros(14))
        for _ in range(count):
            self.assertEqual(runner.step(action), 'ordinary-observation')
            self.assertIs(runner.seen[-1], action)
        return action

    def test_scheduled_policy_brackets_unchanged_actions_at_exact_boundary(self):
        runner = self.scheduled_embodiment()
        self.advance_policy(runner, 1560)
        self.assertEqual(FakeTransition.constructed, 0)
        self.assertAlmostEqual(runner._t, 78.)
        self.advance_policy(runner, 3)
        self.assertEqual(FakeTransition.constructed, 1)
        runner.close()
        cert = self.report('policy-physics-transition.json')
        self.assertEqual(cert['status'], 'complete')
        self.assertEqual(cert['completed_actions'], 1563)
        self.assertEqual(cert['expected_control_intervals'], 3)
        self.assertAlmostEqual(cert['native_transition']['switch_capture_time_s'], 78.)
        self.assertEqual(len(runner.seen), 1563)
        self.assertIsNone(runner.endpoint_load)
        self.assertFalse((self.folder / 'assay-physics-transition.json').exists())
        self.assertTrue(self.report()['finalization']['complete'])
        self.assertEqual(self.metadata['policy_physics_transition']['status'], 'complete')

    def test_scheduled_short_episode_preserves_not_reached_certificate(self):
        runner = self.scheduled_embodiment()
        self.advance_policy(runner, 2)
        runner.close()
        self.assertEqual(self.report('policy-physics-transition.json')['status'], 'not_reached')
        self.assertEqual(FakeTransition.constructed, 0)
        self.assertTrue(self.report()['finalization']['complete'])

    def test_scheduled_caught_native_failure_cannot_be_finalized_as_success(self):
        runner = self.scheduled_embodiment()
        self.advance_policy(runner, 1560)
        FakeTransition.fail_after = 1
        with self.assertRaisesRegex(ValueError, 'native callback mismatch'):
            runner.step(SimpleNamespace(data=np.zeros(14)))
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        cert = self.report('policy-physics-transition.json')
        self.assertEqual(cert['status'], 'failed')
        self.assertTrue(cert['native_transition']['callback_removed'])
        self.assertFalse(self.report()['finalization']['complete'])
        self.assertEqual(self.report()['score'], 0.)
        self.assertEqual(self.report()['geometric_score'], 1.)
        self.assertEqual(self.report()['policy_physics_certification']['status'], 'incomplete')
        self.assertTrue(runner.base_closed)
        self.assertTrue(runner.audit.closed)

    def test_scheduled_ordinary_step_exception_still_removes_callback(self):
        runner = self.scheduled_embodiment()
        self.advance_policy(runner, 1560)
        runner.ordinary_step_error = True
        with self.assertRaisesRegex(RuntimeError, 'ordinary policy step failure'):
            runner.step(SimpleNamespace(data=np.zeros(14)))
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        cert = self.report('policy-physics-transition.json')
        self.assertEqual(cert['status'], 'failed')
        self.assertTrue(cert['native_transition']['callback_removed'])
        self.assertEqual(len(runner.seen), 1560)
        self.assertIsNone(runner.endpoint_load)

    def test_scheduled_repeated_reset_is_rejected(self):
        runner = self.scheduled_embodiment()
        runner._policy_reset_started = True
        with self.assertRaisesRegex(RuntimeError, 'fresh embodiment'):
            runner.reset(None)
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        self.assertFalse(self.report()['finalization']['complete'])

    def test_scheduled_missing_audit_cannot_claim_complete_finalization(self):
        runner = self.scheduled_embodiment()
        runner.policy_physics_audit = None
        with self.assertRaisesRegex(RuntimeError, 'not initialized'):
            runner.step(SimpleNamespace(data=np.zeros(14)))
        with self.assertRaises(BaseExceptionGroup):
            runner.close()
        self.assertFalse(self.report()['finalization']['complete'])
        self.assertEqual(self.report()['policy_physics_transition']['status'], 'missing')
        self.assertEqual(self.report()['score'], 0.)
        self.assertEqual(self.report()['geometric_score'], 1.)
        self.assertTrue(runner.base_closed)

    def test_scheduled_non240_cli_rejected_before_output_creation(self):
        from fixed_policy_physics import SCHEDULE
        output = self.folder / 'must-not-exist'
        source = Path(__file__).with_name('run.py').resolve()
        script = ('import runpy,sys; sys.path.insert(0,sys.argv[1]); from config import CONFIG; '
                  'import json; CONFIG["policy_physics_schedule"]=json.loads(sys.argv[2]); '
                  'script=sys.argv[3]; sys.argv=[script]+sys.argv[4:]; runpy.run_path(script,run_name="__main__")')
        for hz in ('480', '960'):
            result = subprocess.run([sys.executable, '-c', script, str(source.parent),
                json.dumps(SCHEDULE), str(source), '--policy', 'dream', '--physics-hz', hz,
                '--log-dir', str(output)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn('requires initial --physics-hz 240', result.stderr)
            self.assertNotIn('ModuleNotFoundError', result.stderr)
            self.assertFalse(output.exists())


class ExecutionOutcomeTests(unittest.TestCase):
    """Inspect may swallow a PolicyError before the embodiment close hook."""

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.folder = Path(temp.name)
        source = Path(__file__).with_name('run.py')
        names = {'_inspect_execution', '_record_execution_outcome'}
        functions = [node for node in ast.parse(source.read_text()).body
                     if isinstance(node, ast.FunctionDef) and node.name in names]
        namespace = dict(Path=Path, json=json, hashlib=hashlib)
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
        self.check = namespace['_inspect_execution']
        self.record = namespace['_record_execution_outcome']
        self.metadata = dict(audited_final_score=1., geometric_score=1., finalization_complete=True)
        self.summary = dict(score=1., geometric_score=1., candidate_score=1., status='qualified',
            finalization=dict(complete=True, errors=[], interrupted_by=None),
            finalization_certification=dict(complete=True),
            policy_physics_certification=dict(required=True, status='complete', errors=[]))
        (self.folder / 'score-summary.json').write_text(json.dumps(self.summary))

    def log(self, *, status='success', error=None, code_name='adhoc_test.json'):
        log = dict(status=status, error='all 1 trial(s) errored; nothing was scored' if error else None,
            results=dict(total_scenes=1, total_trials=1, errored_trials=int(error is not None)),
            samples=[dict(scene_id='scene-0', status=status, error=error,
                          termination_reasons=['policy_done'] if not error else [None])])
        path = self.folder / code_name
        path.write_text(json.dumps(log))
        return path

    def test_saved_policy_error_overrides_zero_cli_and_already_closed_summary(self):
        self.log(status='error', error="PolicyError: _event() got multiple values for argument 'label'")
        execution = self.check(self.folder, set(), 0)
        self.assertEqual(execution['status'], 'failed')
        self.assertIn('PolicyError:', execution['errors'][0])
        self.assertEqual(execution['logs'][0]['errored_trials'], 1)
        self.record(self.folder, self.metadata, execution)
        result = json.loads((self.folder / 'score-summary.json').read_text())
        self.assertEqual(result['score'], 0.)
        self.assertEqual(result['geometric_score'], 1.)
        self.assertEqual(self.metadata['audited_final_score'], 0.)
        self.assertFalse(result['finalization']['complete'])
        self.assertFalse(result['finalization_certification']['complete'])
        self.assertEqual(result['policy_physics_certification']['status'], 'complete')

    def test_graceful_recorded_policy_stop_remains_eligible(self):
        path = self.log()  # A caught _StopAttempt ends the generator normally.
        for return_code in (0, None):
            execution = self.check(self.folder, set(), return_code)
            self.assertEqual(execution['status'], 'complete')
            self.assertEqual(execution['logs'][0]['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
            self.record(self.folder, self.metadata, execution)
            result = json.loads((self.folder / 'score-summary.json').read_text())
            self.assertEqual(result['score'], 1.)
            self.assertTrue(result['finalization']['complete'])

    def test_nonzero_cli_or_exception_overrides_successful_log(self):
        self.log()
        for code, exceptions in ((1, ()), (0, (RuntimeError('policy transport failed'),))):
            execution = self.check(self.folder, set(), code, exceptions)
            self.assertEqual(execution['status'], 'failed')
            self.record(self.folder, self.metadata, execution)
            self.assertEqual(self.metadata['audited_final_score'], 0.)
            self.assertFalse(self.metadata['finalization_complete'])

    def test_stale_missing_multiple_and_malformed_logs_fail_closed(self):
        old = self.log(code_name='adhoc_old.json')
        self.assertEqual(self.check(self.folder, {old}, 0)['status'], 'failed')
        new = self.log(code_name='adhoc_new.json')
        self.assertEqual(self.check(self.folder, {old}, 0)['status'], 'complete')
        self.assertEqual(self.check(self.folder, set(), 0)['status'], 'failed')
        new.write_text('{')
        self.assertEqual(self.check(self.folder, {old}, 0)['status'], 'failed')


class DreamTransportTests(unittest.TestCase):
    """Exercise the retained real SDK using a fake, credential-free client."""

    def setUp(self):
        import dream
        from run import _register_dream_policy
        self.register_policy = _register_dream_policy
        self.factories, self.audit = {}, []
        self.responses = ['success', 'success']
        responses = self.responses

        class Client:
            calls = 0

            def __init__(self):
                self.session_id = None
                self.last_timing = {'serve_infer_ms': 1}

            def get_models(self):
                return [SimpleNamespace(id='dreamlabs/dream-1-beta')]

            def policy(self, config):
                self.policy_config = config

            def get_action(self, *, commanded=None):
                Client.calls += 1
                response = responses.pop(0)
                if response == 'expired':
                    self.session_id = 'replacement'
                    raise dream.DreamSessionExpired('test session expired', reason='expired')
                self.session_id = 'replacement' if response == 'changed' else 'original'
                return dict(q_l_chunk=np.zeros((48, 7)), q_r_chunk=np.zeros((48, 7)))

            def close(self):
                pass

        self.client = Client
        mock = patch.object(dream, 'DreamClient', Client)
        mock.start()
        self.addCleanup(mock.stop)

    def register(self, kind, name):
        self.assertEqual(kind, 'policy')
        def save(factory):
            self.factories[name] = factory
            return factory
        return save

    def session(self, rows=8):
        from inspect_robots_dream.session import Episode
        name = self.register_policy(rows, self.register, self.audit)
        policy = self.factories[name]()
        self.assertEqual(policy.rows_per_request, rows)
        policy._session.start(Episode())
        return policy._session

    def test_first_session_and_continuing_session_return_unchanged_chunks(self):
        session = self.session(12)
        for _ in range(2):
            chunk = session.act(None)
            self.assertEqual(chunk.rows.shape, (48, 14))
            self.assertEqual(chunk.attempts, 1)
            np.testing.assert_array_equal(chunk.rows, 0.)
        self.assertEqual(self.client.calls, 2)
        self.assertEqual([event['status'] for event in self.audit], ['success', 'success'])
        self.assertIsNone(self.audit[0]['idle_wall_seconds'])
        self.assertGreaterEqual(self.audit[1]['idle_wall_seconds'], 0.)

    def test_expiry_stops_before_adapter_retry(self):
        self.responses[:] = ['success', 'expired', 'success']
        session = self.session()
        session.act(None)
        with self.assertRaisesRegex(RuntimeError, 'stopped before retry'):
            session.act(None)
        self.assertEqual(self.client.calls, 2)
        self.assertEqual(self.responses, ['success'])
        self.assertEqual(self.audit[-1]['status'], 'session_replaced')
        self.assertEqual(self.audit[-1]['reason'], 'expired')

    def test_silent_session_change_cannot_return_replacement_actions(self):
        self.responses[:] = ['success', 'changed']
        session = self.session()
        session.act(None)
        with self.assertRaisesRegex(RuntimeError, 'stopped before executing'):
            session.act(None)
        self.assertEqual(self.client.calls, 2)
        self.assertEqual(self.audit[-1]['status'], 'error')

    def test_actual_cli_default_override_and_demo_branch(self):
        # Extract only the real parser/registration/metadata statements, leaving
        # simulator creation out. This checks the CLI-to-factory connection.
        import run
        source = Path(run.__file__)
        main = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        parsed = next(i for i, node in enumerate(main.body)
                      if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
                      and node.targets[0].id == 'args')
        setup = []
        for node in main.body[parsed + 1:]:
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                if node.targets[0].id in ('policy_name', 'dream_request_audit', 'metadata'):
                    setup.append(node)
            elif isinstance(node, ast.If) and any(
                    isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
                    and child.func.id == '_register_dream_policy' for child in ast.walk(node)):
                setup.append(node)
            elif isinstance(node, ast.If) and "metadata['dream_rows_per_request']" in ast.unparse(node):
                setup.append(node)
        tree = ast.Module(body=main.body[:parsed + 1] + setup, type_ignores=[])
        for policy, option, expected in [('dream', [], 8), ('dream', ['--dream-rows-per-request', '12'], 12),
                                         ('demo', [], None)]:
            with self.subTest(policy=policy, expected=expected):
                namespace = dict(argparse=argparse, CONFIG={'run_limit': 3000}, Path=Path,
                    __doc__='test', _register_dream_policy=self.register_policy,
                    register=self.register, platform=platform, SCORE_VERSION=run.SCORE_VERSION)
                with patch.object(sys, 'argv', ['run.py', '--policy', policy, '--log-dir', '/tmp/test-only', *option]), \
                     patch.dict(sys.modules, {'demo_policy': SimpleNamespace(register_policy=lambda mode: 'scripted') } ):
                    exec(compile(tree, str(source), 'exec'), namespace)
                if expected is None:
                    self.assertEqual(namespace['policy_name'], 'scripted')
                    self.assertNotIn('dream_rows_per_request', namespace['metadata'])
                else:
                    self.assertEqual(namespace['metadata']['dream_rows_per_request'], expected)
                    self.assertEqual(self.factories[namespace['policy_name']]().rows_per_request, expected)


if __name__ == "__main__":
    unittest.main()
