"""Certification tests use synthetic telemetry, not a physical knot demonstration."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from replay import certify_result, replay_trial
from scoring import replay_samples
from test_runner import FakeTransition
from test_scoring import candidate, proof_sample, seating_sample, straight_sample


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.config = {'length': .64, 'mass': .025, 'radius': .004, 'segment': .01,
                       'physics_hz': 240, 'assay_physics_hz': 960, 'proof_seconds': 5.}
        self.samples = [straight_sample()]
        self.samples += [seating_sample(i/20, i/20) for i in range(1, 41)]
        self.samples += [proof_sample(2+i/20, i/20) for i in range(1, 101)]
        topology = patch('scoring.analyze_open_rope', return_value=candidate())
        topology.start()
        self.addCleanup(topology.stop)
        self.result = replay_samples(self.samples, self.config)
        self.assertEqual(self.result['score'], 1.)
        switch = FakeTransition(None, 960, 0.)
        switch.start()
        for i in range(140):
            switch.before_step(i/20, i)
            switch.after_step((i+1)/20, i+1)
        switch.close()
        self.certificate = switch.report

    def certify(self, certificate=None, complete=True):
        return certify_result(self.result, self.config, certificate,
                              finalization_complete=complete)

    def test_complete_native_certificate_preserves_full_score(self):
        result = self.certify(self.certificate)
        self.assertEqual(result['score'], 1.)
        self.assertEqual(result['geometric_score'], 1.)
        self.assertEqual(result['assay_certification']['status'], 'complete')

    def test_missing_failed_or_incomplete_certificate_cannot_award_full_credit(self):
        mutations = [
            lambda c: c.update(status='failed'),
            lambda c: c.update(callback_removed=False),
            lambda c: c.update(validated_control_intervals=139),
            lambda c: c['intervals'].pop(),
            lambda c: c['intervals'][-1].update(validated=False),
            lambda c: c['intervals'][-1].update(native_callback_dt_s=[1/240]*48),
            lambda c: c['intervals'][-1].update(capture_time_s=7.05),
            lambda c: c['intervals'][-1]['cameras']['review_cam'].update(frame_delta=[0]),
            lambda c: c.update(state_sha256_after='changed-state'),
        ]
        for mutation in mutations:
            certificate = deepcopy(self.certificate)
            mutation(certificate)
            with self.subTest(certificate=mutation):
                result = self.certify(certificate)
                self.assertEqual(result['score'], .5)
                self.assertEqual(result['geometric_score'], 1.)
                self.assertEqual(result['assay_certification']['status'], 'incomplete')
        self.assertEqual(self.certify()['score'], .5)
        self.assertEqual(self.certify({'intervals': 'malformed'})['score'], .5)

    def test_complete_native_steps_do_not_override_failed_finalization(self):
        result = self.certify(self.certificate, complete=False)
        self.assertEqual(result['score'], .5)
        self.assertEqual(result['geometric_score'], 1.)
        self.assertIn('trial finalization is not certified complete', result['assay_certification']['errors'])

    def test_policy_only_zero_or_half_needs_no_transition(self):
        result = replay_samples([straight_sample()], self.config)
        for score in (0., .5):
            result['score'] = score
            certified = certify_result(result, self.config)
            self.assertEqual(certified['score'], score)
            self.assertEqual(certified['assay_certification']['status'], 'not_required')

    def test_legacy_diagnostic_remains_zero_with_visible_candidate(self):
        result = replay_samples(self.samples, dict(self.config, diagnostic_only=True))
        certified = certify_result(result, self.config)
        self.assertEqual(certified['score'], 0.)
        self.assertEqual(certified['candidate_score'], 1.)
        self.assertEqual(certified['assay_certification']['status'], 'diagnostic_only')

    def test_replay_uses_saved_certificates_but_never_trusts_saved_scores(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            trajectory = folder / 'trajectory.jsonl'
            trajectory.write_text(''.join(json.dumps(s)+'\n' for s in self.samples))
            runtime = {'task_config': self.config, 'finalization_complete': True, 'audited_final_score': 0.}
            (folder/'runtime.json').write_text(json.dumps(runtime))
            # A geometry-only archive cannot silently earn authoritative 1.0.
            self.assertEqual(replay_trial(trajectory)['score'], .5)
            (folder/'assay-physics-transition.json').write_text(json.dumps(self.certificate))
            summary = {'score': 0., 'finalization': {'complete': True}}
            (folder/'score-summary.json').write_text(json.dumps(summary))
            self.assertEqual(replay_trial(trajectory)['score'], 1.)
            runtime['finalization_complete'] = False
            (folder/'runtime.json').write_text(json.dumps(runtime))
            self.assertEqual(replay_trial(trajectory)['score'], .5)
            runtime['finalization_complete'] = True
            (folder/'runtime.json').write_text(json.dumps(runtime))
            # Reproduce the old runner's misleading score1 plus incomplete close.
            summary.update(score=1., finalization={'complete': False})
            (folder/'score-summary.json').write_text(json.dumps(summary))
            result = replay_trial(trajectory)
            self.assertEqual(result['score'], .5)
            self.assertEqual(result['geometric_score'], 1.)
            # Diagnostic replay is explicit, needs no certificate, and stays0.
            runtime['task_config']['diagnostic_only'] = True
            (folder/'runtime.json').write_text(json.dumps(runtime))
            (folder/'assay-physics-transition.json').unlink()
            result = replay_trial(trajectory)
            self.assertEqual(result['score'], 0.)
            self.assertEqual(result['assay_certification']['status'], 'diagnostic_only')


if __name__ == '__main__':
    unittest.main()
