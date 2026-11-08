"""Assay state-machine tests; synthetic positive telemetry is not a knot demo.

Positive tests inject topology certificates to isolate timing, force and credit
rules. Geometry negatives use real telemetry checks. test_topology.py separately
exercises actual trefoil and non-trefoil curves.
"""
from copy import deepcopy
import json
import unittest
from unittest.mock import patch
import numpy as np
from scoring import (PROOF_PROTOCOL, PROOF_SOURCE, ROBOT_COLLISION_LINKS,
                     KnotScorer, _box_clearance, _jig_clearance, replay_samples)


def candidate(bounds=(.20, .22)):
    return {"status": "overhand_candidate", "is_overhand": True,
            "core_arclength_m": list(bounds), "reason": "test-only injected certificate"}


def straight_sample(time=0., radius=.004):
    n, seg = 64, .01
    nodes = np.column_stack([np.arange(n+1)*seg, np.zeros(n+1), np.zeros(n+1)])
    ends = np.stack([nodes[:-1], nodes[1:]], axis=1)
    boxes = [{"link": link, "center": [2., 2., 2.], "rotation": np.eye(3).tolist(),
              "half_size": [.01, .01, .01]} for link in sorted(ROBOT_COLLISION_LINKS)]
    return {"time": time,
            "hands": [dict(tcp=[2., 2., 2.], opening=1., finger_boxes=deepcopy(boxes)) for _ in range(2)],
            "cord": {"points": ((nodes[:-1]+nodes[1:])/2).tolist(),
                     "quats_xyzw": np.tile([0., 0., 0., 1.], (n, 1)).tolist(),
                     "endpoints": ends.tolist(), "polyline": nodes.tolist(),
                     "joint_gaps": [0.]*(n-1), "max_joint_gap": 0.,
                     "seg_len": seg, "radius": radius,
                     "linear_velocities": np.zeros((n, 3)).tolist(),
                     "angular_velocities": np.zeros((n, 3)).tolist()}}


def proof_sample(time, elapsed, force=.981, phase="proof"):
    sample = straight_sample(time)
    sample.update(phase=phase, proof={
        "protocol": PROOF_PROTOCOL, "source": PROOF_SOURCE, "active": True,
        "endpoint_indices": [0, 63], "endpoint_forces_n": [[-force, 0., 0.], [force, 0., 0.]],
        "applied_force_n": force, "elapsed_s": elapsed})
    return sample


def seating_sample(time, elapsed, force=None):
    # Left endpoint of each integration interval: apply, step, then read back.
    if force is None:
        force = .981*np.clip((elapsed-.05)/2, 0., 1.)
    return proof_sample(time, elapsed, force, phase="seating")


def seat(scorer, *, start=0.):
    scorer.update(straight_sample(start))
    for i in range(1, 41):
        scorer.update(seating_sample(start+i/20, i/20))


def complete_assay(scorer, *, force=.981):
    seat(scorer)
    for i in range(1, 101):
        scorer.update(proof_sample(2+i/20, i/20, force))


class ScorerTests(unittest.TestCase):
    def test_jig_cannot_support_a_qualifying_knot(self):
        sample = straight_sample()
        endpoints = np.asarray(sample['cord']['endpoints']) + [0., .02, .932]
        cord = {'endpoints': endpoints, 'radius': .004}
        self.assertIsNone(_jig_clearance(cord, {}))
        self.assertAlmostEqual(_jig_clearance(cord, {'development_jig': 'three-peg'}), 0., places=7)
        with patch('scoring.analyze_open_rope', return_value=candidate()):
            scorer = KnotScorer({'development_jig': 'three-peg'})
            for field in ('points', 'endpoints', 'polyline'):
                sample['cord'][field] = (np.asarray(sample['cord'][field])+[0., .02, .932]).tolist()
            scorer.update(sample)
            self.assertIsNone(scorer.invalid)
            self.assertTrue(scorer.released)
            self.assertFalse(scorer.jig_clear)
            self.assertEqual(scorer.finalize(), 0.)
            sample['time'] = .05
            for field in ('points', 'endpoints', 'polyline'):
                sample['cord'][field] = (np.asarray(sample['cord'][field])+[0., .15, 0.]).tolist()
            scorer.update(sample)
            self.assertTrue(scorer.jig_clear)
            self.assertEqual(scorer.finalize(), .5)

    def test_actual_straight_rope_and_pickup_get_no_credit(self):
        scorer = KnotScorer()
        scorer.update(straight_sample())
        self.assertEqual(scorer.summary(final=True)["score"], 0.)
        self.assertIsNone(scorer.invalid)
        self.assertEqual(scorer.topology["status"], "unknot")
        lifted = straight_sample(.05)
        for field in ("points", "endpoints", "polyline"):
            lifted["cord"][field] = (np.asarray(lifted["cord"][field])+[0, 0, .2]).tolist()
        scorer.update(lifted)
        self.assertEqual(scorer.finalize(), 0.)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_policy_half_needs_release_but_has_no_inferred_tail_gate(self, detector):
        scorer = KnotScorer()
        scorer.update(straight_sample())
        self.assertEqual(scorer.value, .5)
        detector.return_value = candidate((.02, .04))
        self.assertEqual(scorer.finalize(), .5)
        self.assertTrue(scorer.summary()["core"]["diagnostics_only"])
        detector.return_value = candidate()
        near = straight_sample(.05)
        near["hands"][0]["finger_boxes"][0]["center"] = [.21, .025, 0.]
        scorer.update(near)
        self.assertEqual(scorer.value, 0.)
        self.assertFalse(scorer.released)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_two_second_seating_then_five_full_seconds_earns_one(self, detector):
        scorer = KnotScorer()
        seat(scorer)
        for i in range(1, 100):
            scorer.update(proof_sample(2+i/20, i/20))
        self.assertEqual(scorer.value, .5)
        self.assertAlmostEqual(scorer.summary()["proof"]["elapsed_s"], 4.95)
        scorer.update(proof_sample(7., 5.))
        result = scorer.summary(final=True)
        self.assertIsNone(result["invalid"])
        self.assertEqual(result["score"], 1.)
        self.assertTrue(result["policy_end"]["qualified"])
        self.assertEqual(result["policy_end"]["time"], 0.)
        self.assertTrue(result["seating"]["completed"])
        self.assertEqual(result["proof"]["observed_force_range_n"], [.981, .981])
        self.assertTrue(result["terminal_validated"])
        json.dumps(result, allow_nan=False)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_zero_elapsed_onsets_also_work(self, detector):
        scorer = KnotScorer()
        scorer.update(straight_sample())
        for i in range(40):
            scorer.update(seating_sample(.05+i/20, i/20))
        for i in range(101):
            scorer.update(proof_sample(2.05+i/20, i/20))
        self.assertIsNone(scorer.invalid)
        self.assertEqual(scorer.finalize(), 1.)
        self.assertAlmostEqual(scorer.summary()["proof"]["elapsed_s"], 5.)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_onset_and_phase_timers_cannot_invent_duration(self, detector):
        for time, elapsed, reason in ((.05, .10, "onset"), (.05, .025, "onset"),
                                      (.05, -.05, "negative"), (.10, .10, "missing capture")):
            with self.subTest(time=time, elapsed=elapsed):
                scorer = KnotScorer()
                scorer.update(straight_sample())
                scorer.update(seating_sample(time, elapsed))
                self.assertEqual(scorer.value, 0.)
                self.assertIn(reason, scorer.invalid)
        scorer = KnotScorer()
        seat(scorer)
        scorer.update(proof_sample(2.05, 2.05))  # Must reset phase-local timer.
        self.assertIn("onset", scorer.invalid)
        scorer = KnotScorer()
        seat(scorer)
        scorer.update(proof_sample(2.05, .05))
        scorer.update(proof_sample(2.10, 5.))
        self.assertIn("timer contradicts", scorer.invalid)
        scorer.update(straight_sample(2.15))
        self.assertEqual(scorer.value, 0.)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_short_seating_or_wrong_ramp_cannot_earn_full(self, detector):
        for kind in ("short", "step", "flat", "decreasing"):
            scorer = KnotScorer()
            scorer.update(straight_sample())
            length = 20 if kind == "short" else 40
            for i in range(1, length+1):
                force = None
                if kind == "step": force = .981
                if kind == "flat": force = 0.
                # Remains within the ramp envelope but reverses progression.
                if kind == "decreasing" and i == 20: force = .44135
                scorer.update(seating_sample(i/20, i/20, force))
            for i in range(1, 101):
                scorer.update(proof_sample(length/20+i/20, i/20))
            self.assertIsNone(scorer.invalid, kind)
            self.assertEqual(scorer.finalize(), .5, kind)
            self.assertFalse(scorer.summary()["seating"]["completed"], kind)
            self.assertFalse(scorer.summary()["proof"]["completed"], kind)
            self.assertIsNotNone(scorer.summary()["proof"]["failure"], kind)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_short_proof_or_bad_held_force_preserves_half(self, detector):
        scorer = KnotScorer()
        seat(scorer)
        for i in range(1, 21):
            scorer.update(proof_sample(2+i/20, i/20))
        self.assertEqual(scorer.finalize(), .5)
        self.assertFalse(scorer.summary()["proof"]["completed"])
        scorer = KnotScorer()
        complete_assay(scorer, force=.70)
        self.assertEqual(scorer.finalize(), .5)
        self.assertIn("outside tolerance", scorer.summary()["proof"]["failure"])

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_full_load_five_percent_tolerance_boundary(self, detector):
        for ratio, expected in ((.95, 1.), (1.05, 1.), (.949, .5), (1.051, .5)):
            with self.subTest(ratio=ratio):
                scorer = KnotScorer()
                complete_assay(scorer, force=.981*ratio)
                self.assertIsNone(scorer.invalid)
                self.assertEqual(scorer.finalize(), expected)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_retention_failure_preserves_pretest_half_even_if_knot_recovers(self, detector):
        for failure_phase in ("seating", "proof", "after_proof"):
            with self.subTest(phase=failure_phase):
                scorer = KnotScorer()
                scorer.update(straight_sample())
                for i in range(1, 41):
                    detector.return_value = {"status": "unknot", "is_overhand": False} if failure_phase == "seating" and i == 20 else candidate()
                    scorer.update(seating_sample(i/20, i/20))
                for i in range(1, 101):
                    detector.return_value = {"status": "unknot", "is_overhand": False} if failure_phase == "proof" and i == 20 else candidate()
                    scorer.update(proof_sample(2+i/20, i/20))
                if failure_phase == "after_proof":
                    self.assertEqual(scorer.value, 1.)
                    detector.return_value = {"status": "unknot", "is_overhand": False}
                    scorer.update(proof_sample(7.05, 5.05))
                self.assertIsNone(scorer.invalid)
                self.assertEqual(scorer.finalize(), .5)
                self.assertTrue(scorer.summary()["policy_end"]["qualified"])
                self.assertFalse(scorer.summary()["proof"]["completed"])

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_inferred_tail_changes_during_assay_do_not_change_credit(self, detector):
        scorer = KnotScorer()
        seat(scorer)
        for i in range(1, 101):
            detector.return_value = candidate((.02, .22)) if i == 20 else candidate()
            scorer.update(proof_sample(2+i/20, i/20))
        self.assertEqual(scorer.finalize(), 1.)
        self.assertIsNone(scorer.summary()["proof"]["failure"])

    @patch("scoring.analyze_open_rope")
    def test_assessor_cannot_create_policy_credit(self, detector):
        unknot = {"status": "unknot", "is_overhand": False}
        detector.side_effect = [unknot, unknot] + [candidate()]*142
        scorer = KnotScorer()
        complete_assay(scorer)
        result = scorer.summary(final=True)
        self.assertIsNone(result["invalid"])
        self.assertEqual(result["score"], 0.)
        self.assertFalse(result["policy_end"]["qualified"])
        self.assertEqual(result["topology_status"], "overhand_candidate")

    @patch("scoring.analyze_open_rope", return_value=candidate((.10, .50)))
    def test_core_span_and_crossing_envelope_motion_are_diagnostics_only(self, detector):
        scorer = KnotScorer({"compact_span_max_m": .001, "slip_max_m": .00001})
        seat(scorer)
        detector.return_value = candidate((.30, .55))
        for i in range(1, 101):
            scorer.update(proof_sample(2+i/20, i/20))
        result = scorer.summary(final=True)
        self.assertEqual(result["score"], 1.)
        self.assertGreater(result["core"]["core_span_m"], .045)
        self.assertGreater(result["proof"]["maximum_boundary_slip_m"], .01)
        self.assertTrue(result["proof"]["slip_and_core_span_are_diagnostics_only"])

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_bad_physics_erases_even_frozen_half(self, detector):
        scorer = KnotScorer()
        seat(scorer)
        sample = proof_sample(2.05, .05)
        sample["cord"]["points"][20][0] += .004
        scorer.update(sample)
        self.assertTrue(scorer.summary()["policy_end"]["qualified"])
        self.assertEqual(scorer.finalize(), 0.)
        self.assertIn("endpoints contradict", scorer.invalid)
        # A geometry failure reported by the independent topology audit also
        # erases partial credit rather than being treated as ordinary knot loss.
        scorer = KnotScorer()
        seat(scorer)
        detector.return_value = {"status": "invalid", "is_overhand": None, "reason": "nonlocal_capsule_penetration"}
        scorer.update(proof_sample(2.05, .05))
        self.assertEqual(scorer.finalize(), 0.)
        self.assertIn("penetration", scorer.invalid)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_requested_only_or_inconsistent_force_evidence_is_invalid(self, detector):
        for mutation in ("source", "scalar", "endpoints"):
            scorer = KnotScorer()
            seat(scorer)
            sample = proof_sample(2.05, .05)
            if mutation == "source": sample["proof"]["source"] = "requested"
            elif mutation == "scalar": sample["proof"]["endpoint_forces_n"][0] = [0., 0., 0.]
            else: sample["proof"]["endpoint_indices"] = [2, 61]
            scorer.update(sample)
            self.assertIsNotNone(scorer.invalid)
            self.assertEqual(scorer.value, 0.)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_nonopposing_or_inward_load_does_not_pass(self, detector):
        for forces in ([[.981, 0, 0], [.981, 0, 0]], [[.981, 0, 0], [-.981, 0, 0]]):
            scorer = KnotScorer()
            seat(scorer)
            sample = proof_sample(2.05, .05)
            sample["proof"]["endpoint_forces_n"] = forces
            scorer.update(sample)
            self.assertIsNone(scorer.invalid)
            self.assertEqual(scorer.value, .5)
            self.assertIn("not opposed", scorer.summary()["proof"]["failure"])

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_phase_order_and_no_hidden_assessor_actions(self, detector):
        for samples in ([proof_sample(0., 0.)],
                        [straight_sample(), proof_sample(.05, .05)],
                        [seating_sample(0., 0.)]):
            result = replay_samples(samples)
            self.assertEqual(result["score"], 0.)
            self.assertIsNotNone(result["invalid"])
        scorer = KnotScorer()
        seat(scorer)
        scorer.update(straight_sample(2.05))
        self.assertIn("cannot resume", scorer.invalid)
        scorer = KnotScorer()
        sample = seating_sample(0., 0.)
        sample["phase"] = "policy"
        scorer.update(sample)
        self.assertIn("hidden in the policy phase", scorer.invalid)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_terminal_policy_refresh_catches_between_interval_failure(self, detector):
        scorer = KnotScorer()
        scorer.update(straight_sample())
        detector.return_value = {"status": "ambiguous", "is_overhand": None}
        scorer.update(straight_sample(.05))
        self.assertEqual(scorer.value, .5)
        self.assertFalse(scorer.summary()["terminal_validated"])
        self.assertEqual(scorer.summary(final=True)["score"], 0.)

    @patch("scoring.analyze_open_rope", return_value=candidate())
    def test_diagnostic_flag_permanently_disables_task_credit(self, detector):
        for via_config in (False, True):
            scorer = KnotScorer({"diagnostic_only": True} if via_config else {})
            initial = straight_sample()
            initial["diagnostic_only"] = not via_config
            scorer.update(initial)
            for i in range(1, 41):
                sample = seating_sample(i/20, i/20)
                sample["diagnostic_only"] = False
                scorer.update(sample)
                self.assertEqual(scorer.value, 0.)
            for i in range(1, 101):
                scorer.update(proof_sample(2+i/20, i/20))
                self.assertEqual(scorer.value, 0.)
            result = scorer.summary(final=True)
            self.assertEqual(result["status"], "diagnostic_only")
            self.assertEqual(result["candidate_score"], 1.)

    def test_fabricated_gaps_cannot_hide_disconnected_links(self):
        sample = straight_sample()
        sample["cord"]["points"][20][0] += .004
        scorer = KnotScorer()
        scorer.update(sample)
        self.assertEqual(scorer.value, 0.)
        self.assertIn("endpoints contradict", scorer.invalid)
        sample = straight_sample()
        # Consistent endpoints but forged zero joint-gap summaries.
        sample["cord"]["points"][20][0] += .004
        sample["cord"]["endpoints"][20][0][0] += .004
        sample["cord"]["endpoints"][20][1][0] += .004
        scorer = KnotScorer()
        scorer.update(sample)
        self.assertIn("joint gaps contradict", scorer.invalid)

    def test_measured_joint_gap_over_two_mm_is_invalid(self):
        sample = straight_sample()
        sample["cord"]["points"][20][1] += .003
        for end in (0, 1):
            sample["cord"]["endpoints"][20][end][1] += .003
        sample["cord"]["joint_gaps"][19:21] = [.003, .003]
        sample["cord"]["max_joint_gap"] = .003
        scorer = KnotScorer()
        scorer.update(sample)
        self.assertIn("joint gap exceeds", scorer.invalid)

    def test_missing_robot_envelope_and_nonfinite_telemetry_fail_closed(self):
        for kind in ("robot", "nan", "quaternion"):
            sample = straight_sample()
            if kind == "robot":
                sample["hands"][0]["finger_boxes"].pop()
            elif kind == "nan":
                sample["cord"]["linear_velocities"][0][0] = float("nan")
            else:
                sample["cord"]["quats_xyzw"][0] = [0., 0., 0., 2.]
            scorer = KnotScorer()
            scorer.update(sample)
            self.assertIsNotNone(scorer.invalid)
            self.assertEqual(scorer.value, 0.)
            json.dumps(scorer.summary(final=True), allow_nan=False)

    def test_segment_clearance_checks_between_endpoints(self):
        ends = np.array([[[-1., 0., 0.], [1., 0., 0.]]])
        box = {"center": [0., 0., 0.], "rotation": np.eye(3), "half_size": [.01, .01, .01]}
        self.assertEqual(_box_clearance(ends, .004, box), 0.)
        box["center"] = [0., .05, 0.]
        self.assertAlmostEqual(_box_clearance(ends, .004, box), .036)

    def test_recording_must_start_at_zero_and_time_increase(self):
        for times in ((.05,), (0., 0.), (0., -.05)):
            scorer = KnotScorer()
            for time in times:
                scorer.update(straight_sample(time))
            self.assertIsNotNone(scorer.invalid)

    def test_replay_ignores_recorded_score_and_rejects_empty_recording(self):
        samples = [straight_sample(i/20) for i in range(6)]
        for sample in samples:
            sample["metrics"] = {"score": 1., "terminal_validated": True}
        result = replay_samples(samples)
        self.assertEqual(result["score"], 0.)
        self.assertTrue(result["terminal_validated"])
        self.assertIsNone(result["invalid"])
        self.assertIsNotNone(replay_samples([])["invalid"])


if __name__ == "__main__":
    unittest.main()
