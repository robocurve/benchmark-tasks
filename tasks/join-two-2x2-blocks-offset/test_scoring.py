"""Adversarial geometry and state-transition checks; no simulator required."""

import copy
import math
import unittest

import numpy as np

from scoring import (BrickScorer, TABLE, body_box, brick_envelope,
                     geometric_connection, minimum_clearance, obb_clearance,
                     quaternion_matrix, rotation_distance, score_samples)


FULL = {"lower_shape": (2, 2), "upper_shape": (2, 2), "required_studs": 4}
SINGLE = {"lower_shape": (1, 1), "upper_shape": (1, 1), "required_studs": 1}
MIXED = {"lower_shape": (2, 2), "upper_shape": (1, 1), "required_studs": 1}
OFFSET = {"lower_shape": (2, 2), "upper_shape": (2, 2), "required_studs": 2}
IDENTITY = [0., 0., 0., 1.]


def quat(axis, degrees):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return [*(axis*math.sin(math.radians(degrees)/2)), math.cos(math.radians(degrees)/2)]


def pose(pos, rotation=IDENTITY):
    return {"pos": list(pos), "quat_xyzw": list(rotation)}


def box(center, half_size=(.003, .004, .003), rotation=None, link=None):
    return {"center": list(center), "half_size": list(half_size),
            "rotation": (np.eye(3) if rotation is None else rotation).tolist(), "link": link}


def hands_for(upper, shape, held=True):
    p = np.array(upper["pos"])
    if not held:
        p = np.array([-.05, -.4, TABLE+.3])
    halfwidth = (shape[0]*.016-.0002)/2
    first = {"tcp": p.tolist(), "opening": .2 if held else 1., "finger_boxes": [
        box(p+np.array([halfwidth+.003, 0., .003])),
        box(p+np.array([-halfwidth-.003, 0., .003])),
        box(p+np.array([0., 0., .08])),
    ]}
    far = np.array([.7, -.4, TABLE+.3])
    second = {"tcp": far.tolist(), "opening": 1., "finger_boxes": [
        box(far+np.array([.03, 0, 0])), box(far+np.array([-.03, 0, 0])), box(far+np.array([0, 0, .08]))]}
    for hand in (first, second):
        for envelope, link in zip(hand["finger_boxes"], ("tip_left", "tip_right", "gripper")):
            envelope["link"] = link
        for i, link in enumerate(('base', 'link1', 'link2', 'link3', 'link4', 'link5')):
            hand['finger_boxes'].append(box(np.array(hand['tcp'])+[0., 0., .12+.04*i], link=link))
    return [first, second]


def sample(t, cfg=FULL, kind="pickup", offset=None):
    if offset is None:
        offset = [.008, .008] if cfg == MIXED else ([.016, 0.] if cfg == OFFSET else [0., 0.])
    lower = pose([.36, .16, TABLE+.0096+(0.03 if kind == "proof" else 0.)])
    upper = pose([.2, .16, TABLE+.0096+.04]) if kind == "pickup" else pose([
        lower["pos"][0]+offset[0], lower["pos"][1]+offset[1], lower["pos"][2]+.0192])
    return {"time": t, "upper": upper, "lower": lower,
            "hands": hands_for(upper, cfg["upper_shape"], held=kind != "rest")}


def feed(scorer, kind, frames, start, cfg=FULL, mutate=None):
    for step in range(frames):
        state = sample(round(start+step*.05, 8), cfg, kind)
        if mutate:
            mutate(state)
        scorer.update(state)
    return round(start+frames*.05, 8)


def through_proof(cfg=FULL):
    scorer = BrickScorer(cfg)
    t = feed(scorer, "pickup", 5, 0., cfg)
    t = feed(scorer, "seat", 5, t, cfg)
    t = feed(scorer, "proof", 21, t, cfg)
    return scorer, t


class GeometryTests(unittest.TestCase):
    def test_identity_and_quaternion_sign_equivalent(self):
        q = quat([1, 2, 3], 71)
        np.testing.assert_allclose(quaternion_matrix(q), quaternion_matrix([-v for v in q]))
        np.testing.assert_allclose(quaternion_matrix(IDENTITY), np.eye(3))

    def test_all_four_square_yaw_symmetries(self):
        lower = pose([.3, .1, .8])
        for angle in (0, 90, 180, 270, -90):
            upper = pose([.3, .1, .8192], quat([0, 0, 1], angle))
            result = geometric_connection(upper, lower, FULL)
            self.assertTrue(result["valid"], angle)
            self.assertEqual(result["engaged_studs"], 4)

    def test_single_socket_accepts_arbitrary_yaw(self):
        lower = pose([.3, .1, .8])
        for angle in (17, 45, 123, 281):
            upper = pose([.3, .1, .8192], quat([0, 0, 1], angle))
            self.assertTrue(geometric_connection(upper, lower, SINGLE)["valid"])

    def test_mixed_accepts_every_lower_stud_and_arbitrary_yaw(self):
        lower = pose([.3, .1, .8])
        for x in (-.008, .008):
            for y in (-.008, .008):
                upper = pose([.3+x, .1+y, .8192], quat([0, 0, 1], 31))
                self.assertTrue(geometric_connection(upper, lower, MIXED)["valid"])
        self.assertFalse(geometric_connection(pose([.3, .1, .8192]), lower, MIXED)["valid"])

    def test_offset_accepts_every_edge_but_not_corner_or_full(self):
        lower = pose([.3, .1, .8])
        for x, y in ((.016, 0), (-.016, 0), (0, .016), (0, -.016)):
            self.assertTrue(geometric_connection(pose([.3+x, .1+y, .8192]), lower, OFFSET)["valid"])
        for x, y in ((0., 0.), (.016, .016)):
            self.assertFalse(geometric_connection(pose([.3+x, .1+y, .8192]), lower, OFFSET)["valid"])

    def test_offset_does_not_satisfy_full_task(self):
        lower = pose([.3, .1, .8])
        self.assertFalse(geometric_connection(pose([.316, .1, .8192]), lower, FULL)["valid"])

    def test_lateral_seat_and_tilt_limits(self):
        lower = pose([.3, .1, .8])
        self.assertTrue(geometric_connection(pose([.301, .1, .81995]), lower, SINGLE)["valid"])
        self.assertFalse(geometric_connection(pose([.30101, .1, .8192]), lower, SINGLE)["valid"])
        self.assertFalse(geometric_connection(pose([.3, .1, .81996]), lower, SINGLE)["valid"])
        self.assertFalse(geometric_connection(pose([.3, .1, .8192], quat([1, 0, 0], 3.1)), lower, SINGLE)["valid"])

    def test_loose_stack_on_top_of_studs_is_not_seated(self):
        lower = pose([.3, .1, .8])
        self.assertFalse(geometric_connection(pose([.3, .1, .824]), lower, FULL)["valid"])

    def test_shared_rotation_does_not_change_connection(self):
        q = quat([1, 2, 1], 39)
        r = quaternion_matrix(q)
        lower = pose([.3, .1, .8], q)
        upper = pose(np.array(lower["pos"])+r@np.array([.016, 0., .0192]), q)
        self.assertTrue(geometric_connection(upper, lower, OFFSET)["valid"])

    def test_upside_down_upper_rejected(self):
        self.assertFalse(geometric_connection(pose([.3, .1, .8192], quat([1, 0, 0], 180)), pose([.3, .1, .8]), FULL)["valid"])

    def test_box_distance_overlap_touch_face_and_diagonal(self):
        a = box([0., 0., 0.], [.5, .5, .5])
        self.assertEqual(obb_clearance(a, box([.1, 0., 0.], [.5]*3)), 0.)
        self.assertEqual(obb_clearance(a, box([1., 0., 0.], [.5]*3)), 0.)
        self.assertAlmostEqual(obb_clearance(a, box([1.2, 0., 0.], [.5]*3)), .2)
        self.assertAlmostEqual(obb_clearance(a, box([1.3, 1.4, 0.], [.5]*3)), .5)

    def test_box_distance_rotated_face(self):
        a = box([0, 0, 0], [.5]*3)
        b = box([2, 0, 0], [.5]*3, quaternion_matrix(quat([0, 0, 1], 45)))
        self.assertAlmostEqual(obb_clearance(a, b), 1.5-math.sqrt(.5), places=10)
        self.assertAlmostEqual(obb_clearance(a, b), obb_clearance(b, a), places=10)

    def test_rotated_edge_distance_analytic(self):
        # One thin rod along x and the other along y, separated in z.
        a = box([0, 0, 0], [1, .01, .01])
        b = box([0, 0, .13], [1, .01, .01], quaternion_matrix(quat([0, 0, 1], 90)))
        self.assertAlmostEqual(obb_clearance(a, b), .11, places=10)

    def test_stud_envelope_catches_contact_missed_by_body(self):
        lower = pose([.3, .1, .8])
        finger = box([.308, .108, .813], [.002]*3)
        self.assertGreater(obb_clearance(finger, body_box(lower, (2, 2), FULL)), .001)
        self.assertEqual(minimum_clearance([finger], brick_envelope(lower, (2, 2), FULL)), 0.)

    def test_missing_boxes_fail_closed(self):
        self.assertEqual(minimum_clearance([], [box([0, 0, 0])]), 0.)

    def test_bounding_box_pruning_preserves_exact_minimum(self):
        rng = np.random.default_rng(72)
        for _ in range(6):
            groups = [[box(rng.uniform(-.1, .1, 3), rng.uniform(.001, .02, 3),
                           quaternion_matrix(rng.normal(size=4))) for _ in range(5)] for _ in range(2)]
            exact = min(obb_clearance(a, b) for a in groups[0] for b in groups[1])
            self.assertAlmostEqual(minimum_clearance(*groups), exact, places=12)

    def test_rotation_distance_includes_yaw(self):
        self.assertAlmostEqual(math.degrees(rotation_distance(np.eye(3), quaternion_matrix(quat([0, 0, 1], 10)))), 10.)


class StateTests(unittest.TestCase):
    def test_all_four_complete_sequences(self):
        for cfg in (FULL, SINGLE, MIXED, OFFSET):
            scorer, t = through_proof(cfg)
            self.assertEqual(scorer.score, .5)
            self.assertIsNotNone(scorer.proof_time)
            feed(scorer, "rest", 21, t, cfg)
            self.assertEqual(scorer.score, 1., scorer.result())

    def test_pickup_requires_real_duration(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "pickup", 4, 0.)
        self.assertEqual(scorer.score, 0.)
        scorer.update(sample(.2))
        self.assertEqual(scorer.score, .25)

    def test_pickup_requires_clearance_and_two_near_finger_pads(self):
        for mutation in (lambda s: s["upper"]["pos"].__setitem__(2, TABLE+.0096+.019),
                         lambda s: s["hands"][0]["finger_boxes"][0]["center"].__setitem__(0, .7),
                         lambda s: s["hands"][0].__setitem__("tcp", [0., 0., 1.])):
            scorer = BrickScorer(FULL)
            feed(scorer, "pickup", 6, 0., mutate=mutation)
            self.assertEqual(scorer.score, 0.)

    def test_either_hand_can_pick_up(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "pickup", 5, 0., mutate=lambda s: s["hands"].reverse())
        self.assertEqual(scorer.score, .25)

    def test_grasp_uses_measured_geometry_not_normalized_jaw_cutoff(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "pickup", 5, 0., mutate=lambda s: s["hands"][0].__setitem__("opening", .8))
        self.assertEqual(scorer.score, .25)

    def test_finger_order_does_not_change_grasp(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "pickup", 5, 0., mutate=lambda s: s["hands"][0]["finger_boxes"].reverse())
        self.assertEqual(scorer.score, .25)

    def test_two_boxes_without_tip_identity_do_not_prove_grasp(self):
        scorer = BrickScorer(FULL)
        def anonymize(s):
            for envelope in s["hands"][0]["finger_boxes"]:
                envelope.pop("link")
        feed(scorer, "pickup", 5, 0., mutate=anonymize)
        self.assertEqual(scorer.score, 0.)

    def test_one_frame_pickup_drop_resets_window(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "pickup", 4, 0.)
        bad = sample(.2)
        bad["hands"][0]["finger_boxes"][0]["center"][0] = .7
        scorer.update(bad)
        feed(scorer, "pickup", 4, .25)
        self.assertEqual(scorer.score, 0.)

    def test_initial_stack_without_pickup_cannot_score(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "rest", 25, 0.)
        self.assertEqual(scorer.score, 0.)

    def test_loose_seated_stack_can_only_earn_half(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        feed(scorer, "rest", 30, t)
        self.assertEqual(scorer.score, .5)
        self.assertIsNone(scorer.proof_time)

    def test_lift_falling_lower_cannot_complete_proof(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        t = feed(scorer, "proof", 10, t)
        bad = sample(t, kind="proof")
        bad["lower"]["pos"][2] -= .002
        scorer.update(bad)
        self.assertEqual(scorer.score, .25)
        self.assertIsNone(scorer.proof_time)

    def test_wrong_stud_count_and_stale_connector_cannot_score_half(self):
        scorer = BrickScorer(OFFSET)
        t = feed(scorer, "pickup", 5, 0., OFFSET)
        for i in range(10):
            state = sample(t+i*.05, OFFSET, "seat", offset=[0., 0.])
            state["connector"] = {"attached": True, "engaged_studs": 2, "broke": False}
            scorer.update(state)
        self.assertEqual(scorer.score, .25)

    def test_both_bricks_gripped_blocks_retention_proof(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        def support(s):
            lower = np.array(s["lower"]["pos"])
            s["hands"][1]["finger_boxes"][0] = box(lower, [.003]*3, link="tip_left")
        feed(scorer, "proof", 30, t, mutate=support)
        self.assertEqual(scorer.score, .5)
        self.assertIsNone(scorer.proof_time)

    def test_finger_contact_with_stud_blocks_proof_even_with_remote_tcp(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        def support(s):
            p = np.array(s["lower"]["pos"])+[.008, .008, .012]
            s["hands"][1]["finger_boxes"][0] = box(p, [.002]*3, link="tip_left")
        feed(scorer, "proof", 25, t, mutate=support)
        self.assertIsNone(scorer.proof_time)

    def test_forearm_support_blocks_proof_with_fingers_clear(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        def support(s):
            s["hands"][1]["finger_boxes"].append(box(s["lower"]["pos"], [.003]*3, link="forearm"))
        feed(scorer, "proof", 25, t, mutate=support)
        self.assertIsNone(scorer.proof_time)

    def test_retention_needs_two_millimetres_robot_clearance(self):
        for gap in (.0019, .0021):
            scorer = BrickScorer(FULL)
            t = feed(scorer, "pickup", 5, 0.)
            t = feed(scorer, "seat", 5, t)
            def near_lower(s):
                p = np.array(s["lower"]["pos"])+[.0159+.002+gap, 0., 0.]
                s["hands"][1]["finger_boxes"].append(box(p, [.002]*3, link="forearm"))
            feed(scorer, "proof", 25, t, mutate=near_lower)
            self.assertEqual(scorer.proof_time is not None, gap > .002)

    def test_missing_one_hand_boxes_blocks_proof(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        feed(scorer, "proof", 25, t, mutate=lambda s: s["hands"][1].pop("finger_boxes"))
        self.assertIsNone(scorer.proof_time)

    def test_missing_forearm_envelope_blocks_proof_despite_complete_grippers(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        def omit_forearm(s):
            s['hands'][1]['finger_boxes'] = [b for b in s['hands'][1]['finger_boxes'] if b['link'] != 'link3']
        feed(scorer, "proof", 25, t, mutate=omit_forearm)
        self.assertIsNone(scorer.proof_time)
        self.assertFalse(scorer.result()['finger_evidence_complete'])

    def test_unattributed_top_level_boxes_cannot_prove_a_grasp(self):
        for covered in (False, True):
            scorer = BrickScorer(FULL)
            t = feed(scorer, "pickup", 5, 0.)
            t = feed(scorer, "seat", 5, t)
            def merge(s):
                s["finger_boxes"] = [b for hand in s["hands"] for b in hand.pop("finger_boxes")]
                s["finger_boxes_cover_both_hands"] = covered
            feed(scorer, "proof", 25, t, mutate=merge)
            self.assertIsNone(scorer.proof_time)

    def test_one_frame_support_resets_proof_timer(self):
        scorer = BrickScorer(FULL)
        t = feed(scorer, "pickup", 5, 0.)
        t = feed(scorer, "seat", 5, t)
        t = feed(scorer, "proof", 15, t)
        state = sample(t, kind="proof")
        state["hands"][1]["finger_boxes"][0] = box(state["lower"]["pos"], link="tip_left")
        scorer.update(state)
        feed(scorer, "proof", 15, t+.05)
        self.assertIsNone(scorer.proof_time)

    def test_proof_without_release_is_half(self):
        scorer, t = through_proof()
        feed(scorer, "seat", 30, t)
        self.assertEqual(scorer.score, .5)

    def test_rest_requires_one_second(self):
        scorer, t = through_proof()
        feed(scorer, "rest", 20, t)
        self.assertEqual(scorer.score, .5)
        scorer.update(sample(t+1., kind="rest"))
        self.assertEqual(scorer.score, 1.)

    def test_rest_requires_five_millimetres_robot_clearance(self):
        for gap in (.0049, .0051):
            scorer, t = through_proof()
            def near_lower(s):
                p = np.array(s["lower"]["pos"])+[.0159+.002+gap, 0., 0.]
                s["hands"][1]["finger_boxes"].append(box(p, [.002]*3, link="forearm"))
            feed(scorer, "rest", 25, t, mutate=near_lower)
            self.assertEqual(scorer.score, 1. if gap > .005 else .5)

    def test_rest_translation_accumulates_against_fixed_anchor(self):
        scorer, t = through_proof()
        for i in range(25):
            state = sample(t+i*.05, kind="rest")
            state["upper"]["pos"][0] += i*.0002
            state["lower"]["pos"][0] += i*.0002
            scorer.update(state)
        self.assertEqual(scorer.score, .5)

    def test_rest_yaw_motion_resets_even_with_same_shape(self):
        scorer, t = through_proof()
        for i in range(25):
            state = sample(t+i*.05, kind="rest")
            state["upper"]["quat_xyzw"] = quat([0, 0, 1], i*.5)
            state["lower"]["quat_xyzw"] = quat([0, 0, 1], i*.5)
            scorer.update(state)
        self.assertEqual(scorer.score, .5)

    def test_full_score_is_not_historic_maximum(self):
        scorer, t = through_proof()
        t = feed(scorer, "rest", 21, t)
        self.assertEqual(scorer.score, 1.)
        state = sample(t, kind="rest")
        state["upper"]["pos"][2] += .01
        scorer.update(state)
        self.assertEqual(scorer.score, .25)
        self.assertIsNone(scorer.proof_time)

    def test_hand_returns_after_full_score_erases_full(self):
        scorer, t = through_proof()
        t = feed(scorer, "rest", 21, t)
        scorer.update(sample(t, kind="seat"))
        self.assertEqual(scorer.score, .5)

    def test_disconnect_after_proof_requires_new_proof(self):
        scorer, t = through_proof()
        state = sample(t, kind="seat")
        state["upper"]["pos"][0] += .01
        scorer.update(state)
        feed(scorer, "rest", 30, t+.05)
        self.assertEqual(scorer.score, .5)
        self.assertIsNone(scorer.proof_time)

    def test_regrasp_can_retry_after_failed_connection(self):
        scorer, t = through_proof()
        scorer.update(sample(t, kind="pickup"))
        t = feed(scorer, "seat", 5, t+.05)
        t = feed(scorer, "proof", 21, t)
        feed(scorer, "rest", 21, t)
        self.assertEqual(scorer.score, 1.)

    def test_timestamp_gap_resets_incomplete_pickup(self):
        scorer = BrickScorer(FULL)
        feed(scorer, "pickup", 4, 0.)
        scorer.update(sample(.25))
        self.assertEqual(scorer.score, 0.)

    def test_gap_erases_proof(self):
        scorer, t = through_proof()
        feed(scorer, "rest", 30, t+.2)
        self.assertEqual(scorer.score, .5)
        self.assertIsNone(scorer.proof_time)

    def test_repeated_timestamp_invalidates(self):
        scorer = BrickScorer(FULL)
        for _ in range(10):
            scorer.update(sample(0.))
        self.assertEqual(scorer.score, 0.)
        self.assertIn("strictly increase", scorer.invalid_reason)

    def test_backwards_timestamp_invalidates(self):
        scorer = BrickScorer(FULL)
        scorer.update(sample(.1))
        scorer.update(sample(.05))
        self.assertIsNotNone(scorer.invalid_reason)

    def test_bounds_use_full_oriented_geometry_not_claimed_aabb(self):
        scorer = BrickScorer(FULL)
        state = sample(0.)
        state["upper"]["pos"][0] = -.16
        state["upper"]["low"] = [.1, .1, .8]
        state["upper"]["high"] = [.2, .2, .9]
        scorer.update(state)
        self.assertEqual(scorer.score, 0.)
        self.assertIsNotNone(scorer.invalid_reason)
        feed(scorer, "pickup", 5, .05)
        self.assertEqual(scorer.score, 0.)

    def test_below_table_invalidates(self):
        scorer = BrickScorer(FULL)
        state = sample(0.)
        state["lower"]["pos"][2] = TABLE-.016
        scorer.update(state)
        self.assertIsNotNone(scorer.invalid_reason)

    def test_nonfinite_and_malformed_values_fail_closed(self):
        mutations = [lambda s: s.__setitem__("time", float("nan")),
                     lambda s: s["upper"]["pos"].__setitem__(0, float("inf")),
                     lambda s: s["lower"].__setitem__("quat_xyzw", [0, 0, 0, 0]),
                     lambda s: s["hands"][0].__setitem__("opening", float("nan")),
                     lambda s: s["hands"][0]["finger_boxes"][0].__setitem__("half_size", [-1, 1, 1]),
                     lambda s: s["hands"].pop()]
        for mutation in mutations:
            scorer = BrickScorer(FULL)
            state = sample(0.)
            mutation(state)
            scorer.update(state)
            self.assertEqual(scorer.score, 0.)
            self.assertIsNotNone(scorer.invalid_reason)

    def test_batch_replay_matches_incremental_score(self):
        data = [sample(i*.05) for i in range(5)]
        direct = BrickScorer(FULL)
        for state in data:
            direct.update(state)
        self.assertEqual(score_samples(data, FULL), direct.result())


if __name__ == "__main__":
    unittest.main()
