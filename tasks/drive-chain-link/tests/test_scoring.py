"""Adversarial state tests, independent of the reference controller and Isaac Sim."""

import copy
import json
from pathlib import Path
import unittest
import numpy as np
from chain_link.config import (
    CONNECTOR_POSITION,
    CONNECTOR_PICKUP_POSITION,
    KEEPER_POSITION,
    CHAIN_POSITION,
    NODES,
)
from chain_link.scoring import Scorer


def assembly():
    c = CONNECTOR_POSITION.copy()

    def part(p):
        return {"pose": [*p, 0.0, 0.0, 0.0, 1.0], "velocity": [0.0] * 6}

    bodies = {f"Link{i:02}": part(c + [0.2, 0, 0.007]) for i in range(11)}
    for i in range(1, 10):
        a, b = NODES[i], NODES[i + 1]
        yaw = np.arctan2(b[1] - a[1], b[0] - a[0])
        bodies[f"Link{i:02}"] = part(CHAIN_POSITION + (a + b) / 2)
        bodies[f"Link{i:02}"]["pose"][3:] = [0, 0, np.sin(yaw / 2), np.cos(yaw / 2)]
    bodies["Link00"] = part(c + [0.05, -0.05, 0.007])
    bodies["Link10"] = part(c + [-0.05, 0.05, 0.007])
    return dict(
        time_s=0.0,
        connector=part(c),
        keeper=part(c + [0.014, 0, 0.020]),
        bodies=bodies,
        hands={n: {"opening_m": 0.09} for n in ("left_robot", "right_robot")},
        robot_clearance_m=0.04,
    )


class AssemblyScoreTests(unittest.TestCase):
    def final(self, s, scorer=None):
        scorer = Scorer() if scorer is None else scorer
        scorer.evaluate(s)
        s["time_s"] += 1.01
        return scorer.evaluate(s)

    def resting_layout(self):
        state = assembly()
        state["connector"]["pose"][:3] = CONNECTOR_PICKUP_POSITION.tolist()
        state["keeper"]["pose"][:3] = KEEPER_POSITION.tolist()
        for i in (0, 10):
            a, b = NODES[i], NODES[i + 1]
            yaw = np.arctan2(b[1] - a[1], b[0] - a[0])
            state["bodies"][f"Link{i:02}"]["pose"] = list(
                CHAIN_POSITION + (a + b) / 2
            ) + [0, 0, np.sin(yaw / 2), np.cos(yaw / 2)]
        return state

    def test_loose_connector_start_has_no_assembly_credit(self):
        m = self.final(self.resting_layout())
        self.assertEqual(m["score"], 0)
        self.assertFalse(m["parked"])
        self.assertTrue(m["guide_seated"])

    def test_placing_connector_alone_does_not_join_chain(self):
        state = self.resting_layout()
        state["connector"]["pose"][:3] = CONNECTOR_POSITION.tolist()
        m = self.final(state)
        self.assertTrue(m["parked"])
        self.assertFalse(m["ends_joined"])
        self.assertEqual(m["score"], 0)

    def test_success_requires_one_second(self):
        s = assembly()
        score = Scorer()
        self.assertEqual(score.evaluate(s)["score"], 0.7)
        s["time_s"] = 0.99
        self.assertFalse(score.evaluate(s)["success"])
        s["time_s"] = 1.01
        self.assertEqual(score.evaluate(s)["score"], 1.0)

    def test_either_missing_end_forces_zero(self):
        for name in ("Link00", "Link10"):
            with self.subTest(name=name):
                s = assembly()
                s["bodies"][name]["pose"][0] += 0.03
                self.assertEqual(self.final(s)["score"], 0.0)

    def test_both_ends_without_keeper_only_partial(self):
        s = assembly()
        s["keeper"]["pose"][1] -= 0.3
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_slot_mouths_never_count_as_retained(self):
        s = assembly()
        s["keeper"]["pose"][0] = CONNECTOR_POSITION[0] - 0.018
        m = self.final(s)
        self.assertTrue(m["plate_seated"])
        self.assertFalse(m["retained"])
        self.assertEqual(m["score"], 0.7)

    def test_one_stem_outside_slot(self):
        s = assembly()
        s["keeper"]["pose"][1] += 0.013
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_reversed_keeper_rejected(self):
        s = assembly()
        s["keeper"]["pose"][3:] = [0, 0, 1, 0]
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_plate_above_heads_is_not_seated(self):
        s = assembly()
        s["keeper"]["pose"][2] = CONNECTOR_POSITION[2] + 0.04
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_plate_below_chain_is_not_seated(self):
        s = assembly()
        s["keeper"]["pose"][2] = CONNECTOR_POSITION[2] + 0.002
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_radial_clearance_not_axis_aligned_square(self):
        s = assembly()
        s["bodies"]["Link00"]["pose"][0] += 0.009
        s["bodies"]["Link00"]["pose"][1] += 0.009
        self.assertEqual(self.final(s)["score"], 0.0)

    def test_nearby_valid_hole_offset(self):
        s = assembly()
        s["bodies"]["Link00"]["pose"][0] += 0.010
        self.assertEqual(self.final(s)["score"], 1.0)

    def test_lifted_end_rejected(self):
        s = assembly()
        s["bodies"]["Link10"]["pose"][2] += 0.03
        self.assertEqual(self.final(s)["score"], 0.0)

    def test_hands_must_open_and_clear(self):
        for field in ("opening", "clearance"):
            s = assembly()
            if field == "opening":
                s["hands"]["right_robot"]["opening_m"] = 0.02
            else:
                s["robot_clearance_m"] = 0.001
            self.assertEqual(self.final(s)["score"], 0.7)

    def test_every_chain_link_must_settle(self):
        for name in ("Link00", "Link05", "Link10"):
            s = assembly()
            s["bodies"][name]["velocity"][0] = 0.03
            self.assertEqual(self.final(s)["score"], 0.7)
        s = assembly()
        s["keeper"]["velocity"][5] = 0.4
        self.assertEqual(self.final(s)["score"], 0.7)

    def test_interruption_restarts_hold(self):
        s = assembly()
        score = Scorer()
        score.evaluate(s)
        s["time_s"] = 0.8
        s["robot_clearance_m"] = 0
        score.evaluate(s)
        s["time_s"] = 0.9
        s["robot_clearance_m"] = 0.04
        score.evaluate(s)
        s["time_s"] = 1.1
        self.assertEqual(score.evaluate(s)["score"], 0.7)
        s["time_s"] = 1.91
        self.assertEqual(score.evaluate(s)["score"], 1.0)

    def test_no_peak_score_survives_disassembly(self):
        s = assembly()
        score = Scorer()
        self.assertEqual(self.final(s, score)["score"], 1.0)
        s["bodies"]["Link00"]["pose"][0] += 0.03
        self.assertEqual(score.evaluate(s)["score"], 0.0)

    def test_connector_must_remain_in_cradle(self):
        s = assembly()
        for p in [s["connector"], s["keeper"], *s["bodies"].values()]:
            p["pose"][0] += 0.1
        m = self.final(s)
        self.assertTrue(m["retained"])
        self.assertFalse(m["parked"])
        self.assertEqual(m["score"], 0.7)

    def test_escape_violation_is_permanent(self):
        for p in ([0, 0, 0.73], [-0.2, 0, 0.85], [0.92, 0, 0.85], [0.2, 0.66, 0.85]):
            score = Scorer()
            score.observe_physics({"part": np.array([p])})
            score.observe_physics({"part": np.array([[0.2, 0, 0.85]])})
            self.assertEqual(self.final(assembly(), score)["score"], 0.0)
            self.assertEqual(score.physics_samples, 2)

    def test_nonfinite_physics_coordinate_is_invalid(self):
        score = Scorer()
        score.observe_physics({"part": np.array([[np.nan, 0, 0.85]])})
        m = self.final(assembly(), score)
        self.assertEqual(m["score"], 0.0)
        self.assertIn("invalid_state", m["violations"])

    def test_nonfinite_state_or_velocity_forces_zero(self):
        for field in ("pose", "velocity"):
            s = assembly()
            s["bodies"]["Link05"][field][0] = np.nan
            score = Scorer()
            self.assertEqual(score.evaluate(s)["score"], 0.0)
            self.assertEqual(self.final(assembly(), score)["score"], 0.0)

    def test_folded_middle_link_cannot_receive_full_credit(self):
        s = assembly()
        s["bodies"]["Link05"]["pose"][0] -= 0.050
        m = self.final(s)
        self.assertTrue(m["retained"])
        self.assertFalse(m["guide_seated"])
        self.assertEqual(m["score"], 0.7)

    def test_link_lifted_off_former_cannot_receive_full_credit(self):
        s = assembly()
        s["bodies"]["Link04"]["pose"][2] += 0.010
        self.assertFalse(self.final(s)["guide_seated"])
        self.assertEqual(self.final(s)["score"], 0.7)

    def test_guide_uses_window_shape_instead_of_a_circular_distance(self):
        from chain_link.scoring import rotation

        s = assembly()
        p = s["bodies"]["Link05"]["pose"]
        p[:3] = (np.array(p[:3]) + rotation(p) @ np.array([0.020, 0, 0])).tolist()
        self.assertTrue(self.final(s)["guide_seated"])
        self.assertEqual(self.final(s)["score"], 1)

    def test_tooth_outside_side_rails_is_not_seated(self):
        from chain_link.scoring import rotation

        s = assembly()
        p = s["bodies"]["Link05"]["pose"]
        p[:3] = (np.array(p[:3]) + rotation(p) @ np.array([0, 0.012, 0])).tolist()
        self.assertFalse(self.final(s)["guide_seated"])
        self.assertEqual(self.final(s)["score"], 0.7)

    def test_keeper_measurement_is_not_overwritten_by_guide_diagnostics(self):
        m = Scorer().evaluate(assembly())
        np.testing.assert_allclose(m["keeper_local"], [0.014, 0, 0.020], atol=1e-12)

    def test_trace_inputs_are_not_mutated(self):
        s = assembly()
        before = copy.deepcopy(s)
        Scorer().evaluate(s)
        self.assertEqual(s, before)


class PhysicalApertureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        records = json.loads(
            (Path(__file__).resolve().parents[1] / "scene-inventory.json").read_text()
        )
        cls.cells = [
            r for r in records if r.get("asset") == "keeper" and r.get("kind") == "box"
        ]

    def occupied(self, p):
        p = np.array(p)
        return any(
            np.all(abs(p - r["center_m"]) < np.array(r["size_m"]) / 2 - 1e-9)
            for r in self.cells
        )

    def circle_hits_solid(self, x, y, r):
        return any(
            self.occupied([x + r * np.cos(a), y + r * np.sin(a), 0])
            for a in np.linspace(0, 2 * np.pi, 180)
        )

    def test_heads_cannot_pass_vertically_through_slots(self):
        for y in (-0.05, 0.05):
            self.assertTrue(self.circle_hits_solid(0.018, y, 0.012))
            self.assertFalse(self.occupied([0.018, y, 0]))

    def test_stems_can_enter_from_the_open_edge(self):
        for y in (-0.05, 0.05):
            for x in np.linspace(0.05, -0.014, 50):
                self.assertFalse(self.circle_hits_solid(x, y, 0.004))

    def test_stems_fit_but_heads_are_blocked_by_necks(self):
        for y in (-0.05, 0.05):
            self.assertFalse(self.circle_hits_solid(-0.014, y, 0.004))
            self.assertTrue(self.circle_hits_solid(-0.014, y, 0.012))

    def test_closed_slot_ends_open_mouths_and_connected_bridge(self):
        for y in (-0.05, 0.05):
            self.assertTrue(self.occupied([-0.035, y, 0]))
            self.assertFalse(self.occupied([0.039, y, 0]))
        self.assertTrue(self.occupied([0, 0, 0]))


class BareRailClearanceTests(unittest.TestCase):
    def test_raised_rails_clear_the_retaining_plate(self):
        from chain_link.config import CLOSED_NODES

        records = json.loads(
            (Path(__file__).resolve().parents[1] / "scene-inventory.json").read_text()
        )
        rails = [r for r in records if "RailThickening" in r.get("solid", "")]
        self.assertEqual(len(rails), 4)
        for record in rails:
            i = 0 if "/Link00/" in record["solid"] else 10
            a, b = CLOSED_NODES[i], CLOSED_NODES[i + 1]
            centre = CHAIN_POSITION + (a + b) / 2
            yaw = np.arctan2(b[1] - a[1], b[0] - a[0])
            rotation = np.array(
                [
                    [np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1],
                ]
            )
            world = centre + rotation @ np.array(record["center_m"])
            extent = np.abs(rotation) @ (np.array(record["size_m"]) / 2)
            # The whole raised rail must stay outside the keeper's +/-64mm
            # plate envelope. Pin holes and 5mm support shoulders stay unchanged.
            self.assertGreater(abs(world[1]) - extent[1], 0.064)


if __name__ == "__main__":
    unittest.main()
