"""Checks for measurement bypasses, partial credit, boundary cases and invalidation."""

from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import scoring
import spec as S


def body(position, quaternion=(0, 0, 0, 1), battery=False):
    p, q = np.asarray(position, float), np.asarray(quaternion, float)
    low, high = S.cell_bounds(p, q) if battery else (p-0.01, p+0.01)
    return scoring.Body(p, q, np.zeros(3), np.zeros(3), low, high)


def initial():
    bodies = {name: body(p, battery=True) for name, p in S.CELL_START.items()}
    bodies.update({name: body(p) for name, p in S.PROBE_RESTS.items()})
    return bodies


def connect(bodies, name, reversed=False):
    cell = bodies[name]
    offset = S.CELL_HALF+S.TIP_RADIUS+S.TIP_LOCAL[1]
    plus = body(cell.pos+[0, offset, 0], (0, 0, 1, 0))
    minus = body(cell.pos-[0, offset, 0])
    bodies["red_probe"], bodies["black_probe"] = (minus, plus) if reversed else (plus, minus)
    return {probe: {name: 0.1} for probe in S.PROBE_NAMES}


def measure(trial, bodies, name, reversed=False):
    forces = connect(bodies, name, reversed)
    for _ in range(round(S.CONTACT_HOLD*S.HZ)):
        trial.update(1/S.HZ, bodies, forces)


def deposit(bodies, name, bin_name):
    p = S.BIN_CENTERS[bin_name].copy()
    p[2] = S.BIN_FLOOR+S.CELL_RADIUS+0.0003
    bodies[name] = body(p, battery=True)


class ScoringTests(unittest.TestCase):
    def test_both_opposite_terminals_and_force_required(self):
        for failure in ("missing_force", "one_probe", "same_terminal", "air_gap", "different_cells"):
            with self.subTest(failure=failure):
                trial, bodies = scoring.Trial(), initial()
                name = S.CELL_NAMES[0]
                forces = connect(bodies, name)
                if failure == "missing_force":
                    forces = {}
                elif failure == "one_probe":
                    bodies["black_probe"] = initial()["black_probe"]
                elif failure == "same_terminal":
                    bodies["black_probe"] = deepcopy(bodies["red_probe"])
                elif failure == "air_gap":
                    bodies["red_probe"].pos[1] += 0.004
                else:
                    bodies["black_probe"].pos[0] = bodies[S.CELL_NAMES[1]].pos[0]
                    forces["black_probe"] = {S.CELL_NAMES[1]: 0.1}
                for _ in range(20):
                    trial.update(1/S.HZ, bodies, forces)
                self.assertFalse(trial.measured)
                self.assertIsNone(trial.reading_cv)

    def test_contact_hold_is_continuous(self):
        trial, bodies = scoring.Trial(), initial()
        name = S.CELL_NAMES[0]
        forces = connect(bodies, name)
        for _ in range(5):
            trial.update(1/S.HZ, bodies, forces)
        trial.update(1/S.HZ, bodies, {})
        for _ in range(9):
            trial.update(1/S.HZ, bodies, forces)
        self.assertNotIn(name, trial.measured)
        trial.update(1/S.HZ, bodies, forces)
        self.assertIn(name, trial.measured)

    def test_reversed_polarity_shows_negative_and_sorts_by_magnitude(self):
        trial, bodies = scoring.Trial(), initial()
        name = S.CELL_NAMES[0]
        measure(trial, bodies, name, reversed=True)
        self.assertEqual(trial.reading_cv, -S.VOLTAGES_CV[0])
        self.assertEqual(trial.measured[name], S.VOLTAGES_CV[0])

    def test_guessed_correct_bins_earn_zero_without_measurement(self):
        trial, bodies = scoring.Trial(), initial()
        for name, value in zip(S.CELL_NAMES, S.VOLTAGES_CV, strict=True):
            deposit(bodies, name, "alive" if value >= S.THRESHOLD_CV else "dead")
        trial.update(1/S.HZ, bodies, {})
        self.assertEqual(trial.score, 0)

    def test_partial_credit_tracks_current_final_state(self):
        trial, bodies = scoring.Trial(), initial()
        name = S.CELL_NAMES[0]
        measure(trial, bodies, name)
        deposit(bodies, name, "alive")
        trial.update(1/S.HZ, bodies, {})
        self.assertAlmostEqual(trial.score, 1/len(S.CELL_NAMES))
        bodies[name] = initial()[name]
        trial.update(1/S.HZ, bodies, {})
        self.assertEqual(trial.score, 0)

    def test_wrong_bin_and_held_battery_do_not_count(self):
        trial, bodies = scoring.Trial(), initial()
        name = S.CELL_NAMES[0]
        measure(trial, bodies, name)
        deposit(bodies, name, "dead")
        trial.update(1/S.HZ, bodies, {})
        self.assertEqual(trial.score, 0)
        deposit(bodies, name, "alive")
        cell = bodies[name]
        trial.update(1/S.HZ, bodies, {}, [(cell.low.copy(), cell.high.copy())])
        self.assertEqual(trial.score, 0)

    def test_bin_contact_tolerance_rejects_real_protrusion(self):
        for protrusion, expected in ((0.0005, True), (0.002, False)):
            trial, bodies = scoring.Trial(), initial()
            name = S.CELL_NAMES[0]
            measure(trial, bodies, name)
            p = S.BIN_CENTERS["alive"].copy()
            p[0] -= S.BIN_INNER_HALF[0]-S.CELL_RADIUS+protrusion
            p[2] = S.BIN_FLOOR+S.CELL_RADIUS
            bodies[name] = body(p, battery=True)
            trial.update(1/S.HZ, bodies, {})
            self.assertEqual(trial.details[name]["credited"], expected)

    def test_threshold_equality_is_alive(self):
        values = list(S.VOLTAGES_CV)
        values[0] = S.THRESHOLD_CV
        trial, bodies = scoring.Trial(values), initial()
        name = S.CELL_NAMES[0]
        measure(trial, bodies, name)
        deposit(bodies, name, "alive")
        trial.update(1/S.HZ, bodies, {})
        self.assertTrue(trial.details[name]["credited"])

    def test_every_cell_and_final_hold_needed_for_success(self):
        trial, bodies = scoring.Trial(), initial()
        for name in S.CELL_NAMES:
            measure(trial, bodies, name)
        for name, value in zip(S.CELL_NAMES, S.VOLTAGES_CV, strict=True):
            deposit(bodies, name, "alive" if value >= S.THRESHOLD_CV else "dead")
        for _ in range(19):
            trial.update(1/S.HZ, bodies, {})
        self.assertEqual(trial.score, 1.0)
        self.assertFalse(trial.success)
        trial.update(1/S.HZ, bodies, {})
        self.assertTrue(trial.success)

    def test_workspace_escape_is_permanent(self):
        trial, bodies = scoring.Trial(), initial()
        name = S.CELL_NAMES[0]
        bodies[name].low[2] = S.TOP-0.03
        trial.update(1/S.HZ, bodies, {})
        bodies = initial()
        for name in S.CELL_NAMES:
            measure(trial, bodies, name)
        self.assertEqual(trial.score, 0)
        self.assertIsNotNone(trial.invalid_reason)

    def test_nonfinite_state_invalidates(self):
        trial, bodies = scoring.Trial(), initial()
        bodies["red_probe"].pos[0] = float("nan")
        trial.update(1/S.HZ, bodies, {})
        self.assertIsNotNone(trial.invalid_reason)


if __name__ == "__main__":
    unittest.main()
