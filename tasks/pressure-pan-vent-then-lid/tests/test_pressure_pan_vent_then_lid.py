import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import numpy as np

from geometry import LID_REST_Z_M, PAN_XY, TABLE_TOP, TRIVET_XY, VALVE_XY
from logic import (
    evaluate_state,
    hands_clear,
    in_plume_zone,
    lid_rise,
    lifted_now,
    seated_on_trivet,
    tilt_fault_now,
)

LEVEL = (0.0, 0.0, 0.0, 1.0)


def tilted(deg):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


ON_TRIVET = (TRIVET_XY[0], TRIVET_XY[1], TABLE_TOP + 0.010)
ON_PAN = (PAN_XY[0], PAN_XY[1], TABLE_TOP + LID_REST_Z_M)
PAN = (PAN_XY[0], PAN_XY[1], TABLE_TOP)


def score(lid_pos=ON_TRIVET, quat=LEVEL, press=0.0, plume=False, moved=False, bypass=False, tilt=False, lifted=True,
          unlocked=20, hands=True, still=True):
    return evaluate_state(lid_pos, quat, press, plume, moved, bypass, tilt, lifted, unlocked, hands, still)[0]


class Ladder(unittest.TestCase):
    def test_full(self):
        self.assertEqual(score(), 1.0)

    def test_idle_scores_zero(self):
        self.assertEqual(score(lid_pos=ON_PAN, lifted=False, unlocked=0), 0.0)

    def test_vent_not_released_is_three_quarters(self):
        self.assertEqual(score(press=0.01), 0.75)

    def test_hands_not_clear_or_moving_is_three_quarters(self):
        self.assertEqual(score(hands=False), 0.75)
        self.assertEqual(score(still=False), 0.75)

    def test_lifted_but_elsewhere_is_half(self):
        self.assertEqual(score(lid_pos=(0.28, -0.05, TABLE_TOP + 0.12)), 0.5)

    def test_vent_held_without_lift_is_quarter(self):
        self.assertEqual(score(lid_pos=ON_PAN, lifted=False, unlocked=20), 0.25)
        self.assertEqual(score(lid_pos=ON_PAN, lifted=False, unlocked=19), 0.0)

    def test_faults_zero_everything(self):
        self.assertEqual(score(plume=True), 0.0)
        self.assertEqual(score(moved=True), 0.0)

    def test_caps(self):
        self.assertEqual(score(bypass=True), 0.25)
        self.assertEqual(score(tilt=True), 0.5)
        self.assertEqual(score(bypass=True, tilt=True), 0.25)

    def test_trivet_tolerances(self):
        x, y, z = ON_TRIVET
        self.assertTrue(seated_on_trivet((x, y, z + 0.0029), LEVEL))
        self.assertFalse(seated_on_trivet((x, y, z + 0.0035), LEVEL))
        self.assertFalse(seated_on_trivet((x, y, z - 0.0035), LEVEL))
        self.assertTrue(seated_on_trivet((x + 0.019, y, z), LEVEL))
        self.assertFalse(seated_on_trivet((x + 0.021, y, z), LEVEL))
        self.assertTrue(seated_on_trivet((x, y, z), tilted(2.9)))
        self.assertFalse(seated_on_trivet((x, y, z), tilted(3.5)))

    def test_hover_above_trivet_does_not_count(self):
        x, y, z = ON_TRIVET
        self.assertNotEqual(score(lid_pos=(x, y, z + 0.02), lifted=False, unlocked=0), 1.0)


class Lift(unittest.TestCase):
    def test_rise_and_tilt(self):
        up = (PAN_XY[0], PAN_XY[1], TABLE_TOP + LID_REST_Z_M + 0.031)
        self.assertAlmostEqual(lid_rise(up), 0.031)
        self.assertTrue(lifted_now(up, tilted(14)))
        self.assertFalse(lifted_now(up, tilted(16)))
        self.assertFalse(lifted_now((up[0], up[1], up[2] - 0.005), LEVEL))

    def test_tilt_fault_only_above_the_pan(self):
        up = (PAN_XY[0], PAN_XY[1], TABLE_TOP + LID_REST_Z_M + 0.05)
        self.assertTrue(tilt_fault_now(up, tilted(16), PAN_XY))
        self.assertFalse(tilt_fault_now(up, tilted(10), PAN_XY))
        away = (PAN_XY[0], PAN_XY[1] - 0.2, up[2])
        self.assertFalse(tilt_fault_now(away, tilted(30), PAN_XY))


class Zone(unittest.TestCase):
    def test_inside_and_outside(self):
        z = TABLE_TOP + 0.12
        self.assertTrue(in_plume_zone((VALVE_XY[0], VALVE_XY[1], z)))
        self.assertTrue(in_plume_zone((VALVE_XY[0] - 0.07, VALVE_XY[1], z)))
        self.assertFalse(in_plume_zone((VALVE_XY[0] - 0.08, VALVE_XY[1], z)))
        self.assertFalse(in_plume_zone((0.20, -0.10, z)))          # the demo's waiting point
        self.assertFalse(in_plume_zone((VALVE_XY[0], VALVE_XY[1], TABLE_TOP + 0.03)))   # below the zone
        self.assertFalse(in_plume_zone((VALVE_XY[0], VALVE_XY[1], TABLE_TOP + 0.30)))   # above it


class Release(unittest.TestCase):
    ready = [(0.10, 0.14, TABLE_TOP + 0.12), (0.10, -0.14, TABLE_TOP + 0.12)]

    def test_ready_poses_are_clear(self):
        self.assertTrue(hands_clear(self.ready, ON_TRIVET, LEVEL, PAN, 0.0))

    def test_close_to_the_lid_is_not_clear(self):
        near = [(TRIVET_XY[0], TRIVET_XY[1], TABLE_TOP + 0.04), self.ready[0]]
        self.assertFalse(hands_clear(near, ON_TRIVET, LEVEL, PAN, 0.0))

    def test_close_to_the_button_is_not_clear(self):
        near = [(PAN_XY[0] - 0.045, PAN_XY[1] + 0.165, TABLE_TOP + 0.09), self.ready[1]]
        self.assertFalse(hands_clear(near, ON_TRIVET, LEVEL, PAN, 0.0))

    def test_close_to_the_pan_is_not_clear(self):
        near = [self.ready[0], (PAN_XY[0], PAN_XY[1], TABLE_TOP + 0.10)]
        self.assertFalse(hands_clear(near, ON_TRIVET, LEVEL, PAN, 0.0))


if __name__ == "__main__":
    unittest.main()
