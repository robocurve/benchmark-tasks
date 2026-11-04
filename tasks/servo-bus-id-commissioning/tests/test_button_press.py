"""The cap spring is soft and the stop sets the travel: static equilibrium of arm compliance vs cap spring (VERIFY_F item 2).

Arithmetic (measured from the YAM joint gains at the press pose with compliance.py): vertical tip compliance is about
1.8 mm/N for the left arm (which presses) and 3.1 mm/N for the right arm. A 625 N/m cap commanded 6.8 mm deep only
travels 2-3 mm, below the 6.4 mm trigger; a 100 N/m cap commanded 14 mm deep reaches the 8 mm stop.
"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import geometry as g
from compliance import equilibrium_travel

TRIGGER = g.BUTTON_PRESS_THRESHOLD  # 6.4 mm
C_WORST = 3.2e-3  # m/N: the right arm's measured 3.06 mm/N rounded up; the left (pressing) arm is 1.8 mm/N
FRICTION_SHORT = 0.002  # m: joint friction can stop the tip up to about 2 mm short of the command


class TestButtonPress(unittest.TestCase):
    def test_spring_is_soft(self) -> None:
        self.assertLessEqual(g.BUTTON_RATE, 100.0)
        self.assertLess(g.BUTTON_RATE * g.BUTTON_TRAVEL, 1.0)  # under 1 N reaches the 8 mm stop
        sag = g.BUTTON_MASS * 9.81 / g.BUTTON_RATE
        self.assertLess(sag, 0.0005, "a heavy cap would sag on the soft spring and look pre-pressed")

    def test_old_stiff_design_could_not_trigger(self) -> None:
        """Regression: 625 N/m with the old 6.83 mm commanded press travels well under the 6.4 mm trigger."""
        for c in (1.8e-3, 3.06e-3):
            self.assertLess(equilibrium_travel(c, 0.00683, 625.0, 0.008), TRIGGER - 0.002)

    def test_equilibrium_travel_reaches_trigger_with_margin(self) -> None:
        for c in (1.0e-3, 1.8e-3, 3.06e-3, C_WORST):
            for short in (0.0, FRICTION_SHORT):
                travel = equilibrium_travel(c, g.BUTTON_PRESS_DEPTH_CMD, g.BUTTON_RATE, g.BUTTON_TRAVEL, short)
                self.assertGreaterEqual(travel, TRIGGER + 0.001, f"C={c}, short={short}: travel {travel * 1000:.2f} mm")

    def test_stop_sets_travel_not_arm(self) -> None:
        """For any compliance up to 4 mm/N the cap is at the 8 mm stop, so the press margin is stop - trigger = 1.6 mm."""
        for c in (0.5e-3, 1.8e-3, 3.2e-3, 4e-3):
            travel = equilibrium_travel(c, g.BUTTON_PRESS_DEPTH_CMD, g.BUTTON_RATE, g.BUTTON_TRAVEL, FRICTION_SHORT)
            self.assertAlmostEqual(travel, g.BUTTON_TRAVEL, places=9)
        self.assertGreaterEqual(g.BUTTON_TRAVEL - TRIGGER, 0.001)

    def test_command_is_deeper_than_stop_by_compliance_and_friction(self) -> None:
        need = g.BUTTON_TRAVEL * (1 + C_WORST * g.BUTTON_RATE) + FRICTION_SHORT
        self.assertGreaterEqual(g.BUTTON_PRESS_DEPTH_CMD, need + 0.001, "keep >= 1.0 mm of command beyond the requirement")

    def test_fingertip_stays_above_controller_box(self) -> None:
        tip_cmd = g.BUTTON_PRESS_TCP_Z - g.PAD_TIP_PAST_TCP
        box_top = g.TOP + g.CTRL_SIZE[2]
        self.assertGreaterEqual(tip_cmd - box_top, 0.005)
        # The stop-limited cap top is where the tip really stops
        self.assertAlmostEqual(g.BUTTON_REST_TOP_Z - g.BUTTON_TRAVEL - box_top, 0.012, places=6)

    def test_measured_arm_model_left_arm_both_buttons(self) -> None:
        """Compute the compliance from the real joint gains and IK at both press poses (needs yam_arena)."""
        try:
            from yam_arena import ik, station
            from yam_arena.constants import READY
            from yam_arena.demo import grip_rotation
        except ImportError:
            self.skipTest("yam_arena not importable")
        from compliance import friction_allowance, vertical_compliance

        base = station.BASES["left"]
        for pos2 in (g.ASSIGN_BTN_POS, g.RESET_BTN_POS):
            target = np.array([pos2[0], pos2[1], g.BUTTON_PRESS_TCP_Z])
            q = np.array(READY, float)
            for _ in range(4):
                q, pe, _re = ik.solve(target, ik.matrix_to_quat_xyzw(grip_rotation(0.0)), q, base_pos=base, iters=200)
            self.assertLess(pe, 0.001)
            c = vertical_compliance(q, base, (0.0, 0.0, -g.PAD_TIP_PAST_TCP))
            short = friction_allowance(q, base, (0.0, 0.0, -g.PAD_TIP_PAST_TCP))
            travel = equilibrium_travel(c, g.BUTTON_PRESS_DEPTH_CMD, g.BUTTON_RATE, g.BUTTON_TRAVEL, short)
            self.assertLessEqual(c, C_WORST)
            self.assertGreaterEqual(travel, TRIGGER + 0.001, f"C={c * 1000:.2f} mm/N short={short * 1000:.2f} mm travel={travel * 1000:.2f} mm")
            print(f"press at {pos2[:2]}: C_zz={c * 1000:.2f} mm/N ({1 / c:.0f} N/m), friction_short={short * 1000:.2f} mm, "
                  f"equilibrium cap travel={travel * 1000:.2f} mm (trigger 6.4, stop 8.0)")


if __name__ == "__main__":
    unittest.main()
