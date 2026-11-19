"""Counterexamples to grading the squeegee by its handle height or centre."""
import unittest
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from geometry import BLADE, PCB, STENCIL, WORLD
from surface_scoring import ScoreState, blade_clear_of_pcb


def pose(xyz, angles=(0, 0, 0)):
    return np.r_[xyz, Rotation.from_euler('xyz', angles, degrees=True).as_quat()]


class BladeClearanceTests(unittest.TestCase):
    def test_low_blade_on_original_rest_can_earn_full_credit(self):
        paths = [WORLD + '/pcb_stencil', WORLD + '/pcb_squeegee']
        filters = paths + [WORLD + '/stencil_support0', WORLD + '/stencil_support1']
        metrics = dict(released=True, bridges=0, qualified_pad_fraction=1.,
                       all_pads_nonempty=True, on_pad_fraction=1., deposited_mass_mg=70.)
        state = ScoreState('stencil_paste', paths, filters, [], SimpleNamespace(metrics=lambda: metrics))
        poses = np.array([pose(STENCIL), pose(BLADE)])
        contacts = np.zeros((2, len(filters)))
        contacts[0, 2] = 1

        def step(n):
            for _ in range(n):
                state.step(poses, np.zeros((2, 6)), np.zeros((0, 4)), contacts, .05)
            return state.update()

        self.assertEqual(step(20), 1)
        poses[1] = pose([*PCB[:2], .811])  # blade enters the board while handle stays above
        self.assertEqual(step(1), 0)
        poses[1] = pose(BLADE)
        self.assertEqual(step(19), 0)
        self.assertEqual(step(1), 1)

    def test_hovering_depends_on_lowest_overlapping_part(self):
        # Board top is 780.5 mm. The upright blade edge is 32.5 mm below the handle.
        self.assertTrue(blade_clear_of_pcb(pose([*PCB[:2], .8151])))
        self.assertFalse(blade_clear_of_pcb(pose([*PCB[:2], .8150])))
        self.assertFalse(blade_clear_of_pcb(pose([*PCB[:2], .824], (0, 65, 0))))

    def test_rotated_corner_uses_tool_footprint(self):
        xyz = PCB + [.04929, .04167, .0293]
        self.assertTrue(blade_clear_of_pcb(pose(xyz, (0, 0, -45))))
        self.assertFalse(blade_clear_of_pcb(pose(xyz, (0, 0, 45))))

    def test_upturned_tool_can_foul_board_with_handle_only(self):
        self.assertFalse(blade_clear_of_pcb(pose([*PCB[:2], .789], (180, 0, 0))))


if __name__ == '__main__':
    unittest.main()
