from __future__ import annotations

import importlib.util
import os
import unittest
import numpy as np

import mechkit as mk

# Load task dynamically
_task_path = os.path.join(os.path.dirname(__file__), "clamp_grounding_clamp_fuse_carrier.py")
_spec = importlib.util.spec_from_file_location("clamp_grounding_clamp_fuse_carrier", _task_path)
task = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(task)


class TestDisconnectScoring(unittest.TestCase):
    def setUp(self):
        mk.MEMORY.clear()

    def make_state(
        self,
        clamp_pos: tuple[float, float, float] = task.CLAMP_INIT_POS,
        fuse_pos: tuple[float, float, float] = task.FUSE_SEATED_POS,
        box_pos: tuple[float, float, float] = (task.BOX_X, task.BOX_Y, task.BOX_BASE_Z),
        tray_pos: tuple[float, float, float] = task.TRAY_POS,
        clamp_lifted: bool = False,
        clamp_grounded: bool = False,
        fuse_extracted: bool = False,
        fuse_in_tray: bool = False,
        ungrounded_hazard_tripped: bool = False,
        hold_timer: int = 0,
    ):
        moved = {
            "disconnect_box": (np.array(box_pos), np.array([0.0, 0.0, 0.0, 1.0])),
            "grounding_clamp": (np.array(clamp_pos), np.array([0.0, 0.0, 0.0, 1.0])),
            "fuse_carrier": (np.array(fuse_pos), np.array([0.0, 0.0, 0.0, 1.0])),
            "safety_tray": (np.array(tray_pos), np.array([0.0, 0.0, 0.0, 1.0])),
        }
        mk.MEMORY["clamp_lifted"] = clamp_lifted
        mk.MEMORY["clamp_grounded"] = clamp_grounded
        mk.MEMORY["fuse_extracted"] = fuse_extracted
        mk.MEMORY["fuse_in_tray"] = fuse_in_tray
        mk.MEMORY["ungrounded_hazard_tripped"] = ungrounded_hazard_tripped
        mk.MEMORY["hold_timer"] = hold_timer
        return task.kit.fake_states(moved=moved)

    def test_initial_state(self):
        objs = self.make_state()
        self.assertEqual(task.score(objs), 0.0)
        self.assertFalse(task.success(objs))

    def test_clamp_lifted_partial(self):
        objs = self.make_state(
            clamp_pos=(task.CLAMP_INIT_POS[0], task.CLAMP_INIT_POS[1], task.CLAMP_INIT_POS[2] + 0.050),
            clamp_lifted=True,
        )
        self.assertEqual(task.score(objs), 0.20)
        self.assertFalse(task.success(objs))

    def test_clamp_grounded_stage(self):
        busbar_clamp_p = (task.BUSBAR_POS[0], task.BUSBAR_POS[1] + 0.018, task.BUSBAR_POS[2])
        objs = self.make_state(
            clamp_pos=busbar_clamp_p,
            clamp_lifted=True,
            clamp_grounded=True,
        )
        self.assertEqual(task.score(objs), 0.50)
        self.assertFalse(task.success(objs))

    def test_fuse_extracted_stage(self):
        busbar_clamp_p = (task.BUSBAR_POS[0], task.BUSBAR_POS[1] + 0.018, task.BUSBAR_POS[2])
        objs = self.make_state(
            clamp_pos=busbar_clamp_p,
            fuse_pos=(task.FUSE_SEATED_POS[0], task.FUSE_SEATED_POS[1], task.FUSE_SEATED_POS[2] + 0.080),
            clamp_lifted=True,
            clamp_grounded=True,
            fuse_extracted=True,
        )
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_full_success_hold(self):
        busbar_clamp_p = (task.BUSBAR_POS[0], task.BUSBAR_POS[1] + 0.018, task.BUSBAR_POS[2])
        tray_fuse_p = (task.TRAY_POS[0], task.TRAY_POS[1], task.TRAY_POS[2] + 0.018)
        objs = self.make_state(
            clamp_pos=busbar_clamp_p,
            fuse_pos=tray_fuse_p,
            clamp_lifted=True,
            clamp_grounded=True,
            fuse_extracted=True,
            fuse_in_tray=True,
            hold_timer=16,
        )
        self.assertEqual(task.score(objs), 1.0)
        self.assertTrue(task.success(objs))

    def test_hazard_ungrounded_pull_fails(self):
        objs = self.make_state(
            fuse_pos=(task.FUSE_SEATED_POS[0], task.FUSE_SEATED_POS[1], task.FUSE_SEATED_POS[2] + 0.080),
            ungrounded_hazard_tripped=True,
        )
        self.assertTrue(task.failed(objs))
        self.assertEqual(task.score(objs), 0.0)

    def test_dropped_clamp_fails(self):
        objs = self.make_state(
            clamp_pos=(task.CLAMP_INIT_POS[0], task.CLAMP_INIT_POS[1], task.TOP - 0.060),
        )
        self.assertTrue(task.failed(objs))
        self.assertEqual(task.score(objs), 0.0)

    def test_dropped_fuse_fails(self):
        objs = self.make_state(
            fuse_pos=(task.FUSE_SEATED_POS[0], task.FUSE_SEATED_POS[1], task.TOP - 0.060),
        )
        self.assertTrue(task.failed(objs))
        self.assertEqual(task.score(objs), 0.0)


if __name__ == "__main__":
    unittest.main()
