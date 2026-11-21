"""Offline unit tests for reassemble-commercial-citrus-juicer scoring and mechanics."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import mechkit as mk
import reassemble_commercial_citrus_juicer as task


class TestJuicerScoring(unittest.TestCase):

    def setUp(self):
        mk.MEMORY.clear()
        mk.MEMORY.update({
            "bowl_placed": False,
            "bowl_locked": False,
            "cone_seated": False,
            "lever_stroked": False,
            "min_lever_deg": 0.0,
            "hold_timer": 0,
        })

    def make_state(
        self,
        bowl_pos: tuple[float, float, float] = task.BOWL_INIT_POS,
        bowl_yaw_deg: float = 0.0,
        cone_pos: tuple[float, float, float] = task.CONE_INIT_POS,
        base_pos: tuple[float, float, float] = (task.BASE_X, task.BASE_Y, task.BASE_Z),
        lever_pitch_deg: float = 85.0,
        min_lever_deg: float = 85.0,
        bowl_placed: bool = False,
        bowl_locked: bool = False,
        cone_seated: bool = False,
        lever_stroked: bool = False,
        hold_timer: int = 0,
    ):
        yaw_rad = math.radians(bowl_yaw_deg)
        bowl_quat = np.array([0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)])

        pitch_rad = math.radians(-lever_pitch_deg)
        lever_quat = np.array([0.0, math.sin(pitch_rad / 2.0), 0.0, math.cos(pitch_rad / 2.0)])

        moved = {
            "juicer_base": (np.array(base_pos), np.array([0.0, 0.0, 0.0, 1.0])),
            "press_lever": (np.array(task.LEVER_ANCHOR), lever_quat),
            "collector_bowl": (np.array(bowl_pos), bowl_quat),
            "strainer_cone": (np.array(cone_pos), np.array([0.0, 0.0, 0.0, 1.0])),
        }
        mk.MEMORY["bowl_placed"] = bowl_placed
        mk.MEMORY["bowl_locked"] = bowl_locked
        mk.MEMORY["cone_seated"] = cone_seated
        mk.MEMORY["lever_stroked"] = lever_stroked
        mk.MEMORY["min_lever_deg"] = min_lever_deg
        mk.MEMORY["hold_timer"] = hold_timer
        return task.kit.fake_states(moved=moved)

    def test_initial_state(self):
        objs = self.make_state()
        self.assertEqual(task.score(objs), 0.0)
        self.assertFalse(task.success(objs))

    def test_bowl_placed_only(self):
        # Place bowl in collar at yaw = 0 deg (not locked)
        objs = self.make_state(
            bowl_pos=(task.COLLAR_X, task.COLLAR_Y, task.BOWL_SEAT_Z),
            bowl_yaw_deg=0.0,
            bowl_placed=True,
        )
        self.assertEqual(task.score(objs), 0.25)
        self.assertFalse(task.success(objs))

    def test_bowl_locked(self):
        # Place bowl in collar and rotate 30 deg clockwise (-30 deg yaw)
        objs = self.make_state(
            bowl_pos=(task.COLLAR_X, task.COLLAR_Y, task.BOWL_SEAT_Z),
            bowl_yaw_deg=-30.0,
            bowl_placed=True,
            bowl_locked=True,
        )
        self.assertEqual(task.score(objs), 0.50)
        self.assertFalse(task.success(objs))

    def test_cone_seated(self):
        # Bowl locked + cone seated flush
        objs = self.make_state(
            bowl_pos=(task.COLLAR_X, task.COLLAR_Y, task.BOWL_SEAT_Z),
            bowl_yaw_deg=-30.0,
            cone_pos=(task.COLLAR_X, task.COLLAR_Y, task.CONE_SEAT_Z),
            bowl_placed=True,
            bowl_locked=True,
            cone_seated=True,
        )
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))

    def test_full_success_hold(self):
        # Bowl locked + cone seated + lever stroked + 20 steps hold
        objs = self.make_state(
            bowl_pos=(task.COLLAR_X, task.COLLAR_Y, task.BOWL_SEAT_Z),
            bowl_yaw_deg=-30.0,
            cone_pos=(task.COLLAR_X, task.COLLAR_Y, task.CONE_SEAT_Z),
            lever_pitch_deg=85.0,
            min_lever_deg=20.0,
            bowl_placed=True,
            bowl_locked=True,
            cone_seated=True,
            lever_stroked=True,
            hold_timer=21,
        )
        self.assertEqual(task.score(objs), 1.0)
        self.assertTrue(task.success(objs))

    def test_dropped_bowl_fails(self):
        objs = self.make_state(
            bowl_pos=(task.COLLAR_X, task.COLLAR_Y, task.TOP - 0.050),
            bowl_placed=True,
        )
        self.assertTrue(task.failed(objs))
        self.assertEqual(task.score(objs), 0.0)

    def test_dropped_cone_fails(self):
        objs = self.make_state(
            cone_pos=(task.COLLAR_X, task.COLLAR_Y, task.TOP - 0.050),
        )
        self.assertTrue(task.failed(objs))
        self.assertEqual(task.score(objs), 0.0)

    def test_base_displaced_fails(self):
        objs = self.make_state(
            base_pos=(task.BASE_X + 0.050, task.BASE_Y, task.BASE_Z),
        )
        self.assertTrue(task.failed(objs))
        self.assertEqual(task.score(objs), 0.0)

    def test_cone_without_bowl_fails(self):
        # Placing cone without bowl in place
        objs = self.make_state(
            cone_pos=(task.COLLAR_X, task.COLLAR_Y, task.CONE_SEAT_Z),
            cone_seated=False,
            bowl_locked=False,
        )
        self.assertEqual(task.score(objs), 0.0)

    def test_hold_timer_less_than_20_not_full(self):
        # Everything done, but hold_timer is only 15 (< 20 steps = 1.0 s)
        objs = self.make_state(
            bowl_pos=(task.COLLAR_X, task.COLLAR_Y, task.BOWL_SEAT_Z),
            bowl_yaw_deg=-30.0,
            cone_pos=(task.COLLAR_X, task.COLLAR_Y, task.CONE_SEAT_Z),
            bowl_placed=True,
            bowl_locked=True,
            cone_seated=True,
            lever_stroked=True,
            hold_timer=15,
        )
        self.assertEqual(task.score(objs), 0.75)
        self.assertFalse(task.success(objs))


if __name__ == "__main__":
    unittest.main()
