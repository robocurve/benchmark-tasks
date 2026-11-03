"""Loads task.py the way yam_arena does and runs the scoring glue and the demo plans against fake poses. Needs
``yam_arena`` importable (installed, or a checkout named by $YAM_ARENA_SRC); skipped otherwise. No simulator."""

import math
import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))
try:
    import yam_arena  # noqa: F401
except ImportError:
    yam_arena = None


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


@unittest.skipIf(yam_arena is None, "yam_arena not importable: set YAM_ARENA_SRC to a checkout")
class TaskGlue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from yam_arena import task_file

        import geometry as g
        from parts import PartState

        cls.g, cls.PartState = g, PartState
        cls.task = task_file.load(HERE / "task.py")
        cls.mod = cls.task.module

    def setUp(self):
        self.open_ = True
        self.mod.gripper_open = lambda e, arm: self.open_
        self.mod.env = lambda: None
        self.mod._SCORER.reset()

    def state(self, pos, quat, plunger=0.0):
        z = np.zeros(3)
        return self.PartState(
            pos=np.array(pos, float),
            quat_xyzw=np.array(quat, float),
            lin_vel=z,
            ang_vel=z,
            low=z,
            high=z,
            sliders={"plunger": plunger},
        )

    def objects(self, plug_pos, plug_quat=None, plunger=0.0):
        g = self.g
        block_quat = yaw_quat(g.BLOCK_YAW)
        return {
            "block": self.state(g.BLOCK_POS, block_quat, plunger),
            "plug": self.state(plug_pos, block_quat if plug_quat is None else plug_quat, plunger),
        }

    def seated(self):
        x, y = self.g.block_to_world_xy((0.0, self.g.MOUTH_Y))
        return (x, y, self.g.SEAT_Z)

    def test_declares_the_two_parts_the_scene_needs(self):
        self.assertEqual(self.task.names, ("block", "plug"))
        self.assertEqual(self.task.free_objects, ("block", "plug"))
        self.assertEqual(self.task.scene, "station")
        self.assertEqual(self.task.docs.split(":")[0][:2], "On")

    def test_the_block_is_turned_so_the_socket_faces_the_arms(self):
        block = self.task.objects[0]
        self.assertAlmostEqual(block.yaw, self.g.BLOCK_YAW)
        mouth_x, _ = self.g.block_to_world_xy((0.0, self.g.MOUTH_Y))
        self.assertLess(mouth_x, self.g.BLOCK_XY[0])  # the mouth is nearer the arms than the block's centre

    def test_start_layout_scores_zero(self):
        g = self.g
        start = self.objects((*g.PLUG_XY0, g.PLUG_Z0), yaw_quat(g.PLUG_YAW0))
        self.assertEqual(self.task.score(start), 0.0)
        self.assertFalse(self.task.success(start))

    def test_seated_and_released_is_full_score(self):
        seated = self.objects(self.seated())
        self.task.score(seated)  # the first call captures the starting poses
        self.assertEqual(self.task.score(seated), 1.0)
        self.assertTrue(self.task.success(seated))

    def test_seated_but_still_held_is_three_quarters(self):
        self.open_ = False
        seated = self.objects(self.seated())
        self.task.score(seated)
        self.assertEqual(self.task.score(seated), 0.75)
        self.assertFalse(self.task.success(seated))

    def test_plunger_pressed_back_is_not_seated(self):
        pressed = self.objects(self.seated(), plunger=-0.0005)
        self.task.score(pressed)
        self.assertLess(self.task.score(pressed), 0.75)

    def test_demo_plans(self):
        plan = self.task.demo()
        self.assertEqual(set(plan), {"left", "right"})
        os.environ["DEMO_NO_BRACE"] = "1"
        try:
            self.assertEqual(set(self.task.demo()), {"right"})
        finally:
            del os.environ["DEMO_NO_BRACE"]
        os.environ["DEMO_HOLD_ONLY"] = "1"
        try:
            self.assertEqual(self.task.demo(), {})
        finally:
            del os.environ["DEMO_HOLD_ONLY"]

    def test_demo_ends_with_the_plug_pushed_to_the_mouth(self):
        right = self.task.demo()["right"]
        push = next(w for w in right if w.label == "push in")
        g = self.g
        import demo_plan as d

        # the TCP sits over the housing centre, 9 mm behind the plug origin, which ends at the mouth (plus the overshoot)
        x, y = g.block_to_world_xy((d.DEFAULT_ALIGN_DX, g.MOUTH_Y + d.DEFAULT_OVERSHOOT - g.HOUSING[1] / 2))
        self.assertAlmostEqual(push.pos[0], x, delta=1e-9)
        self.assertAlmostEqual(push.pos[1], y, delta=1e-9)
        self.assertAlmostEqual(push.pos[2], g.SEAT_Z + d.DEFAULT_ALIGN_DZ + 0.007, delta=1e-9)

    def test_the_grips_close_across_the_parts(self):
        plan = self.task.demo()
        g = self.g
        right = {w.label: w for w in plan["right"]}
        # grip yaw = the part's yaw + 90 degrees (+ the 0 or -180 degree choice): fingers close along the part's x axis
        self.assertAlmostEqual(math.remainder(right["grip the plug"].yaw - (g.PLUG_YAW0 + math.pi / 2), math.pi), 0.0, places=9)
        self.assertAlmostEqual(math.remainder(right["push in"].yaw - (g.BLOCK_YAW + math.pi / 2), math.pi), 0.0, places=9)
        left = {w.label: w for w in plan["left"]}
        self.assertAlmostEqual(math.remainder(left["grip the block"].yaw - (g.BLOCK_YAW + math.pi / 2), math.pi), 0.0, places=9)

    def test_left_hand_holds_the_block_away_from_the_socket_end(self):
        left = {w.label: w for w in self.task.demo()["left"]}
        import demo_plan as d

        grip = left["grip the block"].pos
        g = self.g
        x, y = g.block_to_world_xy((0.0, d.LEFT_GRIP_DY))
        self.assertAlmostEqual(grip[0], x + d.LEFT_GRIP_SHIFT_W[0], delta=1e-9)
        self.assertAlmostEqual(grip[1], y + d.LEFT_GRIP_SHIFT_W[1], delta=1e-9)
        # within the block's length, with room for the 25 mm wide pads
        self.assertLess(0.03 + 0.0125, g.BLOCK_SIZE[1] / 2)

    def test_left_arm_holds_until_the_right_hand_has_lifted_clear(self):
        plan = self.task.demo()
        right_t = np.cumsum([w.duration for w in plan["right"]])
        labels = [w.label for w in plan["right"]]
        backed_off = right_t[labels.index("lift clear")]
        left_labels = [w.label for w in plan["left"]]
        left_t = np.cumsum([w.duration for w in plan["left"]])
        let_go = left_t[left_labels.index("let go of the block") - 1]  # the hold ends where "let go" begins
        self.assertGreaterEqual(let_go, backed_off - 1e-9)


if __name__ == "__main__":
    unittest.main()
