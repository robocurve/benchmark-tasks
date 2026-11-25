"""The optional local expert changes only this task's future scripted waypoints."""

from pathlib import Path
from types import SimpleNamespace
import unittest

from pose import install_expert, ready_pose
from yam_arena.demo import FINGER_OPEN, Waypoint
from yam_arena.scripted import YamScriptedPolicy


class ExpertCalibrationTests(unittest.TestCase):
    def test_calibration_is_scoped_once_and_preserves_the_executed_prefix(self):
        task_path = Path(__file__).with_name("task.py").resolve()
        calls = []
        steered = []

        def plan(dx=0.):
            result = {}
            for side in ("left","right"):
                p,r = ready_pose(side)
                target = (p[0]+dx,p[1],p[2])
                result[side] = [
                    Waypoint(.10,p,FINGER_OPEN,"park",r),
                    Waypoint(.10,target,FINGER_OPEN,"pickup",r),
                ]
            return result

        def replan(stage):
            calls.append(stage)
            return plan(.002)

        def steer(target):
            steered.append(target[-1])
            return target

        cls = YamScriptedPolicy
        old_reset,old_act = cls.reset,cls.act
        try:
            install_expert(task_path,{"pickup":"shade"},replan,steer=steer)
            policy = cls()
            policy._dt = .05
            scene = SimpleNamespace(instruction="",target=None)
            policy._task = SimpleNamespace(path=task_path,demo=plan)
            policy.reset(scene)
            original = policy._plans["left"].waypoints.copy()
            policy.act(SimpleNamespace(state_time=0.))
            self.assertEqual(calls,[])
            policy.act(SimpleNamespace(state_time=.10))
            policy.act(SimpleNamespace(state_time=.14))
            self.assertEqual(calls,["shade"])
            self.assertEqual(steered,["park","pickup","pickup"])
            self.assertNotIn("target", policy._plans["right"].__dict__)
            waypoints = policy._plans["left"].waypoints
            self.assertIs(waypoints[0],original[0])
            self.assertAlmostEqual(waypoints[1].pos[0]-original[1].pos[0],.002)

            policy._task = SimpleNamespace(path=task_path.with_name("other.py"),demo=plan)
            policy.reset(scene)
            policy.act(SimpleNamespace(state_time=.14))
            self.assertEqual(calls,["shade"])
            self.assertEqual(steered,["park","pickup","pickup"])
        finally:
            cls.reset,cls.act = old_reset,old_act
            if hasattr(cls, "_harp_expert_installed"):
                del cls._harp_expert_installed


if __name__ == "__main__":
    unittest.main()
