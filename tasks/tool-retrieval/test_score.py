"""History and scorer edge cases; run with python -m unittest discover."""
import unittest
from types import SimpleNamespace
import numpy as np
from tool_retrieval.scoring import History, evaluate

TOP = .7475


def state(center, size, still=True):
    p = np.array(center, dtype=float)
    h = np.array(size, dtype=float)/2
    return SimpleNamespace(pos=p, low=p-h, high=p+h, resting=lambda: still)


def objects():
    return {"part": state((.235,-.175,TOP+.009),(.03,.025,.01)),
            "spatula": state((.24,.27,TOP+.039),(.273,.059,.0074))}


class ScorerTests(unittest.TestCase):
    def test_placement_without_extraction_is_zero(self):
        self.assertEqual(evaluate(objects(), History(), TOP), 0)

    def test_valid_full_completion(self):
        self.assertEqual(evaluate(objects(), History(tool_contact=True,extracted=True,open_clear=True), TOP), 1)

    def test_direct_touch_before_extraction_invalidates(self):
        h = History()
        h.contact('/World/envs/env_0/part','/World/envs/env_0/left_robot/gripper')
        h.tool_contact = h.extracted = h.open_clear = True
        self.assertEqual(evaluate(objects(), h, TOP), 0)

    def test_grasp_after_extraction_allowed(self):
        h = History(extracted=True)
        h.contact('/World/envs/env_0/part','/World/envs/env_0/right_robot/gripper')
        self.assertFalse(h.invalid)

    def test_only_part_tool_pair_counts(self):
        h = History()
        h.contact('/World/envs/env_0/spatula','/World/envs/env_0/table')
        self.assertFalse(h.tool_contact)
        h.contact('/World/envs/env_0/part','/World/envs/env_0/spatula')
        self.assertTrue(h.tool_contact)

    def test_partial_extraction_and_unreturned_tool(self):
        o = objects();h = History(extracted=True,tool_contact=True)
        self.assertEqual(evaluate(o,h,TOP), .75)
        o['part'] = state((.25,.075,TOP+.005),(.03,.025,.01))
        self.assertEqual(evaluate(o,h,TOP), .25)

    def test_dropped_part_invalidates(self):
        o = objects();h=History(extracted=True,tool_contact=True,open_clear=True)
        o['part'] = state((.235,-.175,TOP-.1),(.03,.025,.01))
        h.update(o,TOP)
        self.assertEqual(evaluate(o,h,TOP),0)

    def test_extraction_needs_tool_contact_and_full_clearance(self):
        o=objects();o['part']=state((.31,.075,TOP+.005),(.03,.025,.01))
        h=History(tool_contact=True);h.update(o,TOP)
        self.assertFalse(h.extracted)
        o['part']=state((.28,.075,TOP+.005),(.03,.025,.01));h.update(o,TOP)
        self.assertTrue(h.extracted)

    def test_returning_part_under_guard_loses_extraction_credit(self):
        o=objects();o['part']=state((.40,.075,TOP+.005),(.03,.025,.01))
        self.assertEqual(evaluate(o,History(tool_contact=True,extracted=True),TOP),0)

    def test_held_or_tilted_tool_is_not_parked(self):
        o=objects();h=History(tool_contact=True,extracted=True,open_clear=True)
        o['spatula']=state((.24,.27,TOP+.06),(.273,.059,.06))
        self.assertEqual(evaluate(o,h,TOP),.75)


class ActualRobotPrimPaths(unittest.TestCase):
    def test_physx_robot_prim_paths_are_rejected(self):
        for arm in ('LeftArm', 'RightArm'):
            history = History()
            history.contact('/World/envs/env_0/part', f'/World/envs/env_0/{arm}/link_6')
            self.assertTrue(history.invalid, arm)


if __name__ == '__main__':
    unittest.main()
