"""Unit tests verifying glue pattern and reset functionality for segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import unittest
from types import SimpleNamespace
import numpy as np

import task
import geometry as g


class TestTaskGlue(unittest.TestCase):
    def test_glue_pattern(self):
        task.reset(0)
        obj_bleach = SimpleNamespace(
            pos=np.array(g.BAY_LEFT_POS, dtype=float),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
            lin_vel=np.array([0.0, 0.0, 0.0], dtype=float),
        )
        obj_ammonia = SimpleNamespace(
            pos=np.array(g.BAY_RIGHT_POS, dtype=float),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0], dtype=float),
            lin_vel=np.array([0.0, 0.0, 0.0], dtype=float),
        )

        objects = None
        for _ in range(45):
            objects = {"bottle_bleach": obj_bleach, "bottle_ammonia": obj_ammonia}
            res = task._update(objects)

        self.assertAlmostEqual(res.score, 1.00)
        self.assertTrue(res.success)
        self.assertEqual(task.score(objects), 1.00)
        self.assertTrue(task.success(objects))

        # Check caching on object identity
        res2 = task._update(objects)
        self.assertIs(res, res2)

        # Check reset clears state
        task.reset(0)
        self.assertIsNone(task._last_states_ref)
        self.assertIsNone(task._last_result)


if __name__ == "__main__":
    unittest.main()
