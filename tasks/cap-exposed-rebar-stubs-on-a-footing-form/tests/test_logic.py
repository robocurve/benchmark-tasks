"""The scoring rules, driven through logic.Scorer with made-up cap poses (no simulator)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import geometry as g  # noqa: E402
import logic  # noqa: E402

UP = (0.0, 0.0, 0.0, 1.0)


def tilted(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def seated(stub, dx=0.0, dz=0.0, quat=UP):
    return ((g.FORM_X + dx, g.STUB_Y[stub], g.SEATED_Z + dz), quat)


def start_caps():
    caps = {"cap_fitted_1": seated(0), "cap_fitted_2": seated(4)}
    for k, y in enumerate(g.CAP_START_Y):
        caps[f"cap_loose_{k + 1}"] = ((g.PAD_X, y, g.CAP_START_Z), UP)
    return caps


FAR = [(0.0, 0.305, 1.0), (0.0, -0.305, 1.0)]
NEAR = [(g.FORM_X, 0.0, g.TOP + 0.10), (0.0, -0.305, 1.0)]


class TestLogic(unittest.TestCase):
    def run_steps(self, caps, tcps=FAR, n=1):
        sc = logic.Scorer()
        r = None
        for _ in range(n):
            r = sc.step(caps=caps, tcps=tcps)
        return r

    def finish(self, n_capped=3):
        caps = start_caps()
        for k, i in enumerate(g.BARE[:n_capped]):
            caps[f"cap_loose_{k + 1}"] = seated(i)
        return caps

    def test_start_state_scores_zero(self):
        r = self.run_steps(start_caps())
        self.assertEqual(r["score"], 0.0)
        self.assertEqual(r["n_capped"], 2)

    def test_ladder(self):
        for n, want in ((1, 0.25), (2, 0.5), (3, 0.75)):
            self.assertEqual(self.run_steps(self.finish(n))["score"], want, n)

    def test_full_score_needs_the_hold_and_clear_arms(self):
        caps = self.finish(3)
        self.assertEqual(self.run_steps(caps, FAR, 59)["score"], 0.75)
        self.assertEqual(self.run_steps(caps, FAR, 60)["score"], 1.0)
        self.assertEqual(self.run_steps(caps, NEAR, 200)["score"], 0.75)       # an arm left over the form

    def test_knocking_a_cap_off_lowers_the_score(self):
        sc = logic.Scorer()
        r = sc.step(caps=self.finish(3), tcps=FAR)
        self.assertEqual(r["score"], 0.75)
        caps = self.finish(3)
        caps["cap_fitted_1"] = ((g.FORM_X, g.STUB_Y[0] - 0.05, g.TOP + 0.03), tilted(90))
        self.assertEqual(sc.step(caps=caps, tcps=FAR)["score"], 0.5)

    def test_seat_conditions(self):
        i = g.BARE[0]
        self.assertTrue(logic.seated_on(*seated(i), i))
        self.assertFalse(logic.seated_on(*seated(i, dx=0.004), i))              # 4 mm off the bar axis
        self.assertTrue(logic.seated_on(*seated(i, dx=0.0025), i))
        self.assertFalse(logic.seated_on(*seated(i, dz=0.006), i))              # bar only 24 mm up the bore
        self.assertTrue(logic.seated_on(*seated(i, dz=0.003), i))
        self.assertFalse(logic.seated_on(*seated(i, quat=tilted(9)), i))
        self.assertTrue(logic.seated_on(*seated(i, quat=tilted(6)), i))

    def test_a_cap_on_its_side_or_perched_on_the_tip_is_not_seated(self):
        i = g.BARE[0]
        self.assertFalse(logic.seated_on((g.FORM_X, g.STUB_Y[i], g.BAR_TOP - 0.010), UP, i))   # on the tip: 10 mm up
        self.assertFalse(logic.seated_on((g.FORM_X, g.STUB_Y[i], g.TOP + 0.03), tilted(90), i))

    def test_one_cap_cannot_count_for_two_stubs(self):
        caps = start_caps()
        caps["cap_loose_1"] = seated(g.BARE[0])
        self.assertEqual(self.run_steps(caps)["n_capped"], 3)

    def test_arms_clear_rule(self):
        self.assertTrue(logic.arms_clear(FAR))
        self.assertFalse(logic.arms_clear([(g.FORM_X + 0.10, 0.0, g.TOP + 0.10)] + FAR[1:]))     # 10 cm from the middle stub
        self.assertTrue(logic.arms_clear([(g.FORM_X, 0.0, g.TOP + 0.30)] + FAR[1:]))              # high above it


if __name__ == "__main__":
    unittest.main()
