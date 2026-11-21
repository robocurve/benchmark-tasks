"""The scoring rules on hand-built states: guard seats, hooked horns, every score band, the locks, and the operator's
test (lift to 50 N, hold 2 s, bare-steel check over the last second). Standard library only."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_sling_guards import grading  # noqa: E402
from rdb_sling_guards import hook as HK  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402

EDGES = grading.EDGE_NAMES
FAR = ((.130, .300, .010), (0., 0., 0., 1.))           # a guard still in its tray (far from every edge)


def quat_from_matrix(R):
    """(x, y, z, w) from a rotation matrix given as rows."""
    t = R[0][0] + R[1][1] + R[2][2]
    if t > -.99:
        w = math.sqrt(1 + t) / 2
        return ((R[2][1] - R[1][2]) / (4 * w), (R[0][2] - R[2][0]) / (4 * w), (R[1][0] - R[0][1]) / (4 * w), w)
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(1e-12, 1 + R[i][i] - R[j][j] - R[k][k])) * 2
    q = [0., 0., 0., 0.]
    q[3] = (R[k][j] - R[j][k]) / s
    q[i] = .25 * s
    q[j] = (R[j][i] + R[i][j]) / s
    q[k] = (R[k][i] + R[i][k]) / s
    return tuple(q)


def seated(edge, dx=0., d=(0., 0.), turn=None):
    (y, z), _, _ = L.EDGES[edge]
    R = grading.seat_rotation(edge)
    if turn is not None:
        R = grading.matmul(R, turn)
    return ((L.XS + dx, y + d[0], z + d[1]), quat_from_matrix(R))


def rx(deg):
    a = math.radians(deg)
    return ((1., 0., 0.), (0., math.cos(a), -math.sin(a)), (0., math.sin(a), math.cos(a)))


ALL = {f"guard_{k + 1}": seated(e) for k, e in enumerate(EDGES)}
BOTTOM = HK.C - (L.BOWL_R_IN - L.FIT_BAR_R)
APEX_IN = {"fitA": (HK.C, -(L.BOWL_R_IN - L.FIT_BAR_R)), "fitB": (-HK.C, -(L.BOWL_R_IN - L.FIT_BAR_R))}
APEX_OUT = {"fitA": (.20, -.20), "fitB": (-.20, -.20)}


def m(guards=None, apex=None, latch=(0., 0.), clear=False, under=True, pull=0., bare=None, on_guard=None, apex_x=None):
    return {"guards": guards if guards is not None else {n: FAR for n in ALL}, "apex": apex or APEX_OUT,
            "apex_x": apex_x or {"fitA": 0., "fitB": 0.},
            "latch_deg": {"L": latch[0], "R": latch[1]}, "hands_clear": clear, "under_bar": under, "pull_n": pull,
            "bare_n": bare or {e: 0. for e in EDGES}, "guard_n": on_guard or {e: 5. for e in EDGES}}


def run(grade, state, ticks):
    out = None
    for _ in range(ticks):
        out = grade.update(state)
    return out


class Seats(unittest.TestCase):
    def test_each_edge_and_the_flipped_guard(self):
        for edge in EDGES:
            p, q = seated(edge)
            self.assertEqual(grading.guard_seat(p, q)[0], edge)
            p, q = seated(edge, turn=grading.FLIP)          # the same angle turned end for end
            self.assertEqual(grading.guard_seat(p, q)[0], edge)

    def test_limits(self):
        tol = 1000 * L.SEAT_TOL
        self.assertEqual(grading.guard_seat(*seated("TL", d=(.0029, 0.)))[0], "TL")
        self.assertIsNone(grading.guard_seat(*seated("TL", d=(.0031, 0.)))[0])
        self.assertEqual(grading.guard_seat(*seated("BR", turn=rx(9.5)))[0], "BR")
        self.assertIsNone(grading.guard_seat(*seated("BR", turn=rx(10.5)))[0])
        along = L.GUARD_LEN / 2 - L.SLING_W / 2 - L.COVER_MARGIN
        self.assertEqual(grading.guard_seat(*seated("TR", dx=along - .0005))[0], "TR")
        self.assertIsNone(grading.guard_seat(*seated("TR", dx=along + .0005))[0])
        self.assertGreater(tol, 0)

    def test_tray_guard_is_not_seated(self):
        self.assertIsNone(grading.guard_seat(*FAR)[0])


class Horns(unittest.TestCase):
    def test_bowl_floor_counts_and_outside_does_not(self):
        for side, c in (("L", HK.C), ("R", -HK.C)):
            self.assertTrue(HK.in_bowl((c, -(L.BOWL_R_IN - L.FIT_BAR_R)), side))
            self.assertFalse(HK.in_bowl((c, .030), side))                 # above the latch's end
            self.assertFalse(HK.in_bowl((3 * c, -.010), side))           # outside the horn
            self.assertFalse(HK.in_bowl((-c, -(L.BOWL_R_IN - L.FIT_BAR_R)), side))   # the other horn's bowl


class Bands(unittest.TestCase):
    def test_nothing_done(self):
        g = grading.RiggingGrade().update(m())
        self.assertEqual((g["score"], g["test"]), (0., "waiting"))

    def test_guards_count_a_tenth(self):
        for n in range(5):
            guards = {k: (v if i < n else FAR) for i, (k, v) in enumerate(ALL.items())}
            self.assertAlmostEqual(grading.RiggingGrade().update(m(guards=guards))["score"], .1 * n)

    def test_one_hooked_fitting_counts_a_fifth(self):
        apex = {"fitA": APEX_IN["fitA"], "fitB": APEX_OUT["fitB"]}
        self.assertAlmostEqual(grading.RiggingGrade().update(m(apex=apex))["score"], .2)
        self.assertAlmostEqual(grading.RiggingGrade().update(m(guards=ALL, apex=apex))["score"], .6)

    def test_apex_beside_the_hook_does_not_count(self):
        apex = {"fitA": APEX_IN["fitA"], "fitB": APEX_OUT["fitB"]}
        for x, score in ((0., .2), (L.APEX_OFF_PLANE, .2), (L.APEX_OFF_PLANE + .001, 0.), (.100, 0.)):
            g = grading.RiggingGrade().update(m(apex=apex, apex_x={"fitA": x, "fitB": 0.}))
            self.assertAlmostEqual(g["score"], score, msg=x)

    def test_open_latch_does_not_count(self):
        apex = {"fitA": APEX_IN["fitA"], "fitB": APEX_OUT["fitB"]}
        self.assertAlmostEqual(grading.RiggingGrade().update(m(apex=apex, latch=(-12., 0.)))["score"], 0.)

    def test_both_hooked_with_an_edge_unguarded_is_zero(self):
        guards = dict(ALL)
        guards["guard_2"] = FAR
        self.assertEqual(grading.RiggingGrade().update(m(guards=guards, apex=APEX_IN))["score"], 0.)

    def test_both_hooked_and_guarded_before_the_test(self):
        g = grading.RiggingGrade().update(m(guards=ALL, apex=APEX_IN))
        self.assertEqual((g["score"], g["test"]), (.7, "waiting"))


class Test(unittest.TestCase):
    def ready(self, **kw):
        return m(guards=ALL, apex=APEX_IN, clear=True, **kw)

    def test_full_score_after_the_hold(self):
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        g = run(grade, self.ready(pull=0.), int(L.CLEAR_S * hz))
        self.assertEqual(g["test"], "lifting")
        g = run(grade, self.ready(pull=L.PULL_N + 1), 1)
        self.assertEqual(g["test"], "holding")
        g = run(grade, self.ready(pull=L.PULL_N), int(L.HOLD_S * hz))
        self.assertEqual((g["test"], g["result"], g["score"]), ("done", "passed", 1.))
        self.assertFalse(grade.finished)
        run(grade, self.ready(pull=L.PULL_N), int(grading.SUCCESS_S * hz))
        self.assertTrue(grade.finished)

    def test_no_test_until_the_hands_have_been_clear_a_second(self):
        grade = grading.RiggingGrade()
        run(grade, self.ready(), int(L.CLEAR_S * grading.CONTROL_HZ) - 1)
        g = grade.update(m(guards=ALL, apex=APEX_IN, clear=False))
        self.assertEqual(g["test"], "waiting")

    def bare_test(self, bare):
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        run(grade, self.ready(), int(L.CLEAR_S * hz))
        run(grade, self.ready(pull=L.PULL_N + 1), 1)
        return run(grade, self.ready(pull=L.PULL_N, bare=bare), int(L.HOLD_S * hz))

    def test_web_on_bare_steel_locks_zero(self):
        g = self.bare_test({"TL": 0., "BL": L.BARE_LIMIT_N + .1, "TR": 0., "BR": 0.})
        self.assertEqual((g["score"], g["result"]), (0., "the web bore on bare steel at BL"))
        self.assertEqual(g["locked"], g["result"])

    def test_bare_contact_below_the_limit_passes(self):
        g = self.bare_test({e: L.BARE_LIMIT_N - .1 for e in EDGES})
        self.assertEqual(g["score"], 1.)

    def test_sling_out_from_under_the_bar_locks_zero(self):
        grade = grading.RiggingGrade()
        grade.update(m(guards=ALL, under=False))
        g = run(grade, self.ready(), 3)
        self.assertEqual((g["score"], g["locked"]), (0., "the sling was pulled out from under the bar"))

    def test_test_with_an_edge_unguarded_locks_zero(self):
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        guards = dict(ALL)
        guards["guard_1"] = FAR
        state = m(guards=guards, apex=APEX_IN, clear=True)
        run(grade, state, int(L.CLEAR_S * hz))
        run(grade, dict(state, pull_n=L.PULL_N + 1), 1)
        g = run(grade, dict(state, pull_n=L.PULL_N), int(L.HOLD_S * hz))
        self.assertEqual((g["score"], g["locked"]), (0., "the load was taken with an edge unguarded"))
        g = grade.update(self.ready())                       # guarding it afterwards does not help
        self.assertEqual(g["score"], 0.)

    def test_a_one_step_spike_does_not_make_a_hold(self):
        # a native run: the web snagged on a bare corner, the pull read 55 N for one step, then the sling's own 11 N
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        run(grade, self.ready(), int(L.CLEAR_S * hz))
        g = run(grade, self.ready(pull=55.), 1)
        self.assertEqual(g["test"], "holding")
        g = run(grade, self.ready(pull=11.), 1)
        self.assertEqual((g["test"], grade.command), ("lifting", "lift"))
        g = run(grade, self.ready(pull=11.), int(L.HOLD_S * hz))
        self.assertEqual((g["test"], g["score"]), ("lifting", .7))

    def test_a_dropped_load_starts_the_hold_over(self):
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        run(grade, self.ready(), int(L.CLEAR_S * hz))
        run(grade, self.ready(pull=L.PULL_N + 1), 1)
        run(grade, self.ready(pull=L.PULL_N), int(L.HOLD_S * hz) - 2)
        g = run(grade, self.ready(pull=L.HOLD_MIN_N - 1), 1)
        self.assertEqual(g["test"], "lifting")
        run(grade, self.ready(pull=L.PULL_N + 1), 1)
        g = run(grade, self.ready(pull=L.HOLD_MIN_N), int(L.HOLD_S * hz) - 1)    # 40 N still counts as held
        self.assertEqual(g["test"], "holding")
        g = run(grade, self.ready(pull=L.HOLD_MIN_N), 1)
        self.assertEqual((g["test"], g["result"], g["score"]), ("done", "passed", 1.))

    def test_a_hold_not_made_within_the_time_limit_fails(self):
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        run(grade, self.ready(), int(L.CLEAR_S * hz))
        start = grade.test_start
        g = run(grade, self.ready(pull=11.), int(L.TEST_LIMIT_S * hz) - (grade.tick - start) - 1)
        self.assertEqual((g["test"], g["score"]), ("lifting", .7))
        g = run(grade, self.ready(pull=11.), 1)
        self.assertEqual((g["test"], g["score"]), ("done", 0.))
        self.assertTrue(g["locked"].startswith("the hook did not hold 50 N for 2 s within 30 s"), g["locked"])

    def test_a_hand_back_in_during_the_test_locks_zero(self):
        grade = grading.RiggingGrade()
        run(grade, self.ready(), int(L.CLEAR_S * grading.CONTROL_HZ))
        run(grade, self.ready(pull=L.PULL_N), 1)
        g = run(grade, m(guards=ALL, apex=APEX_IN, clear=False, pull=L.PULL_N), 1)
        self.assertEqual(g["score"], 0.)
        g = run(grade, self.ready(pull=L.PULL_N), int(L.HOLD_S * grading.CONTROL_HZ))
        self.assertEqual((g["score"], g["locked"]), (0., "a hand came back into the rigging during the test lift"))

    def test_a_restarted_hold_reads_its_own_last_second(self):
        grade = grading.RiggingGrade()
        hz = grading.CONTROL_HZ
        run(grade, self.ready(), int(L.CLEAR_S * hz))
        run(grade, self.ready(pull=L.PULL_N), 1)
        run(grade, self.ready(pull=L.PULL_N), int((L.HOLD_S - 1.) * hz))
        run(grade, self.ready(pull=L.PULL_N, bare={e: 2. for e in EDGES}), 5)     # a snag, then the load drops
        run(grade, self.ready(pull=L.HOLD_MIN_N - 1), 1)
        run(grade, self.ready(pull=L.PULL_N), 1)
        g = run(grade, self.ready(pull=L.PULL_N), int(L.HOLD_S * hz))
        self.assertEqual((g["result"], g["score"]), ("passed", 1.))

    def test_hoist_commands(self):
        grade = grading.RiggingGrade()
        self.assertEqual(grade.command, "rest")
        run(grade, self.ready(), int(L.CLEAR_S * grading.CONTROL_HZ))
        self.assertEqual(grade.command, "lift")
        grade.update(self.ready(pull=L.PULL_N + 1))
        self.assertEqual(grade.command, "hold")


if __name__ == "__main__":
    unittest.main()
