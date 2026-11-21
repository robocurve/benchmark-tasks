"""The scoring rules on hand-built states: every band and its edges, the zero conditions, the grippers-clear check,
the final check (a pluck after the G button's last movement) and the end of the trial. Standard library only."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_ukulele_tuning import fixed_scene as fixture  # noqa: E402
from rdb_ukulele_tuning import grading  # noqa: E402
from rdb_ukulele_tuning import layout as L  # noqa: E402
from rdb_ukulele_tuning.grading import TuningGrade  # noqa: E402
from rdb_ukulele_tuning.tuning import Pitch  # noqa: E402


def to_instrument(point):
    return fixture.to_instrument(point)


AWAY = [to_instrument((.02, .305, .26)), to_instrument((.02, -.305, .26))]       # both grippers well back
ON_BUTTON = [AWAY[0], L.button_tip("g")]                                           # right gripper on the G button
TURNED = 455.                                   # where the G button ends after tuning (degrees from rest)


def state(g=0., c=0., e=0., a=0.):
    return {"g": g, "c": c, "e": e, "a": a}


def grade_once(g, *, tools=AWAY, checked=True, **others):
    """One step that ends the trial: with ``checked`` the G string is plucked on it (after any movement)."""
    return TuningGrade().update(state(g, **others), tools, checked, TURNED)


def settled(grade, a=0., tools=AWAY, steps=1, plucked_first=True, button=TURNED, **others):
    """Steps with the G button standing at ``button`` and the G string at ``a`` cents; the first one plucks the G
    string if ``plucked_first``."""
    result = None
    for k in range(steps):
        result = grade.update(state(a, **others), tools, plucked_first and k == 0, button)
    return result


class Bands(unittest.TestCase):
    def test_every_band_and_its_edges(self):
        cases = [(0., 1.), (5., 1.), (-5., 1.), (5.01, .75), (-15., .75), (15., .75), (15.01, .5), (-30., .5),
                 (30., .5), (30.01, .25), (-30.01, .25), (50., .25), (-50., .25), (50.01, 0.), (-50.01, 0.),
                 (-100., 0.), (100., 0.), (-75., 0.)]
        for a, score in cases:
            self.assertEqual(grade_once(a)["score"], score, a)

    def test_lower_bands_do_not_need_the_final_check(self):
        for a, score in ((10., .75), (-20., .5), (40., .25), (-100., 0.)):
            self.assertEqual(grade_once(a, checked=False)["score"], score, a)

    def test_possible_scores(self):
        seen = {grade_once(a)["score"] for a in range(-120, 121)}
        seen |= {grade_once(a, tools=ON_BUTTON)["score"] for a in range(-120, 121)}
        seen |= {grade_once(a, checked=False)["score"] for a in range(-120, 121)}
        self.assertEqual(seen, set(grading.SCORES))

    def test_quarter_either_way_within_50_cents(self):
        # the bands depend on the G string's final pitch only, not on where it started
        for cents in (-40., 40., -50., 50.):
            self.assertEqual(grading.band(cents, True, True), .25, cents)
            self.assertEqual(grading.band(cents, False, False), .25, cents)
        for cents in (-50.01, 50.01, -99., 99.):
            self.assertEqual(grading.band(cents, True, True), 0., cents)

    def test_full_marks_need_both_grippers_clear(self):
        self.assertEqual(grade_once(1., tools=ON_BUTTON)["score"], .75)
        self.assertEqual(grade_once(1., tools=None)["score"], .75)  # unreadable: not clear
        self.assertFalse(grade_once(1., tools=None)["grippers_clear"])
        tip = L.button_tip("g")                                     # the G button points up (instrument +x)
        near = [AWAY[0], (tip[0] + .049, tip[1], tip[2])]
        self.assertEqual(grade_once(0., tools=near)["score"], .75)  # 49 mm above the G button's end
        far = [AWAY[0], (tip[0] + .051, tip[1], tip[2])]
        self.assertEqual(grade_once(0., tools=far)["score"], 1.)


class FinalCheck(unittest.TestCase):
    """Full marks need the G string plucked after its button's last movement (more than the gear's free play)."""

    def test_in_tune_with_a_final_pluck_scores_full(self):
        grade = TuningGrade()
        result = settled(grade, a=1.5, steps=30)
        self.assertEqual(result["score"], 1.)
        self.assertTrue(result["final_check"])
        self.assertEqual(result["last_pluck_step"], 1)

    def test_in_tune_without_one_is_capped_at_0_75(self):
        grade = TuningGrade()
        result = settled(grade, a=1.5, steps=60, plucked_first=False)          # never plucked
        self.assertEqual(result["score"], .75)
        self.assertFalse(result["final_check"])
        self.assertIsNone(result["last_pluck_step"])
        self.assertFalse(grade.finished)

    def test_a_pluck_before_the_last_movement_does_not_count(self):
        grade = TuningGrade()
        settled(grade, a=-100., button=0., steps=5)                            # plucked at the start ...
        for k in range(1, 46):                                                 # ... then turned 450 degrees
            result = grade.update(state(-100. + .22 * 10. * k), AWAY, False, 10. * k)
        self.assertLessEqual(abs(result["g_cents"]), 1.)
        self.assertEqual(result["score"], .75)
        self.assertFalse(result["final_check"])
        self.assertEqual(result["last_pluck_step"], 1)
        # plucked again after the turn: full marks
        self.assertEqual(grade.update(state(-1.), AWAY, True, 450.)["score"], 1.)

    def test_a_turn_within_the_free_play_is_not_a_movement(self):
        grade = TuningGrade()
        settled(grade, button=TURNED, steps=3)
        for shift in (1., -3., grading.CHECK_TURN_DEG, -grading.CHECK_TURN_DEG):
            self.assertEqual(grade.update(state(0.), AWAY, False, TURNED + shift)["score"], 1., shift)
        result = grade.update(state(0.), AWAY, False, TURNED + grading.CHECK_TURN_DEG + .01)
        self.assertEqual(result["score"], .75)
        self.assertEqual(grade.update(state(0.), AWAY, False, TURNED)["score"], .75)    # turning back does not undo it

    def test_the_last_pluck_is_the_one_that_counts(self):
        grade = TuningGrade()
        settled(grade, a=-100., button=0., steps=2)                            # plucked at the start (step 1)
        settled(grade, a=-1., button=450., steps=2)                            # turned, plucked again (step 3)
        self.assertEqual(grade.update(state(-1.), AWAY, False, 450.)["score"], 1.)
        result = grade.update(state(3.4), AWAY, False, 470.)                    # 20 degrees more, not plucked
        self.assertFalse(result["final_check"])
        self.assertEqual(result["score"], .75)                                 # within 5 cents, never shown
        self.assertEqual(result["last_pluck_step"], 3)
        self.assertEqual(result["g_turned_since_pluck_deg"], 20.)

    def test_the_check_and_the_other_conditions_are_separate(self):
        grade = TuningGrade()
        self.assertEqual(settled(grade, a=0., tools=ON_BUTTON, steps=3)["score"], .75)  # plucked but still holding
        self.assertEqual(settled(TuningGrade(), a=0., c=16., steps=3)["score"], 0.)     # plucked but C moved


class ZeroConditions(unittest.TestCase):
    def test_another_string_moved_more_than_15_cents(self):
        for name in ("c", "e", "a"):
            for moved in (15.01, -15.01, 40.):
                result = grade_once(0., **{name: moved})
                self.assertEqual(result["score"], 0., (name, moved))
                self.assertIn(name.upper() + " string", result["void"])
            self.assertEqual(grade_once(0., **{name: 15.})["score"], 1.)
            self.assertEqual(grade_once(0., **{name: -14.9})["score"], 1.)

    def test_turning_the_c_button_instead_fails(self):
        # the C string's button sits beside the G string's on the same edge: 70 degrees of it is 15.4 cents
        c = Pitch("c")
        moved = c.update(math.radians(70.) * L.tighten_sign("c"))
        self.assertGreater(moved, grading.MOVED_CENTS)
        self.assertEqual(TuningGrade().update(state(-100., c=moved), AWAY, True, 0.)["score"], 0.)

    def test_untouched_and_loosened(self):
        self.assertEqual(grade_once(-100.)["score"], 0.)                         # nothing done
        self.assertEqual(grade_once(-130.)["score"], 0.)                         # turned the wrong way
        self.assertEqual(grade_once(100.)["score"], 0.)                          # a whole semitone too far: G#4

    def test_measured_from_each_strings_start(self):
        grade = TuningGrade(start={"g": -100., "c": 2., "e": -1., "a": 0.})
        self.assertEqual(grade.update(state(0., c=16.), AWAY, True, 0.)["score"], 1.)    # 14 cents from its start
        self.assertEqual(grade.update(state(0., c=-13.1), AWAY, False, 0.)["score"], 0.)  # 15.1 from it


class Clearance(unittest.TestCase):
    def test_box_distance(self):
        box = ((0., 1.), (0., 1.), (0., 1.))
        self.assertEqual(grading.box_distance((.5, .5, .5), box), 0.)
        self.assertAlmostEqual(grading.box_distance((2., .5, .5), box), 1.)
        self.assertAlmostEqual(grading.box_distance((2., 2., .5), box), math.sqrt(2.))

    def test_ready_pose_is_clear_and_the_parts_are_not(self):
        for point in AWAY:
            self.assertGreater(grading.clearance(point), .2)
        for point in (L.button_tip("g"), L.button_centre("c"), L.pluck_point(), L.TUNER_CENTRE,
                      L.on_screen(0., 0.), (0., L.SOUNDHOLE_Y, L.TOP_Z), (0., L.TAIL_Y, L.BACK_Z)):
            self.assertEqual(grading.clearance(point), 0., point)

    def test_boxes_cover_the_ukulele(self):
        boxes = grading.BOXES
        x = [v for b in boxes for v in b[0]]
        y = [v for b in boxes for v in b[1]]
        self.assertAlmostEqual(max(x) - min(x), 2 * L.BODY_OUTLINE[12][1])          # the lower bout, 178 mm
        self.assertAlmostEqual(max(x) - min(x), L.LOWER_BOUT)
        self.assertLessEqual(min(y), L.TIP_Y - L.CLIP_SPINE)
        self.assertAlmostEqual(max(y), L.TAIL_Y)
        self.assertAlmostEqual(L.TAIL_Y - L.TIP_Y, L.LENGTH)                            # 535 mm long


class EndOfTrial(unittest.TestCase):
    def test_ends_one_second_after_full_marks(self):
        grade = TuningGrade()
        self.assertEqual(settled(grade, a=1.)["score"], 1.)                    # the final pluck
        self.assertFalse(grade.finished)
        for _ in range(grading.END_TICKS - 1):
            self.assertEqual(settled(grade, a=1., plucked_first=False)["score"], 1.)
            self.assertFalse(grade.finished)
        settled(grade, a=1., plucked_first=False)
        self.assertTrue(grade.finished)

    def test_cannot_end_before_the_final_pluck(self):
        grade = TuningGrade()
        settled(grade, a=1., steps=3 * grading.END_TICKS, plucked_first=False)  # in tune and clear, not plucked
        self.assertFalse(grade.finished)
        settled(grade, a=1., steps=grading.END_TICKS + 1)                       # then plucked
        self.assertTrue(grade.finished)

    def test_hold_restarts_if_full_marks_are_lost(self):
        grade = TuningGrade()
        settled(grade, a=1., steps=grading.END_TICKS)
        self.assertEqual(settled(grade, a=1., tools=ON_BUTTON, plucked_first=False)["score"], .75)   # a hand back
        self.assertEqual(grade.success_ticks, 0)
        settled(grade, a=1., plucked_first=False)
        self.assertFalse(grade.finished)
        grade = TuningGrade()
        settled(grade, a=1., steps=grading.END_TICKS)
        settled(grade, a=1., button=TURNED + 10., plucked_first=False)          # the button turned: check void
        self.assertEqual(grade.success_ticks, 0)
        settled(grade, a=1., button=TURNED + 10., steps=2 * grading.END_TICKS, plucked_first=False)
        self.assertFalse(grade.finished)

    def test_nothing_else_ends_a_trial_early(self):
        grade = TuningGrade()
        for a in (-100., 0., 100.):
            grade.update(state(a, e=40.), ON_BUTTON, True, 0.)
            self.assertFalse(grade.finished)

    def test_grade_row_is_plain_json(self):
        import json
        row = grade_once(-3.2, e=1.5)
        self.assertEqual(json.loads(json.dumps(row)), row)
        self.assertEqual(row["moved_cents"]["e"], 1.5)
        self.assertEqual(row["g_start_cents"], -100.)
        self.assertEqual(row["g_turned_since_pluck_deg"], 0.)

    def test_scorer_name_and_entry_point(self):
        self.assertEqual(grading.make_scorer().name, "ukulele_tuning_score")
        text = (ROOT / "pyproject.toml").read_text()
        self.assertIn('ukulele_tuning_score = "rdb_ukulele_tuning.grading:make_scorer"', text)


if __name__ == "__main__":
    unittest.main()
