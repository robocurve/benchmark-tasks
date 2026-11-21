"""The rules that stand in for sound: pitch from the button's angle with the gear's free play, the pluck rule and
the tuner's display. Standard library only: ``python -m unittest discover -s tests`` (or pytest)."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_ukulele_tuning import layout as L  # noqa: E402
from rdb_ukulele_tuning import tuning  # noqa: E402
from rdb_ukulele_tuning.tuning import Pitch, PluckDetector, TunerDisplay  # noqa: E402

HZ = tuning.CONTROL_HZ


def tighten(name, degrees):
    """The button's joint angle (radians, about +x) that is ``degrees`` of tightening from rest."""
    return math.radians(degrees) * L.tighten_sign(name)


class PitchModel(unittest.TestCase):
    def test_start_and_rate(self):
        a = Pitch("g")
        self.assertEqual(a.cents, -100.)
        self.assertAlmostEqual(a.update(tighten("g", 100.)), -100. + 22.)
        self.assertAlmostEqual(L.CENTS_PER_DEG * 360., 79.2)           # about 80 cents per turn of the button

    def test_a_full_semitone_takes_more_than_a_turn(self):
        a = Pitch("g")
        turn = 100. / L.CENTS_PER_DEG
        self.assertGreater(turn, 360.)
        self.assertAlmostEqual(a.update(tighten("g", turn)), 0., places=9)

    def test_tightening_direction_per_button(self):
        # each joint turns about the instrument's +x (up on the table); the G and C buttons point up, E and A down
        self.assertEqual(L.tighten_sign("a"), 1)
        self.assertEqual(L.tighten_sign("e"), 1)
        self.assertEqual(L.tighten_sign("g"), -1)
        self.assertEqual(L.tighten_sign("c"), -1)
        for name in L.STRINGS:   # clockwise seen from the button's end raises the pitch
            # a turn by -x degrees about the button's own outward axis is clockwise seen from its end
            outward = L.SIDE[name]
            joint = math.radians(-10.) * outward
            self.assertGreater(Pitch(name).update(joint), Pitch(name).start)

    def test_free_play_on_reversal(self):
        a = Pitch("g")
        a.update(tighten("g", 50.))
        at = a.cents
        a.update(tighten("g", 50. - 3.9))        # back inside the free play: the post does not move
        self.assertAlmostEqual(a.cents, at)
        a.update(tighten("g", 50. - 5.))          # 1 degree past it
        self.assertAlmostEqual(a.cents, at - L.CENTS_PER_DEG)
        a.update(tighten("g", 50. - 5. + 3.9))    # forward again inside the free play
        self.assertAlmostEqual(a.cents, at - L.CENTS_PER_DEG)
        a.update(tighten("g", 50. + 1.))
        self.assertAlmostEqual(a.cents, at + L.CENTS_PER_DEG)

    def test_free_play_is_taken_up_toward_tightening_at_reset(self):
        a = Pitch("g")
        self.assertAlmostEqual(a.play, L.FREE_PLAY_DEG / 2)
        self.assertAlmostEqual(a.update(tighten("g", .5)), -100. + .5 * L.CENTS_PER_DEG)     # at once
        b = Pitch("g")
        self.assertEqual(b.update(tighten("g", -L.FREE_PLAY_DEG + .01)), -100.)              # loosening: play first
        self.assertLess(b.update(tighten("g", -L.FREE_PLAY_DEG - 1.)), -100.)

    def test_other_strings_start_in_tune_and_move_alike(self):
        for name in ("c", "e", "a"):
            p = Pitch(name)
            self.assertEqual(p.cents, 0.)
            self.assertAlmostEqual(p.update(tighten(name, 68.2)), 68.2 * L.CENTS_PER_DEG)     # 15 cents

    def test_unwrap_continues_through_half_turns(self):
        angle, readings = 0., []
        for k in range(1, 200):
            true = .05 * k                          # 10 rad: more than one and a half turns
            wrapped = math.atan2(math.sin(true), math.cos(true))
            angle = tuning.unwrap(angle, wrapped)
            readings.append((true, angle))
        for true, got in readings:
            self.assertAlmostEqual(true, got, places=9)
        self.assertAlmostEqual(tuning.unwrap(3.1, -3.1), 3.1 + (2 * math.pi - 6.2))


def run_pluck(detector, displacements, touched=None, start=0):
    """Feed one displacement per step; returns the steps on which a pluck counted."""
    counted = []
    for i, d in enumerate(displacements):
        tick = start + i
        if detector.update(tick, d, bool(touched and touched[i])):
            counted.append(tick)
    return counted


class PluckRule(unittest.TestCase):
    def test_pushed_2_mm_and_springing_back_is_a_pluck(self):
        p = PluckDetector()
        self.assertEqual(run_pluck(p, [0, .001, .002, .0015, .0004, 0], [0, 1, 1, 0, 0, 0]), [4])
        self.assertEqual(p.count, 1)

    def test_less_than_2_mm_is_not(self):
        p = PluckDetector()
        self.assertEqual(run_pluck(p, [0, .001, .0019, .0019, .001, 0, 0], [0, 1, 1, 1, 0, 0, 0]), [])

    def test_either_direction(self):
        self.assertEqual(run_pluck(PluckDetector(), [0, -.003, -.0001], [0, 1, 0]), [2])

    def test_eased_back_by_the_finger_is_not(self):
        p = PluckDetector()
        out = [0, .002, .003, .0015, .0008, .0003, 0]
        self.assertEqual(run_pluck(p, out, [0, 1, 1, 1, 1, 1, 0]), [])
        self.assertEqual(p.phase, "rest")
        self.assertEqual(p.damped, 1)

    def test_a_touch_at_any_step_of_the_return_spoils_it(self):
        p = PluckDetector()
        self.assertEqual(run_pluck(p, [0, .003, .0015, .0006, .0002], [0, 1, 0, 1, 0]), [])

    def test_slow_return_is_not(self):
        slow = [0, .003] + [.0018] * 6 + [.0004]                 # back within 0.5 mm 0.35 s after it was 2 mm out
        self.assertEqual(run_pluck(PluckDetector(), slow, [0, 1] + [0] * 7), [])
        quick = [0, .003] + [.0018] * 5 + [.0004]                # back 0.3 s after
        self.assertEqual(run_pluck(PluckDetector(), quick, [0, 1] + [0] * 6), [7])

    def test_held_out_then_let_go(self):
        p = PluckDetector()
        held = [0] + [.004] * 40 + [.0012, .0001]                                         # 2 s held, then released
        self.assertEqual(run_pluck(p, held, [0] + [1] * 40 + [0, 0]), [42])
        self.assertAlmostEqual(p.last_peak, .004)

    def test_plucks_closer_than_0_3_s_count_once(self):
        p = PluckDetector()
        one = [.003, .0001]
        counted = run_pluck(p, one + one + one + [0] * 2 + one, [1, 0] * 3 + [0, 0] + [1, 0])
        # plucks at steps 1, 3 (too soon), 5 (4 steps after 1: too soon), 9 (8 after 1: counts)
        self.assertEqual(counted, [1, 9])
        self.assertEqual(p.ignored, 2)

    def test_brushing_without_displacement_and_resting_finger(self):
        p = PluckDetector()
        self.assertEqual(run_pluck(p, [0, .0003, 0, 0], [1, 1, 1, 0]), [])


class TunerScreen(unittest.TestCase):
    def test_blank_until_plucked(self):
        display = TunerDisplay()
        shown = display.update(0, False, -100.)
        self.assertFalse(shown["active"])
        self.assertEqual(shown["needle_deg"], L.NEEDLE_REST_DEG)
        self.assertFalse(any(shown["lamps"].values()))

    def test_reads_for_three_seconds_after_a_pluck(self):
        display = TunerDisplay()
        self.assertTrue(display.update(10, True, -100.)["active"])
        self.assertTrue(display.update(10 + 3 * HZ - 1, False, -100.)["active"])
        self.assertFalse(display.update(10 + 3 * HZ, False, -100.)["active"])
        self.assertTrue(display.update(10 + 3 * HZ + 5, True, -100.)["active"])          # a new pluck
        self.assertTrue(display.update(10 + 6 * HZ + 4, False, -100.)["active"])
        self.assertFalse(display.update(10 + 6 * HZ + 5, False, -100.)["active"])

    def test_live_while_reading(self):
        display = TunerDisplay()
        first = display.update(0, True, -100.)
        later = display.update(20, False, -96.)                                          # turned while it rings
        self.assertEqual(first["needle_deg"], 0.)
        self.assertAlmostEqual(later["needle_deg"], 4. * .9)

    def test_start_reads_f_sharp_and_green(self):
        shown = tuning.reading(-100.)
        self.assertEqual(shown["note"], "F#")
        self.assertTrue(shown["lamps"]["lamp_fs"] and shown["lamps"]["lamp_in_tune"])
        self.assertFalse(shown["lamps"]["lamp_g"])
        self.assertEqual(shown["needle_deg"], 0.)

    def test_needle_and_lamps(self):
        cases = {   # cents from G4: (note, needle degrees, in tune)
            -2.: ("G", -1.8, True), 3.: ("G", 2.7, True), 4.: ("G", 3.6, False), -60.: ("F#", 36., False),
            -40.: ("G", -36., False), 55.: ("G#", -40.5, False), 49.: ("G", 44.1, False), -150.: ("F#", -45., False),
        }
        for cents, (note, needle, green) in cases.items():
            shown = tuning.reading(cents)
            self.assertEqual(shown["note"], note, cents)
            self.assertAlmostEqual(shown["needle_deg"], needle, msg=cents)
            self.assertEqual(shown["lamps"]["lamp_in_tune"], green, cents)
            self.assertEqual(sum(shown["lamps"][k] for k in L.NOTE_LAMP.values()), 1)

    def test_beyond_the_three_notes(self):
        for cents, needle in ((-151., -45.), (-200., -45.), (151., 45.), (300., 45.)):
            shown = tuning.reading(cents)
            self.assertTrue(shown["active"])
            self.assertIsNone(shown["note"])
            self.assertEqual(shown["needle_deg"], needle)
            self.assertFalse(any(shown["lamps"].values()))

    def test_needle_scale(self):
        self.assertEqual(L.NEEDLE_MAX_DEG / L.NEEDLE_RANGE_CENTS, .9)                     # 50 cents -> 45 degrees


class FinalCheckRule(unittest.TestCase):
    def test_no_pluck_no_check(self):
        check = tuning.FinalCheck()
        self.assertFalse(check.update(1, False, 0.))
        self.assertIsNone(check.turned_since_pluck(0.))

    def test_a_pluck_stands_until_the_button_turns_past_the_free_play(self):
        check = tuning.FinalCheck()
        self.assertTrue(check.update(1, True, 100.))
        self.assertTrue(check.update(2, False, 100. + L.FREE_PLAY_DEG))          # within the free play: no movement
        self.assertTrue(check.update(3, False, 100. - L.FREE_PLAY_DEG))
        self.assertFalse(check.update(4, False, 100. + L.FREE_PLAY_DEG + .01))   # moved: the pluck came before it
        self.assertEqual(check.moved_tick, 4)
        self.assertFalse(check.update(5, False, 100.))                           # turning back does not undo it
        self.assertTrue(check.update(6, True, 100.))                             # a new pluck after the movement

    def test_with_the_pluck_rule_and_the_pitch(self):
        """Plucked at the start, turned up to pitch: no check; plucked again: check."""
        detector, check, a = PluckDetector(), tuning.FinalCheck(), Pitch("g")
        tick, checked = 0, None

        def step(angle_deg, displacement, touched):
            nonlocal tick, checked
            tick += 1
            a.update(tighten("g", angle_deg))
            checked = check.update(tick, detector.update(tick, displacement, touched), a.button)

        for d, t in ((-.003, True), (-.0002, False)):                            # pluck at the start (F#): pressed down
            step(0., d, t)
        self.assertTrue(checked)
        for k in range(1, 92):                                                   # 455 degrees in 5-degree steps
            step(5. * k, 0., False)
        self.assertAlmostEqual(a.cents, -100. + .22 * 455., places=6)
        self.assertFalse(checked)
        for d, t in ((-.003, True), (-.0002, False)):                            # the check pluck
            step(455., d, t)
        self.assertTrue(checked)


class DemoArithmetic(unittest.TestCase):
    def test_turn_for_an_error(self):
        half = L.FREE_PLAY_DEG / 2
        self.assertAlmostEqual(tuning.button_turn_for(-100., half), 100. / .22)          # engaged: straight on
        self.assertAlmostEqual(tuning.button_turn_for(10., half), -(L.FREE_PLAY_DEG + 10. / .22))   # reverse
        self.assertAlmostEqual(tuning.button_turn_for(10., -half), -10. / .22)
        self.assertAlmostEqual(tuning.button_turn_for(-1., 0.), half + 1. / .22)
        self.assertEqual(tuning.button_turn_for(0., 0.), 0.)

    def test_turning_by_the_rule_tunes_the_model(self):
        a = Pitch("g")
        angle = 0.
        for error in (-100.,):
            angle += tighten("g", tuning.button_turn_for(error, a.play))
            a.update(angle)
        self.assertAlmostEqual(a.cents, 0., places=9)
        angle += tighten("g", 30.)                                                         # overshoot 6.6 cents
        a.update(angle)
        angle += tighten("g", tuning.button_turn_for(a.cents, a.play))                    # back, through the play
        self.assertAlmostEqual(a.update(angle), 0., places=9)

    def test_roll_change(self):
        self.assertAlmostEqual(tuning.roll_change(90., 1, 1), math.pi / 2)
        self.assertAlmostEqual(tuning.roll_change(90., 1, -1), -math.pi / 2)
        self.assertAlmostEqual(tuning.roll_change(-45., -1, 1), math.pi / 4)

    def test_equivalent_roll(self):
        lo, hi = math.radians(-90.), math.radians(90.)
        self.assertAlmostEqual(tuning.equivalent_roll(math.radians(85.), lo, hi, +1), math.radians(85.))
        self.assertAlmostEqual(tuning.equivalent_roll(math.radians(89.), lo, hi, +1), math.radians(89.))
        self.assertAlmostEqual(tuning.equivalent_roll(math.radians(90.), lo, hi, +1), math.radians(-90.))
        self.assertAlmostEqual(tuning.equivalent_roll(math.radians(-90.), lo, hi, -1), math.radians(90.))
        self.assertAlmostEqual(tuning.equivalent_roll(math.radians(200.), lo, hi, +1), math.radians(20.))
        self.assertIsNone(tuning.equivalent_roll(0., math.radians(10.), math.radians(20.), 1))


if __name__ == "__main__":
    unittest.main()
