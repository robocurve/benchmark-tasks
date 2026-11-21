"""The scorer (grading.py): twelve checks, the 11/12 cap, the zero conditions, the end of the trial. Standard library
only."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from rdb_tlc_spotting import fixed_scene as F  # noqa: E402
from rdb_tlc_spotting import grading, tlc  # noqa: E402
from rdb_tlc_spotting import layout as L  # noqa: E402
from test_rules import IDENTITY, mul, standing_plate  # noqa: E402

AWAY = [(0., .3, .3), (0., -.3, .3)]
READY_POSES = {n: (p, q) for n, (p, q) in F.START.items()}


def finished_poses(from_wall=.032, lean=None, lid=True, tank=(L.TANK_X, L.TANK_Y, 0.)):
    lean = math.degrees(math.asin(from_wall / L.PLATE_LEN)) if lean is None else lean
    poses = dict(READY_POSES)
    poses["tank"] = (tank, IDENTITY)
    poses["plate"] = standing_plate(from_wall, lean, tank=tank)
    rest = (tank[0], tank[1], tank[2] + L.TANK[2] + L.LID[2] / 2)
    poses["lid"] = (rest, IDENTITY) if lid else F.START["lid"]
    return poses


def spot_all(grade, lanes=(0, 1, 2), steps=4, force=.3, samples=None):
    """Fill spotter k from vial k, then press it on tick k (the spotter is moved by setting its pose)."""
    need = round(L.FILL_S * L.CONTROL_HZ)
    plate = F.START["plate"]
    for k in lanes:
        sample = k if samples is None else samples[k]
        for _ in range(need + 1):
            poses = dict(READY_POSES)
            vx, vy, vz = L.vial_point(sample)
            poses[F.SPOTTERS[k]] = ((vx, vy, vz - .004 + L.CAP_LEN / 2), IDENTITY)
            grade.update(poses, [0., 0., 0.], [0., 0., 0.], AWAY)
        tick = tlc.to_world(plate, L.tick_point(k))
        for i in range(steps + 1):
            poses = dict(READY_POSES)
            poses[F.SPOTTERS[k]] = ((tick[0], tick[1], tick[2] + L.CAP_LEN / 2), IDENTITY)
            f = [0., 0., 0.]
            f[k] = force if i < steps else 0.
            grade.update(poses, f, [0., 0., 0.], AWAY)


class Scoring(unittest.TestCase):
    def test_start_scores_zero(self):
        result = grading.TLCGrade().update(READY_POSES, [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual(result["score"], 0.)
        self.assertIsNone(result["void"])
        self.assertEqual(result["racked"], [0, 1, 2])

    def test_full_marks(self):
        grade = grading.TLCGrade()
        spot_all(grade)
        result = grade.update(finished_poses(), [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual(result["points"], 12, result)
        self.assertEqual(result["score"], 1.)
        self.assertLessEqual(result["plate"]["lean_deg"], L.LEAN_MAX_DEG)

    def test_eleven_twelfths_without_the_spotters_back_or_with_a_hand_close(self):
        grade = grading.TLCGrade()
        spot_all(grade)
        poses = finished_poses()
        poses["spotter_1"] = ((.165, .19, .30), IDENTITY)                    # still held up in the air
        result = grade.update(poses, [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual((result["points"], result["score"]), (12, round(11 / 12, 4)))
        near = [AWAY[0], (L.TANK_X, L.TANK_Y, .14)]
        result = grade.update(finished_poses(), [0.] * 3, [0.] * 3, near)
        self.assertEqual(result["score"], round(11 / 12, 4))

    def test_plate_checks(self):
        grade = grading.TLCGrade()
        spot_all(grade)
        steep = grade.update(finished_poses(from_wall=.048), [0.] * 3, [0.] * 3, AWAY)    # 40 degrees
        self.assertTrue(steep["standing"])
        self.assertFalse(steep["leaning"])
        self.assertEqual(steep["points"], 11)
        open_ = grading.TLCGrade()
        spot_all(open_)
        no_cover = open_.update(finished_poses(lid=False), [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual(no_cover["points"], 11)
        face_in = grading.TLCGrade()
        spot_all(face_in)
        poses = finished_poses()
        p, q = poses["plate"]
        poses["plate"] = (p, mul(q, (1., 0., 0., 0.)))
        self.assertFalse(face_in.update(poses, [0.] * 3, [0.] * 3, AWAY)["leaning"])

    def test_lane_checks(self):
        grade = grading.TLCGrade()
        spot_all(grade, steps=14)                                            # 0.7 s: spots over 3 mm
        result = grade.update(finished_poses(), [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual(result["points"], 9)
        swapped = grading.TLCGrade()
        spot_all(swapped, samples=(1, 0, 2))                                  # spotters 1 and 2 filled from the wrong vials
        result = swapped.update(finished_poses(), [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual(result["points"], 3 + 3)

    def test_zero_conditions(self):
        grade = grading.TLCGrade()
        broken = grade.update(READY_POSES, [0.] * 3, [0., 3.5, 0.], AWAY)
        self.assertEqual(broken["score"], 0.)
        self.assertIn("broke", broken["void"])
        self.assertTrue(grade.finished)
        dropped = grading.TLCGrade()
        poses = dict(READY_POSES)
        poses["spotter_2"] = ((.30, -.05, .00075), mul((0., math.sqrt(.5), 0., math.sqrt(.5)), IDENTITY))
        self.assertIn("dropped", dropped.update(poses, [0.] * 3, [0.] * 3, AWAY)["void"])
        shoved = grading.TLCGrade()
        spot_all(shoved)
        result = shoved.update(finished_poses(tank=(L.TANK_X + .006, L.TANK_Y, 0.)), [0.] * 3, [0.] * 3, AWAY)
        self.assertEqual(result["score"], 0.)
        self.assertIn("tank", result["void"])
        # standing on its bottom edge the baseline only reaches 5 mm of water past 60 degrees, so the rule is checked
        # with 12 mm of water instead of 5
        saved = L.SOLVENT_Z
        try:
            L.SOLVENT_Z = L.TANK_FLOOR + .012
            drowned = grading.TLCGrade()
            self.assertIn("wash", drowned.update(finished_poses(), [0.] * 3, [0.] * 3, AWAY)["void"])
        finally:
            L.SOLVENT_Z = saved
        held = grading.TLCGrade()                                              # stood on its bottom edge: not dropped
        poses = dict(READY_POSES)
        poses["plate"] = ((.30, .02, L.PLATE_LEN / 2 + .0002), mul((0., -math.sqrt(.5), 0., math.sqrt(.5)), IDENTITY))
        self.assertIsNone(held.update(poses, [0.] * 3, [0.] * 3, AWAY)["void"])

    def test_a_plate_lying_face_up_is_not_a_zero(self):
        grade = grading.TLCGrade()
        spot_all(grade)
        poses = dict(READY_POSES)
        poses["plate"] = ((.30, -.02, L.PLATE_T / 2), IDENTITY)                 # put down flat, silica up, off the bar
        result = grade.update(poses, [0.] * 3, [0.] * 3, AWAY)
        self.assertIsNone(result["void"])
        self.assertEqual(result["points"], 9)                                    # the spots stand; no plate checks
        face_down = grading.TLCGrade()
        poses["plate"] = ((.30, -.02, L.PLATE_T / 2), (1., 0., 0., 0.))         # turned over: the spots on the bench
        self.assertIn("face down", face_down.update(poses, [0.] * 3, [0.] * 3, AWAY)["void"])
        floor = grading.TLCGrade()
        poses["plate"] = ((.30, -.45, -.30), IDENTITY)                          # off the bench, face up or not
        self.assertIn("fell off", floor.update(poses, [0.] * 3, [0.] * 3, AWAY)["void"])

    def test_a_spotter_past_60_degrees_is_dropped_only_when_not_held(self):
        flat = mul((0., math.sqrt(.5), 0., math.sqrt(.5)), IDENTITY)             # lying along x, 90 degrees over
        poses = dict(READY_POSES)
        poses["spotter_2"] = ((.20, .05, .12), flat)                             # in the air, its tip well above the bench
        for pads, void in (([0., 6., 0.], None), ([0., L.HELD_N + .1, 0.], None), ([0., .5, 0.], "spotter 2 dropped"),
                           (None, "spotter 2 dropped")):
            result = grading.TLCGrade().update(poses, [0.] * 3, [0.] * 3, AWAY, pads)
            self.assertEqual(result["void"], void, pads)
        tilted = mul((0., math.sin(math.radians(25.)), 0., math.cos(math.radians(25.))), IDENTITY)   # 50 degrees
        poses["spotter_2"] = ((.20, .05, .12), tilted)
        self.assertIsNone(grading.TLCGrade().update(poses, [0.] * 3, [0.] * 3, AWAY)["void"])
        poses["spotter_2"] = ((.30, -.05, .00075), flat)                        # lying on the bench, held or not
        self.assertIn("dropped", grading.TLCGrade().update(poses, [0.] * 3, [0.] * 3, AWAY, [0., 6., 0.])["void"])

    def test_trial_ends_one_second_after_full_marks(self):
        grade = grading.TLCGrade()
        spot_all(grade)
        for i in range(grading.END_TICKS):
            grade.update(finished_poses(), [0.] * 3, [0.] * 3, AWAY)
            self.assertFalse(grade.finished)
        grade.update(finished_poses(), [0.] * 3, [0.] * 3, AWAY)
        self.assertTrue(grade.finished)

    def test_possible_scores(self):
        self.assertEqual(grading.SCORES[0], 0.)
        self.assertEqual(grading.SCORES[-1], 1.)
        self.assertEqual(len(grading.SCORES), 13)


if __name__ == "__main__":
    unittest.main()
