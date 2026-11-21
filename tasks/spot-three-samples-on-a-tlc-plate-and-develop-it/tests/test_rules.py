"""The stated rules (tlc.py): filling, spotting, gouging, the plate in the developing tank, its lid, the rack.
Standard library only."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_tlc_spotting import fixed_scene as F  # noqa: E402
from rdb_tlc_spotting import layout as L  # noqa: E402
from rdb_tlc_spotting import tlc  # noqa: E402

IDENTITY = (0., 0., 0., 1.)
HANG = (0., -math.sqrt(.5), 0., math.sqrt(.5))


def quat_y(angle):
    return (0., math.sin(angle / 2), 0., math.cos(angle / 2))


def mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def standing_plate(from_wall, lean_deg, tank=(L.TANK_X, L.TANK_Y, 0.)):
    """A plate with its bottom edge on the tank's floor ``from_wall`` in front of the back wall, leaning back."""
    q = mul(quat_y(math.radians(lean_deg)), HANG)
    bottom = (tank[0] + L.TANK_IN[0] / 2 - from_wall, tank[1], tank[2] + L.TANK_FLOOR)
    centre = tlc.to_world((bottom, q), (L.PLATE_LEN / 2, 0., 0.))
    low = min(c[2] for c in tlc.plate_corners((centre, q)))
    return (centre[0], centre[1], centre[2] + (tank[2] + L.TANK_FLOOR - low)), q


class Poses(unittest.TestCase):
    def test_quaternion_round_trip(self):
        pose = ((.1, .2, .3), mul(quat_y(.4), (0., 0., math.sin(.3), math.cos(.3))))
        p = (.05, -.02, .07)
        back = tlc.to_local(pose, tlc.to_world(pose, p))
        self.assertTrue(all(abs(a - b) < 1e-12 for a, b in zip(back, p)))

    def test_spotter_tip_and_rack(self):
        for k, name in enumerate(F.SPOTTERS):
            tip = tlc.spotter_tip(F.START[name])
            self.assertAlmostEqual(tip[2], L.hole_point(k)[2])
            self.assertEqual(tlc.in_rack(F.START[name]), k)
        lifted = ((L.RACK_X, L.HOLE_Y[0], .2), IDENTITY)
        self.assertIsNone(tlc.in_rack(lifted))


class Filling(unittest.TestCase):
    def tip(self, j, depth):
        return (L.VIAL_X, L.VIAL_Y[j], L.LIQUID_Z - depth)

    def test_fills_after_dip_time_only_when_deep_enough(self):
        loads = tlc.Loads()
        need = round(L.FILL_S * L.CONTROL_HZ)
        away = (0., 0., .2)
        for _ in range(need - 1):
            loads.update([self.tip(0, .003), away, away])
        self.assertIsNone(loads.sample(0))
        loads.update([self.tip(0, .003), away, away])
        self.assertEqual(loads.sample(0), 0)
        shallow = tlc.Loads()
        for _ in range(3 * need):
            shallow.update([away, self.tip(1, .001), away])            # wet, but less than 2 mm under
        self.assertIsNone(shallow.sample(1))

    def test_a_spotter_wetted_in_two_samples_is_mixed(self):
        loads = tlc.Loads()
        need = round(L.FILL_S * L.CONTROL_HZ)
        away = (0., 0., .2)
        for _ in range(need):
            loads.update([self.tip(0, .004), away, away])
        loads.update([self.tip(1, .0005), away, away])
        self.assertEqual(loads.sample(0), "mixed")
        self.assertIn(1, loads.mixed_vials)


class Spotting(unittest.TestCase):
    def press(self, spots, k, force, steps, at=None, sample=None):
        tick = L.tick_point(k) if at is None else at
        for _ in range(steps):
            spots.update(k, force, (tick[0], tick[1], L.PLATE_T / 2), k if sample is None else sample)

    def test_no_spot_without_contact_force_or_dwell(self):
        spots = tlc.Spots()
        self.press(spots, 0, .03, 10)                                    # under the band: a brush
        self.press(spots, 0, 0., 1)
        self.press(spots, 0, .3, round(L.DWELL_S * L.CONTROL_HZ) - 1)    # too short
        self.press(spots, 0, 0., 1)
        self.assertEqual(spots.spots, [])

    def test_spot_grows_with_contact_time(self):
        spots = tlc.Spots()
        self.press(spots, 0, .3, 4)
        lane = spots.lane(0)
        self.assertTrue(lane["covered"] and lane["small"] and lane["clean"])
        self.assertAlmostEqual(lane["diameter_mm"], round(1000 * L.spot_diameter(.2), 2))
        self.press(spots, 0, .3, 10)
        lane = spots.lane(0)
        self.assertFalse(lane["small"])                                  # 0.7 s in contact: over 3 mm
        self.assertGreater(lane["diameter_mm"], 3.)

    def test_second_touch_on_the_same_place_grows_the_spot(self):
        spots = tlc.Spots()
        self.press(spots, 1, .3, 4)
        self.press(spots, 1, 0., 1)
        self.press(spots, 1, .3, 4)
        self.assertEqual(len(spots.spots), 1)
        self.assertAlmostEqual(spots.spots[0]["seconds"], .4)

    def test_wrong_sample_or_two_spots_fail_the_lane(self):
        spots = tlc.Spots()
        self.press(spots, 0, .3, 4, sample=1)
        self.assertFalse(spots.lane(0)["covered"])
        spots = tlc.Spots()
        tx, ty, _ = L.tick_point(2)
        self.press(spots, 2, .3, 4)
        self.press(spots, 2, 0., 1)
        self.press(spots, 2, .3, 4, at=(tx + .002, ty, 0.))
        self.assertEqual(spots.lane(2)["spots"], 2)
        self.assertFalse(spots.lane(2)["covered"])

    def test_spot_off_its_tick_does_not_cover_it(self):
        spots = tlc.Spots()
        tx, ty, _ = L.tick_point(0)
        self.press(spots, 0, .3, 4, at=(tx + .0015, ty, 0.))            # 2.2 mm spot, 1.5 mm off
        self.assertFalse(spots.lane(0)["covered"])

    def test_pressing_too_hard_or_sliding_gouges(self):
        spots = tlc.Spots()
        self.press(spots, 0, 1.5, 4)
        lane = spots.lane(0)
        self.assertTrue(lane["gouged"])
        self.assertFalse(lane["clean"])
        spots = tlc.Spots()
        tx, ty, _ = L.tick_point(1)
        for i in range(6):
            spots.update(1, .3, (tx + .0004 * i, ty, L.PLATE_T / 2), 1)
        self.assertTrue(spots.lane(1)["gouged"])

    def test_empty_spotter_leaves_nothing(self):
        spots = tlc.Spots()
        tick = L.tick_point(0)
        for _ in range(6):
            spots.update(0, .3, (tick[0], tick[1], L.PLATE_T / 2), None)
        self.assertEqual(spots.spots, [])


class Tank(unittest.TestCase):
    def test_plate_leaning_on_the_back_wall(self):
        lean = math.degrees(math.asin(.032 / L.PLATE_LEN))
        state = tlc.plate_in_tank(standing_plate(.032, lean), F.START["tank"])
        self.assertTrue(state["inside"] and state["bottom_wet"] and state["face_out"] and state["right_way_up"])
        self.assertTrue(state["on_wall"])
        self.assertLess(abs(state["lean_deg"] - lean), .01)
        self.assertGreater(state["baseline_above_mm"], 1000 * L.BASELINE_CLEAR)

    def test_standing_free_is_not_on_the_wall(self):
        state = tlc.plate_in_tank(standing_plate(.050, 3.), F.START["tank"])
        self.assertFalse(state["on_wall"])

    def test_wrong_way_round_or_upside_down(self):
        p, q = standing_plate(.032, 25.)
        face_in = (p, mul(q, (1., 0., 0., 0.)))                           # turned over about its own length
        self.assertFalse(tlc.plate_in_tank(face_in, F.START["tank"])["face_out"])
        upside = (p, mul(q, (0., 1., 0., 0.)))                            # turned end over end
        self.assertFalse(tlc.plate_in_tank(upside, F.START["tank"])["right_way_up"])

    def test_lid_on_and_off(self):
        rest = (L.TANK_X, L.TANK_Y, L.TANK[2] + L.LID[2] / 2)
        self.assertTrue(tlc.lid_on((rest, IDENTITY), F.START["tank"])["on"])
        self.assertFalse(tlc.lid_on(((rest[0] + .01, rest[1], rest[2]), IDENTITY), F.START["tank"])["on"])
        turned = (rest, (0., 0., math.sin(math.radians(10)), math.cos(math.radians(10))))      # 20 degrees askew
        self.assertFalse(tlc.lid_on(turned, F.START["tank"])["on"])
        self.assertFalse(tlc.lid_on(F.START["lid"], F.START["tank"])["on"])


if __name__ == "__main__":
    unittest.main()
