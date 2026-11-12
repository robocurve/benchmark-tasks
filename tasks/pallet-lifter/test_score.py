"""CPU scorer regressions. Synthetic states are not a physical trajectory replay."""

from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from pallet_lifter.scoring import History, RAIL_HEIGHT, evaluate

TOP = .7475
PALLET = '/World/envs/env_0/pallet'
LIFTER = '/World/envs/env_0/lifter'


def state(xy, bottom, size, still=True, quat=(0, 0, 0, 1)):
    """Build a state in the runtime's world AABB-centre convention."""
    size = np.array(size, dtype=float)
    pos = np.array((*xy, TOP + bottom + size[2] / 2))
    low, high = pos - size / 2, pos + size / 2
    # Specify the bottom directly, including strict-boundary test cases.
    low[2], high[2] = TOP + bottom, TOP + bottom + size[2]
    return SimpleNamespace(pos=pos, low=low, high=high,
                           resting=lambda: still,
                           quat_xyzw=np.array(quat, dtype=float))


def pallet(xy=(.37, -.12), bottom=.015, **kwargs):
    # Actual asset bounds, including the non-colliding face grooves.
    return state(xy, bottom, (.12, .16, .0384), **kwargs)


def objects():
    return {'pallet': pallet((.40, .08), bottom=0),
            'lifter': state((.2575, .28), .035, (.305, .08, .025))}


def transfer(clearance=.020, contact=True):
    """Source -> lift -> destination -> settle; no injected lift-history flag."""
    o, h = objects(), History()
    h.update(o, TOP)
    if contact:
        h.contact(LIFTER, PALLET)
    for xy, bottom, still in (((.40, .08), clearance, False),
                              ((.37, -.12), clearance, False),
                              ((.37, -.12), .015, True)):
        o['pallet'] = pallet(xy, bottom, still=still)
        h.update(o, TOP)
    return o, h


class Scores(unittest.TestCase):
    def test_low_clearance_transfer_scores_without_60_mm_lift(self):
        o, h = transfer(clearance=.020)
        self.assertEqual(evaluate(o, h, TOP), .75)
        self.assertTrue(h.tool_contact)
        self.assertTrue(h.lifted)
        self.assertFalse(h.invalid)
        h.open_clear = True
        self.assertEqual(evaluate(o, h, TOP), 1.)

    def test_seating_does_not_require_any_historical_clearance_gate(self):
        # A sampled history may miss the clearance peak. Include both sides
        # of the existing +/-4 mm seating tolerance, then update before scoring.
        for bottom in (.0111, .015, .0189):
            with self.subTest(bottom=bottom):
                o, h = transfer(clearance=.015)
                self.assertFalse(h.lifted)
                o['pallet'] = pallet(bottom=bottom)
                h.update(o, TOP)
                if bottom <= .015:
                    self.assertFalse(h.lifted)
                self.assertEqual(evaluate(o, h, TOP), .75)
                h.open_clear = True
                self.assertEqual(evaluate(o, h, TOP), 1.)

    def test_existing_high_lift_transfer_still_scores(self):
        o, h = transfer(clearance=.080)
        h.open_clear = True
        self.assertEqual(evaluate(o, h, TOP), 1.)

    def test_contact_without_lift_or_transfer_scores_zero(self):
        o, h = objects(), History(open_clear=True)
        h.contact(LIFTER, PALLET)
        h.update(o, TOP)
        self.assertFalse(h.lifted)
        self.assertEqual(evaluate(o, h, TOP), 0.)

    def test_no_contact_transfer_or_staging_scores_zero(self):
        o, h = transfer(contact=False)
        h.open_clear = True
        self.assertFalse(h.lifted)
        self.assertEqual(evaluate(o, h, TOP), 0.)
        # A stale diagnostic flag cannot replace contact proof.
        h.lifted = True
        self.assertEqual(evaluate(o, h, TOP), 0.)

    def test_airborne_partial_credit_uses_rail_collision_height(self):
        for bottom, expected in ((.0149, 0.), (.015, 0.),
                                 (.0151, .25), (.020, .25), (.080, .25)):
            with self.subTest(bottom=bottom):
                o, h = objects(), History()
                h.contact(LIFTER, PALLET)
                o['pallet'] = pallet((.40, .08), bottom, still=False)
                h.update(o, TOP)
                self.assertEqual(evaluate(o, h, TOP), expected)

    def test_lift_before_tool_contact_does_not_count(self):
        o, h = objects(), History()
        o['pallet'] = pallet((.40, .08), .080, still=False)
        h.update(o, TOP)
        self.assertFalse(h.lifted)
        self.assertEqual(evaluate(o, h, TOP), 0.)
        h.contact(LIFTER, PALLET)
        o['pallet'] = pallet((.40, .08), 0)
        h.update(o, TOP)
        self.assertFalse(h.lifted)
        self.assertEqual(evaluate(o, h, TOP), 0.)

    def test_lift_credit_is_not_a_retained_maximum(self):
        o, h = transfer()
        o['pallet'] = pallet((.40, .08), 0)
        h.update(o, TOP)
        self.assertTrue(h.lifted)
        self.assertEqual(evaluate(o, h, TOP), 0.)

    def test_each_seating_condition_still_required(self):
        yaw = np.radians(9) / 2
        cases = {
            'xy too far': pallet((.383, -.12)),
            'too low': pallet(bottom=.0109),
            'too high': pallet(bottom=.0191),
            'yaw too large': pallet(quat=(0, 0, np.sin(yaw), np.cos(yaw))),
            'moving': pallet(still=False),
            'too tall': state((.37, -.12), .015, (.12, .16, .043)),
        }
        for name, s in cases.items():
            with self.subTest(condition=name):
                o, h = transfer()
                h.open_clear = True
                o['pallet'] = s
                h.update(o, TOP)
                # A nonseated state can only get airborne partial credit.
                expected = .25 if s.low[2] > TOP + .015 else 0.
                self.assertEqual(evaluate(o, h, TOP), expected)

    def test_tool_parking_and_open_clear_remain_required_for_full_score(self):
        o, h = transfer()
        self.assertEqual(evaluate(o, h, TOP), .75)
        h.open_clear = True
        for name, s in {
            'wrong xy': state((.30, .28), .035, (.305, .08, .025)),
            'wrong height': state((.2575, .28), .042, (.305, .08, .025)),
            'not flat': state((.2575, .28), .035, (.305, .08, .036)),
            'moving': state((.2575, .28), .035, (.305, .08, .025), still=False),
        }.items():
            with self.subTest(condition=name):
                o['lifter'] = s
                h.update(o, TOP)
                self.assertEqual(evaluate(o, h, TOP), .75)

    def test_robot_contact_permanently_invalidates_even_after_success(self):
        for robot in ('left_robot/finger', 'right_robot/finger',
                      'LeftArm/link_6', 'RightArm/link_6'):
            for reverse in (False, True):
                with self.subTest(robot=robot, reverse=reverse):
                    o, h = transfer()
                    h.open_clear = True
                    self.assertEqual(evaluate(o, h, TOP), 1.)
                    pair = (PALLET, '/World/envs/env_0/' + robot)
                    h.contact(*(pair[::-1] if reverse else pair))
                    self.assertTrue(h.invalid)
                    h.update(o, TOP)
                    self.assertEqual(evaluate(o, h, TOP), 0.)
                    h.contact(LIFTER, PALLET)
                    h.update(o, TOP)
                    self.assertEqual(evaluate(o, h, TOP), 0.)

    def test_drop_or_unsafe_state_permanently_invalidates(self):
        for key in ('pallet', 'lifter'):
            for failure in ('drop', 'nonfinite', 'x low', 'x high', 'y outside'):
                with self.subTest(object=key, failure=failure):
                    o, h = transfer()
                    safe = o[key]
                    s = state((.37, -.12), .015, (.12, .16, .0384))
                    if failure == 'drop':
                        s.low[2] = TOP - .0201
                    elif failure == 'nonfinite':
                        s.pos[0] = np.nan
                    elif failure == 'x low':
                        s.pos[0] = -.101
                    elif failure == 'x high':
                        s.pos[0] = .951
                    else:
                        s.pos[1] = .621
                    o[key] = s
                    h.update(o, TOP)
                    self.assertTrue(h.invalid)
                    self.assertEqual(evaluate(o, h, TOP), 0.)
                    o[key] = safe
                    h.update(o, TOP)
                    h.open_clear = True
                    self.assertEqual(evaluate(o, h, TOP), 0.)

    def test_contact_paths_include_children_and_ignore_unrelated_pairs(self):
        h = History()
        h.contact(LIFTER, '/World/table')
        self.assertFalse(h.tool_contact)
        h.contact(PALLET + '/foot_0', LIFTER + '/tine_0')
        self.assertTrue(h.tool_contact)
        self.assertFalse(h.invalid)


class CollisionGeometry(unittest.TestCase):
    def test_rail_clearance_matches_checked_in_collision_geometry(self):
        # usd-core only: inspect the actual collision boxes without Isaac Sim.
        from pxr import Usd, UsdGeom, UsdPhysics

        assets = Path(__file__).parent / 'assets'
        stage = Usd.Stage.Open(str(assets / 'rests.usdc'))
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
        self.assertEqual({p.GetName() for p in colliders}, {'rail_0', 'rail_1'})
        for prim in colliders:
            bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            self.assertAlmostEqual(bounds.GetMax()[2], RAIL_HEIGHT, places=8)
        # The 16.6 mm visual AABB is not the collision clearance height.
        visual = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
        self.assertAlmostEqual(visual.GetMax()[2], .0166, places=8)

        stage = Usd.Stage.Open(str(assets / 'pallet.usdc'))
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        feet = [p for p in stage.Traverse() if p.GetName().startswith('foot_')]
        self.assertEqual(len(feet), 4)
        for prim in feet:
            self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI))
            bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            self.assertAlmostEqual(bounds.GetMin()[2], -.023, places=8)


if __name__ == '__main__':
    unittest.main()
