import math
import unittest

import geometry as geo
from scoring import PITCH


class SaddleFitTests(unittest.TestCase):
    def test_crown_reaches_both_legs_and_the_published_height(self):
        points = [v for sector in geo.harp_crown() for v in sector]
        for sign in (-1,1):
            self.assertTrue(any(abs(y-sign*geo.LEG_Y)<.002 and
                                abs(z-(geo.HARP_HEIGHT-.04375))<1e-6 for _,y,z in points))
        self.assertAlmostEqual(max(v[2] for v in points)+geo.SADDLE_PLATE,
                               geo.ASSEMBLED_HARP_HEIGHT)

    def test_squeezed_leg_can_enter_ordinary_open_side_of_notch(self):
        inserted_y = geo.LEG_Y-.0095
        toe_outer = inserted_y+geo.TOE_OUT
        self.assertLess(toe_outer,geo.LIP_INNER)
        self.assertGreater(geo.LEG_Y+geo.TOE_OUT-geo.LIP_INNER,.0012)

    def test_notch_neck_clears_the_roof_after_spring_return(self):
        for s in (-1,1):
            _,_,(pos,size,_),(toe,toe_size,_) = geo.leg_boxes(s)
            outer = geo.LEG_Y+s*pos[1]+size[1]/2
            self.assertLess(outer,geo.LIP_INNER)
            self.assertLess(s*toe[1]-toe_size[1]/2,s*pos[1]-size[1]/2)
            self.assertGreater(s*toe[1]+toe_size[1]/2,s*pos[1]+size[1]/2)

    def test_sleeve_guide_throat_clears_the_wire_section(self):
        self.assertGreater(geo.COLLAR_THROAT*math.cos(math.pi/16),math.hypot(.00175,.00175))

    def test_formed_foot_keeps_the_sleeve_on_the_wire(self):
        for s in (-1,1):
            _,(shoulder,size,_),_,_ = geo.leg_boxes(s)
            self.assertGreater(abs(shoulder[1])+size[1]/2,geo.COLLAR_THROAT)

    def test_cheek_bevel_opens_at_entry_and_keeps_the_seated_gap(self):
        positive = geo.saddle_entrances()[-1]
        opening = min(x for x,y,_ in positive if y == .035)
        throat = min(x for x,y,_ in positive if y == .038)
        self.assertGreater(opening,throat)
        self.assertAlmostEqual(2*throat,.006)

    def test_sleeve_mouth_bevel_ends_below_the_retaining_height(self):
        sleeve = geo.sleeve_body()[0]
        self.assertGreater(math.hypot(*sleeve[0][:2]),math.hypot(*sleeve[3][:2]))
        top_of_bevel = geo.COLLAR_Z-geo.COLLAR_LENGTH/2+geo.COLLAR_BEVEL
        self.assertLess(top_of_bevel,geo.LIP_Z[0])

    def test_band_covers_the_actual_shoulder_only_when_cups_are_low(self):
        self.assertGreater(geo.sleeve_blocking_band(.00755,.02952),.002)
        self.assertGreater(geo.sleeve_blocking_band(.00914,.03114),.002)
        self.assertEqual(geo.sleeve_blocking_band(.04446,.06646),0.)

    def test_down_sleeve_blocks_full_notch_disengagement(self):
        sidewall_outer = .0475
        # Wing corner must fit in the sleeve centered on a free leg.
        self.assertLess(math.hypot(.003,sidewall_outer-geo.LEG_Y),
                        geo.COLLAR_BORE*math.cos(math.pi/16))
        first_interference = sidewall_outer-geo.COLLAR_BORE
        full_disengagement = geo.LIP_INNER-geo.TOE_OUT
        self.assertGreater(first_interference,full_disengagement)
        lo = geo.COLLAR_Z-geo.COLLAR_LENGTH/2
        self.assertLess(lo,geo.LIP_Z[0])
        self.assertGreater(lo+geo.COLLAR_LENGTH,geo.LIP_Z[1])
        self.assertGreater(lo+geo.CARRY_SLEEVE_RISE,geo.LIP_Z[1])

    def test_finial_has_a_clear_thread_entry(self):
        bore = geo.finial_body()[0]
        inner_at_mouth = math.hypot(*bore[0][:2])
        inner_above_mouth = math.hypot(*bore[3][:2])
        self.assertGreater(inner_at_mouth,inner_above_mouth)
        self.assertGreater(inner_above_mouth,geo.POST_ROOT_RADIUS)

    def test_finial_thread_has_real_pitch_and_complementary_phase(self):
        male,female = geo.threads(False),geo.threads(True)
        self.assertAlmostEqual(male[20][1][2]-male[0][1][2],PITCH)
        self.assertAlmostEqual(female[20][0][2]-female[0][0][2],PITCH)
        self.assertAlmostEqual(female[0][0][2]+.0016-male[0][1][2],PITCH/2)
        self.assertLess(math.hypot(*female[0][0][:2]),math.hypot(*male[0][1][:2]))


if __name__ == "__main__":
    unittest.main()
