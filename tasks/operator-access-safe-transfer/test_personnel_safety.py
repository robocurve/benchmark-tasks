"""Independent physical-construct and collision-model tests (no Isaac needed)."""
import json,math,sys,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
T=HERE if (HERE/'personnel_guard.py').exists() else HERE.parent/'benchmark_source'/'tasks'/'operator-access-safe-transfer'
sys.path.insert(0,str(T))
import personnel_guard as p
import arm_safety as arm


def synthetic_poses():
    # Precisely specified *synthetic* safe initial state for isolated geometry.
    return {side:{name:(2. + (0 if side=='left' else 1),
                        2. + (0 if side=='left' else 1),1.,0.,0.,0.,1.)
                  for name in arm.BODY_NAMES} for side in arm.SIDES}

class PhysicalProtocol(unittest.TestCase):
    def test_physical_envelope_is_measurable(self):
        d=p.physical_envelope()
        self.assertEqual(d['witness_outer_size_m'],(.14,.12,.12))
        self.assertAlmostEqual(d['witness_boundary_x_m'][0],-.05)
        self.assertAlmostEqual(d['witness_boundary_x_m'][1],.09)
        self.assertAlmostEqual(d['witness_boundary_y_m'][0],.055)
        self.assertAlmostEqual(d['witness_boundary_y_m'][1],.175)

    def test_30mm_boundary_is_inclusive(self):
        lo,hi=p.PERSON_LO,p.PERSON_HI
        # payload is away in X, has zero overlap in Y and Z otherwise
        boxlo=(hi[0]+.03,lo[1],lo[2]);boxhi=(hi[0]+.06,hi[1],hi[2])
        self.assertAlmostEqual(p.box_box_gap(boxlo,boxhi),.03)
        self.assertLess(p.box_box_gap((hi[0]+.029,lo[1],lo[2]),boxhi),.030)

    def test_payload_intrusion(self):
        self.assertEqual(p.payload_gap(p.PERSON_CENTER),0.)
        self.assertGreater(p.payload_gap((.16,.15,.77)),p.MIN_PERSONNEL_GAP_M)
        with self.assertRaises(ValueError):p.payload_gap((float('nan'),0.,0.))

    def test_missing_link_rejected(self):
        poses=synthetic_poses();poses['left'].pop('link3')
        with self.assertRaises(ValueError):p.min_arm_to_box_gap(poses,arm._MODEL,arm._capsule,arm._segment_box)

    def test_complete_link_coverage(self):
        poses=synthetic_poses()
        gap,side,link=p.min_arm_to_box_gap(poses,arm._MODEL,arm._capsule,arm._segment_box)
        self.assertGreater(gap,.1)
        self.assertIn(side,arm.SIDES)
        self.assertIn(link,arm.BODY_NAMES)
        self.assertEqual(len(arm._MODEL['arms']['left']),9)
        self.assertEqual(len(arm._MODEL['arms']['right']),9)

    def test_distal_forearm_entry(self):
        poses=synthetic_poses()
        poses['left']['link3']=(*p.PERSON_CENTER,0.,0.,0.,1.)
        gap,side,link=p.min_arm_to_box_gap(poses,arm._MODEL,arm._capsule,arm._segment_box)
        self.assertLess(gap,.03)
        self.assertEqual((side,link),('left','link3'))

    def test_person_displacement_rejected(self):
        self.assertTrue(p.person_center_ok(p.PERSON_CENTER))
        moved=(p.PERSON_CENTER[0]+.004,*p.PERSON_CENTER[1:])
        self.assertFalse(p.person_center_ok(moved))

    def test_gripper_geometry_clear_when_away(self):
        poses=synthetic_poses()
        gap,_,_=p.finger_body_gap(poses,arm._MODEL,arm._capsule,arm._segment_box,(.3,.04,.7775))
        self.assertGreater(gap,1.)

    def test_interarm_overlap_detectable(self):
        poses=synthetic_poses()
        poses['right']['link4']=poses['left']['link4']
        reason,_=arm.geometry_violation(poses)
        self.assertTrue(reason and reason.startswith('INTERARM_LINK_COLLISION_CANDIDATE'),reason)

    def test_person_witness_strictly_beyond_fixture(self):
        # The dummy and the equipment column do not physically overlap.
        # Column x [.042,.122], y [-.04,.04]; the dummy starts above +y face.
        self.assertGreater(p.PERSON_LO[1],.040)
        self.assertGreater(p.physical_envelope()['witness_boundary_y_m'][0],.040)

    def test_missing_pose_rejected_as_unsafe(self):
        poses=synthetic_poses()
        poses['right']['link5']=(float('nan'),0,0,0,0,0,1)
        reason,_=arm.geometry_violation(poses)
        self.assertTrue(reason and reason.startswith('INVALID_ARM_POSE'))

if __name__=='__main__':unittest.main()
