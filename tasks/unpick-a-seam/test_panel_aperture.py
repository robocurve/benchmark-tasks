"""Physical-edge diagnostics distinguish a hole from compressed/sliding panels."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import Mesh
from facet_cloth import mechanics,measured_surface,geometric_edge_measurements,ShellMaterial


def pair():
    mesh=Mesh(np.array(((0.,0.,0.),(.01,0.,0.),(.02,0.,0.),
                        (0.,.01,0.),(.01,.01,0.),(.02,.01,0.))),
              np.array(((0,1,4),(0,4,3),(1,2,5),(1,5,4))))
    model=mechanics(mesh,primitive_boxes=True)
    return dict(mesh=mesh,model=model,material=ShellMaterial())


class ApertureTests(unittest.TestCase):
    def geometry(self,shift):
        model=pair()['model'];world=model['points'].copy();world[0]+=shift
        return geometric_edge_measurements(model,world,np.tile(np.eye(3),(2,1,1)))

    def test_compression_is_not_a_hole(self):
        row=self.geometry((.00012,0,0))
        self.assertAlmostEqual(row['maximum_empty_aperture_m'],0.)
        self.assertAlmostEqual(row['maximum_across_edge_compression_m'],.00012)

    def test_opening_is_retained(self):
        row=self.geometry((-.00012,0,0))
        self.assertAlmostEqual(row['maximum_empty_aperture_m'],.00012)
        self.assertAlmostEqual(row['maximum_across_edge_compression_m'],0.)

    def test_clearance_uses_actual_400micron_material_thickness(self):
        self.assertAlmostEqual(self.geometry((0,0,.00039))['maximum_empty_normal_clearance_m'],0.)
        self.assertAlmostEqual(self.geometry((0,0,.00045))['maximum_empty_normal_clearance_m'],.00005)

    def test_slip_and_overlap_are_independent(self):
        row=self.geometry((0,.002,0))
        self.assertAlmostEqual(row['maximum_empty_aperture_m'],0.)
        self.assertAlmostEqual(row['maximum_along_edge_slip_m'],.002)
        self.assertAlmostEqual(row['minimum_shared_edge_overlap_m'],.008)
        self.assertEqual(row['edges_without_overlap'],0)

    def test_rotated_geometry_is_invariant(self):
        model=pair()['model'];world=model['points'].copy();world[0, :, 0]-=.00012
        r=Rotation.from_rotvec((.6,-.9,.2)).as_matrix()
        row=geometric_edge_measurements(model,world@r.T+(.1,-.3,.4),np.tile(r,(2,1,1)))
        self.assertAlmostEqual(row['maximum_empty_aperture_m'],.00012)
        self.assertAlmostEqual(row['maximum_empty_normal_clearance_m'],0.)

    def test_diagnostic_preserves_nodes_and_legacy_warning(self):
        shell=pair();poses=np.c_[shell['model']['centres'],np.tile((0.,0.,0.,1.),(2,1))]
        poses[0,0]+=.0003;before=poses.copy()
        old_nodes,old=measured_surface(shell,poses)
        new_nodes,new=measured_surface(shell,poses,geometry_aperture_m=.0002)
        self.assertFalse(old['valid']);self.assertTrue(new['valid'])
        self.assertTrue(new['legacy_endpoint_warning'])
        self.assertAlmostEqual(new['geometry']['maximum_empty_aperture_m'],0.)
        np.testing.assert_array_equal(old_nodes,new_nodes)
        np.testing.assert_array_equal(before,poses)

    def test_actual_opening_and_lost_overlap_stop(self):
        shell=pair();poses=np.c_[shell['model']['centres'],np.tile((0.,0.,0.,1.),(2,1))]
        poses[0,0]-=.00021
        self.assertFalse(measured_surface(shell,poses,geometry_aperture_m=.0002)[1]['valid'])
        poses[0,0]+=.00021;poses[0,1]+=.011
        row=measured_surface(shell,poses,geometry_aperture_m=.0002)[1]
        self.assertFalse(row['valid']);self.assertEqual(row['geometry']['edges_without_overlap'],1)

    def test_diagonal_empty_aperture_retains_both_components(self):
        row=self.geometry((-.00015,0,.00055))
        self.assertAlmostEqual(row['maximum_empty_aperture_m'],np.hypot(.00015,.00015))

    def test_practical_panel_slip_still_has_a_bounded_junction(self):
        shell=pair();poses=np.c_[shell['model']['centres'],np.tile((0.,0.,0.,1.),(2,1))]
        poses[0,1]+=.002
        self.assertTrue(measured_surface(shell,poses,geometry_aperture_m=.005,
            require_edge_overlap=False)[1]['valid'])
        poses[0,1]+=.004
        self.assertFalse(measured_surface(shell,poses,geometry_aperture_m=.005,
            require_edge_overlap=False)[1]['valid'])

    def test_passage_envelope_retains_warning_and_rejects_larger_hole(self):
        shell=pair();poses=np.c_[shell['model']['centres'],np.tile((0.,0.,0.,1.),(2,1))]
        poses[0,0]-=.00025
        _,warning=measured_surface(shell,poses,geometry_aperture_m=.0004)
        self.assertTrue(warning['valid']);self.assertTrue(warning['aperture_warning'])
        self.assertAlmostEqual(warning['geometry']['maximum_empty_aperture_m'],.00025)
        poses[0,0]-=.00016
        self.assertFalse(measured_surface(shell,poses,geometry_aperture_m=.0004)[1]['valid'])

    def test_invalid_numeric_domain_rejected(self):
        model=pair()['model'];rot=np.tile(np.eye(3),(2,1,1))
        for thickness in (0,-.1,float('nan')):
            with self.assertRaises(ValueError):
                geometric_edge_measurements(model,model['points'],rot,thickness)
        poses=np.c_[model['centres'],np.tile((0.,0.,0.,1.),(2,1))]
        for limit in (0,-.1,float('nan')):
            with self.assertRaises(ValueError):
                measured_surface(pair(),poses,geometry_aperture_m=limit)


if __name__=='__main__':unittest.main()
