import unittest
import numpy as np
from geometry import audit_scene,patch,stitch_loop
from cutting import ThreadCutter,DamageAudit,segment_triangle_intersections
from scoring import assess
from inspection import point_triangle_distance,inspect_removed_threads,StrainAudit


class MechanismTests(unittest.TestCase):
    def test_cloth_is_continuous_with_real_holes(self):
        r=audit_scene()
        self.assertEqual([s['boundary_loops'] for s in r['cloth']],[13,13])
        for s in r['stitches']:self.assertGreater(s['minimum_link_m'],.0004)

    def test_needle_path_passes_hole_not_material(self):
        m=patch()
        hole=segment_triangle_intersections([.295,-.030,.74],[.295,-.030,.76],m.rest,m.faces)
        cloth=segment_triangle_intersections([.300,-.030,.74],[.300,-.030,.76],m.rest,m.faces)
        self.assertEqual(len(hole),0)
        self.assertGreater(len(cloth),0)

    def test_proximity_and_wrong_force_direction_cannot_cut(self):
        c=ThreadCutter([28]*6)
        f=np.zeros((168,3))
        for _ in range(100):self.assertEqual(c.update(f,[1,0,0],1/240),[])
        f[0,0]=1.
        for _ in range(100):self.assertEqual(c.update(f,[1,0,0],1/240),[])

    def test_actual_sustained_edge_load_severs_only_contacted_thread(self):
        c=ThreadCutter([28]*6);f=np.zeros((168,3));f[33,0]=-.3
        events=[]
        for _ in range(12):events.extend(c.update(f,[1,0,0],1/240))
        self.assertEqual(events,[33]);self.assertEqual(c.severed,{33})

    def test_interrupted_contact_does_not_accumulate_as_sustained(self):
        c=ThreadCutter([28]*6);f=np.zeros((168,3));f[0,0]=-.3
        for _ in range(30):
            c.update(f,[1,0,0],.02);c.update(np.zeros_like(f),[1,0,0],.001)
        self.assertFalse(c.severed)

    def test_blade_swept_through_cloth_latches_damage(self):
        m=patch();a=DamageAudit()
        self.assertTrue(a.update([[.30,0,.75],[.301,0,.75]],[[.30,0,.76],[.301,0,.76]],
                                 [m.rest],[m.faces],.1))
        self.assertTrue(a.update([[.1,0,.8]],[[.1,0,.81]],[m.rest],[m.faces],.2))

    def test_submillimetre_blade_crossing_is_not_lost_to_absolute_determinant(self):
        vertices=np.array(((0.,0.,0.),(.0002,0.,0.),(0.,.0002,0.)))
        hit=segment_triangle_intersections([.00005,.00005,-.00001],
                                           [.00005,.00005,.00001],vertices,[[0,1,2]])
        self.assertEqual(hit.tolist(),[0])

    def test_cloth_moving_through_stationary_blade_is_invalid(self):
        vertices=np.array(((0.,0.,0.),(.0002,0.,0.),(0.,.0002,0.)))
        edge=np.array(((.00005,.00005,.00001),(.00006,.00005,.00001)))
        audit=DamageAudit()
        self.assertFalse(audit.update(edge,edge,[vertices],[[[0,1,2]]],0.))
        self.assertTrue(audit.update(edge,edge,[vertices+[0,0,.00002]],[[[0,1,2]]],.001))

    def test_cut_count_alone_is_not_task_completion(self):
        r=assess(released_stitches=0,blade_crossed_fabric=False,
                 maximum_new_tear_m=0,complete=True,thread_bridge_present=True,tools_clear=True)
        self.assertEqual(r['score'],0.)

    def test_all_removed_and_damage_gate(self):
        kw=dict(released_stitches=6,blade_crossed_fabric=False,
                maximum_new_tear_m=0,complete=True,thread_bridge_present=False,tools_clear=True,material_valid=True)
        self.assertEqual(assess(**kw)['score'],1.)
        kw['blade_crossed_fabric']=True
        self.assertEqual(assess(**kw)['score'],0.)

    def test_full_removal_does_not_require_patch_relocation_but_requires_safe_release(self):
        kw=dict(released_stitches=6,blade_crossed_fabric=False,maximum_new_tear_m=0.,
                complete=True,thread_bridge_present=False,tools_clear=True,material_valid=True)
        self.assertEqual(assess(**kw)['score'],1.)
        kw['tools_clear']=False
        self.assertEqual(assess(**kw)['score'],.75)
        kw['thread_bridge_present']=True
        self.assertEqual(assess(**kw)['score'],0.)

    def test_invalid_judgment_types_fail_closed(self):
        base=dict(released_stitches=6,blade_crossed_fabric=False,
                  maximum_new_tear_m=0,complete=True,thread_bridge_present=False,tools_clear=True,material_valid=True)
        for field,value in [('complete','false'),('tools_clear',1),('released_stitches',True),
                            ('maximum_new_tear_m',float('nan'))]:
            data=dict(base);data[field]=value
            self.assertEqual(assess(**data)['score'],0.)

    def test_invalid_material_cannot_earn_partial_or_full_credit(self):
        for count in (2,4,6):
            r=assess(released_stitches=count,blade_crossed_fabric=False,
                     maximum_new_tear_m=0.,complete=True,thread_bridge_present=False,tools_clear=True,material_valid=False)
            self.assertEqual(r['score'],0.)

    def test_thread_clearance_checks_segment_interiors(self):
        vertices=np.array(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)))
        faces=np.array(((0,1,2),))
        crossing=np.array([[[.2,.2,-.01],[.2,.2,.01]]])
        away=np.array([[[.2,.2,.003],[.3,.2,.003]]])
        r=inspect_removed_threads([crossing,away],[vertices],[faces])
        self.assertEqual(r['removed_count'],1)
        self.assertFalse(r['threads'][0]['removed'])

    def test_mesh_distance_uses_edges_and_faces(self):
        v=np.array(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)))
        d=point_triangle_distance([[.2,.2,.3],[.6,.6,0],[2,0,0]],v,[[0,1,2]])
        np.testing.assert_allclose(d,[.3,np.sqrt(.02),1.])

    def test_declared_five_percent_domain_keeps_warning_and_failure_separate(self):
        v=np.array(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)))
        audit=StrainAudit(v,[[0,1,2]],maximum_stretch=1.05)
        self.assertFalse(audit.update(v*np.array((1.04,1.,1.)),.1))
        self.assertAlmostEqual(audit.first_warning['principal_stretch'],1.04)
        self.assertIsNone(audit.first_excursion)
        self.assertTrue(audit.update(v*np.array((1.06,1.,1.)),.2))
        self.assertAlmostEqual(audit.first_excursion['principal_stretch'],1.06)
        self.assertTrue(audit.update(v,.3))
        self.assertEqual(audit.first_warning['time_s'],.1)
        self.assertEqual(audit.first_excursion['time_s'],.2)

    def test_strain_gate_is_rigid_motion_invariant_and_latches(self):
        v=np.array(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)))
        audit=StrainAudit(v,[[0,1,2]])
        moved=v@np.array(((0.,1.,0.),(-1.,0.,0.),(0.,0.,1.)))+[2,3,4]
        self.assertFalse(audit.update(moved,0.))
        self.assertTrue(audit.update(v*np.array((1.2,1.,1.)),.1))
        self.assertTrue(audit.update(v,.2))
        self.assertAlmostEqual(audit.maximum_observed,1.2)


if __name__=='__main__':unittest.main()
