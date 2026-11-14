import math,unittest
import numpy as np
import geometry as g
import collision as c
from scoring import Observation,Scorer

TOP=.7475
def final(t=0.):
    p=np.array([[.489,-.060,TOP+g.FLOOR],[.489,.060,TOP+g.FLOOR],[*g.TRAY_XY,TOP]])
    r=np.array([np.eye(3)]*3)
    tips=np.array([[.15,-.31,1.0],[.15,-.29,1.0],[.15,.29,1.0],[.15,.31,1.0]])
    return Observation(t,p,r,tips,tips[[0,3]]+[0,0,.05],np.array([.04,.04]))
def settle(s,make=final,start=0.,n=21):
    for j in range(n): s.update(make(start+j*.05))
    return s.result

class MechanismTests(unittest.TestCase):
    def test_start_scene_does_not_intersect_tray(self):
        tray=c.placed(g.tray(),[*g.TRAY_XY,TOP],np.eye(3))
        for i in range(2):
            placed=c.placed(g.half(i),[*g.PIVOT_XY,TOP],g.rz(g.START_ANGLES[i]))
            self.assertEqual(c.intersections(placed,tray),[])

    def test_actual_key_retention_not_an_angle_flag(self):
        for angle in (0,30,60,75,80):
            self.assertFalse(g.head_fits(math.radians(angle)))
            # Axial extraction intersects the actual pivot-head and slot meshes.
            hits=c.half_intersections([[0,0,0],[0,0,.004]], [g.rz(0),g.rz(-math.radians(angle))])
            self.assertIn(('head','pivot'),hits)

    def test_aligned_head_passes_actual_mesh_hole(self):
        self.assertTrue(g.head_fits(math.pi/2,clearance=.0003))
        for z in np.linspace(0,.020,41):
            self.assertEqual(c.half_intersections([[0,0,0],[0,0,z]],[g.rz(0),g.rz(-math.pi/2)]),[])

    def test_ordinary_rotation_is_collision_free(self):
        for a in np.linspace(30,45,31):
            self.assertEqual(c.half_intersections([[0,0,0]]*2,[g.rz(math.radians(a)),g.rz(-math.radians(a))]),[])

    def test_real_size_and_open_handle_loops(self):
        for i in range(2):
            v=g.vertices(i)
            self.assertGreater(np.ptp(v[:,0]),.195)
            self.assertLess(np.ptp(v[:,0]),.210)
            self.assertAlmostEqual(v[:,2].min(),0.)
            self.assertEqual(sum(s.kind=='handle' for s in g.half(i)),49)

    def test_shape_meshes_are_finite_closed_indexed(self):
        for solids in (g.half(0),g.half(1),g.tray()):
            for s in solids:
                self.assertTrue(np.isfinite(s.vertices).all())
                self.assertGreaterEqual(s.triangles.min(),0)
                self.assertLess(s.triangles.max(),len(s.vertices))
                edges={}
                for tri in s.triangles:
                    for a,b in zip(tri,np.roll(tri,-1)):
                        key=tuple(sorted((int(a),int(b)))); edges[key]=edges.get(key,0)+1
                self.assertTrue(all(v==2 for v in edges.values()))

    def test_distance_sees_blade_interior_not_just_vertices(self):
        p=np.array([[.04,0,.0075],[.04,0,.0275]])
        d=g.blade_distance(p,np.zeros(3),np.eye(3),0)
        self.assertEqual(d[0],0.)
        self.assertAlmostEqual(d[1],.0185)

class ScoringTests(unittest.TestCase):
    def setUp(self): self.s=Scorer(TOP)
    def test_full(self): self.assertEqual(settle(self.s),1.)
    def test_21_samples_needed(self): self.assertEqual(settle(self.s,n=20),0.)
    def test_separated_on_table_partial(self):
        def make(t):
            o=final(t); o.positions[:2,0]=.20; o.positions[:2,2]=TOP; return o
        self.assertEqual(settle(self.s,make),.5)
    def test_historical_full_does_not_persist(self):
        self.assertEqual(settle(self.s),1.)
        o=final(1.05); o.positions[0,2]+=.02
        self.assertEqual(self.s.update(o),0.)
    def test_assembled_parts_no_credit(self):
        def make(t):
            o=final(t); o.positions[:2]=[.30,0,TOP]
            o.rotations[:2]=[g.rz(a) for a in g.START_ANGLES]; return o
        self.assertEqual(settle(self.s,make),0.)
    def test_centres_inside_but_handle_overhang_zero(self):
        def make(t):
            o=final(t); o.positions[1,1]=.12; return o
        self.assertEqual(settle(self.s,make),0.)
    def test_floating_zero(self):
        def make(t):
            o=final(t); o.positions[1,2]+=.01; return o
        self.assertEqual(settle(self.s,make),0.)
    def test_tray_off_table_zero(self):
        def make(t):
            o=final(t); o.positions[:,2]+=.01; return o
        self.assertEqual(settle(self.s,make),0.)
    def test_finger_near_handle_zero(self):
        def make(t):
            o=final(t); o.fingertips[0]=o.positions[0]+g.grasp(0)+[0,0,.03]; return o
        self.assertEqual(settle(self.s,make),0.)
    def test_palm_support_zero(self):
        def make(t):
            o=final(t); o.hand_centers[0]=o.positions[0]+g.grasp(0); return o
        self.assertEqual(settle(self.s,make),0.)
    def test_blade_contact_latched_even_if_undone(self):
        o=final(); o.fingertips[0]=o.positions[0]+[.04,0,.0075]; self.s.update(o)
        self.assertEqual(settle(self.s,start=.05),0.)
        self.assertIn('hand contacts metal outside molded handles',self.s.violations)
    def test_pivot_head_grasp_is_not_an_allowed_handle(self):
        o=final(); o.fingertips[0]=o.positions[0]+[0,0,.0138]; self.s.update(o)
        self.assertEqual(settle(self.s,start=.05),0.)
        self.assertIn('hand contacts metal outside molded handles',self.s.violations)
    def test_harmless_low_path_can_recover(self):
        # A low pose alone cannot establish impact, damage or an uncontrolled drop.
        o=final(); o.positions[0]=[.10,-.40,TOP-.020]; self.s.update(o)
        self.assertEqual(self.s.result,0.)
        self.assertEqual(settle(self.s,start=.05),1.)
        self.assertEqual(self.s.violations,set())
    def test_damage_latched_and_reset(self):
        o=final(); o.damage=True; self.s.update(o)
        self.assertEqual(settle(self.s,start=.05),0.)
        self.s.reset(); self.assertEqual(settle(self.s),1.)
    def test_rotate_return_fails_stillness(self):
        for j in range(21):
            o=final(j*.05)
            if j==10:o.rotations[0]=g.rz(.03)
            self.s.update(o)
        self.assertEqual(self.s.result,0.)
    def test_nan_and_reflection_invalid(self):
        for r in (np.full((3,3),np.nan),np.diag([1,1,-1])):
            self.s.reset(); o=final(); o.rotations[0]=r; self.s.update(o)
            self.assertEqual(settle(self.s,start=.05),0.)
    def test_time_gap_invalid(self):
        self.s.update(final(0)); self.s.update(final(.1))
        self.assertEqual(settle(self.s,start=.15),0.)

if __name__=='__main__': unittest.main()
