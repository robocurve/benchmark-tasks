import math
import unittest
from scene import layout,rotate_z,rotate_x,add


class GeometryTests(unittest.TestCase):
    def test_inventory_is_four_walls_and_eight_flaps(self):
        parts,joints=layout()
        self.assertEqual(len(parts),12);self.assertEqual(len(joints),12)
        self.assertEqual(sum(h['sealed'] for h in joints),2)
        self.assertEqual(len({p['name'] for p in parts}),12)

    def test_hinge_anchors_coincide_in_initial_configuration(self):
        parts,joints=layout();lookup={p['name']:p for p in parts}
        def transform(p,v):
            w,x,y,z=p['quat']
            # Unit quaternion vector rotation.
            cross=(y*v[2]-z*v[1],z*v[0]-x*v[2],x*v[1]-y*v[0])
            second=(y*cross[2]-z*cross[1],z*cross[0]-x*cross[2],x*cross[1]-y*cross[0])
            return tuple(a+b+2*w*c+2*d for a,b,c,d in zip(p['pos'],v,cross,second))
        for h in joints:
            a=transform(lookup[h['a']],h['anchor_a']);b=transform(lookup[h['b']],h['anchor_b'])
            self.assertLess(math.dist(a,b),1e-9,h['name'])

    def test_top_flaps_can_fold_back_outside_wall_without_overlap(self):
        parts,joints=layout(top_angle=270.);p={x['name']:x for x in parts}
        # North flap is vertical outside the north wall at the 270 degree fold.
        self.assertAlmostEqual(p['north_top']['pos'][1]-p['north']['pos'][1],.003)
        self.assertTrue(all(h['low']<=-270. for h in joints if h['name'].endswith('top_hinge')))

    def test_bottom_up_rotated_start_preserves_every_hinge_anchor(self):
        from config import starting_pose
        parts,hs=layout();world={}
        for p in parts:
            pos,q=starting_pose(p);world[p['name']]={'pos':pos,'quat':q}
        def apply(p,v):
            w,x,y,z=p['quat'];cross=(y*v[2]-z*v[1],z*v[0]-x*v[2],x*v[1]-y*v[0]);second=(y*cross[2]-z*cross[1],z*cross[0]-x*cross[2],x*cross[1]-y*cross[0]);return tuple(a+b+2*w*c+2*d for a,b,c,d in zip(p['pos'],v,cross,second))
        for h in hs:self.assertLess(math.dist(apply(world[h['a']],h['anchor_a']),apply(world[h['b']],h['anchor_b'])),1e-9)

if __name__=='__main__':unittest.main()
