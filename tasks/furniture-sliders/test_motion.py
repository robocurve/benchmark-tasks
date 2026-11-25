"""Collision regressions that caused actual approach failures."""
import unittest
import json
from pathlib import Path
import numpy as np
from motion_plan import World,overlap,path,released_parking_goal
from control_utils import solve,rot_from
from yam_arena import analysis
from yam_arena.constants import READY,ARM_JOINTS
from config import FLOOR_Z,F


class MotionTests(unittest.TestCase):
    def test_rear_shorter_approach_clears_other_arm(self):
        f=json.loads(Path(__file__).with_name('regression-rear-close.json').read_text())
        w=World(f['furniture'],f['other_q'],'right',narrow=True)
        self.assertFalse(w.clear(f['goals_mm']['70']))
        route=path(f['start'],f['goals_mm']['50'],w)
        self.assertTrue(all(w.edge(a,b,resolution=.012) for a,b in zip(route,route[1:])))

    def test_rear_approach_uses_native_hull_bounds(self):
        fixture=json.loads(Path(__file__).with_name('regression-rear-approach.json').read_text())
        broad=World(fixture['furniture'],fixture['other_q'],fixture['side'])
        self.assertFalse(broad.clear(fixture['goal']))
        narrow=World(fixture['furniture'],fixture['other_q'],fixture['side'],narrow=True)
        route=path(fixture['start'],fixture['goal'],narrow)
        self.assertTrue(all(narrow.edge(a,b,resolution=.012) for a,b in zip(route,route[1:])))

    def test_turn_retreat_replans_at_fine_resolution(self):
        fixture=json.loads(Path(__file__).with_name('regression-retreat.json').read_text())
        goal=released_parking_goal(fixture['furniture'],fixture['side'])
        world=World(fixture['furniture'],fixture['other_q'],fixture['side'],narrow=True)
        route=path(fixture['start'],goal,world)
        self.assertTrue(all(world.edge(a,b,resolution=.012) for a,b in zip(route,route[1:])))

    def test_release_parking_after_quarter_turn(self):
        from scipy.spatial.transform import Rotation
        from config import PAD_H
        r=Rotation.from_euler('z',-90,degrees=True)*Rotation.from_euler('y',np.arcsin(PAD_H/.3))
        pose=[.32,0,FLOOR_Z+.175*PAD_H/.3,*r.as_quat()]
        q=released_parking_goal(pose,'right')
        # The old READY target was only 46 mm from the tilted slab, below the
        # released-state requirement. The new target physically moves outward.
        self.assertLess(q[0],0)
        poses=analysis.fk_link_poses_world(dict(zip(ARM_JOINTS,q))|{'joint7':-.04695,'joint8':-.04695},(0,-.305,.75))
        v=np.concatenate([h.vertices for _,h in analysis.world_hulls(poses,links=['gripper','tip_left','tip_right'])])
        center=np.asarray(pose[:3])+r.as_matrix()@np.array((0,0,.335))
        self.assertGreater(analysis.box_signed_distance(v,center,r.as_matrix(),(.35,.35,.03)).min(),.055)
        world=World(pose,READY,'right',narrow=True)
        self.assertTrue(world.clear(q),world.last_collision)

    def test_mirrored_right_pad_approach(self):
        # Keeping the left closing axis on the right hit a wrist limit in the
        # first two-pad trial. Reflecting the full frame restores clearance.
        rot=rot_from((-1,0,0),(0,.342,-.940))
        q=solve(np.array((.17,-.35045,FLOOR_Z+.122)),rot,np.array(READY),(0,-.305,.75))
        world=World([.32,0,FLOOR_Z+.00758,0,.02167,0,.999765],READY,'right',grip=0.,narrow=True)
        self.assertTrue(world.clear(q),world.last_collision)
        for y,z in [(-.35045,FLOOR_Z+.022),(-.29,FLOOR_Z+.022),(-.19,FLOOR_Z+.022)]:
            q=solve(np.array((.17,y,z)),rot,q,(0,-.305,.75))
            # The last point is intentionally in the pad/foot contact region;
            # only feasibility of the bounded stock joint solution is required.
            from yam_arena import ik
            self.assertTrue(np.all(q>=ik.LOWER) and np.all(q<=ik.UPPER))

    def test_crossing_rods_without_contained_vertices(self):
        a=('a',np.zeros(3),np.eye(3),np.array((1.,.01,.01)))
        b=('b',np.zeros(3),np.eye(3),np.array((.01,1.,.01)))
        self.assertTrue(overlap(a,b))
        self.assertFalse(overlap(a,('c',np.array((0,0,.1)),np.eye(3),b[3])))

    def test_approach_avoids_observed_tabletop_collision(self):
        q=solve(np.array((.10,.15,FLOOR_Z+.1325)),rot_from((0,1,0),(.866,0,-.5)),
                np.radians([87.09,132.13,115.92,-68.77,59.87,-95.8]),(0,-.305,.75))
        w=World([.32,0,FLOOR_Z,0,0,0,1],READY,'right')
        self.assertTrue(w.clear(np.array(READY))); self.assertTrue(w.clear(q))
        self.assertFalse(w.edge(np.array(READY),q))
        qs=path(READY,q,w)
        # Independent native convex-vertex screen against the actual slab:
        # the original straight path penetrated by about 15 mm.
        center=np.array((.32,0,FLOOR_Z+F['size_m'][2]-F['top_thickness_m']/2))
        size=(*F['size_m'][:2],F['top_thickness_m'])
        for a,b in zip(qs,qs[1:]):
            for s in np.linspace(0,1,12):
                v=a+s*(b-a)
                poses=analysis.fk_link_poses_world(dict(zip(ARM_JOINTS,v))|{'joint7':-.04695,'joint8':-.04695},(0,-.305,.75))
                clearance,_=analysis.hulls_to_box(analysis.world_hulls(poses),center,np.eye(3),size)
                self.assertGreater(clearance,0.)


if __name__=='__main__':unittest.main()
