"""Geometry and scoring checks; independent of Isaac Sim."""
import itertools
import math
import unittest
import numpy as np
import geometry
import task
from yam_arena.task_file import ObjectState


def state(xy,yaw=0):
    p=np.array([*xy,task.TOP+task.HEIGHT/2])
    return ObjectState(pos=p,quat_xyzw=np.array([0.,0.,math.sin(yaw/2),math.cos(yaw/2)]),
                       lin_vel=np.zeros(3),ang_vel=np.zeros(3),low=p-.01,high=p+.01)


class PuzzleTests(unittest.TestCase):
    def test_key_dimensions_and_matching_clearance(self):
        male = geometry.cells_for(0)
        female = geometry.cells_for(1)
        self.assertEqual(sum(x == 25 for x, _ in male), geometry.NECK_WIDTH)
        self.assertEqual(sum(x == 30 for x, _ in male), geometry.HEAD_WIDTH)
        self.assertEqual(sum((-25, y) not in female for y in range(-25, 25)),
                         geometry.NECK_WIDTH + geometry.CLEARANCE)
        self.assertEqual(sum((-20, y) not in female for y in range(-25, 25)),
                         geometry.HEAD_WIDTH + geometry.CLEARANCE)

    def test_unique_square_at_right_angle_orientations(self):
        offsets=[(-25,25),(25,25),(-25,-25),(25,-25)]
        variants={}
        for p in range(4):
            cells=geometry.cells_for(p)
            for r in range(4):
                for slot,(dx,dy) in enumerate(offsets):
                    shape={(x+dx,y+dy) for x,y in cells}
                    if all(-50<=x<50 and -50<=y<50 for x,y in shape): variants[p,r,slot]=shape
                cells={(-y-1,x) for x,y in cells}
        valid=[]
        for order in itertools.permutations(range(4)):
            for rotations in itertools.product(range(4),repeat=4):
                shapes=[variants.get((order[i],rotations[i],i)) for i in range(4)]
                if any(s is None for s in shapes): continue
                if any(shapes[i]&shapes[j] for i in range(4) for j in range(i)): continue
                valid.append((order,rotations))
        expected={((0,1,2,3),(0,0,0,0)),((1,3,0,2),(1,1,1,1)),
                  ((2,0,3,1),(3,3,3,3)),((3,2,1,0),(2,2,2,2))}
        self.assertEqual(set(valid),expected)

    def test_final_corner_fit_uses_original_seam_and_position_tolerances(self):
        objects={f'square_api_piece_{i}':state(xy) for i,xy in enumerate(task.GOAL,1)}
        fit=task.fit_final_corner(objects,task.GOAL[3],np.eye(3))
        self.assertGreater(fit['clearance'],.0002)
        objects['square_api_piece_4']=state(fit['xy'],fit['yaw'])
        self.assertEqual(task.score(objects),1.)

    def test_side_grasps_contact_full_body_rows_clear_of_keys(self):
        # Actual fingertip contact-band width is 15.5 mm (the higher carriage is wider).
        for i in range(4):
            cells=geometry.cells_for(i)
            rows=range(9,25) if i<2 else range(-25,-14)
            self.assertTrue(all((x,y) in cells for y in rows for x in range(-25,25)))
            self.assertFalse(any(y in rows and not -25<=x<25 for x,y in cells))

    def test_scripted_insertion_plan_fits_control_budget(self):
        plan=task.demo()
        self.assertLessEqual(max(sum(w.duration for w in moves) for moves in plan.values()),120.)
        for i in (2,3,4):
            moves=plan['left' if i==2 else 'right']
            holds=[w for w in moves if w.label==f'{i}:align']
            self.assertTrue(any(abs(w.pos[2]-(task.TOP+task.HEIGHT/2+task.GRASP_Z_OFFSET+.030))<1e-8 for w in holds))

    def test_measured_anchor_preserves_rigid_keyed_body_grid(self):
        yaw=math.radians(2)
        c,s=math.cos(yaw),math.sin(yaw)
        R=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        base0=np.asarray(task.GOAL[0])-task.rotated_bounds_center(0,np.eye(3))
        anchor_root=base0+np.array([.0005,-.0005])
        anchor=state(anchor_root+task.rotated_bounds_center(0,R),yaw)
        objects={'square_api_piece_1':anchor}
        for i in range(1,4):
            xy,rot=task.assembly_pose(i,anchor)
            base=np.asarray(task.GOAL[i])-task.rotated_bounds_center(i,np.eye(3))
            delta=base-base0+task.ASSEMBLY_CLEARANCE*np.array([i in (1,3),-int(i in (2,3))])
            expected=anchor_root+R[:2,:2]@delta
            np.testing.assert_allclose(xy-task.rotated_bounds_center(i,R),expected,atol=1e-10)
            np.testing.assert_allclose(rot,R,atol=1e-10)
            objects[f'square_api_piece_{i+1}']=state(xy,yaw)
        self.assertEqual(task.score(objects),1.)
        # At the near-zero anchor yaw used by the demo, existing seam tolerances admit the clearance.
        flat=state(task.GOAL[0]);objects={'square_api_piece_1':flat}
        for i in range(1,4):objects[f'square_api_piece_{i+1}']=state(task.assembly_pose(i,flat)[0])
        self.assertEqual(task.score(objects),1.)

    def test_controller_calibration_does_not_move_scoring_targets(self):
        original=task.DEMO_CALIBRATION
        goals=list(task.GOAL)
        objects={f'square_api_piece_{i}':state(xy) for i,xy in enumerate(goals,1)}
        try:
            task.DEMO_CALIBRATION={'1':{'xy':[.01,-.01],'yaw_deg':10.}}
            self.assertEqual(task.score(objects),1.)
            self.assertEqual(task.GOAL,goals)
            self.assertEqual(task.score({f'square_api_piece_{i}':state(start[:2],start[2]) for i,start in enumerate(task.START,1)}),0.)
        finally:
            task.DEMO_CALIBRATION=original

    def test_scoring_requires_correct_position_and_orientation(self):
        objs={f'square_api_piece_{i}':state(xy) for i,xy in enumerate(task.GOAL,1)}
        self.assertEqual(task.score(objs),1.0)
        objs['square_api_piece_2']=state(task.GOAL[1],yaw=math.pi/2)
        self.assertLess(task.score(objs),1.0)
        objs['square_api_piece_2']=state((task.GOAL[1][0]+.01,task.GOAL[1][1]))
        self.assertLess(task.score(objs),1.0)
        self.assertEqual(task.score({f'square_api_piece_{i}':state(s[:2],s[2]) for i,s in enumerate(task.START,1)}),0.0)

if __name__=='__main__': unittest.main()
