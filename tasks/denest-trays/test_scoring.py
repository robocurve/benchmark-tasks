"""Offline scorer regression tests; no simulator or API calls."""
import json
from pathlib import Path
import unittest
import numpy as np
from yam_arena.task_file import ObjectState
import task


def scene(offset=0, yaw_deg=0):
    states={};poses={}
    for name,xy in task.GOALS.items():
        yaw=np.deg2rad(yaw_deg if name=='outer' else 0)
        q=np.array([0,0,np.sin(yaw/2),np.cos(yaw/2)])
        root=np.array([xy[0]+(offset if name=='outer' else 0),xy[1],task.TOP])
        R=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        vertices=np.array([root+R@(np.array(p)+np.array(size)/2*np.array([x,y,z]))
            for p,size in task.PARTS[name] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
        lo=vertices.min(0);hi=vertices.max(0)
        states[name]=ObjectState((lo+hi)/2,q,np.zeros(3),np.zeros(3),lo,hi)
        poses[name]=np.r_[root,q]
    robot={side:{'fingers':[task.OPEN,task.OPEN],'tcp':[3.,3.,3.]} for side in ('left','right')}
    return states,poses,robot


def advance(scorer,inputs,frames=1):
    states,poses,robot=inputs
    for _ in range(frames):
        result=scorer.update(step=scorer.last_step+1,objects=states,root_poses=poses,robot=robot)
    return result


class ScoringTests(unittest.TestCase):
    def test_full_score_requires_40_consecutive_frames(self):
        scorer=task.make_scorer();inputs=scene()
        self.assertEqual(advance(scorer,inputs,39)['score'],.5)
        self.assertTrue(advance(scorer,inputs)['success'])

    def test_closed_grippers_are_zero(self):
        inputs=scene();inputs[2]['left']['fingers']=[0,0]
        self.assertEqual(advance(task.make_scorer(),inputs,40)['score'],0)

    def test_hand_close_to_tray_is_zero(self):
        inputs=scene();inputs[2]['left']['tcp']=inputs[0]['outer'].pos.tolist()
        self.assertEqual(advance(task.make_scorer(),inputs)['score'],0)

    def test_hold_resets_when_hands_return(self):
        scorer=task.make_scorer();inputs=scene()
        advance(scorer,inputs,39)
        inputs[2]['right']['fingers']=[0,0]
        self.assertEqual(advance(scorer,inputs)['held_frames'],0)
        inputs[2]['right']['fingers']=[task.OPEN,task.OPEN]
        self.assertEqual(advance(scorer,inputs,39)['score'],.5)
        self.assertEqual(advance(scorer,inputs)['score'],1)

    def test_duplicate_frame_does_not_advance_hold(self):
        scorer=task.make_scorer();s,p,r=scene()
        scorer.update(step=1,objects=s,root_poses=p,robot=r)
        with self.assertRaises(ValueError):scorer.update(step=1,objects=s,root_poses=p,robot=r)
        self.assertEqual(scorer.held,1)

    def test_drop_is_latched(self):
        scorer=task.make_scorer();inputs=scene()
        inputs[0]['outer'].low[2]=task.TOP-.03
        self.assertTrue(advance(scorer,inputs)['dropped'])
        self.assertEqual(advance(scorer,scene(),40)['score'],0)

    def test_rotated_inside_boundary_is_full(self):
        self.assertEqual(advance(task.make_scorer(),scene(.0199,5),40)['score'],1)

    def test_rotated_outside_boundary_is_partial(self):
        self.assertEqual(advance(task.make_scorer(),scene(.0201,5),40)['score'],.5)

    def test_root_pose_and_robot_evidence_are_mandatory(self):
        s,p,r=scene()
        with self.assertRaises(TypeError):task.make_scorer().update(step=1,objects=s)
        with self.assertRaises(ValueError):task.make_scorer().update(step=1,objects=s,root_poses={},robot=r)

    def test_initial_nesting_is_zero(self):
        inputs=scene()
        for name in task.START:
            delta=np.array(task.START[name])-inputs[1][name][:3]
            inputs[0][name].pos[:]+=delta;inputs[0][name].low[:]+=delta;inputs[0][name].high[:]+=delta
            inputs[1][name][:3]=task.START[name]
        self.assertEqual(advance(task.make_scorer(),inputs)['score'],0)

    def test_moving_airborne_and_wrong_orientation_are_zero(self):
        for condition in ('moving','airborne','tilted','yaw'):
            with self.subTest(condition=condition):
                inputs=scene();obj=inputs[0]['inner']
                if condition=='moving':obj.lin_vel[0]=.03
                if condition=='airborne':obj.low[2]+=.05;obj.high[2]+=.05
                if condition=='tilted':obj.quat_xyzw[:]=[1,0,0,0]
                if condition=='yaw':obj.quat_xyzw[:]=[0,0,np.sin(np.deg2rad(15)/2),np.cos(np.deg2rad(15)/2)]
                self.assertEqual(advance(task.make_scorer(),inputs)['score'],0)

    def test_instruction_matches_manifest(self):
        self.assertEqual(task.INSTRUCTION,json.loads(Path(task.__file__).with_name('task.json').read_text())['instruction'])

if __name__=='__main__':unittest.main()
