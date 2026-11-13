"""Controller gates and kinematic feasibility; run in the Arena environment."""
import unittest
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from yam_arena import ik, station, task_file
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from demo_policy import ParcelFeedbackPolicy, flap_target, rx
from config import TOP,CUT_SEGMENTS
import telemetry


def initial_sample():
    hands=[]
    for side in ('left','right'):
        p,r=ik.tcp_pose(np.array(READY),station.BASES[side])
        hands.append(dict(tcp=p.tolist(),tcp_rotation=r.tolist(),joints=list(READY),opening=1.))
    return dict(time=0.,hands=hands,flap_angles=[0.,0.],held=False,
                metrics=dict(score=0.,cut_segments=0),flaps={})

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.task=task_file.load(Path(__file__).with_name('task.py'))
        telemetry.latest=initial_sample();telemetry.events.clear()
        self.policy=ParcelFeedbackPolicy()
        self.policy._bases=station.BASES;self.policy._task=self.task
        self.policy.reset(None);self.policy.initialize()

    def test_incomplete_cut_cannot_enter_opening(self):
        telemetry.latest['metrics']['cut_segments']=CUT_SEGMENTS-1
        t=self.policy.cut_plan.duration+.1;telemetry.latest['time']=t
        chunk=self.policy.act(SimpleNamespace(state_time=t))
        self.assertTrue(chunk.actions[0].meta['request_stop'])
        self.assertEqual(self.policy.phase,'failed')
        self.assertFalse(any(e['kind']=='approach_flap' for e in telemetry.events))

    def test_tool_must_be_released_before_opening(self):
        telemetry.latest['metrics']['cut_segments']=CUT_SEGMENTS;telemetry.latest['held']=True
        t=self.policy.cut_plan.duration+.1;telemetry.latest['time']=t
        self.policy.act(SimpleNamespace(state_time=t))
        self.assertEqual(self.policy.phase,'failed')

    def test_stalled_flap_stops_after_bounded_retries(self):
        p=self.policy;p.phase='rotate_flap';p.side='left';p.arm='right';p.now=3.
        p.last_angle=0.;p.last_progress=0.;p.rotate_start=0.;p.retries['left']=2
        p.engagement_start=0.;p.engagement_verified=False
        p.advance()
        self.assertEqual(p.phase,'failed')
        self.assertIn('stalled',p.failure)

    def test_transport_preserves_gripper_preload(self):
        # A loaded jaw remains wider than its commanded gap. Starting a carry
        # from the measured width would release the established contact force.
        p=self.policy
        telemetry.latest['hands'][1]['opening']=.426
        tcp=np.array(telemetry.latest['hands'][1]['tcp'])
        r=np.array(telemetry.latest['hands'][1]['tcp_rotation'])
        p.target['right']=(tcp,r,.405)
        p.move('right',tcp+[.05,0.,0.],r,.405,2.,'item_front')
        p.now=1.;p.advance()
        self.assertEqual(p.target['right'][2],.405)
        self.assertEqual(p.motion['g0'],p.motion['g1'])

    def test_inward_flap_lift_starts_at_measured_angle(self):
        from config import CENTER,WIDTH,LID_Z,FLAP_WIDTH
        p=self.policy;p.side='right';p.arm='left'
        angle=-18.;rbody=rx(angle)
        root=np.array([CENTER[0],-WIDTH/2,LID_Z])+rbody@np.array([0.,FLAP_WIDTH/2,0.])
        pos,rot=flap_target('right',angle)
        telemetry.latest['flap_angles']=[110.,angle]
        telemetry.latest['flaps']={'right':dict(pos=root.tolist(),quat=ik.matrix_to_quat_xyzw(rbody).tolist())}
        h=telemetry.latest['hands'][0]
        h.update(tcp=pos.tolist(),tcp_rotation=rot.tolist(),opening=.04)
        p.start_rotation()
        self.assertEqual(p.command_angle,angle)
        p.now=.05;p.advance()
        self.assertLess(p.command_angle,-16.)
        self.assertLess(np.linalg.norm(p.target['left'][0]-pos),.003)

    def test_complete_three_cut_path_is_reachable(self):
        plan=self.policy.cut_plan
        for i in range(1,int(plan.duration*20)+1):plan.action7(i/20)
        self.assertLess(plan.worst_ik_mm,.5)
        self.assertLess(plan.worst_ik_deg,.1)

    def test_thin_edge_probe_requires_observed_motion(self):
        from config import CENTER,WIDTH,LID_Z,FLAP_WIDTH
        p=self.policy;p.side='right';p.arm='left'
        root=np.array([CENTER[0],-WIDTH/2+FLAP_WIDTH/2,LID_Z])
        pos,rot=flap_target('right',0.)
        telemetry.latest['flaps']={'right':dict(pos=root.tolist(),quat=[0.,0.,0.,1.])}
        h=telemetry.latest['hands'][0]
        h.update(tcp=pos.tolist(),tcp_rotation=rot.tolist(),opening=.026)
        p.start_rotation()
        self.assertEqual(p.phase,'rotate_flap')
        self.assertFalse(p.engagement_verified)
        p.now=3.;p.retries['right']=2;p.advance()
        self.assertEqual(p.phase,'failed')

    def test_precut_diagnostic_cannot_earn_task_credit(self):
        m=self.task.module;m._tracker=SimpleNamespace(value=1.);m._diagnostic=True
        self.assertFalse(m.success({}));self.assertEqual(m.score({}),0.)
        m._diagnostic=False
        self.assertTrue(m.success({}));self.assertEqual(m.score({}),1.)

    def test_reachable_plain_edge_opening_arc(self):
        for side in ('left','right'):
            arm='right' if side=='left' else 'left'
            q=np.array(READY)
            for angle in range(-20,109,4):
                pos,rot=flap_target(side,angle)
                q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),q,base_pos=station.BASES[arm],iters=180)
                self.assertLess(pe,.0005,(side,angle,pe))
                self.assertLess(re,.002,(side,angle,re))
                self.assertTrue(np.all(q>=ik.LOWER) and np.all(q<=ik.UPPER))

    def test_edge_grasp_does_not_start_without_access(self):
        p=self.policy;p.side='left';p.arm='right'
        p.transition('support_check')
        self.assertEqual(p.phase,'failed')
        self.assertIn('expose',p.failure)

if __name__=='__main__':unittest.main()
