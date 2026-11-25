"""Policy privacy, installed integration, action and termination counterexamples."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from inspect_robots import Action,ActionChunk,Observation,Scene
from inspect_robots.types import Termination,StepResult
from dream_control import (CAMERAS,MODEL,policy_observation,action_from_chunk,terminal_reason,
                           make_policy,source_hashes,validate_preflight)


def observed(t=0.):
    return Observation(images={n:np.full((352,640,3),i*50,np.uint8) for i,n in enumerate((*CAMERAS,'observer_cam'))},
        state={'joint_pos':np.array([0,1,1,-.6,0,0,1]*2,dtype=float),'object_pose':np.ones(7)},
        instruction='untrusted extra instruction',state_time=t,extra={'contacts':[10.]})


class DreamBoundaryTests(unittest.TestCase):
    def test_only_permitted_inputs_and_canonical_instruction(self):
        o=observed();p=policy_observation(o,'Fit the sliders.')
        self.assertEqual(tuple(p.images),CAMERAS);self.assertEqual(set(p.state),{'joint_pos'})
        self.assertEqual(p.extra,{});self.assertEqual(p.instruction,'Fit the sliders.')
        for i,name in enumerate(CAMERAS):
            self.assertEqual(p.images[name].shape,(176,320,3));self.assertTrue((p.images[name]==i*50).all())
        p.state['joint_pos'][0]=9;self.assertEqual(o.state['joint_pos'][0],0)

    def test_invalid_state_shape_and_camera_type_rejected(self):
        from dataclasses import replace
        for q in [np.zeros((2,7)),np.full(14,np.nan),np.array([0]*6+[2]+[0]*7)]:
            with self.assertRaises(ValueError):policy_observation(replace(observed(),state={'joint_pos':q}),'x')
        o=observed();images=dict(o.images);images['top_cam']=images['top_cam'].astype(float)
        with self.assertRaises(ValueError):policy_observation(replace(o,images=images),'x')

    def test_no_target_assistance_or_clipping_before_native_driver(self):
        q=np.arange(14,dtype=float)
        action,end=action_from_chunk(ActionChunk([Action(q)],control_hz=20.))
        np.testing.assert_array_equal(action.data,q);self.assertIsNone(end)

    def test_bad_action_or_rate_rejected(self):
        for q in [np.zeros((2,7)),np.full(14,np.inf),np.zeros(13)]:
            with self.assertRaises(ValueError):action_from_chunk(ActionChunk([Action(q)],control_hz=20.))
        with self.assertRaises(ValueError):action_from_chunk(ActionChunk([Action(np.zeros(14))],control_hz=30.))

    def test_terminal_without_action_and_after_action(self):
        end=Termination('give_up','No progress')
        action,t=action_from_chunk(ActionChunk([],control_hz=20.,terminal=end))
        self.assertIsNone(action);self.assertEqual(t,end)
        action,t=action_from_chunk(ActionChunk([Action(np.zeros(14))],control_hz=20.,terminal=end))
        self.assertIsNone(t);self.assertEqual(terminal_reason(StepResult(observed()),action),'give_up')
        success=StepResult(observed(),terminated=True,termination_reason='success')
        self.assertEqual(terminal_reason(success,action),'success')
        self.assertEqual(terminal_reason(StepResult(observed(),truncated=True)),'environment_truncated')

    def test_offline_uses_installed_policy_queue_without_network_session(self):
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        with tempfile.TemporaryDirectory() as d,patch('inspect_robots_dream.session.DreamSession',side_effect=AssertionError('network session')):
            policy,session=make_policy(d,False)
            policy.bind(yam_arena_embodiment(cam_width=640,cam_height=352,control_hz=20).info)
            policy.reset(Scene(id='check',instruction='Fit the sliders.'))
            first=observed().state['joint_pos']
            for step in range(96):
                chunk=policy.act(policy_observation(observed(step/20),'Fit the sliders.'))
                action,_=action_from_chunk(chunk)
                if step<32:np.testing.assert_array_equal(action.data,first)
                elif step<80:
                    self.assertAlmostEqual(action.data[0],.02)
                    self.assertAlmostEqual(action.data[7],-.02)
                    self.assertEqual(action.data[6],.8)
                else:np.testing.assert_array_equal(action.data,first)
            self.assertEqual(session.requests,2);self.assertEqual(session.replies,2)
            self.assertEqual(policy.transcript()['model'],MODEL)
            policy.close()
            lines=[json.loads(v) for v in (Path(d)/'inference.jsonl').read_text().splitlines()]
            self.assertEqual([v['observation_step'] for v in lines if v['event']=='reply'],[32,80])
            self.assertTrue(all(not v['hosted'] for v in lines))

    def test_live_guard_rejects_changed_source_or_failed_check(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);code=root/'code';code.mkdir();(code/'a.py').write_text('x=1\n')
            (root/'spec-used.json').write_text('{}')
            result={'mode':'interface-check','completed':True,'interface_validation':{'passed':True},
                    'device':{'verified':True},'stock_robot_verified':True,
                    'evaluation_source_sha256':source_hashes(code)}
            p=root/'result.json';p.write_text(json.dumps(result));validate_preflight(p,code,{})
            (code/'a.py').write_text('x=2\n')
            with self.assertRaises(ValueError):validate_preflight(p,code,{})


if __name__=='__main__':unittest.main()
