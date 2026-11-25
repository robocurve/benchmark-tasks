"""Independently reconcile a saved Dream run's inputs, chunks, executed actions and score."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


def audit(run):
    result=json.loads((run/'result.json').read_text())
    if result['mode'] not in ('interface-check','dream'):raise ValueError('Not a policy run')
    rows=[json.loads(v) for v in (run/'measurements.jsonl').read_text().splitlines()]
    inputs=[json.loads(v) for v in (run/'policy-inputs.jsonl').read_text().splitlines()]
    events=[json.loads(v) for v in (run/'inference.jsonl').read_text().splitlines()]
    replies={x['observation_step']:x for x in events if x['event']=='reply'}
    assert rows[0]['measurement']['step']==0 and rows[0]['action'] is None
    assert not rows[0]['scoring']['matches'] and len(rows[0]['measurement']['pads'])==4
    assert len(rows)==result['steps']+1 and len(inputs)>=result['steps']
    from yam_arena.constants import ARM_JOINTS,FINGER_OPEN
    from yam_arena import ik
    bounds=(np.tile([*ik.LOWER,0.],2),np.tile([*ik.UPPER,1.],2))
    clock_error=0.;state_error=0.;action_error=0.;clipped=0;request_steps=[]
    pending=[];hold=None
    preroll=result['policy_transcript']['preroll_steps']
    instruction=result['instruction']
    for i,(before,after) in enumerate(zip(rows,rows[1:])):
        observation=inputs[i]
        assert observation['step']==i and observation['instruction']==instruction
        assert observation['camera_order']==['top_cam','left_cam','right_cam']
        assert set(observation['jpeg_sha256'])=={'top_cam','left_cam','right_cam'}
        q=[]
        for side in ('left','right'):
            arm=before['diagnostics']['arms'][side];names=arm['joint_names'];positions=arm['joint_positions']
            q.extend(positions[names.index(n)] for n in ARM_JOINTS)
            q.append(float(np.clip(np.mean([positions[names.index(n)] for n in ('joint7','joint8')])/FINGER_OPEN,0,1)))
        state_error=max(state_error,float(np.max(np.abs(np.asarray(q)-observation['joint_pos']))))
        clock_error=max(clock_error,abs(observation['time_s']-i/20))
        if hold is None:hold=observation['joint_pos']
        if i<preroll:expected=hold
        else:
            if not pending:
                reply=replies[i];request_steps.append(i)
                pending=list(reply['rows'][:result['policy_transcript']['rows_per_request']])
            expected=pending.pop(0)
        action_error=max(action_error,float(np.max(np.abs(np.asarray(expected)-after['policy_action']))))
        command=np.clip(np.asarray(expected),*bounds)
        action_error=max(action_error,float(np.max(np.abs(command-after['action']))))
        clipped+=int(np.any(command!=expected))
        assert after['policy_observation_step']==i
    assert state_error<=1e-6 and clock_error<=1e-7 and action_error<=1e-9
    assert not result.get('scripted_controller_loaded') and not result.get('scripted_assistance')
    replay=json.loads(subprocess.run([sys.executable,'-B',str(run/'task-bundle/audit_run.py'),str(run)],
                                     text=True,capture_output=True,check=True).stdout)
    assert replay['recorded_score_matches'] and replay['actions_14d_finite'] and replay['device_verified']
    versions=result['interface_validation']['source_versions'] if 'interface_validation' in result else None
    return {'run':str(run),'mode':result['mode'],'completed':result['completed'],'steps':result['steps'],
            'score':replay['replayed_scoring']['score'],'termination_reason':result.get('termination_reason'),
            'bare_start_verified':True,'one_episode':True,'no_scripted_assistance':True,
            'three_camera_only_input':True,'recorded_state_matches_native':state_error<=1e-6,
            'maximum_native_state_error':state_error,'maximum_policy_clock_error_s':clock_error,
            'model_chunks_match_played_actions':True,'maximum_action_difference':action_error,
            'native_limit_clipped_actions':clipped,'inference_request_observation_steps':request_steps,
            'hosted_requests':result.get('hosted_inference_requests',0),'hosted_replies':result.get('hosted_inference_replies',0),
            'transport_attempts_for_returned_chunks':sum(x['attempts'] for x in replies.values()),
            'distinct_reported_server_session_ids':len({x['session_id'] for x in replies.values() if x.get('session_id')}),
            'retry_scope':'Installed SDK transport/session retries within the single continuous simulator episode; no task reset or repeat trial.',
            'genuine_hosted_evaluation':result.get('genuine_hosted_evaluation',False),
            'source_versions':versions,'independent_score_audit':replay,
            'inputs_sha256':hashlib.sha256((run/'policy-inputs.jsonl').read_bytes()).hexdigest(),
            'inference_sha256':hashlib.sha256((run/'inference.jsonl').read_bytes()).hexdigest()}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--out',type=Path)
    a=p.parse_args();answer=audit(a.run.resolve());encoded=json.dumps(answer,indent=2)+'\n'
    if a.out:
        with a.out.open('x') as f:f.write(encoded)
    print(json.dumps({k:v for k,v in answer.items() if k not in ('independent_score_audit','inference_request_observation_steps')},indent=2))
