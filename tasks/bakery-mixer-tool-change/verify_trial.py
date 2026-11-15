#!/usr/bin/env python3
"""Audit saved native mixer evidence; does not run or replace the simulator scorer."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value, path='value'):
    if isinstance(value, dict):
        for key, item in value.items():
            finite(item, path+'.'+key)
    elif isinstance(value, list):
        for item in value:
            finite(item, path)
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        require(math.isfinite(value), 'Non-finite '+path)


def number(value, name):
    require(isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value), 'Invalid number: '+name)
    return value


def vector(value, length, name):
    require(isinstance(value, list) and len(value)==length, 'Invalid vector: '+name)
    return [number(v, name) for v in value]


def subtract(a, b):
    return [x-y for x, y in zip(a, b)]


def norm(a):
    return math.sqrt(sum(x*x for x in a))


def transpose(a):
    return list(map(list, zip(*a)))


def multiply(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def apply(a, v):
    return [sum(x*y for x, y in zip(row, v)) for row in a]


def rotation(q):
    x, y, z, w=vector(q, 4, 'quaternion')
    require(abs(norm(q)-1)<1e-4, 'Non-unit recorded quaternion')
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def physical(row, cfg):
    states={}
    for key in ['beater', 'guard', 'mixer/shaft', 'mixer/carrier', 'mixer/bowl',
                'mixer/start', 'mixer/stop']:
        entry=row['objects'][key]
        require(isinstance(entry, list) and len(entry)==3, 'Invalid body state: '+key)
        states[key]=(vector(entry[0], 3, key), rotation(entry[1]))
        vector(entry[2], 6, key+' velocity')
    p, r=states['beater'];sp, sr=states['mixer/shaft']
    relative=apply(transpose(sr), subtract(p, sp));rr=multiply(transpose(sr), r)
    yaw=math.atan2(rr[1][0], rr[0][0])-cfg['hub_yaw']
    yaw=math.atan2(math.sin(yaw), math.cos(yaw))
    retained=(norm(relative[:2])<.0005 and abs(relative[2]-cfg['hub_rest_offset'])<.0006
              and rr[2][2]>math.cos(math.radians(3)) and abs(yaw)<math.radians(10))
    bp, br=states['mixer/bowl'];gp, gr=states['guard']
    guard=apply(transpose(br), subtract(gp, bp));rr=multiply(transpose(br), gr)
    seated=(norm(guard[:2])<.003 and abs(guard[2]-cfg['guard_height'])<.0015
            and rr[2][2]>math.cos(math.radians(2))
            and abs(math.atan2(rr[1][0], rr[0][0]))<math.radians(5))
    lift=bp[2]-(cfg['poses']['mixer'][2]+.070)
    raised=abs(lift-cfg['lift_travel'])<.0015
    tcps=[vector(row['tcp'][side], 3, side+' TCP') for side in ['left', 'right']]
    points=[[p[i]+r[i][2]*z for i in range(3)] for z in [0, -.04, -.105, -.17]]
    beater_clear=all(min(norm(subtract(t, point)) for point in points)>.075 for t in tcps)
    guard_clear=all(norm(subtract(t, gp))>.19 for t in tcps)
    outside=any(point[2]<cfg['table_top']-.02 or point[2]>1.5 or abs(point[1])>.65
                or not -.18<point[0]<.90 for point in [p, gp])
    intrusion=any(norm(subtract(t[:2], bp[:2]))<.145 and
                  bp[2]-.02<t[2]<cfg['poses']['mixer'][2]+.350 for t in tcps)
    return dict(retained=retained, seated=seated, raised=raised, lift=lift,
                beater_clear=beater_clear, guard_clear=guard_clear, outside=outside,
                intrusion=intrusion, start=states['mixer/start'][0][0]-cfg['button_rest_x'],
                stop=states['mixer/stop'][0][0]-cfg['button_rest_x'], states=states)


def media(run, receipt, steps):
    probe=shutil.which('ffprobe')
    if not probe:
        return {'checked': False, 'reason': 'ffprobe unavailable'}
    result={}
    for camera in ['top_cam', 'left_cam', 'right_cam']:
        path=run/(camera+'.mp4')
        require(path.is_file(), 'Missing video: '+path.name)
        output=subprocess.check_output([probe, '-v', 'error', '-select_streams', 'v:0',
            '-show_entries', 'stream=codec_name,pix_fmt,width,height,r_frame_rate,nb_frames',
            '-of', 'json', str(path)], text=True)
        streams=json.loads(output)['streams']
        require(len(streams)==1, 'Missing video stream: '+camera)
        stream=streams[0]
        require(stream['codec_name']=='h264' and stream['pix_fmt']=='yuv420p',
                'Video must be H.264 yuv420p: '+camera)
        require([stream['width'], stream['height']]==receipt['camera_resolution'],
                'Video resolution differs from receipt: '+camera)
        n, d=map(int, stream['r_frame_rate'].split('/'))
        require(d and n/d==20, 'Video frame rate differs from 20 Hz: '+camera)
        require(int(stream['nb_frames'])==steps, 'Video frame count differs from trace: '+camera)
        result[camera]=dict(stream, bytes=path.stat().st_size,
                           sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return {'checked': True, 'videos': result}


def verify(run, mode, final_media=False):
    cfg=json.loads(Path(__file__).with_name('pilot.json').read_text())
    receipt=json.loads((run/'receipt.json').read_text())
    require(receipt['status']=='completed', 'Native run is not completed')
    require(isinstance(receipt['hostname'], str) and bool(receipt['hostname']), 'Missing native hostname')
    require(receipt['task']=='bakery-mixer-tool-change', 'Wrong task')
    require(receipt['scope']=='Actual native Isaac pilot execution', 'Wrong execution scope')
    require(receipt['source_unchanged'] is True, 'Task source changed during execution')
    require(not receipt.get('error') and not receipt.get('video_errors') and not receipt.get('reset_error'),
            'Run contains execution, video or reset errors')
    require(re.fullmatch('[0-9a-f]{40}', receipt['git_revision']) is not None, 'Invalid source revision')
    for key in ['task_fingerprint', 'runner_sha256', 'source_patch_sha256']:
        require(re.fullmatch('[0-9a-f]{64}', receipt[key]) is not None, 'Invalid provenance hash: '+key)
    require(hashlib.sha256((run/'source.patch').read_bytes()).hexdigest()==receipt['source_patch_sha256'],
            'Source patch hash mismatch')
    require(abs(receipt['control_dt_s']-.05)<1e-9 and abs(receipt['physics_dt_s']-1/960)<1e-9,
            'Unexpected native timesteps')
    require(receipt['camera_resolution'] in [[640, 360], [1280, 720]], 'Unexpected station video resolution')
    require(receipt['max_steps']==3600, 'Unexpected control budget')
    rows=[json.loads(line) for line in (run/'trace.jsonl').read_text().splitlines()]
    require(0<len(rows)<=3600 and receipt['steps']==len(rows), 'Trace/receipt step count mismatch or budget exceeded')
    require(receipt['final_metrics']==rows[-1]['metrics'], 'Receipt does not match final trace metrics')
    bool_fields=['success', 'unsafe_event', 'beater_retained', 'guard_released', 'motor_on',
                 'verified_run', 'stop_after_run', 'hands_clear', 'retained_geometry', 'guard_seated', 'bowl_raised']
    number_fields=['score', 'run_controls', 'beater_rpm', 'shaft_rpm', 'orbit_rpm', 'lift_m']
    previous={key:rotation(receipt['object_reset_state'][name][1]) for key, name in
              [('beater_rpm', 'beater'), ('shaft_rpm', 'mixer/shaft'), ('orbit_rpm', 'mixer/carrier')]}
    previous_physical=physical(dict(objects=receipt['object_reset_state'],
        tcp={side:receipt['reset_state'][side]['tcp'] for side in ['left', 'right']}), cfg)
    retained_count=guard_count=run_count=hold_count=longest_run=0
    qualified=stopped=False;run_end=stop_step=None;pressed=[];motor_ever=False
    expected_motor=start_pressed=False
    max_lift=0.
    for step, row in enumerate(rows, 1):
        finite(row)
        require(row['step']==step and abs(number(row['time_s'], 'time_s')-step/20)<1e-9,
                'Trace controls are not consecutive at 20 Hz')
        vector(row['action'], 14, 'action')
        m=row['metrics']
        for key in bool_fields:
            require(type(m[key]) is bool, 'Non-boolean metric '+key)
        for key in number_fields:
            number(m[key], key)
        require(0<=m['score']<=1 and m['run_controls']>=0 and m['run_controls']%1==0,
                'Invalid score or running count')
        external=m.get('external_beater_contact',False)
        require(type(external) is bool, 'Invalid external-contact metric')
        if mode=='full':
            require('external_beater_contact' in m, 'Full audit requires native external-contact checks')
        p=physical(row, cfg);max_lift=max(max_lift, p['lift'])
        interlock=previous_physical['seated'] and previous_physical['raised']
        if not interlock:
            expected_motor=False
        if previous_physical['start']>=.003 and not start_pressed:
            expected_motor=interlock;start_pressed=True
        elif previous_physical['start']<.001:
            start_pressed=False
        if previous_physical['stop']>=.003:
            expected_motor=False
        require(m['motor_on']==expected_motor,
                'Motor state disagrees with recorded buttons/interlocks at control '+str(step))
        previous_physical=p
        require(m['guard_seated']==p['seated'] and m['bowl_raised']==p['raised'],
                'Recorded assembly metrics disagree with body poses at control '+str(step))
        require(abs(m['lift_m']-p['lift'])<1e-6, 'Recorded lift differs from bowl pose')
        for key, name in [('beater_rpm', 'beater'), ('shaft_rpm', 'mixer/shaft'), ('orbit_rpm', 'mixer/carrier')]:
            current=p['states'][name][1];delta=multiply(current, transpose(previous[key]))
            rpm=-math.atan2(delta[1][0]-delta[0][1], delta[0][0]+delta[1][1])*20*60/math.tau
            require(abs(rpm-m[key])<.01, 'Recorded RPM differs from body rotation: '+key+' at '+str(step))
            previous[key]=current
        motor_ever|=m['motor_on']
        moving=motor_ever and max(abs(m['shaft_rpm']), abs(m['orbit_rpm']))>5
        require(not m['unsafe_event'] and not p['outside'] and
                not (moving and (p['intrusion'] or not p['seated'] or not p['raised'])),
                'Unsafe event or independently observed hazard at control '+str(step))
        retained_count=retained_count+1 if p['retained'] and p['beater_clear'] and not external else 0
        guard_count=guard_count+1 if p['seated'] and p['guard_clear'] else 0
        ready=retained_count>=20 and p['seated'] and p['raised']
        if not ready:
            qualified=stopped=False
        rotating=(abs(m['beater_rpm']-136)<8 and abs(m['shaft_rpm']-136)<8 and abs(m['orbit_rpm']+60)<5)
        good=ready and m['motor_on'] and rotating and p['beater_clear'] and p['guard_clear']
        run_count=run_count+1 if good else 0;longest_run=max(longest_run, run_count)
        if run_count>=100:
            qualified=True;run_end=step
        if qualified and p['stop']>=.003 and not m['motor_on']:
            stopped=True;stop_step=step
        stationary=max(abs(m[key]) for key in ['beater_rpm', 'shaft_rpm', 'orbit_rpm'])<1
        endpoint=(ready and guard_count>=20 and qualified and stopped and not m['motor_on']
                  and stationary and p['beater_clear'] and p['guard_clear'] and p['start']<.001 and p['stop']<.001)
        hold_count=hold_count+1 if endpoint else 0
        if p['start']>=.003:
            valid=(p['raised'] and not p['seated']) if mode=='absent-guard' else (p['seated'] and abs(p['lift'])<.0015)
            if mode!='full':
                require(valid, 'START pressed without the requested negative-control prerequisites')
                pressed.append(step)
    videos=media(run, receipt, len(rows))
    if final_media:
        require(videos['checked'], 'Final media requires ffprobe')
        require(receipt['camera_resolution']==[1280, 720], 'Final media must be 1280x720')
        require(all(v['bytes']<=50_000_000 for v in videos['videos'].values()), 'A final video exceeds 50 MB')
    if mode=='full':
        require(longest_run>=100, 'No 100-control measured powered run')
        require(hold_count>=40 and stopped, 'No final 40-control stationary, released, hands-clear hold after STOP')
        require(receipt['final_metrics']['success'] and receipt['final_metrics']['score']==1,
                'Native scorer did not report full score')
    else:
        require(pressed and not motor_ever, 'No valid inhibited START press, or motor enabled')
    return dict(verified=True, mode=mode, source_revision=receipt['git_revision'], steps=len(rows),
                simulated_seconds=len(rows)/20, final_score=receipt['final_metrics']['score'],
                longest_measured_run_controls=longest_run, qualified_run_last_step=run_end,
                stop_last_step=stop_step, final_endpoint_controls=hold_count,
                inhibited_start_controls=len(pressed), motor_ever_on=motor_ever, max_lift_m=max_lift,
                receipt_sha256=hashlib.sha256((run/'receipt.json').read_bytes()).hexdigest(), media=videos,
                limitations='Checks saved 20 Hz poses/RPM and native external-contact flags plus declared provenance; does not authenticate execution, certify safety, or infer camera extrinsics from video metadata.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--mode', choices=['full', 'absent-guard', 'lowered-bowl'], required=True)
    parser.add_argument('--final-media', action='store_true', help='Require 1280x720 media, ffprobe and <=50 MB per video')
    args=parser.parse_args()
    try:
        result=verify(args.run, args.mode, args.final_media)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(verified=False, error=str(error)), indent=2))
        return 1
    print(json.dumps(result, indent=2))
    return 0


if __name__=='__main__':
    sys.exit(main())
