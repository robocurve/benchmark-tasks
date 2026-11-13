"""Independently replay and audit a finalized full-task trial (not pre-cut diagnostics)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
from scoring import UnboxScorer
from config import DESTINATION, TOP


def audit(folder,expected=1.):
    folder=Path(folder)
    logs=list(folder.glob('adhoc_*.json'))
    if len(logs)!=1: raise ValueError('expected exactly one finalized Inspect log')
    log=json.loads(logs[0].read_text());stats=log['stats']
    rows=[json.loads(line) for line in (folder/'trajectory.jsonl').read_text().splitlines()]
    if not rows or any(s.get('diagnostic') for s in rows): raise ValueError('not a full-task trajectory')
    tracker=UnboxScorer()
    for row in rows: tracker.update(row)
    checks={
        'completed_without_error':log['status']=='success' and log['results']['errored_trials']==0,
        'expected_score':log['results']['metrics'].get('task_score')==expected,
        'independent_replay_matches':tracker.value==expected,
        'trace_length':len(rows)==stats['total_steps']+1,
        'control_timestamps':all(abs(s['time']-i/20)<1e-7 for i,s in enumerate(rows)),
        'no_invalid_outcomes':not any(s['metrics']['invalid'] for s in rows),
    }
    hashes=json.loads((folder/'source-sha256.json').read_text())
    here=Path(__file__).resolve().parent
    core=('config.py','mechanics.py','task.py','scoring.py','run.py','demo_policy.py','telemetry.py','replay.py')
    checks['current_runtime_source_matches']=all(
        name in hashes and hashlib.sha256((here/name).read_bytes()).hexdigest()==hashes[name] for name in core)
    actions=[]
    for p in folder.glob('actions/**/*.jsonl'):
        actions.extend(json.loads(l)['action'] for l in p.read_text().splitlines() if '"action"' in l)
    if not actions and (folder/'actions.jsonl').exists():
        actions=[json.loads(l)['action'] for l in (folder/'actions.jsonl').read_text().splitlines() if '"action"' in l]
    checks['action_count']=len(actions)==stats['total_steps']
    bounds_file=folder/'robot-limits.json'
    action_report={}
    if actions and bounds_file.exists():
        limits=json.loads(bounds_file.read_text())
        lo,hi,delta=limits['low'],limits['high'],limits['max_step_at_20_hz']
        valid=all(len(a)==14 and all(math.isfinite(x) and lo[i]-1e-8<=x<=hi[i]+1e-8 for i,x in enumerate(a)) for a in actions)
        initial=[v for h in rows[0]['hands'] for v in [*h['joints'],h['opening']]]
        ordered=[initial]+actions
        peaks=[max((abs(b[i]-a[i]) for a,b in zip(ordered,ordered[1:])),default=0) for i in range(14)]
        checks['action_bounds']=valid
        checks['command_rate_limits']=all(v<=lim+1e-8 for v,lim in zip(peaks,delta))
        action_report=dict(maximum_command_steps=peaks,limits=limits)
    else: checks['action_limits_available']=False
    events=json.loads((folder/'controller-events.json').read_text())
    checks['no_failed_controller_event']=not any(e['kind']=='failure' for e in events)
    final=rows[-1]
    phase=final.get('controller') or {}
    checks['no_controller_failure']=phase.get('failure') is None
    if expected==1.:
        checks['final_flaps_open']=min(final['flap_angles'])>=70.
        checks['item_on_pad']=(abs(final['item']['pos'][0]-DESTINATION[0])<=.025 and
                               abs(final['item']['pos'][1]-DESTINATION[1])<=.025 and
                               abs(final['item']['pos'][2]-(TOP+.022))<=.008)
        checks['item_released']=final['item_released']
    clips=list(folder.glob('*.mp4'))+list(folder.glob('frames/**/*.mp4'))
    video_frames={}
    for clip in clips:
        probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
            '-show_entries','stream=nb_frames','-of','json',str(clip)]))
        video_frames[clip.name]=int(probe['streams'][0]['nb_frames'])
    checks['complete_camera_videos']=len(video_frames)==3 and all(n==len(rows) for n in video_frames.values())
    errors=[v['position_m'] for s in rows if s.get('controller') for v in s['controller']['ik_errors'].values()]
    return dict(passed=all(checks.values()),checks=checks,score=tracker.value,
                control_steps=stats['total_steps'],simulated_seconds=final['time'],
                wall_seconds=stats['duration_s'],final_flap_angles=final['flap_angles'],
                final_item=final['item'],final_cutter=final['cutter'],
                maximum_ik_residual_m=max(errors,default=None),video_frames=video_frames,
                action_audit=action_report,source_sha256={n:hashes.get(n) for n in core})


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path)
    p.add_argument('--capture-limits-only',action='store_true',help='capture installed robot limits without running media checks')
    p.add_argument('--capture-limits',action='store_true',help='capture installed YAM limits before auditing (Arena environment only)')
    p.add_argument('--expected-score',type=float,default=1.);p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.capture_limits or a.capture_limits_only:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment
        from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        emb=YamArenaEmbodiment(scene='station',control_hz=20)
        step=[SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20]
        limits=dict(low=emb._low.tolist(),high=emb._high.tolist(),max_step_at_20_hz=step*2,
                    configured_action_max_step=emb.info.action_space.semantics.max_step,
                    rate_check_basis='Independent check against configured actuator velocity at 20 Hz')
        (a.run_dir/'robot-limits.json').write_text(json.dumps(limits,indent=2)+'\n')
    if a.capture_limits_only:
        print(a.run_dir/'robot-limits.json');return
    result=audit(a.run_dir,a.expected_score)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    print(text)
    raise SystemExit(0 if result['passed'] else 1)

if __name__=='__main__':main()
