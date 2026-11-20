"""Replay recorded geometry through the current primary scorer, read-only."""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
from drawstring_score import DrawstringJudge

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    trace=json.loads((a.run/'trace.json').read_text());states={int(s['step']):s for s in trace}
    if sorted(states)!=list(range(max(states)+1)):raise ValueError('Every control-step geometry sample is required')
    params=json.loads((a.run/'task_parameters.json').read_text());params['sample_interval_steps']=1
    judge=DrawstringJudge(params);digest=hashlib.sha256();start=time.monotonic()
    # A dense lossless NPY replaces 1,020 small NPZ files in the portable packet.
    dense=np.load(a.run/'geometry.npy',mmap_mode='r') if (a.run/'geometry.npy').exists() else None
    if dense is not None and len(dense)!=len(states):raise ValueError('Geometry/trace length mismatch')
    for step,s in sorted(states.items()):
        v=dense[step] if dense is not None else np.load(a.run/f'state_{step:04d}.npz')['loaded_bag'][0]
        digest.update(v.tobytes());judge.update(s,v)
        if step%100==0:print(json.dumps({'step':step,'score':judge.result().get('score'),'elapsed':round(time.monotonic()-start,2)}),flush=True)
        if judge.failure is not None:break
    result=dict(judge.result(),last_replayed_step=step,expected_final_step=max(states),all_steps_replayed=step==max(states),geometry_sha256=digest.hexdigest(),source_sha256={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in ['drawstring_score.py','drawstring_physics.py','evaluate_removal.py','containment.py','replay_run.py']})
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return int(not result.get('full_task_success'))

if __name__=='__main__':raise SystemExit(main())
