"""Re-evaluate recorded measured physics/robot state through the task's scorer."""
import argparse
import json
from pathlib import Path
import numpy as np
from yam_arena.task_file import ObjectState
import task

def rescore(path):
    scorer=task.make_scorer(); result=None
    with Path(path).open() as trace:
        for line in trace:
            row=json.loads(line)
            states={name:ObjectState(**{k:np.asarray(v,dtype=float) for k,v in obj.items()}) for name,obj in row['objects'].items()}
            result=scorer.update(step=row['step'],objects=states,
                root_poses={n:np.asarray(v) for n,v in row['root_poses'].items()},robot=row['robot'])
    if result is None:raise ValueError('Empty trajectory')
    return {'steps':scorer.last_step,'score':result['score'],'success':result['success'],'held_frames':result['held_frames']}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trajectory',type=Path);p.add_argument('--expect',type=Path)
    args=p.parse_args(); result=rescore(args.trajectory)
    if args.expect:
        expected=json.loads(args.expect.read_text())
        for key in ('score','success','steps'):
            if expected[key]!=result[key]:raise SystemExit(f'Score replay mismatch for {key}')
    print(json.dumps(result,indent=2))
