"""Replay the measured hand poses, then score the released physical outcome."""
import argparse, gzip, hashlib, json
from pathlib import Path
import numpy as np
from material import Tie, score

def rescore(directory):
    directory=Path(directory)
    report=json.loads((directory/'run.json').read_text())
    if report['status']!='complete':
        raise ValueError('A completed native run is required')
    expected=report['source_sha256']['material.py']
    if hashlib.sha256(Path(__file__).with_name('material.py').read_bytes()).hexdigest()!=expected:
        raise ValueError('Use the material.py version recorded with this trial')
    path=directory/'trace.jsonl'
    if path.exists():stream=path.open()
    else:stream=gzip.open(directory/'trace.jsonl.gz','rt')
    tie=Tie();last=-1;max_error=0.
    with stream:
        for line in stream:
            row=json.loads(line)
            if row['step']!=last+1:raise ValueError('Missing or reordered trace rows')
            last=row['step']
            if last:
                # Native quaternion arithmetic produces float32 rotations. Keep
                # that precision when replaying atan2 used for blade twist.
                hands={s:{k:np.asarray(v,dtype=np.float32 if k=='rotation' else float) if k!='opening' else float(v) for k,v in h.items()} for s,h in row['hands'].items()}
                tie.update(hands)
            actual=tie.measures()
            for key,value in actual.items():
                error=abs(float(value)-float(row['measurements'][key]));max_error=max(max_error,error)
    if last!=report['steps'] or max_error>1e-7:
        raise ValueError(f'Incomplete or inconsistent material record: {last=}, {max_error=}')
    result=dict(score=score(tie.measures()),steps=last,maximum_replay_error=max_error,
                measurements=tie.measures(),events=tie.events,
                visual_review_required=True)
    if result['score']!=report['score']:raise ValueError('Recorded and replayed scores disagree')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--out',type=Path)
    a=p.parse_args();text=json.dumps(rescore(a.directory),indent=2)+'\n'
    if a.out:a.out.write_text(text)
    else:print(text,end='')
