"""Independently replay published native hands and all collider bounds on CPU."""
import argparse,gzip,hashlib,json
from pathlib import Path
import numpy as np
from model import Model
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('mode',choices=['demo','dream']);a=p.parse_args()
report=json.loads((HERE/'evidence'/(a.mode+'-run.json')).read_text())
for name in ['model.py','common.py','liquid.py']:
    assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==report['source_sha256'][name],name
packed=np.load(HERE/'evidence'/(a.mode+'-bounds.npz'));names=packed['names'];bounds=packed['bounds']
m=Model();count=0
with gzip.open(HERE/'evidence'/(a.mode+'-trace.jsonl.gz'),'rt') as f:
    for line in f:
        row=json.loads(line);assert row['step']==count
        if count:
            hands={s:{k:np.asarray(v) if k in ['tcp','rotation'] else v for k,v in h.items()} for s,h in row['hands'].items()}
            hands['left']['native_body_bounds']={n:b for n,b in zip(names,bounds[count])}
            m.update(hands)
        count+=1
assert count-1==report['steps'] and count==len(bounds)
assert abs(m.score()-report['score'])<1e-9
print(json.dumps(dict(score=m.score(),steps=count-1,score_agrees=True,measurements=m.measure()),indent=2))
