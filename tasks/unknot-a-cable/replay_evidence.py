"""Re-score the compact public traces without Isaac or a GPU."""
from pathlib import Path
import gzip,hashlib,json
import numpy as np
from model import Model
from topology import analyze_open_rope
HERE=Path(__file__).resolve().parent
for mode in ['demo','dream']:
 report=json.loads((HERE/'evidence'/f'{mode}-run.json').read_text())
 for name in ['model.py','topology.py','initial-nodes.json']:
  current=(HERE/name).read_bytes();digest=hashlib.sha256(current).hexdigest()
  if name=='model.py' and digest!=report['source_sha256'][name]:
   proof=json.loads((HERE/'VALIDATION.json').read_text())['scoring_review'];assert proof['verified'] and digest==proof['new_sha256']
   assert current.decode().count(proof['new_code'])==1
   restored=current.decode().replace(proof['new_code'],proof['old_code']);assert hashlib.sha256(restored.encode()).hexdigest()==report['source_sha256'][name]
  else:assert digest==report['source_sha256'][name],name
 m=Model();count=0
 with gzip.open(HERE/'evidence'/f'{mode}-trace.jsonl.gz','rt') as trace:
  for line in trace:
   row=json.loads(line);assert row['step']==count
   m.points=np.asarray(row['state']['polyline']);m.gap=float(row['state']['max_joint_gap'])
   assert np.isfinite(m.points).all()
   if count%20==0:
    m.kind=analyze_open_rope(m.points,.010)['status']
    if count==0:m.initial_valid=m.kind=='overhand_candidate'
   if count:
    m.failed=m.failed or m.gap>.005
    clear=all(h['opening']>.6 and np.min(np.linalg.norm(m.points-np.asarray(h['tcp']),axis=1))>.05 for h in row['hands'].values())
    m.release=m.release+.05 if clear else 0.
   count+=1
 assert count-1==report['steps'] and m.score()==report['score']
 print(json.dumps(dict(mode=mode,score=m.score(),steps=count-1,score_agrees=True,measurements=m.measure())))
