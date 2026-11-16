"""Recompute topology and scoring from the recorded native cord and hands."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
r=json.loads((a.run/'run.json').read_text());assert r['status']=='complete' and not r['cpu_preview']
source=a.run/'source'
for name,digest in r['source_sha256'].items():
 assert hashlib.sha256((source/name).read_bytes()).hexdigest()==digest,name
sys.path.insert(0,str(source.resolve()))
from model import Model
from topology import analyze_open_rope
m=Model();count=0;certificates=[]
for line in (a.run/'trace.jsonl').open():
 row=json.loads(line);assert row['step']==count
 state=row['state'];m.points=np.asarray(state['polyline']);m.gap=float(state['max_joint_gap'])
 assert np.isfinite(m.points).all()
 if count==0:
  result=analyze_open_rope(m.points,.010);m.initial_valid=result['status']=='overhand_candidate';m.kind=result['status'];certificates.append(dict(step=count,analysis=result))
 else:
  m.failed=m.failed or m.gap>.005
  if count%20==0:
   result=analyze_open_rope(m.points,.010);m.kind=result['status'];certificates.append(dict(step=count,analysis=result))
  clear=all(h['opening']>.6 and np.min(np.linalg.norm(m.points-np.asarray(h['tcp']),axis=1))>.05 for h in row['hands'].values())
  m.release=m.release+.05 if clear else 0.
 count+=1
out=dict(steps=count-1,score=m.score(),recorded_score=r['score'],score_agrees=m.score()==r['score'],measurements=m.measure(),frozen_source_verified=True,visual_review_still_required=True)
assert out['score_agrees'] and count-1==r['steps']
(a.run/'rescore.json').write_text(json.dumps(out,indent=2)+'\n')
(a.run/'topology-certificates.json').write_text(json.dumps(certificates)+'\n')
print(json.dumps(out,indent=2))
