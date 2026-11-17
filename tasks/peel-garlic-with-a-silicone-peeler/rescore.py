"""Replay the saved measured hands through the frozen material source."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--allow-preview',action='store_true');a=p.parse_args()
r=json.loads((a.run/'run.json').read_text())
if r['cpu_preview'] and not a.allow_preview:raise SystemExit('CPU previews are not submission evidence')
if r['status']!='complete':raise SystemExit('Run incomplete')
source=a.run/'source'
for name,digest in r['source_sha256'].items():
 if hashlib.sha256((source/name).read_bytes()).hexdigest()!=digest:raise SystemExit('Frozen source mismatch: '+name)
sys.path.insert(0,str(source.resolve()));from model import Model
m=Model();count=0
with (a.run/'trace.jsonl').open() as f:
 for line in f:
  row=json.loads(line)
  if row['step']!=count:raise SystemExit('Trace index gap')
  if count:
   hands={side:{k:np.asarray(v) if k in ['tcp','rotation'] else v for k,v in h.items()} for side,h in row['hands'].items()}
   m.update(hands)
  count+=1
out=dict(samples=count,steps=count-1,score=m.score(),recorded_score=r['score'],score_agrees=abs(m.score()-r['score'])<1e-9,measurements=m.measure(),cpu_preview=r['cpu_preview'],frozen_source_verified=True,visual_review_still_required=True)
if count-1!=r['steps'] or not out['score_agrees']:raise SystemExit('Recorded result does not replay')
(a.run/'rescore.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
