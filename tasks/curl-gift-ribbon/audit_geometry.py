"""Check recorded ribbon surface crossings independently of the score report."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
r=json.loads((a.run/'run.json').read_text());source=a.run/'source'
assert r['status']=='complete' and r['submission_evidence'] and not r['cpu_preview']
assert hashlib.sha256((source/'model.py').read_bytes()).hexdigest()==r['source_sha256']['model.py']
sys.path.insert(0,str(source.resolve()));from model import Model
m=Model();overlap=[];turns=[];rows=0
with (a.run/'trace.jsonl').open() as f:
 for line in f:
  row=json.loads(line);assert row['step']==rows;rows+=1
  m.p=np.array(row['state']['points']);actual=m.overlap_mm();overlap.append(actual)
  assert abs(actual-row['measurements']['strip_interpenetration_mm'])<1e-6
  turns.append(m.measure()['retained_turns'])
assert rows-1==r['steps']
out={'samples':rows,'maximum_width_overlap_mm':max(overlap),'final_three_seconds_maximum_width_overlap_mm':max(overlap[-61:]),'final_three_seconds_minimum_retained_turns':min(turns[-61:]),'passes_geometry_guard':max(overlap)<=1.,'note':'Sub-millimetre penetration is numerical contact error in the disclosed coarse strip model; this is simulation integrity, not a physical judging requirement.'}
assert out['passes_geometry_guard']
(a.run/'geometry-audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
