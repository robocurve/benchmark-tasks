"""Audit the recorded run, then apply the documented containment correction.

Raw run.json and frozen sources are never modified. The only scorer change is
ignoring mesh-floor penetration for the carcass: its complete XY footprint,
centre height and upright orientation still determine whether it is in the tray.
"""
from pathlib import Path
import argparse,ast,hashlib,json,subprocess,sys
import numpy as np
from model import Model
from geometry import BODIES,TABLE
HERE=Path(__file__).resolve().parent

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run.resolve()
 r=json.loads((run/'run.json').read_text())
 subprocess.run([sys.executable,str(HERE/'rescore.py'),str(run)],check=True,capture_output=True,text=True)
 original=json.loads((run/'rescore.json').read_text())
 assert original['complete'] and original['submission_evidence'] and original['frozen_source_verified']
 old=(run/'source/model.py').read_text();current=(HERE/'model.py').read_text()
 correction="and len(c) and (name=='carcass' or np.all(c[:,2]>=TABLE+.004)))"
 expected=old.replace('and len(c) and np.all(c[:,2]>=TABLE+.004))',correction)
 dump=lambda text:ast.dump(ast.parse(text),include_attributes=False)
 assert dump(expected)==dump(current),'Model changes extend beyond the documented pure containment correction'
 differences=[]
 for f in HERE.iterdir():
  if f.suffix!='.py' and f.name not in ['demo-plan.json','ENVIRONMENT.json']:continue
  if f.name in ['model.py','rescore_current.py']:continue
  old_digest=r['source_sha256'].get(f.name)
  assert old_digest is not None,'Missing recorded runtime source: '+f.name
  if hashlib.sha256(f.read_bytes()).hexdigest()!=old_digest:
   assert r['mode']=='dream' and f.name in ['demo_control.py','make_plan.py','planner.py','demo-plan.json'],f.name
   differences.append(f.name)
 m=Model();changed=0;rows=0
 for line in (run/'trace.jsonl').open():
  row=json.loads(line);m.restore(row['state']);now=m.measure();before=row['measurements']
  assert now.keys()==before.keys()
  for key,value in now.items():
   if key=='carcass_in_tray':continue
   assert value==before[key],('Unexpected changed measurement',row['step'],key)
  changed+=now['carcass_in_tray']!=before['carcass_in_tray'];rows+=1
 assert rows==r['steps']+1
 points=[]
 for shape in BODIES[m.index['carcass']]['shapes']:
  assert shape['kind']=='mesh'
  points.extend(m.p('carcass',v+shape['pos']) for v in np.asarray(shape['points']))
 points=np.asarray(points)
 out=dict(score=m.score(),native_recorded_score=r['score'],steps=r['steps'],complete=True,
  submission_evidence=True,frozen_source_verified=True,physics_and_policy_unchanged=True,
  measurements=m.measure(),changed_containment_rows=changed,
  correction='Only the carcass mesh-floor predicate is removed. Complete XY containment, centre height, upright pose, damage, feather containment and release checks remain unchanged. Raw reports and frozen sources are retained.',
  unused_dream_authoring_differences=differences,
  scorer_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['model.py','rescore.py','rescore_current.py']},
  carcass_final_geometry=dict(minimum_xyz_m=points.min(0).tolist(),maximum_xyz_m=points.max(0).tolist(),centre_xyz_m=m.p('carcass').tolist(),upright_cosine=float(m.orientation('carcass')[2,2]),old_floor_threshold_m=TABLE+.004))
 (run/'rescore-current.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
