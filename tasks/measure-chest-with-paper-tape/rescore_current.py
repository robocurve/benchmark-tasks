"""Audit frozen evidence, then replay the geometric scoring correction."""
import argparse,ast,hashlib,json,subprocess,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent

def stripped(path,kind):
 tree=ast.parse(path.read_text())
 if kind=='camera':
  tree.body=[n for n in tree.body if not (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='CONFIGS' for t in n.targets))]
 else:
  for n in tree.body:
   if isinstance(n,ast.ClassDef) and n.name=='Model':n.body=[m for m in n.body if not (isinstance(m,ast.FunctionDef) and m.name=='measure')]
 return ast.dump(tree,include_attributes=False)

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run.resolve();r=json.loads((run/'run.json').read_text())
 subprocess.run([sys.executable,str(HERE/'rescore.py'),str(run)],check=True)
 assert r['status']=='complete' and r['submission_evidence'] and r['capture_every']==1
 authoring={'demo-plan.json','make_plan.py','demo_control.py','planner.py'}
 checks={}
 for f in HERE.iterdir():
  if f.suffix!='.py' or f.name in {'rescore_current.py','score_checks.py'}:continue
  old=run/'source'/f.name
  if r['mode']=='dream' and f.name in authoring:continue
  if f.name=='model.py':assert stripped(old,'model')==stripped(f,'model'),'Physics/update changed';checks[f.name]='Only measurement method may differ'
  elif f.name=='review_views.py':assert stripped(old,'camera')==stripped(f,'camera'),'Policy camera code changed';checks[f.name]='Only recording-camera poses may differ'
  else:assert old.read_bytes()==f.read_bytes(),f.name;checks[f.name]='Identical'
 if r['mode']=='demo':assert (run/'source/demo-plan.json').read_bytes()==(HERE/'demo-plan.json').read_bytes()
 oldtask=json.loads((run/'source/task.json').read_text());task=json.loads((HERE/'task.json').read_text())
 for key in ['instruction','action_limit']:assert oldtask[key]==task[key]
 from model import Model
 m=Model();hold=0.;count=0
 for line in (run/'trace.jsonl').open():
  row=json.loads(line);assert row['step']==count;m.restore(row['state']);q=m.measure()
  if count:hold=hold+.05 if q['readable_loop'] else 0.
  m.hold=hold;count+=1
 assert count==r['steps']+1
 out=dict(score=float(m.score()),rows=count,complete=True,final_measurements=m.measure(),native_recorded_score=r['score'],physics_and_policy_unchanged=True,source_checks=checks,scoring_correction='Signed shortest distance to the native 64-sided chest replaces conservative normalized-radius gap; rubric thresholds unchanged.',scorer_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['model.py','rescore_current.py']})
 encoded=json.dumps(out,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else str(x));(run/'rescore-current.json').write_text(encoded+'\n');print(encoded)
if __name__=='__main__':main()
