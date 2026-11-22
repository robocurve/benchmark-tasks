"""Audit frozen native evidence and verify the current three-strip scorer."""
from pathlib import Path
import argparse,hashlib,json,subprocess,sys
HERE=Path(__file__).resolve().parent

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run.resolve();report=json.loads((run/'run.json').read_text())
 subprocess.run([sys.executable,str(HERE/'rescore.py'),str(run)],check=True)
 frozen=json.loads((run/'rescore.json').read_text());assert frozen['frozen_source_verified']
 assert (run/'source/model.py').read_bytes()==(HERE/'model.py').read_bytes(),'Current scorer must match the recorded three-strip run.'
 authoring={'demo-plan.json','make_plan.py','demo_control.py','planner.py'}
 for f in HERE.iterdir():
  if f.suffix!='.py' or f.name in {'model.py','rescore_current.py'}:continue
  if report['mode']=='dream' and f.name in authoring:continue
  if report['mode']=='dream' and f.name=='joint_tracking.py':
   prior=(run/'source'/f.name).read_text();current=f.read_text()
   before='proportional=1.5,integral=2.,bias_limit=.12,slew_limit=.006'
   after='proportional=.3,integral=.25,bias_limit=.025,slew_limit=.001'
   assert prior==current or (prior.count(before)==1 and prior.replace(before,after)==current)
   # Same runner: tracker.action is exclusively inside the demo branch.
   runner=(HERE/'run.py').read_text()
   assert runner== (run/'source/run.py').read_text()
   demo=runner.split("   if a.mode=='demo':",1)[1].split('   elif policy is not None:',1)[0]
   assert 'tracker.action' in demo and runner.count('tracker.action')==1
   continue
  assert hashlib.sha256(f.read_bytes()).hexdigest()==report['source_sha256'][f.name],f.name
 from model import Model
 m=Model();changed=0
 for line in (run/'trace.jsonl').open():
  row=json.loads(line);m.restore(row['state']);current=m.measure()
  assert current['failed']==row['measurements']['failed']
  changed+=current['rows']!=row['measurements']['rows']
 score=m.score();out=dict(frozen,score=score,measurements=m.measure(),native_recorded_score=report['score'],physics_and_policy_unchanged=True,changed_measurement_frames=changed,scoring_correction='None: current three-strip scorer matches the recorded native run',scorer_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['model.py','rescore_current.py']})
 def scalar(value):
  import numpy as np
  if isinstance(value,np.generic):return value.item()
  raise TypeError(type(value).__name__)
 (run/'rescore-current.json').write_text(json.dumps(out,indent=2,default=scalar)+'\n');print(json.dumps(out,default=scalar))
if __name__=='__main__':main()
