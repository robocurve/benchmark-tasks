"""Replay the carton outcome rubric from complete native 240Hz evidence."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from carton_probe.scoring import Outcome
p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
p.add_argument('--output-name',default='score-replay.json',help='Use a separate file when evaluating a newer rubric on historical evidence')
p.add_argument('--output',type=Path,help='Write outside the original run, preserving its evidence unchanged')
a=p.parse_args()
if Path(a.output_name).name!=a.output_name or not a.output_name.endswith('.json'):p.error('Output name must be a JSON filename')
c=json.loads((a.run/'mechanism-contract.json').read_text());z=np.load(a.run/'physics-substeps.npz')
if 'support_contact_counts' not in z:raise SystemExit('No support evidence in this historical run; no full-score replay possible.')
if len(z['support_contact_counts'])!=len(z['poses']):raise SystemExit('Incomplete contact recording')
s=Outcome(c,z['paths']);events=json.loads((a.run/'break-events.json').read_text());breaks={e['sample'] for e in events};trace=[];first_full=None
poses=z['poses'];robot_counts=z['contact_counts'];support_counts=z['support_contact_counts']
for i,q in enumerate(poses):
 value=s.update(q,robot_counts[i],support_counts[i],broken=i in breaks)
 if value==1 and first_full is None:first_full=i
 if i%12==11 or i==len(poses)-1:trace.append(dict(sample=i,**s.detail))
source=ROOT/'carton_probe/scoring.py'
r=dict(score=s.value,first_full_sample=first_full,first_full_action=None if first_full is None else (first_full+1-120)/12,
       final=s.detail,scorer_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),samples=s.samples,
       note='Outcome replay of physical evidence; no full material-fidelity claim.',trace=trace)
output=a.output or a.run/a.output_name
output.parent.mkdir(parents=True,exist_ok=True)
output.write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v for k,v in r.items() if k!='trace'},indent=2))
