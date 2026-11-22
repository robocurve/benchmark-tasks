"""Audit native traces with the submitted scorer and exercise visible failure cases."""
from pathlib import Path
import sys,json,hashlib,copy,math
import numpy as np
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R))
from model import Model
from geometry import LEAVES,BUCKET
from trace_audit import audit
out={}
assert len(sys.argv)>1, 'Pass the demo run directory, optionally followed by the model run directory'
for argument in sys.argv[1:]:
 p=Path(argument).resolve();name=p.name
 report=json.loads((p/'run.json').read_text())
 for fn,h in report['source_sha256'].items():assert hashlib.sha256((p/'source'/fn).read_bytes()).hexdigest()==h
 frozen=json.loads((p/'source/task.json').read_text());current=json.loads((R/'task.json').read_text())
 for field in ['instruction','scoring','action_limit','fidelity']:assert frozen[field]==current[field],field
 changed=[]
 for fn,h in report['source_sha256'].items():
  if hashlib.sha256((R/fn).read_bytes()).hexdigest()!=h:changed.append(fn)
 assert set(changed)<= {'README.md','task.json','ENVIRONMENT.json','model.py'},changed
 with (p/'trace.jsonl').open() as f:res=audit((json.loads(x) for x in f),report)
 out[name]={'submitted_scorer_replay':res,'changed_files':changed,'scorer_sha256':hashlib.sha256((R/'model.py').read_bytes()).hexdigest()}
 if report['mode']=='demo':
  with (p/'trace.jsonl').open() as f:
   initial=json.loads(next(f));last=initial
   for line in f:last=json.loads(line)
  cases={}
  def check(label,state,expected):
   m=Model();m.restore(state);score=m.score();assert math.isclose(score,expected), (label,score,expected);cases[label]=score
  check('reset',initial['state'],0.)
  check('complete_collection',last['state'],1.)
  for count in [1,2]:
   s=copy.deepcopy(last['state']);m=Model()
   for leaf in LEAVES[count:]:
    for body in leaf:s['poses'][m.index[body]]=initial['state']['poses'][m.index[body]]
   check(str(count)+'_whole_leaves',s,count/3)
  for label,key,value in [('hands_not_released','release',0.5),('lost_off_table','spilled',True),('equipment_broken','broken',['leaf_flex_0_1'])]:
   s=copy.deepcopy(last['state']);s[key]=value;check(label,s,0.)
  s=copy.deepcopy(last['state']);s['poses'][Model().index['bucket']][0]=float(BUCKET[0]+.05);check('bucket_displaced_50mm',s,0.)
  s=copy.deepcopy(last['state']);s['poses'][Model().index['bucket']][3:]=[math.sin(math.pi/12),0,0,math.cos(math.pi/12)];check('bucket_tilted_30degrees',s,0.)
  out['counterfactual_checks']=cases
print(json.dumps(out,indent=2,default=lambda v:v.item() if isinstance(v,np.generic) else v))
