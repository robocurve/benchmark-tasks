"""CPU outcome and trace audit. This is not a physics replay or a feasibility test."""
import hashlib,json
from pathlib import Path
import numpy as np
from model import Model
from geometry import BODIES,JOINTS,CELLS

LOW=np.tile([-2.61799,0.,0.,-1.69297,-1.5708,-2.0944,0.],2)
HIGH=np.tile([3.14159,3.66519,3.14159,1.5708,1.5708,2.0944,1.],2)
DELTA=np.tile([.275,.275,.275,1.,1.,1.,.1556109919714454],2)
READY=np.tile([0.,1.,1.,-.6,0.,0.,1.],2)

def compare(a,b,label):
 if isinstance(a,dict):
  assert a.keys()==b.keys(),label
  for k in a:compare(a[k],b[k],label+'.'+k)
 elif isinstance(a,(list,tuple,np.ndarray)):
  assert np.allclose(a,b,atol=1e-6,rtol=1e-6),label
 elif isinstance(a,(int,float,np.number)) and not isinstance(a,(bool,np.bool_)):
  assert np.isfinite(a) and abs(a-b)<=1e-6*max(1,abs(a)),label
 else:assert a==b,label

def audit(rows,report):
 m=Model();previous=None;release=0.;count=0;prior_failed=False
 for row in rows:
  assert row['step']==count,'Skipped control step'
  a=np.asarray(row['action14']);assert a.shape==(14,) and np.isfinite(a).all()
  assert np.all(a>=LOW-1e-5) and np.all(a<=HIGH+1e-5),'Joint limit'
  assert np.allclose(a,READY,atol=1e-5) if previous is None else np.all(abs(a-previous)<=DELTA+1e-5),'Action-rate limit'
  m.restore(row['state']);assert not prior_failed or m.failed,'Damage was cleared'
  if count:
   clear=all(h['opening']>.4 and min(np.linalg.norm(np.asarray(h['tcp'])-m.p(n)) for n in CELLS+['plate','roller_handle']) > .05 for h in row['hands'].values())
   release=release+.05 if clear else 0.
  assert abs(release-m.release)<1e-6,'Release duration'
  assert np.array_equal(m.leak_check(),m.sealed),'Seal connectivity does not match native film state'
  assert not (m.peak_gap>.001) or m.failed,'Film tear not scored'
  compare(m.measure(),row['measurements'],'measurements')
  if not count:assert m.score()==0. and not m.failed
  previous=a;prior_failed=m.failed;count+=1
 assert count==report['steps']+1 and count>1
 assert report['steps']<=report['requested_steps']
 score=m.score();assert abs(score-report['score'])<1e-9
 complete=report['status']=='complete' and report['steps']==report['requested_steps']
 return dict(score=score,steps=count-1,complete=complete,measurements=m.measure(),visual_review_still_required=True,submission_evidence=complete and not report.get('cpu_preview',True) and report.get('capture_every')==1)
