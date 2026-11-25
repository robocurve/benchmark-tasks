"""CPU outcome and trace audit. This is not a physics replay or a feasibility test."""
import hashlib,json
from pathlib import Path
import numpy as np
from model import Model
from geometry import BODIES,JOINTS,BAND_ANCHORS,BAND_REST

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
 m=Model();previous=None;broken=set();release=0.;retained=0.;retained_any=0.;count=0;peak=0.;strain=1.
 joints={j['name'] for j in JOINTS}
 for row in rows:
  assert row['step']==count,'Trace skipped a control step'
  a=np.asarray(row['action14']);assert a.shape==(14,) and np.isfinite(a).all()
  assert np.all(a>=LOW-1e-5) and np.all(a<=HIGH+1e-5),'Joint limit'
  assert np.allclose(a,READY,atol=1e-5) if previous is None else np.all(abs(a-previous)<=DELTA+1e-5),'Action-rate limit'
  m.restore(row['state']);assert set(m.broken)<=joints and broken<=m.broken
  if count:
   clear=all(h['opening']>.4 and min(np.linalg.norm(np.asarray(h['tcp'])-m.p(n)) for n in ['left_tab','right_tab'])>.055 for h in row['hands'].values())
   release=release+.05 if clear else 0.
   retained=retained+.05 if clear and m.hooks()==2 and m.untwisted() and not m.failed else 0.
   retained_any=retained_any+.05 if clear and m.hooks()>=1 and m.untwisted() and not m.failed else 0.
  assert abs(retained_any-m.retained_any)<1e-6,'Partial retention duration'
  assert abs(release-m.release)<1e-6 and abs(retained-m.retained)<1e-6,'Release/retention duration'
  lengths={side:np.linalg.norm(m.p(side+'_tab',[0,sign*.015,0])-BAND_ANCHORS[side]) for side,sign in [('left',1),('right',-1)]}
  peak=max(peak,max(lengths[side]-BAND_REST[side] for side in lengths));strain=max(strain,max(lengths[side]/BAND_REST[side] for side in lengths))
  if count:assert abs(m.max_extension-peak)<1e-6 and abs(m.max_strain-strain)<1e-6,'Elastic strain history'
  else:peak=m.max_extension;strain=m.max_strain
  compare(m.measure(),row['measurements'],'measurements')
  if not count:assert m.score()==0. and not m.failed
  previous=a;broken=m.broken.copy();count+=1
 assert count==report['steps']+1 and count>1
 assert report['steps']<=report['requested_steps']
 score=m.score();assert abs(score-report['score'])<1e-9
 complete=report['status']=='complete' and report['steps']==report['requested_steps']
 return dict(score=score,steps=count-1,complete=complete,measurements=m.measure(),visual_review_still_required=True,submission_evidence=complete and not report.get('cpu_preview',True) and report.get('capture_every')==1)
