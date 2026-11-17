"""CPU outcome and trace audit. This is not a physics replay or a feasibility test."""
import hashlib,json
from pathlib import Path
import numpy as np
from model import Model
from geometry import BODIES,JOINTS

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
 m=Model();previous=None;broken=set();release=0.;count=0
 joints={j['name'] for j in JOINTS};dynamic=[i for i,b in enumerate(BODIES) if not b.get('fixed')]
 for row in rows:
  assert row['step']==count,'Trace skipped/duplicated a control step'
  a=np.asarray(row['action14']);assert a.shape==(14,) and np.isfinite(a).all()
  assert np.all(a>=LOW-1e-5) and np.all(a<=HIGH+1e-5),'Joint limit'
  assert np.allclose(a,READY,atol=1e-5) if previous is None else np.all(abs(a-previous)<=DELTA+1e-5),'Action-rate limit'
  m.restore(row['state']);assert set(m.broken)<=joints and broken<=m.broken,'Unknown or healed joint'
  if count:
   clear=all(h['opening']>.4 and np.min(np.linalg.norm(m.poses[dynamic,:3]-np.asarray(h['tcp']),axis=1))>.05 for h in row['hands'].values())
   release=release+.05 if clear else 0.
  assert abs(release-m.release)<1e-6,'Release duration disagrees with native hand/body poses'
  compare(m.measure(),row['measurements'],'measurements')
  if not count:assert m.score()==0. and not m.failed,'Invalid initial state'
  previous=a;broken=m.broken.copy();count+=1
 assert count==report['steps']+1 and count>1,'Trace length'
 assert report['steps']<=report['requested_steps'],'Run overran action request'
 events={e['joint'] for e in report.get('events',[]) if e['event']=='native_joint_break'}
 assert events==broken,'Native break-event history differs from final state'
 score=m.score();assert abs(score-report['score'])<1e-9,'Reported score differs'
 complete=report['status']=='complete' and report['steps']==report['requested_steps']
 return dict(score=score,steps=count-1,complete=complete,measurements=m.measure(),native_break_events=len(events),visual_review_still_required=True,submission_evidence=complete and not report.get('cpu_preview',True))
