"""Independent replay of every recorded state, including score persistence."""
import numpy as np
from model import Model

def audit(rows,report):
 m=Model();count=0;prior=None;full_hold=0.;events=[]
 for row in rows:
  assert row['step']==count
  m.restore(row['state']);q=m.measure();score=m.score()
  for k,v in q.items():
   old=row['measurements'][k]
   if isinstance(v,(float,int,np.number)) and not isinstance(v,bool):assert np.isclose(v,old,atol=1e-6),k
   else:assert v==old,k
  if count:
   full_hold=full_hold+.05 if q['readable_loop'] else 0.
   assert np.isclose(m.hold,full_hold,atol=1e-6),'Hold history mismatch'
  if prior is not None:assert np.isfinite(np.asarray(row['action14'])).all()
  prior=row;count+=1
 assert count==report['steps']+1
 assert np.isclose(score,report['score'])
 return dict(score=float(score),rows=count,complete=report['status']=='complete',final_measurements=q,requires_visual_review=True)
