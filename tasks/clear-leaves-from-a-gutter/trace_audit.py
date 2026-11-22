"""Replay all recorded body states with the frozen scorer."""
import numpy as np
from model import Model
def audit(rows,report):
 m=Model();count=0
 for row in rows:
  assert row['step']==count;m.restore(row['state']);q=m.measure()
  for k,v in q.items():assert np.isclose(v,row['measurements'][k],atol=1e-6),k
  count+=1
 assert count==report['steps']+1 and np.isclose(m.score(),report['score'])
 return dict(score=float(m.score()),rows=count,complete=report['status']=='complete',final_measurements=m.measure(),requires_visual_review=True)
