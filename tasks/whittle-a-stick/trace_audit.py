"""Replay all recorded body states with the frozen scorer."""
import numpy as np
from model import Model
def audit(rows,report):
 m=Model();count=0
 for row in rows:
  assert row['step']==count;m.restore(row['state']);q=m.measure()
  for k,v in q.items():assert np.allclose(v,row['measurements'][k],atol=1e-6),k
  assert np.all(m.work>=0) and np.isfinite(m.work).all()
  for i in range(5):
   if 'grain_'+str(i) in m.disabled:
    assert m.work[i]>.16 and m.progress[i]>.040, 'Severed without sufficient loaded cut'
    assert all('grain_'+str(j) in m.disabled for j in range(i)), 'Out-of-order shaving'
  count+=1
 assert count==report['steps']+1 and np.isclose(m.score(),report['score'])
 return dict(score=float(m.score()),rows=count,complete=report['status']=='complete',final_measurements=m.measure(),requires_visual_review=True)
