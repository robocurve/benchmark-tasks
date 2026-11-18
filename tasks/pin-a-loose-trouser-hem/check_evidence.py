"""Check the published outcome window and complete action log, without Isaac.

This complements the full native-trace audit report and uploaded video. It is
not a physics replay, and does not contain the earlier body-state history.
"""
from pathlib import Path
import argparse,gzip,hashlib,json
import numpy as np
from model import Model
from geometry import BODIES
from trace_audit import compare,LOW,HIGH,DELTA,READY

CORE=['geometry.py','native_physics.py','native_model.py','model.py','common.py','fabric_view.py','geometry_helpers.py','task.py','review_views.py','run.py','ENVIRONMENT.json']
def check(path):
 with gzip.open(path,'rt') as f:d=json.load(f)
 assert d['format']=='native-outcome-window-v1'
 report=d['report'];root=Path(__file__).parent
 for name in CORE:
  assert hashlib.sha256((root/name).read_bytes()).hexdigest()==report['source_sha256'][name],name
 actions=np.asarray(d['actions']);assert actions.shape==(report['steps']+1,14)
 assert np.isfinite(actions).all() and np.allclose(actions[0],READY,atol=1e-5)
 assert np.all(actions>=LOW-1e-5) and np.all(actions<=HIGH+1e-5)
 assert np.all(np.abs(np.diff(actions,axis=0))<=DELTA+1e-5)
 rows=d['outcome_window'];assert len(rows)>=61
 assert [r['step'] for r in rows]==list(range(report['steps']-len(rows)+1,report['steps']+1))
 m=Model();dynamic=[i for i,b in enumerate(BODIES) if not b.get('fixed')]
 held=[]
 for row in rows:
  m.restore(row['state']);compare(m.measure(),row['measurements'],'measurement')
  clear=all(h['opening']>.4 and np.min(np.linalg.norm(m.poses[dynamic,:3]-h['tcp'],axis=1))>.05 for h in row['hands'].values())
  v=m.measure();held.append(clear and v['both_layers_held'] and v['point_enclosed'] and v['fabric_intact'])
 assert abs(m.score()-report['score'])<1e-9
 if m.score()==1.:assert all(held[-61:]),'No complete three-second secured release'
 assert d['full_trace_audit']['complete'] and d['full_trace_audit']['score']==m.score()
 return dict(score=m.score(),actions_checked=len(actions)-1,outcome_window_seconds=(len(rows)-1)/20,measurements=m.measure())
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('record',type=Path);a=p.parse_args();print(json.dumps(check(a.record),indent=2))
