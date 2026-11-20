"""Read-only, independent capture-location audit against native finger bounds."""
from pathlib import Path
import json,sys,numpy as np
p=Path(sys.argv[1]);kind=sys.argv[2];r=json.loads((p/'run.json').read_text());sys.path.insert(0,str(p/'source'))
from model import Model
m=Model();events=[e for e in r.get('events',[]) if e['event']=='pinch_captured'];by_step={}
for e in events:by_step.setdefault(round(e['time']*20),[]).append(e)
geometry=json.loads((p/'native-gripper-geometry.json').read_text());bounds={}
for side,d in geometry.items():
 shapes=[s for s in d['shapes'] if s['path'].endswith('precision_finger_extension')]
 assert len(shapes)==2
 pts=np.concatenate([np.asarray(s['vertices_closed_tcp_m']) for s in shapes]);bounds[side]=(pts.min(0),pts.max(0))
checked=[]
for line in (p/'trace.jsonl').open():
 row=json.loads(line)
 for e in by_step.get(row['step'],[]):
  m.restore(row['state']);h=row['hands'][e['side']];R=np.asarray(h['rotation']);tcp=np.asarray(h['tcp']);q=np.asarray(h['finger_joint_positions']);lo,hi=bounds[e['side']]
  thickness=(hi[0]-lo[0])/2;left=q[0]+thickness;right=-q[1]-thickness
  if kind=='pony':
   radius=.0015;margin=.001;points=m.nodes[e['contact_vertices']]
  else:
   radius=.0006;margin=.0006;points=np.array([m.p(e['body'],[0,0,.0015])])
   assert right-left>=2*radius-.000301
  local=(points-tcp)@R
  assert np.all(abs(local[:,0]-radius-left)<=margin+2e-6),(row['step'],'left face')
  assert np.all(abs(local[:,0]+radius-right)<=margin+2e-6),(row['step'],'right face')
  assert np.all(local[:,1]>=lo[1]-2e-6) and np.all(local[:,1]<=hi[1]+2e-6),'outside pad width'
  assert np.all(local[:,2]>=lo[2]-2e-6) and np.all(local[:,2]<=hi[2]+2e-6),'outside pad length'
  checked.append({'step':row['step'],'side':e['side'],'body':e['body'],'contact_points':len(points),'jaw_gap_mm':float((right-left)*1000)})
assert len(checked)==len(events)
out={'passed':True,'captures':checked,'method':'Each recorded capture lies between opposed native finger faces and inside their measured collision bounds. Native-contact tolerance is explicitly included; this does not claim measured friction calibration.'}
(p/'contact-audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
