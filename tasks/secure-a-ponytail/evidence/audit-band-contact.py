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
checked=[];continuity_peak=0.;continuity_step=0
if kind=='pony':
 from geometry import M
 band_ids=[m.names.index(f'band_segment_{i:03d}') for i in range(M)]
for line in (p/'trace.jsonl').open():
 row=json.loads(line)
 if kind=='pony':
  band=np.asarray(row['state']['poses'])[band_ids,:3];spacing=float(np.linalg.norm(np.roll(band,-1,axis=0)-band,axis=1).max())
  if spacing>continuity_peak:continuity_peak=spacing;continuity_step=row['step']
 for e in by_step.get(row['step'],[]):
  m.restore(row['state']);h=row['hands'][e['side']];R=np.asarray(h['rotation']);tcp=np.asarray(h['tcp']);q=np.asarray(h['finger_joint_positions']);lo,hi=bounds[e['side']]
  thickness=(hi[0]-lo[0])/2;left=q[0]+thickness;right=-q[1]-thickness
  if kind=='pony':
   radius=.0015;margin=.001;points=m.nodes[e['contact_vertices']]
   if 'aperture>2*self.radius+.00075' in (p/'source/finite_pinch.py').read_text():
    assert right-left<=2*radius+.00075+2e-6,'Capture preceded close opposing-face contact'
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
if kind=='pony':
 out['band_contact_continuity']={'maximum_adjacent_centre_spacing_mm':continuity_peak*1000,'step':continuity_step,'overlap_limit_for_5mm_locks_mm':8.,'passed':continuity_peak<.008,'method':'Expanded native contact capsules overlap for a 5 mm lock even when conservatively treated as 3 mm spheres at their centres.'}
 assert out['band_contact_continuity']['passed'],'Inspect stretched contact gaps before relying on this recording'
(p/'contact-audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
