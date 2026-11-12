"""Measure pose oscillation at the actual physics frequency during fixed commands."""
import argparse,json
from pathlib import Path
import numpy as np

def summarize(path):
 rows=[json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()];reports=[]
 for phase in dict.fromkeys(r['phase'] for r in rows):
  sample=[r for r in rows if r['phase']==phase];count=max(2,round(.5/sample[-1]['dt']));sample=sample[-count:]
  bodies={};arms={}
  for name in sample[0]['bodies']:
   pose=np.array([r['bodies'][name]['pose'] for r in sample]);q=pose[:,3:];q/=np.linalg.norm(q,axis=1,keepdims=True)
   angle=2*np.arccos(np.clip(np.abs(q@q[-1]),0,1))
   bodies[name]=dict(position_peak_to_peak_m=float(np.linalg.norm(np.ptp(pose[:,:3],axis=0))),rotation_deviation_max_deg=float(np.degrees(angle).max()))
  for side in ['left','right']:
   q=np.array([r['arms'][side]['joint_pos'] for r in sample])
   key='net_normal_contact_forces_n' if 'net_normal_contact_forces_n' in sample[0]['arms'][side] else 'contact_forces_n'
   force=np.array([r['arms'][side][key] for r in sample]);norm=np.linalg.norm(force,axis=2)
   arms[side]=dict(joint_peak_to_peak_rad=float(np.ptp(q,axis=0).max()),peak_normal_contact_force_n=float(norm.max()),normal_force_p95_n=float(np.percentile(norm,95)))
  passed=all(b['position_peak_to_peak_m']<=.0015 and b['rotation_deviation_max_deg']<=2. for b in bodies.values()) and all(a['joint_peak_to_peak_rad']<=.012 for a in arms.values())
  reports.append(dict(phase=phase,physics_hz=1/sample[-1]['dt'],sample_count=len(sample),bodies=bodies,arms=arms,passed=passed))
 return dict(passed=bool(reports) and all(p['passed'] for p in reports),holds=reports,limits=dict(position_peak_to_peak_m=.0015,rotation_deviation_max_deg=2.,joint_peak_to_peak_rad=.012),limitations='Last 0.5 seconds of each fixed-command hold. Contact sensor reports net normal force, not friction or full pairwise impulses. This screen supplements video and movement checks.')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('trace',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args();r=summarize(a.trace);a.out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 2)
