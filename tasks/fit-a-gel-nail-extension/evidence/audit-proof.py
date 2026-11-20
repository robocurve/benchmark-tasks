"""Independently reconstruct the released 1 N proof from recorded native poses."""
from pathlib import Path
import json,sys,numpy as np
p=Path(sys.argv[1]);sys.path.insert(0,str(p/'source'))
from model import Model
from geometry import PATCHES
m=Model();elapsed=0.;origin=None;slip=0.;peak_force=0.;rows=0
for line in (p/'trace.jsonl').open():
 row=json.loads(line);m.restore(row['state']);force=0.
 if rows:
  if m.release>=3 and np.count_nonzero(m.dose>=40)>=len(PATCHES)//2:
   if origin is None:origin=m.p('extension').copy()
   slip=max(slip,float(np.linalg.norm(m.p('extension')-origin)))
   elapsed=min(4.,elapsed+.05);force=min(1.,elapsed/.5,max(0.,(4-elapsed)/.5))
  else:elapsed=0.;origin=None;slip=0.
  assert abs(elapsed-m.proof_elapsed)<1e-8,(rows,'proof duration')
  assert abs(slip-m.proof_slip)<1e-8,(rows,'proof displacement')
  assert abs(force-m.proof_force)<1e-8,(rows,'force ramp')
  assert (origin is None)==(m.proof_origin is None),(rows,'proof origin')
  if origin is not None:assert np.allclose(origin,m.proof_origin,atol=1e-9,rtol=0),(rows,'proof origin')
 peak_force=max(peak_force,force);rows+=1
r={'passed':True,'records':rows,'proof_complete':elapsed>=4.-1e-8,'maximum_applied_force_N':peak_force,'maximum_proof_displacement_mm':slip*1000,'method':'Reconstruct proof trigger, force ramp, origin, duration and peak displacement from native tip poses and independently audited release/dose.'}
(p/'proof-audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
