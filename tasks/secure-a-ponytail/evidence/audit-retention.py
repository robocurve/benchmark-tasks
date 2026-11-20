"""Independent command-duration and displacement audit of a completed native demo."""
from pathlib import Path
import json,sys,hashlib,numpy as np
p=Path(sys.argv[1]);r=json.loads((p/'run.json').read_text());assert r['status']=='complete' and r['score']==1.
rows=[json.loads(l) for l in (p/'trace.jsonl').open()];final=rows[-1];origin=final['state']['retention']['origin'];assert origin is not None
sys.path.insert(0,str(p/'source'));from geometry import M
selected=[q for q in rows if q['state']['retention']['origin']==origin]
forces=np.array([q['state']['proof_force_n'] for q in selected]);assert forces.min()>=0 and forces.max()<=1.+1e-8
plateau=float(np.isclose(forces,1.,atol=1e-8,rtol=0).sum()/20);impulse=float(forces.sum()/20)
assert plateau>=3. and abs(impulse-3.5)<1e-6
centres=np.array([np.asarray(q['state']['nodes'],dtype=np.float32)[M:2*M].mean(0) for q in selected]);travel=np.linalg.norm(centres-np.asarray(origin),axis=1)*1000
assert max(travel)<=10. and final['measurements']['secured_locks']>=11
assert all(q['measurements']['released_seconds']>=3.-1e-8 for q in selected)
assert final['state']['retention']['completed'] and forces[-1]==0.
record={'passed':True,'maximum_force_n':float(forces.max()),'plateau_seconds':plateau,'force_impulse_ns':impulse,'maximum_centre_travel_mm':float(max(travel)),'retained_locks':final['measurements']['secured_locks'],'all_test_frames_hands_released':True,'native_wrench_source_sha256':hashlib.sha256((p/'source/native_wrench.py').read_bytes()).hexdigest(),'scope':'Independent trace computation, not a new physics trial. Actual force application uses the separately validated native permanent-wrench API.'}
(p/'retention-audit.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record))
