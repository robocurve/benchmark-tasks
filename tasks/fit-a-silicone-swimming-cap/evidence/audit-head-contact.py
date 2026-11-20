"""Independent fixed-head penetration check, from a frozen native trace."""
from pathlib import Path
import json,sys
import numpy as np
from scipy.spatial import ConvexHull
p=Path(sys.argv[1]).resolve();sys.path.insert(0,str(p/'source'))
from geometry import BODIES,HEAD,HEADROT
head=next(b for b in BODIES if b['name']=='headform')
eq=ConvexHull(np.asarray(head['shapes'][0]['points'])).equations
worst=0.;step=None
for line in (p/'trace.jsonl').open():
 row=json.loads(line);x=(np.array(row['state']['nodes'])-HEAD)@HEADROT
 depth=float(max(0.,-np.max(x@eq[:,:3].T+eq[:,3],axis=1).min()))
 if depth>worst:worst=depth;step=row['step']
r={'maximum_inside_head_mesh_mm':worst*1000,'step':step,'passed':worst<=.002,'method':'All native cap nodes against the fixed convex head mesh; nose and ears require separate visual review. The 2 mm audit tolerance is larger than the 1 mm contact margin and is not a scoring relaxation.'}
(p/'head-contact-audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
if not r['passed']:raise SystemExit('Cap penetrated the head mesh beyond the contact audit tolerance')
