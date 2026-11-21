"""Adversarial CPU predicates; no claim of physical rollout validation."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import tag_closure as s
root=Path(__file__).parent
row=json.loads((root/'score_fixture.json').read_text())
(root/'runs').mkdir(exist_ok=True)
x=np.array(row['panel_xyz']);R=Rotation.from_quat(row['panel_quat_xyzw']).as_matrix()
assert s.barcode_exposed(x,R,1.0965)
# Put a separate paper face above each printed panel: normal alone must not pass.
y=x.copy();Q=R.copy()
for src,dst in ((1,3),(5,7)):
 n=R[src,:,2]; n=n if n[2]>0 else -n
 y[dst]=x[src]+.003*n;Q[dst]=R[src]
assert not s.barcode_exposed(y,Q,1.0965)
# Rigid-centre rotation and returning oscillation must fail the speed gate.
c=s.panel_corners(x,R)
assert s.motion_speed([c,c+np.array([0,0,.01]),c],.05)>.015
Q=R.copy();Q[1]=R[1]@Rotation.from_euler('x',20,degrees=True).as_matrix()
assert s.motion_speed([c,s.panel_corners(x,Q)],.05)>.015
assert s.motion_speed([c,c],.05)==0
# An invalid barcode/extraction must reset a continuous hold, not merely gate output.
for invalid_index in (0,4):
 args=[True,True,False,True,True,True,True,0.]
 stable=1.95
 args[invalid_index]=False
 stable=stable+.05 if s.final_valid(*args) else 0.
 assert stable==0
 args[invalid_index]=True
 stable=stable+.05 if s.final_valid(*args) else 0.
 assert s.outcome_score(True,True,False,True,True,stable,True)==.75
s.reset();s.STATE['bonded']=True
s.on_joint_break('/World/printer/frangible_joint');assert not s.STATE['broken']
s.on_joint_break(s.JOINT_PATH);assert s.STATE['broken']
assert s.outcome_score(True,True,s.STATE['broken'],True,True,3.)==0
out={'passed':True,'kind':'CPU synthetic predicates only','checks':['final known geometry exposed','paper occludes printed panels','rotating corners','returning oscillation','barcode resets hold','extraction resets hold','printer break ignored','adhesive break latches zero']}
(root/'runs/score-geometry-regression.json').write_text(json.dumps(out,indent=2))
print(out)
