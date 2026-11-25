"""Where is the illustrated lower-leg posture in the maker model's joint angle? The maker's calibration picture shows
each lower leg level, lying on the table from the heel under the knee to the round toe pad. Turn each lower-leg mesh
about its knee axis until its lowest heel point (behind 30 mm from the knee) and its lowest toe-pad point (beyond
50 mm) are at the same height. Prints the angle for each leg and writes fixture/flat-axis-check.json.
Run from the task folder: python checks/flat_axis.py (needs mujoco and scipy)."""
from pathlib import Path
import json,math
import mujoco,numpy as np
from scipy.optimize import brentq
P=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(P/'assets/offline.xml'));d=mujoco.MjData(m)
out={}
for leg in ('FL','FR','RR','RL'):
 g=m.geom(leg+'_shank_collision').id;mid=m.geom_dataid[g]
 def gap(q):
  d.qpos[:]=0;d.qpos[3]=1;d.qpos[m.jnt_qposadr[m.joint(leg+'_shank_joint').id]]=q;mujoco.mj_forward(m,d)
  v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
  x=v[:,0]-d.xpos[m.body(leg+'_shank').id][0]
  return float(v[x>.050,2].min()-v[x<.030,2].min())
 q=brentq(gap,-.4,0.)
 out[leg]=dict(model_angle_rad=round(q,5),model_angle_deg=round(math.degrees(q),3),heel_minus_toe_at_model_zero_mm=round(-1000*gap(0.),2))
 print(leg,'lower leg lies level at model angle',round(math.degrees(q),3),'deg')
 assert -12.5<math.degrees(q)<-11.8
(P/'fixture/flat-axis-check.json').write_text(json.dumps(dict(method='Turn each lower-leg mesh about its knee axis until its lowest heel point (behind 30 mm from the knee) and its lowest toe-pad point (beyond 50 mm) are at the same height; the heel-minus-toe height is given at the model zero.',legs=out),indent=2))
