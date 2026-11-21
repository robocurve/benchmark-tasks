from pathlib import Path
import json,numpy as np,math
p=Path(__file__).resolve().parents[1]
n=json.loads((p/'assets/native.json').read_text())
for leg in ('fl','fr','br','bl'):
 r=next(r for r in n['links'] if r['name']==leg+'_calf_joint');mesh=r['meshes'][0];v=np.array(mesh['vertices']);f=np.array(mesh['faces'])
 e=np.unique(np.sort(np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[0,2]])),axis=1),axis=0);a=v[e[:,0]];b=v[e[:,1]];pts=[]
 for x in np.linspace(v[:,0].min()+.028,v[:,0].max()-.025,10):
  use=(a[:,0]-x)*(b[:,0]-x)<0;u=a[use];w=b[use];t=(x-u[:,0])/(w[:,0]-u[:,0]);c=u+t[:,None]*(w-u);pts.append((x,(c[:,2].min()+c[:,2].max())/2))
 angle=math.degrees(math.atan(np.polyfit(np.array(pts)[:,0],np.array(pts)[:,1],1)[0]));print(leg,round(angle,6),'deg shaft slope at simulation q=0');assert 14<angle<16
