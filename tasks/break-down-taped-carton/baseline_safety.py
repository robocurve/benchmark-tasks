"""Read-only corrected315-piece robot geometry for an UNSCORED scoop coupon."""
import itertools,struct,xml.etree.ElementTree as ET
import numpy as np
from yam_arena import YAM_URDF
from yam_arena.urdf import forward_kinematics,rpy_matrix,quat_xyzw_to_matrix
from scene import layout
from config import TOP


def convex_box_gap(tri,p,R,size):
 v=(tri-p)@R;flat=v.reshape(-1,3);half=np.asarray(size)/2
 bbox=np.maximum(flat.min(0)-half,-half-flat.max(0))
 if bbox.max()>.0003:return float(bbox.max())
 edges=np.concatenate([v[:,1]-v[:,0],v[:,2]-v[:,1],v[:,0]-v[:,2]])
 normals=np.cross(v[:,1]-v[:,0],v[:,2]-v[:,0])
 axes=np.concatenate([np.eye(3),normals,np.cross(edges,[1,0,0]),np.cross(edges,[0,1,0]),np.cross(edges,[0,0,1])]);norm=np.linalg.norm(axes,axis=1);axes=axes[norm>1e-10]/norm[norm>1e-10,None];axes=np.unique(np.round(axes,7),axis=0)
 proj=flat@axes.T;radius=abs(axes)@half
 return float(np.max(np.maximum(proj.min(0)-radius,-radius-proj.max(0))))


class ScoopSafety:
 def __init__(self):
  self.raw=[];self.defs={p['name']:p for p in layout()[0]}
  for link in ET.parse(YAM_URDF).getroot().findall('link'):
   for c in link.findall('collision'):
    mesh=c.find('geometry/mesh')
    if mesh is None:raise ValueError('unsupported ordinary arm collider')
    data=(YAM_URDF.parent/mesh.attrib['filename']).read_bytes();count=struct.unpack_from('<I',data,80)[0]
    tri=np.ndarray(count,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),buffer=data,offset=84)['v'].astype(float)
    origin=c.find('origin');p=np.fromstring(origin.attrib.get('xyz','0 0 0'),sep=' ');R=rpy_matrix(np.fromstring(origin.attrib.get('rpy','0 0 0'),sep=' '));scale=np.fromstring(mesh.attrib.get('scale','1 1 1'),sep=' ')
    self.raw.append((link.attrib['name'],(tri*scale)@R.T+p))
  if len(self.raw)!=315:raise ValueError('reviewed arm geometry count changed')
 def world(self,q,fingers,base):
  fk=forward_kinematics(YAM_URDF,{**{f'joint{i+1}':float(v)for i,v in enumerate(q)},'joint7':float(fingers[0]),'joint8':float(fingers[1])})
  return [(name,t@fk[name][:3,:3].T+fk[name][:3,3]+base)for name,t in self.raw]
 def check(self,q,fingers,poses,cutter,right_q,right_fingers,*,contact=False):
  vectors=[q,fingers,cutter,right_q,right_fingers,*poses.values()]
  if not all(np.isfinite(v).all()for v in vectors):raise ValueError('nonfinite native scoop geometry')
  obs={n:(np.asarray(v[:3]),quat_xyzw_to_matrix(np.asarray(v[3:])),self.defs[n]['size'])for n,v in poses.items()}
  cr=quat_xyzw_to_matrix(np.asarray(cutter[3:]));cp=np.asarray(cutter[:3])
  for name,size,off in [('cutter_handle',(.1,.025,.025),(0,0,0)),('cutter_guard',(.02,.03,.02),(.052,0,.002)),('cutter_blade',(.014,.0005,.008),(.062,0,-.016))]:obs[name]=(cp+cr@off,cr,size)
  left=self.world(q,fingers,np.array([0,.305,.75]));right=self.world(right_q,right_fingers,np.array([0,-.305,.75]));hard=(10.,None,None);intended=(10.,None,None)
  for name,t in left:
   gap=float(t[:,:,2].min()-TOP)
   if gap<hard[0]:hard=(gap,name,'table')
   for n,(p,R,size)in obs.items():
    gap=convex_box_gap(t,p,R,size)
    if contact and name=='tip_right' and n=='north_bottom':
     if gap<intended[0]:intended=(gap,name,n)
    elif gap<hard[0]:hard=(gap,name,n)
  lo=np.array([t.reshape(-1,3).min(0)for _,t in left]);hi=np.array([t.reshape(-1,3).max(0)for _,t in left]);rlo=np.array([t.reshape(-1,3).min(0)for _,t in right]);rhi=np.array([t.reshape(-1,3).max(0)for _,t in right]);gaps=np.max(np.maximum(lo[:,None,:]-rhi[None,:,:],rlo[None,:,:]-hi[:,None,:]),axis=2)
  mutual=float(gaps.min())
  return dict(hard_gap_m=hard[0],hard_pair=list(hard[1:]),other_arm_aabb_gap_m=mutual,intended_target_gap_m=intended[0]if contact else None,intended_pair=list(intended[1:])if contact else None,safe=bool(hard[0]>=.0002 and mutual>=.0002),scope='actualorcommandFK315pieces,current12panels+cutter+table;onlytip_right/north_bottom contactexception;otherarmpositivepieceAABBcertificate')
