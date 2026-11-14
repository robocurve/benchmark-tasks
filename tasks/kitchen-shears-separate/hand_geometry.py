"""Read installed URDF convex collision meshes for offline clearance checks.

No simulator, no mesh modification and no third-party assets are bundled.
Convexity is checked rather than assuming a visual mesh is a convex solid.
"""
from dataclasses import dataclass
import hashlib,pathlib,struct,xml.etree.ElementTree as ET
import numpy as np

@dataclass(frozen=True)
class Hull:
    link: str
    source: str
    sha256: str
    vertices: np.ndarray
    planes: np.ndarray

def read_stl(path):
    data=pathlib.Path(path).read_bytes()
    if len(data)>=84:
        n=struct.unpack_from('<I',data,80)[0]
        if len(data)==84+50*n:
            dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
            return np.frombuffer(data,dtype=dtype,count=n,offset=84)['v'].astype(float)
    vertices=[]
    for line in data.decode('ascii').splitlines():
        fields=line.split()
        if fields and fields[0]=='vertex':vertices.append([float(v) for v in fields[1:]])
    v=np.asarray(vertices)
    if v.ndim!=2 or v.shape[1]!=3 or len(v)%3:raise ValueError(f'invalid STL: {path}')
    return v.reshape(-1,3,3)

def convex_planes(triangles,tolerance=1e-4):
    t=np.asarray(triangles,dtype=float)
    if t.ndim!=3 or t.shape[1:]!=(3,3) or not len(t) or not np.isfinite(t).all():
        raise ValueError('invalid triangles')
    v=np.unique(t.reshape(-1,3),axis=0);center=v.mean(0)
    normals=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);length=np.linalg.norm(normals,axis=1)
    keep=length>1e-13
    n=normals[keep]/length[keep,None];p=t[keep,0]
    n=np.where((np.sum(n*(center-p),axis=1)>0)[:,None],-n,n)
    d=-np.sum(n*p,axis=1);planes=np.c_[n,d]
    if len(planes)<4 or np.linalg.matrix_rank(n,tol=1e-8)<3:raise ValueError('degenerate collision hull')
    error=float(np.max(v@n.T+d))
    if error>tolerance:raise ValueError(f'collision mesh convexity error {error*1000:.6f} mm')
    # STL float precision leaves small face-plane inconsistencies. Shift each
    # plane outward to contain every vertex; this bounds the cooked convex hull
    # conservatively without editing the mesh or shrinking any collision shape.
    planes[:,3]=-np.max(v@n.T,axis=0)
    _,idx=np.unique(np.round(planes,9),axis=0,return_index=True)
    return v,planes[np.sort(idx)]

def rotation(rpy):
    r,p,y=rpy;cr,sr=np.cos(r),np.sin(r);cp,sp=np.cos(p),np.sin(p);cy,sy=np.cos(y),np.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])

def vector(text,default):return np.array(list(map(float,text.split())) if text else default)

def load_hand_hulls(urdf):
    urdf=pathlib.Path(urdf).resolve();result=[]
    for link in ET.parse(urdf).getroot().findall('link'):
        name=link.get('name')
        if name not in ('gripper','tip_left','tip_right'):continue
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is None:raise ValueError(f'{name}: unsupported nonmesh collision')
            rel=mesh.get('filename')
            if '://' in rel:raise ValueError('package URI needs explicit resolution')
            path=(urdf.parent/rel).resolve()
            if not path.is_file():raise FileNotFoundError(path)
            origin=collision.find('origin')
            xyz=vector(origin.get('xyz') if origin is not None else None,[0,0,0])
            r=rotation(vector(origin.get('rpy') if origin is not None else None,[0,0,0]))
            scale=vector(mesh.get('scale'),[1,1,1])
            t=(read_stl(path)*scale)@r.T+xyz
            try:v,p=convex_planes(t)
            except ValueError as e:raise ValueError(f'{rel}: {e}') from e
            result.append(Hull(name,rel,hashlib.sha256(path.read_bytes()).hexdigest(),v,p))
    if {h.link for h in result}!={'gripper','tip_left','tip_right'}:raise ValueError('missing hand links')
    return result

def world_planes(planes,rotation,position):
    n=planes[:,:3]@np.asarray(rotation).T
    return np.c_[n,planes[:,3]-n@np.asarray(position)]

def point_near_hull(points,planes,margin=0.):
    return np.max(np.asarray(points)@planes[:,:3].T+planes[:,3],axis=-1)<=margin

def segment_intersects(a,b,planes,margin=0.):
    """Exact intersection with a convex polytope; optional conservative expansion."""
    a=np.asarray(a);b=np.asarray(b);start=planes[:,:3]@a+planes[:,3]-margin
    delta=planes[:,:3]@(b-a);low=0.;high=1.
    for s,d in zip(start,delta):
        if abs(d)<1e-14:
            if s>0:return False
        elif d>0:high=min(high,-s/d)
        else:low=max(low,-s/d)
        if low>high:return False
    return True
