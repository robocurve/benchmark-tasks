"""Installed-robot continuation IK and convex-body interference screen."""
import json,pathlib,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
import task,geometry as g
from transfer import Transfer
from table_plan import grasp_target
from hand_geometry import read_stl,rotation,vector,world_planes
from yam_arena import ik,YAM_URDF
from yam_arena.constants import LEFT_BASE_POS,RIGHT_BASE_POS,READY
from yam_arena.urdf import forward_kinematics

urdf=pathlib.Path(YAM_URDF)
shapes=[]
for link in ET.parse(urdf).getroot().findall('link'):
    name=link.get('name')
    if name in ('base','world'):continue
    for c in link.findall('collision'):
        mesh=c.find('geometry/mesh')
        if mesh is None:continue
        path=(urdf.parent/mesh.get('filename')).resolve()
        o=c.find('origin');r=rotation(vector(o.get('rpy') if o is not None else None,[0,0,0]))
        p=vector(o.get('xyz') if o is not None else None,[0,0,0])
        v=(read_stl(path)*vector(mesh.get('scale'),[1,1,1]))@r.T+p
        v=np.unique(np.round(v.reshape(-1,3),7),axis=0)
        hull=ConvexHull(v)
        shapes.append((name,v[hull.vertices],hull.equations.copy()))

# Reproduce the c1 final observed geometry, with the unchanged nominal grasp transforms.
e=json.loads((pathlib.Path(__file__).parent/'c1-evidence.json').read_text())
p=np.array(e['positions']);r=np.array(e['rotations'])
cmd0=grasp_target(0,np.array([*g.PIVOT_XY,task.TOP]),np.radians(30.))
a=np.radians(-60+e['feedback']['angle_trim_deg'])
cmd1=grasp_target(1,np.array([*g.PIVOT_XY,task.TOP])+np.array(e['feedback']['translation_mm'])/1000,a)
cmd1=(cmd1[0]+[0,0,e['feedback']['lift_mm']/1000],cmd1[1])
tr=Transfer(task.TOP);tr.begin(0.,p,r,[cmd0,cmd1])
qs=[np.array(READY,float),np.array(READY,float)]
bases=[np.array(RIGHT_BASE_POS),np.array(LEFT_BASE_POS)]
worst_pos=worst_rot=0.;hits=[];below=[];nonmesh=sum(c.find('geometry/mesh') is None for l in ET.parse(urdf).getroot().findall('link') for c in l.findall('collision'))
def placed(i,q,f):
    joints={f'joint{k+1}':float(q[k]) for k in range(6)};joints.update(joint7=f,joint8=f)
    fk=forward_kinematics(urdf,joints);out=[]
    for name,v,planes in shapes:
        tf=fk[name].copy();tf[:3,3]+=bases[i]
        vv=v@tf[:3,:3].T+tf[:3,3]
        out.append((name,vv,world_planes(planes,tf[:3,:3],tf[:3,3]),vv.min(0),vv.max(0)))
    return out
for t in np.arange(0,tr.duration+.001,.1):
    bodies=[]
    for i in range(2):
        tcp,rot,f,label=tr.target(i,float(t))
        qs[i],pe,re=ik.solve(tcp,ik.matrix_to_quat_xyzw(rot),qs[i],base_pos=bases[i],iters=300)
        worst_pos=max(worst_pos,pe*1000);worst_rot=max(worst_rot,float(np.degrees(re)))
        bodies.append(placed(i,qs[i],f))
    alo=np.array([v[3] for v in bodies[0]]);ahi=np.array([v[4] for v in bodies[0]])
    blo=np.array([v[3] for v in bodies[1]]);bhi=np.array([v[4] for v in bodies[1]])
    candidates=np.all(np.minimum(ahi[:,None],bhi[None,:])-np.maximum(alo[:,None],blo[None,:])>1e-6,axis=-1)
    for ia,ib in np.argwhere(candidates):
        aa,bb=bodies[0][ia],bodies[1][ib]
        sep=np.any(np.min(bb[1]@aa[2][:,:3].T+aa[2][:,3],axis=0)>1e-6) or np.any(np.min(aa[1]@bb[2][:,:3].T+bb[2][:,3],axis=0)>1e-6)
        if not sep:hits.append([round(float(t),2),aa[0],bb[0]])
    for i in range(2):
        lows=[(v[0],float(v[3][2]-task.TOP)*1000) for v in bodies[i] if v[0] in ('gripper','tip_left','tip_right')]
        if min(v[1] for v in lows)<-.2:below.append([float(t),i,min(lows,key=lambda v:v[1])])
out=dict(scope='Observed-anchor continuation, 10 Hz IK + installed mesh convex-body pair screen; no loaded-grasp proof.',
         shape_count=len(shapes),nonmesh_colliders=nonmesh,max_position_mm=worst_pos,max_rotation_deg=worst_rot,
         pair_candidates=len(hits),below_table=len(below),examples=hits[:10]+below[:5],duration=tr.duration)
print(json.dumps(out,indent=2))
raise SystemExit(bool(worst_pos>1 or worst_rot>1 or hits or below))
