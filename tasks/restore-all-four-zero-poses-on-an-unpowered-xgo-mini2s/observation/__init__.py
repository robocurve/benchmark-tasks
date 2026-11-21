"""Native sampled hand loads and full face-sampled hand geometry."""
import math
from functools import lru_cache
import numpy as np
import parts
_tip_contact_view=None
_arm_support_view=None
_arm_support_shape=None
_foot_view=None
_foot_shape=None

@lru_cache(maxsize=1)
def hand_mesh():
    from yam_arena import analysis
    out={}
    for name in ('gripper','tip_left','tip_right'):
        cloud=[]
        for h in analysis.link_hulls().get(name,[]):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7),axis=0):
                face=h.vertices[np.abs(h.vertices@pl[:3]-pl[3])<2e-6]
                if len(face)<3:raise ValueError('unknown hull face')
                c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(pl[:3],u)
                face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
                for a,b in zip(face,np.roll(face,-1,axis=0)):
                    n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
                    for i in range(n+1):
                        for j in range(n-i+1):cloud.append(c+(a-c)*(i/n)+(b-c)*(j/n))
        if not cloud:raise ValueError('missing hand geometry')
        cloud=np.asarray(cloud)
        _,keep=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True)
        out[name]=cloud[keep]
    return out

def fingertip_contact_forces(sim):
    global _tip_contact_view
    base=sim.env.unwrapped
    if _tip_contact_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths();env_path=base.scene.env_prim_paths[0]
        paths=[env_path+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in ('tip_left','tip_right')]
        _tip_contact_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            paths,filter_patterns=[],max_contact_data_count=256)
    raw=_tip_contact_view.get_net_contact_forces(dt=base.sim.get_physics_dt())
    if hasattr(raw,'torch'):raw=raw.torch
    force=raw.detach().cpu().numpy() if hasattr(raw,'detach') else np.asarray(raw.numpy())
    force=np.asarray(force).reshape(-1,3)
    if force.shape!=(4,3) or not np.isfinite(force).all():raise ValueError('unknown fingertip contact data')
    return force

def hand_state(sim):
    base=sim.env.unwrapped
    origin=parts._np(base.scene.env_origins[0]);clouds=[];finger=0.
    if origin.shape!=(3,) or not np.isfinite(origin).all():raise ValueError('unknown environment origin')
    for arm in ('left_robot','right_robot'):
        asset=base.scene[arm];poses=parts._np(asset.data.body_link_pose_w)[0]
        if poses.ndim!=2 or poses.shape[1]!=7 or not np.isfinite(poses).all() or np.any(np.linalg.norm(poses[:,3:],axis=1)<1e-6):raise ValueError('unknown robot body poses')
        if not np.isfinite(parts._np(asset.data.joint_pos)).all():raise ValueError('unknown robot joint state')
        for name,p in hand_mesh().items():
            pose=poses[asset.body_names.index(name)]
            if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all():raise ValueError('unknown hand mesh')
            cloud=p@parts._quat_to_matrix(pose[3:]).T+pose[:3]-origin
            if not np.isfinite(cloud).all():raise ValueError('unknown hand surface')
            clouds.append(cloud)
        # Native actuator/joint effort is recorded; this is not a calibration of
        # grip force. These20Hz samples flag simulator feasibility separately from the physical endpoint score.
        raw=asset.root_view.get_dof_projected_joint_forces()
        raw=raw.torch if hasattr(raw,'torch') else raw
        efforts=raw.detach().cpu().numpy()[0] if hasattr(raw,'detach') else np.asarray(raw.numpy())[0]
        values=[float(efforts[asset.joint_names.index(n)]) for n in ('joint7','joint8')]
        if not np.isfinite(values).all():raise ValueError('unknown finger forces')
        finger=max(finger,max(map(abs,values)))

    finger=max(finger,float(np.linalg.norm(fingertip_contact_forces(sim),axis=1).max()))
    return clouds,finger

def reset():
    global _tip_contact_view,_arm_support_view,_foot_view
    _tip_contact_view=None;_arm_support_view=None;_foot_view=None


def arm_support_force(sim):
    """Maximum native contact force between any arm link and any robot link."""
    global _arm_support_view,_arm_support_shape
    if _arm_support_view is None:
        from isaaclab_physx.physics import PhysxManager
        from yam_arena.robot import asset_link_paths
        env=sim.env.unwrapped.scene.env_prim_paths[0];links=asset_link_paths()
        paths=[env+'/'+arm+path for arm in ('LeftArm','RightArm') for name,path in links.items()]
        robot=[parts.ROOT_PATHS['xgo']+'/'+name for name,_,_ in parts._SPECS['xgo'].links]
        _arm_support_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(paths,filter_patterns=[robot]*len(paths),max_contact_data_count=1024)
        _arm_support_shape=(len(paths),len(robot),3)
    raw=_arm_support_view.get_contact_force_matrix(dt=sim.env.unwrapped.sim.get_physics_dt())
    raw=raw.torch if hasattr(raw,'torch') else raw
    value=raw.detach().cpu().numpy() if hasattr(raw,'detach') else np.asarray(raw.numpy())
    if value.shape!=_arm_support_shape or not np.isfinite(value).all():raise ValueError('unknown whole-arm contact data')
    return float(np.linalg.norm(value.reshape(-1,3),axis=1).max())


def foot_supports(sim):
    global _foot_view,_foot_shape
    if _foot_view is None:
        from isaaclab_physx.physics import PhysxManager
        from isaaclab.sim.utils import get_current_stage
        from pxr import UsdPhysics
        env=sim.env.unwrapped.scene.env_prim_paths[0]
        table=[str(p.GetPath()) for p in get_current_stage().Traverse() if str(p.GetPath()).startswith(env+'/') and 'table' in str(p.GetPath()).lower() and p.HasAPI(UsdPhysics.CollisionAPI)]
        if not table:raise ValueError('table contact geometry unavailable')
        feet=[parts.ROOT_PATHS['xgo']+'/'+leg+'_calf_joint' for leg in ('fl','fr','br','bl')]
        _foot_view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(feet,filter_patterns=[table]*4,max_contact_data_count=256)
        _foot_shape=(4,len(table),3)
    data=tuple(_contact_array(x) for x in _foot_view.get_contact_data(dt=sim.env.unwrapped.sim.get_physics_dt()))
    origin=_contact_array(sim.env.unwrapped.scene.env_origins[0])
    return distal_supports(sim.states()['xgo'],origin,data)

def _contact_array(value):
    value=value.torch if hasattr(value,'torch') else value
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    if hasattr(value,'numpy'):return np.asarray(value.numpy())
    return np.asarray(value)

@lru_cache(maxsize=1)
def _distal_regions():
    import pathlib,json
    rows=json.loads((pathlib.Path(__file__).resolve().parents[1]/'assets/native.json').read_text())['links']
    out=[]
    for leg in ('fl','fr','br','bl'):
        row=next(r for r in rows if r['name']==leg+'_calf_joint');piece=row['meshes'][0]['collision_parts'][2]
        v=np.asarray(piece['vertices']);centroid=v.mean(0);planes=[]
        for face in piece['faces']:
            a,b,c=v[face];normal=np.cross(b-a,c-a);length=np.linalg.norm(normal)
            if length<1e-12:continue
            normal/=length
            if normal@(centroid-a)>0:normal=-normal
            planes.append(np.r_[normal,normal@a])
        if not planes:raise ValueError('distal foot geometry unavailable')
        out.append((row['name'],np.asarray(row['origin']),np.asarray(planes)))
    return out

TOUCH=.0001  # a toe-hull contact point at most 0.1 mm from the tabletop: a 0.1 mm leaf cannot pass under the toe
def distal_supports(state,origin,data):
    """A toe is down when a contact point on its own toe hull touches the tabletop (separation at most 0.1 mm)
    or carries load. Heel or shaft contact never counts."""
    from yam_arena.task_file import TABLE_TOP
    fs,ps,ns,seps,counts,starts=data
    fs=np.asarray(fs,float).reshape(-1);ps=np.asarray(ps,float).reshape(-1,3);ns=np.asarray(ns,float).reshape(-1,3);seps=np.asarray(seps,float).reshape(-1)
    counts=np.asarray(counts);starts=np.asarray(starts)
    if counts.ndim!=2 or counts.shape[0]!=4 or starts.shape!=counts.shape:raise ValueError('unknown foot contact indices')
    if not np.isfinite(np.r_[counts.ravel(),starts.ravel()]).all() or np.any(counts!=np.floor(counts)) or np.any(starts!=np.floor(starts)):raise ValueError('invalid foot contact indices')
    if any(len(x)!=len(fs) for x in (ps,ns,seps)) or counts.sum()>=len(fs):raise ValueError('foot contact buffer may be truncated')
    result=[]
    for i,(name,datum,planes) in enumerate(_distal_regions()):
        p,q=state.links[name];r=parts._quat_to_matrix(q);load=0.;touch=False
        for j in range(counts.shape[1]):
            count=int(counts[i,j]);start=int(starts[i,j])
            if count<0 or count>64 or (count and (start<0 or start+count>len(fs))):raise ValueError('foot contact buffer overflow')
            for k in range(start,start+count):
                if not np.isfinite(np.r_[fs[k],ps[k],ns[k],seps[k]]).all() or fs[k]<0 or abs(np.linalg.norm(ns[k])-1)>.001:raise ValueError('invalid foot contact point')
                point=ps[k]-origin;local=(point-p)@r+datum
                in_shoe=bool(np.all(planes[:,:3]@local<=planes[:,3]+.0005))
                if in_shoe and abs(point[2]-TABLE_TOP['station'])<=.001 and seps[k]<=.001 and abs(ns[k,2])>=.5:
                    load+=fs[k]*abs(ns[k,2]);touch=touch or bool(seps[k]<=TOUCH)
        result.append(bool(touch or load>.02))
    return result
