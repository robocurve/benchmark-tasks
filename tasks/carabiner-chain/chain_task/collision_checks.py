"""Optional offline HPP-FCL checks using the actual converted robot hulls."""
from pathlib import Path
import numpy as np
import hppfcl as fcl
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from yam_arena import require_usd,YAM_URDF,station
from yam_arena.constants import ARM_JOINTS,FINGER_OPEN
from yam_arena.urdf import forward_kinematics

def collision_object(shape,rotation,position):return fcl.CollisionObject(shape,fcl.Transform3f(np.asarray(rotation,dtype=float),np.asarray(position,dtype=float)))

def hulls():
    stage=Usd.Stage.Open(str(require_usd()));cache=UsdGeom.XformCache();out=[]
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
        body=prim
        while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):body=body.GetParent()
        if not body or not prim.IsA(UsdGeom.Mesh):raise RuntimeError('Unsupported converted collision shape')
        xf=cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(body).GetInverse()
        points=[xf.Transform(Gf.Vec3d(*p)) for p in UsdGeom.Mesh(prim).GetPointsAttr().Get()]
        vector=(getattr(fcl,"StdVec_Vec3f",None) or getattr(fcl,"StdVec_Vec3s"))()
        for p in points:vector.append(np.array(p,dtype=float))
        out.append((body.GetName(),fcl.ConvexBase.convexHull(vector,True,'Qt'),float(np.linalg.norm(np.array(points),axis=1).max())))
    return out

HULLS=hulls()

def robot(q,grip,side):
    joints=dict(zip(ARM_JOINTS,map(float,q)));joints.update(joint7=float(FINGER_OPEN*grip),joint8=float(FINGER_OPEN*grip))
    tf=forward_kinematics(YAM_URDF,joints)
    return [(name,tf[name][:3,3]+station.BASES[side],collision_object(hull,tf[name][:3,:3],tf[name][:3,3]+station.BASES[side]),radius) for name,hull,radius in HULLS]

def asset(name,path,position,rotation=np.eye(3)):
    stage=Usd.Stage.Open(str(path));cache=UsdGeom.XformCache();out=[]
    for p in stage.Traverse():
        if not p.HasAPI(UsdPhysics.CollisionAPI):continue
        tf=cache.GetLocalToWorldTransform(p);center=np.array(tf.Transform(Gf.Vec3d(0.)))
        axes=np.column_stack([np.array(tf.Transform(Gf.Vec3d(*a)))-center for a in np.eye(3)])
        scale=np.linalg.norm(axes,axis=0);r=axes/scale
        if p.IsA(UsdGeom.Cube):shape=fcl.Box(*map(float,float(UsdGeom.Cube(p).GetSizeAttr().Get())*scale))
        elif p.IsA(UsdGeom.Capsule) or p.IsA(UsdGeom.Cylinder):
            obj=UsdGeom.Capsule(p) if p.IsA(UsdGeom.Capsule) else UsdGeom.Cylinder(p)
            if not np.allclose(scale,1.):raise RuntimeError('Nonuniform primitive scaling unsupported')
            shape=(fcl.Capsule if p.IsA(UsdGeom.Capsule) else fcl.Cylinder)(float(obj.GetRadiusAttr().Get()),float(obj.GetHeightAttr().Get()))
            axis=obj.GetAxisAttr().Get()
            if axis=='Y':r=r@np.array([[1.,0,0],[0,0,1],[0,-1,0]])
            elif axis=='X':r=r@np.array([[0.,0,1],[0,1,0],[-1,0,0]])
        else:raise RuntimeError('Unsupported task collider')
        out.append((name+'_'+p.GetName() if name=='stand' else name,collision_object(shape,rotation@r,position+rotation@center)))
    return out

def static_scene(task):
    from chain_task.geometry import STAND_POS
    return [('table',collision_object(fcl.Box(*station.TABLE_SIZE),np.eye(3),station.TABLE_CENTER))]+asset('stand',task/'assets/stand.usda',STAND_POS)

def against(shapes,obstacles,allowed=()):
    hits=set()
    for name,p,body,radius in shapes:
        if name in ('base','link1'):continue
        for label,obstacle in obstacles:
            if (name,label) in allowed:continue
            if fcl.collide(body,obstacle,fcl.CollisionRequest(),fcl.CollisionResult()):hits.add((name,label))
    return sorted(hits)

def cross(left,right):
    hits=set();p=np.array([x[1] for x in left]);q=np.array([x[1] for x in right]);ra=np.array([x[3] for x in left]);rb=np.array([x[3] for x in right]);close=np.linalg.norm(p[:,None,:]-q[None,:,:],axis=-1)<=ra[:,None]+rb[None,:]+.001
    for i,j in np.argwhere(close):
        n,_,a,_=left[i];m,_,b,_=right[j]
        if fcl.collide(a,b,fcl.CollisionRequest(),fcl.CollisionResult()):hits.add((n,m))
    return sorted(hits)
