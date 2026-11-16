"""Procedural free carton: four walls, eight ordinary flaps and twelve hinges.

Geometry export is implemented; native dynamics and a robot runtime are not
validated. In particular, closed-loop creases and finite-thickness folding
clearance require a real PhysX probe before claiming flattening works.
"""
import math
from pathlib import Path
from model import LENGTH as L,WIDTH as W,HEIGHT as H,BOARD as T
from crease import BOTTOM_CENTER,BOTTOM_LOW,BOTTOM_HIGH,BOTTOM_OFFSET,joint_parameters


def rotate_z(v,angle):
    c,s=math.cos(angle),math.sin(angle)
    return (c*v[0]-s*v[1],s*v[0]+c*v[1],v[2])


def rotate_x(v,angle):
    c,s=math.cos(angle),math.sin(angle)
    return (v[0],c*v[1]-s*v[2],s*v[1]+c*v[2])


def add(a,b):return tuple(x+y for x,y in zip(a,b))


def quat(z,x=0.):
    # USD scalar-first quaternion, Rz(z) Rx(x).
    return (math.cos(z/2)*math.cos(x/2),math.cos(z/2)*math.sin(x/2),
            math.sin(z/2)*math.sin(x/2),math.sin(z/2)*math.cos(x/2))


def layout(top_angle=270.,bottom_angle=0.):
    parts=[];hinges=[]
    walls=[('north',(0.,W/2,H/2),L,0.),
           ('east',(L/2,0.,H/2),W,-math.pi/2),
           ('south',(0.,-W/2,H/2),L,math.pi),
           ('west',(-L/2,0.,H/2),W,math.pi/2)]
    for name,center,length,yaw in walls:
        parts.append(dict(name=name,pos=center,quat=quat(yaw),size=(length-T,T,H),mass=.035))
        for end,angle,sign in [('top',top_angle,-1),('bottom',bottom_angle,1)]:
            theta=math.radians(sign*angle)
            # Inner short flaps lie one board thickness inside the outer pair.
            inset=T if name in ('east','west') else 0.
            outset=T if end=='top' else 0.
            hinge=add(center,rotate_z((0.,outset,(H/2-inset)*(1 if end=='top' else -1)),yaw))
            depth=W/2 if end=='top' else W/2-T
            # Shift BOTH bottom anchor Z values. At angle zero their shifts
            # cancel exactly; nonzero poses must be rebuilt from those anchors.
            shift=BOTTOM_OFFSET if end=='bottom' else 0.
            anchor_a=(0.,outset,hinge[2]-center[2]-shift)
            anchor_b=(0.,depth/2,-shift)
            relative=tuple(a-b for a,b in zip(anchor_a,rotate_x(anchor_b,theta)))
            flap=name+'_'+end
            parts.append(dict(name=flap,pos=add(center,rotate_z(relative,yaw)),quat=quat(yaw,theta),
                              size=(length-2*T,depth,T),mass=.018))
            hinges.append(dict(name=flap+'_hinge',a=name,b=flap,axis='X',
                               anchor_a=anchor_a,anchor_b=anchor_b,
                               low=-275. if end=='top' else BOTTOM_LOW,
                               high=0. if end=='top' else BOTTOM_HIGH,
                               frame_center=-135. if end=='top' else BOTTOM_CENTER,
                               spring=.0002 if end=='top' else 0.,
                               damping=.001,drive_units='degrees' if end=='top' else 'radians',
                               target=-top_angle if end=='top' else BOTTOM_CENTER,
                               sealed=end=='bottom' and name in ('north','south')))
    # Corner anchors are expressed in each body's local frame.
    corners=[(L/2,W/2,H/2),(L/2,-W/2,H/2),(-L/2,-W/2,H/2),(-L/2,W/2,H/2)]
    for i,(name,center,_,yaw) in enumerate(walls):
        next_name,next_center,_,next_yaw=walls[(i+1)%4];corner=corners[i]
        a=rotate_z(tuple(v-c for v,c in zip(corner,center)),-yaw)
        b=rotate_z(tuple(v-c for v,c in zip(corner,next_center)),-next_yaw)
        hinges.append(dict(name='corner_'+str(i),a=name,b=next_name,axis='Z',
                           anchor_a=a,anchor_b=b,low=-180.,high=180.,
                           spring=.002,damping=.001,drive_units='radians',target=-90.,sealed=False))
    return parts,hinges


def export(path,sealed_bottom=True):
    """Native USD export, requiring pxr from the pinned Isaac environment."""
    from pxr import Usd,UsdGeom,UsdPhysics,Gf,PhysxSchema
    stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    root=UsdGeom.Xform.Define(stage,'/Carton');stage.SetDefaultPrim(root.GetPrim())
    parts,hinges=layout()
    for part in parts:
        body=UsdGeom.Xform.Define(stage,'/Carton/'+part['name'])
        body.AddTranslateOp().Set(Gf.Vec3d(*part['pos']))
        q=part['quat'];body.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(part['mass'])
        panel=UsdGeom.Cube.Define(stage,str(body.GetPath())+'/board')
        panel.CreateSizeAttr(1.);panel.AddScaleOp().Set(Gf.Vec3f(*part['size']))
        panel.CreateDisplayColorAttr([Gf.Vec3f(.58,.37,.19)])
        UsdPhysics.CollisionAPI.Apply(panel.GetPrim())
        material=PhysxSchema.PhysxCollisionAPI.Apply(panel.GetPrim())
        material.CreateContactOffsetAttr(.0005);material.CreateRestOffsetAttr(0.)
    for h in hinges:
        joint=UsdPhysics.RevoluteJoint.Define(stage,'/Carton/joints/'+h['name'])
        joint.CreateBody0Rel().SetTargets(['/Carton/'+h['a']])
        joint.CreateBody1Rel().SetTargets(['/Carton/'+h['b']])
        joint.CreateAxisAttr(h['axis'])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*h['anchor_a']))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*h['anchor_b']))
        parameters=joint_parameters(h,sealed_bottom)
        center=parameters['center']
        if center:joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(math.radians(center)/2),Gf.Vec3f(math.sin(math.radians(center)/2),0,0)))
        joint.CreateLowerLimitAttr(parameters['low']);joint.CreateUpperLimitAttr(parameters['high'])
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateCollisionEnabledAttr(parameters['collision_enabled'])
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular')
        drive.CreateTypeAttr('force');drive.CreateTargetPositionAttr(parameters['target'])
        drive.CreateStiffnessAttr(parameters['stiffness']);drive.CreateDampingAttr(parameters['damping'])
    stage.GetRootLayer().Save();return str(Path(path).resolve())

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
    export(p.parse_args().output)
