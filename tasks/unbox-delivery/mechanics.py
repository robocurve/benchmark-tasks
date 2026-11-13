"""USD colliders, physical flap hinges and segmented tape latch.

Only tape fracture is procedural. Robot commands never set flap or item poses.
Import this module after Kit has started.
"""
from pathlib import Path
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
from config import *

def cube(stage,path,size,pos,color,collision=True):
    c=UsdGeom.Cube.Define(stage,path)
    c.CreateSizeAttr(1.)
    c.AddTranslateOp().Set(Gf.Vec3d(*pos)); c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        p=PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim())
        p.CreateContactOffsetAttr(.001); p.CreateRestOffsetAttr(0.)
    return c

def cutter_usd(path):
    stage=Usd.Stage.CreateNew(str(path)); UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    root=UsdGeom.Xform.Define(stage,'/Cutter'); stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.055)
    cube(stage,'/Cutter/handle',(.11,.025,.025),(0.,0.,0.),(1.,.28,.03))
    cube(stage,'/Cutter/guard',(.025,.03,.02),(.057,0.,.002),(.12,.12,.14))
    cube(stage,'/Cutter/blade',(.014,BLADE_THICKNESS,.008),(.065,0.,-.016),(.7,.73,.76))
    stage.GetRootLayer().Save(); return str(Path(path).resolve())

def add_flap(stage,prim_path,side):
    root=stage.GetPrimAtPath(prim_path)
    # Each plain flap is a rigid Xform with one cardboard collider. Joint anchors
    # are in metres and are not affected by a geometry scale.
    sign=1 if side=='left' else -1
    hinge=UsdPhysics.RevoluteJoint.Define(stage, prim_path+'/hinge')
    hinge.CreateBody1Rel().SetTargets([root.GetPath()])
    hinge.CreateAxisAttr('X')
    hinge.CreateLocalPos0Attr(Gf.Vec3f(CENTER[0], sign*WIDTH/2,LID_Z))
    hinge.CreateLocalPos1Attr(Gf.Vec3f(0.,sign*FLAP_WIDTH/2,0.))
    hinge.CreateLowerLimitAttr(0.); hinge.CreateUpperLimitAttr(0.)
    hinge.CreateExcludeFromArticulationAttr(True)
    return hinge

def wrapped_tape_usd(path,end):
    """Visible continuous wall-to-lid film; holding/fracture use the seam latch.

    Film is not an uncuttable rigid barrier. A cut through each top end fold is
    still required by the scorer before either physical hinge can be released.
    """
    stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    root=UsdGeom.Xform.Define(stage,'/Tape');stage.SetDefaultPrim(root.GetPrim())
    front=end=='front';i=0 if front else 1
    x=SEAM[i]+(-.0002 if front else .0002)
    inward=FLAP_END_X[i]-x
    color=(.34,.20,.08)
    cube(stage,'/Tape/wall',(.0001,TAPE_WIDTH,TAPE_DROP),(0.,0.,.0016),color,False)
    cube(stage,'/Tape/top_fold',(abs(inward),TAPE_WIDTH,.0001),
         (inward/2,0.,TAPE_DROP/2+.0016),color,False)
    stage.GetRootLayer().Save();return str(Path(path).resolve())
