"""Finite wound ribbon, ordinary hollow roll and native peel constraints.

The active outer 360 mm is represented explicitly, as inextensible rigid film
cells with width-axis hinges. This is a coarse ribbon approximation, not film
FEM. Wound tack uses native breakable constraints; pulling must do physical
work to free it. There are no target-following tape nodes or pose writes.
"""
from pathlib import Path
import math
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from config import *

def cube(s,p,size,pos,color,contact=.00001):
 c=UsdGeom.Cube.Define(s,p);c.CreateSizeAttr(1.);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size));c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim());a=PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim());a.CreateContactOffsetAttr(contact);a.CreateRestOffsetAttr(0.);return c

def export(path,name):
 s=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');r=UsdGeom.Xform.Define(s,'/Object');s.SetDefaultPrim(r.GetPrim());UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());mass=.16 if name=='roll' else 950.*CELL_LENGTH*TAPE_THICKNESS*TAPE_WIDTH;UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass);PhysxSchema.PhysxContactReportAPI.Apply(r.GetPrim()).CreateThresholdAttr(0.);body=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim());body.CreateSolverPositionIterationCountAttr(128);body.CreateSolverVelocityIterationCountAttr(4)
 if name=='roll':
  # Compound annulus leaves the 76.2 mm core open to normal fingers.
  ri=CORE_RADIUS;ro=ACTIVE_INNER_RADIUS-TAPE_THICKNESS/2;rm=(ri+ro)/2
  for k in range(96):
   theta=2*math.pi*k/96;c=cube(s,f'/Object/wound_{k}',(ro-ri,2*rm*math.tan(math.pi/96),TAPE_WIDTH),(rm*math.cos(theta),rm*math.sin(theta),0),(.72,.56,.27),.00002);ops=c.GetOrderedXformOps();turn=c.AddRotateZOp();turn.Set(math.degrees(theta));c.SetXformOpOrder([ops[0],turn,ops[1]])
 else:cube(s,'/Object/film',(CELL_LENGTH,TAPE_THICKNESS,TAPE_WIDTH),(0,0,0),(.76,.61,.33))
 s.GetRootLayer().Save();return str(Path(path).resolve())

def install_joints(s,parent):
 from carton_native import install_joints as carton_joints
 carton_joints(s,parent,sealed=True)
 for i in range(CELLS):
  path=parent+f'/tape_{i}';pos,yaw=tape_cell(i);relative=tuple(a-b for a,b in zip(pos,ROLL_START))
  # A genuinely physical, finite-strength wound bond. Native solver may break
  # it under load; no controller method deletes it or declares peeled material.
  j=UsdPhysics.FixedJoint.Define(s,path+'/wound_tack');j.CreateBody0Rel().SetTargets([parent+'/roll']);j.CreateBody1Rel().SetTargets([path]);j.CreateLocalPos0Attr(Gf.Vec3f(*relative));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateLocalRot0Attr(Gf.Quatf(math.cos(yaw/2),Gf.Vec3f(0,0,math.sin(yaw/2))));j.CreateBreakForceAttr(2.);j.CreateBreakTorqueAttr(.008);j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
  if i:
   # Film remains connected after peeling. A blade still has to sever a hinge.
   prev,pyaw=tape_cell(i-1);anchor=tuple((u+v)/2 for u,v in zip(prev,pos))
   def local(v,origin,angle):
    x,y,z=(a-b for a,b in zip(v,origin));return (math.cos(angle)*x+math.sin(angle)*y,-math.sin(angle)*x+math.cos(angle)*y,z)
   j=UsdPhysics.RevoluteJoint.Define(s,path+'/film_bend');j.CreateBody0Rel().SetTargets([parent+f'/tape_{i-1}']);j.CreateBody1Rel().SetTargets([path]);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(*local(anchor,prev,pyaw)));j.CreateLocalPos1Attr(Gf.Vec3f(*local(anchor,pos,yaw)));j.CreateLowerLimitAttr(-180.);j.CreateUpperLimitAttr(180.);j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.);d.CreateStiffnessAttr(.0000001);d.CreateDampingAttr(.00000001)
