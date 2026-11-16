"""Native articulated flexible-lock approximation and a physical dryer slider.

Refined rigid bundle capsules connected with passive bending joints. No fixed painted
hair sheet, no scripted lock animation and no state-driven drying shortcuts.
"""
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from config import *

def collider(g):
 UsdPhysics.CollisionAPI.Apply(g.GetPrim());a=PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim());a.CreateContactOffsetAttr(.00015);a.CreateRestOffsetAttr(0.)
def box(s,p,size,pos,color):
 c=UsdGeom.Cube.Define(s,p);c.CreateSizeAttr(1.);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size));c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);collider(c);return c
def export(path,name):
 s=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');r=UsdGeom.Xform.Define(s,'/Object');s.SetDefaultPrim(r.GetPrim())
 if name=='dryer_holder':
  x,y,z=DRYER_START
  box(s,'/Object/foot',(.12,.15,.012),(x+.035,y,TOP+.006),(.27,.30,.33))
  height=z-.026-TOP-.012
  for i,sign in enumerate([-1,1]):
   box(s,f'/Object/post_{i}',(.016,.016,height),(x+.060,y+sign*.055,TOP+.012+height/2),(.27,.30,.33))
   box(s,f'/Object/cradle_{i}',(.080,.015,.020),(x+.005,y+sign*.032,z-.026),(.27,.30,.33))
   box(s,f'/Object/bracket_{i}',(.016,.035,.020),(x+.060,y+sign*.043,z-.026),(.27,.30,.33))
 elif name=='scalp_collision':
  # A separate, physically attached skin body avoids the backend's adjacent
  # articulation-link collision filtering. Total head+skin mass remains650g.
  UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(.05)
  b=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim());b.CreateSolverPositionIterationCountAttr(32);b.CreateSolverVelocityIterationCountAttr(4)
  c=UsdGeom.Sphere.Define(s,'/Object/skin');c.CreateRadiusAttr(HEAD_RADIUS);c.CreateVisibilityAttr('invisible');collider(c)
 elif name=='head':
  UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(.60);b=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim());b.CreateSolverPositionIterationCountAttr(32);b.CreateSolverVelocityIterationCountAttr(4)
  c=UsdGeom.Sphere.Define(s,'/Object/scalp');c.CreateRadiusAttr(HEAD_RADIUS);c.CreateDisplayColorAttr([Gf.Vec3f(.75,.55,.42)])
  box(s,'/Object/neck',(.038,.045,.13),(0,0,-.11),(.75,.55,.42))
  nose=UsdGeom.Sphere.Define(s,'/Object/nose');nose.CreateRadiusAttr(.01);nose.AddTranslateOp().Set(Gf.Vec3d(-.065,0,0));nose.CreateDisplayColorAttr([Gf.Vec3f(.75,.55,.42)]);collider(nose)
 else:
  UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());mass=.48 if name=='dryer' else .003 if name=='switch' else DRY_MASS_PER_SEGMENT+PATCHES_PER_SEGMENT*WATER_PER_PATCH
  UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass);b=PhysxSchema.PhysxRigidBodyAPI.Apply(r.GetPrim());b.CreateSolverPositionIterationCountAttr(16);b.CreateSolverVelocityIterationCountAttr(4)
  if name=='dryer':
   c=UsdGeom.Cylinder.Define(s,'/Object/motor');c.CreateRadiusAttr(.029);c.CreateHeightAttr(.09);c.CreateAxisAttr('X');c.CreateDisplayColorAttr([Gf.Vec3f(.85,.035,.19)]);collider(c)
   c=UsdGeom.Cylinder.Define(s,'/Object/nozzle');c.CreateRadiusAttr(.018);c.CreateHeightAttr(.045);c.CreateAxisAttr('X');c.AddTranslateOp().Set(Gf.Vec3d(.065,0,0));c.CreateDisplayColorAttr([Gf.Vec3f(.12,.14,.17)]);collider(c)
   box(s,'/Object/handle',(.030,.026,.085),(-.019,0,-.065),(.85,.035,.19))
  elif name=='switch':box(s,'/Object/slider',(.006,.013,.014),(0,0,0),(.06,.07,.08))
  else:
   c=UsdGeom.Capsule.Define(s,'/Object/lock');c.CreateRadiusAttr(RADIUS);c.CreateHeightAttr(LENGTH-2*RADIUS);c.CreateAxisAttr('Z');c.CreateDisplayColorAttr([Gf.Vec3f(.055,.025,.012)]);collider(c)
 s.GetRootLayer().Save();return str(Path(path).resolve())
def install_joints(s,parent,include_dryer=True):
 # The ordinary stand fixes the mannequin head to the world. The head's
 # declared HEAD/HEAD_QUAT rest pose is physical, never maintained by pose writes.
 pivot=UsdPhysics.FixedJoint.Define(s,parent+'/head/neck_bearing');pivot.CreateBody1Rel().SetTargets([parent+'/head']);pivot.CreateLocalPos0Attr(Gf.Vec3f(HEAD[0],HEAD[1],TOP+.12));pivot.CreateLocalPos1Attr(Gf.Vec3f(0,0,TOP+.12-HEAD[2]));pivot.CreateLocalRot0Attr(Gf.Quatf(HEAD_QUAT[3],Gf.Vec3f(*HEAD_QUAT[:3])));pivot.CreateLocalRot1Attr(Gf.Quatf(1.));pivot.CreateExcludeFromArticulationAttr(True)
 for k in range(LOCKS):
  for n in range(SEGMENTS):
   child=f'{parent}/hair_{k}_{n}';child_q=segment_quat(k,n)
   if n:parent_name=f'{parent}/hair_{k}_{n-1}';parent_pos=segment(k,n-1);parent_q=segment_quat(k,n-1)
   else:parent_name=parent+'/head';parent_pos=HEAD;parent_q=HEAD_QUAT
   anchor=points(k)[n];a0=rotate(conjugate(parent_q),tuple(x-y for x,y in zip(anchor,parent_pos)));a1=rotate(conjugate(child_q),tuple(x-y for x,y in zip(anchor,segment(k,n))));relative=multiply(conjugate(child_q),parent_q)
   j=UsdPhysics.RevoluteJoint.Define(s,child+'/bend_'+str(k)+'_'+str(n));j.CreateBody0Rel().SetTargets([parent_name]);j.CreateBody1Rel().SetTargets([child]);j.CreateAxisAttr(bend_axis(k,n));j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(n == 0);j.CreateLocalPos0Attr(Gf.Vec3f(*a0));j.CreateLocalPos1Attr(Gf.Vec3f(*a1));j.CreateLocalRot1Attr(Gf.Quatf(relative[3],Gf.Vec3f(*relative[:3])));j.CreateLowerLimitAttr(-BEND_LIMIT_DEG);j.CreateUpperLimitAttr(BEND_LIMIT_DEG)
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.);d.CreateStiffnessAttr(BEND_STIFFNESS);d.CreateDampingAttr(BEND_DAMPING)
 if include_dryer:install_slider(s,parent)

def install_slider(s,parent):
 # A passive sampled two-well spring. A physics-substep callback selects the
 # nearest well from measured joint displacement; native drive supplies the
 # equal/opposite joint reaction. No policy phase or power flag selects it.
 j=UsdPhysics.PrismaticJoint.Define(s,parent+'/dryer/power_slider');j.CreateBody0Rel().SetTargets([parent+'/dryer']);j.CreateBody1Rel().SetTargets([parent+'/switch']);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(-.037,0,-.045));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateLowerLimitAttr(0.);j.CreateUpperLimitAttr(SWITCH_TRAVEL);j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
 # Preserve the authored dimensionless friction coefficient. PhysX applies
 # this property to articulations only; this excluded standalone joint does
 # not acquire a 0.08 N retention force from it.
 PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.08)
 d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateTargetPositionAttr(0.)
 d.CreateStiffnessAttr(SWITCH_DETENT_STIFFNESS);d.CreateDampingAttr(SWITCH_DETENT_DAMPING)


def export_hair(path):
 """One reduced-coordinate tree: scalp plus all passive lock segments."""
 directory=Path(path).parent
 names=['head']+[f'hair_{k}_{j}' for k in range(LOCKS) for j in range(SEGMENTS)]
 assets={name:export(directory/(name+'.usda'),name) for name in names}
 s=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');rootprim=UsdGeom.Xform.Define(s,'/Mannequin');s.SetDefaultPrim(rootprim.GetPrim())
 for name in names:
  prim=UsdGeom.Xform.Define(s,'/Mannequin/'+name);prim.GetPrim().GetReferences().AddReference(assets[name]);pose_hair_body(prim,name)
 install_joints(s,'/Mannequin',include_dryer=False)
 for prim in s.Traverse():
  if prim.IsA(UsdPhysics.Joint) and prim.GetName().startswith('bend_'):UsdPhysics.Joint(prim).GetExcludeFromArticulationAttr().Set(False)
 head=s.GetPrimAtPath('/Mannequin/head');UsdPhysics.ArticulationRootAPI.Apply(head);a=PhysxSchema.PhysxArticulationAPI.Apply(head);a.CreateEnabledSelfCollisionsAttr(True);a.CreateSolverPositionIterationCountAttr(32);a.CreateSolverVelocityIterationCountAttr(4)
 s.GetRootLayer().Save();return str(Path(path).resolve())

def install_skin_attachment(stage,parent):
 # Anatomical rigid attachment, never a robot grasp or controller constraint.
 j=UsdPhysics.FixedJoint.Define(stage,parent+'/scalp_collision/skin_attachment')
 j.CreateBody0Rel().SetTargets([parent+'/mannequin/head']);j.CreateBody1Rel().SetTargets([parent+'/scalp_collision'])
 j.CreateLocalPos0Attr(Gf.Vec3f(0,0,0));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)


def pose_hair_body(prim,name):
 """Author each rest body in the same declared head frame (USD XYZW)."""
 pos=HEAD if name=='head' else segment(*map(int,name.split('_')[1:]))
 q=HEAD_QUAT if name=='head' else segment_quat(*map(int,name.split('_')[1:]))
 prim.AddTranslateOp().Set(Gf.Vec3d(*pos))
 prim.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
