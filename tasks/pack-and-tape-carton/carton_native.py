"""Independent finite-thickness rigid panels, native creases, measured tape cuts.

Cardboard is a hinged-panel approximation. Tape uses passive hinge restraint
until measured blade coverage cuts all three paths; it is not a film FEM.
"""
from pathlib import Path
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from model import CARTON_HEIGHT as H,CARTON_WIDTH as W
T=.003
from config import *
from carton_scene import layout

def box(s,p,size,pos,color,collision=True):
 c=UsdGeom.Cube.Define(s,p);c.CreateSizeAttr(1.);c.AddTranslateOp().Set(Gf.Vec3d(*pos));c.AddScaleOp().Set(Gf.Vec3f(*size));c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 if collision:
  UsdPhysics.CollisionAPI.Apply(c.GetPrim());a=PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim());a.CreateContactOffsetAttr(.0002);a.CreateRestOffsetAttr(0.)
 return c

def export(path,name):
 s=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');r=UsdGeom.Xform.Define(s,'/Object');s.SetDefaultPrim(r.GetPrim());UsdPhysics.RigidBodyAPI.Apply(r.GetPrim())
 parts,_=layout();parts={p['name']:p for p in parts};mass=.055 if name=='cutter' else parts[name]['mass'];UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass)
 if name=='cutter':
  box(s,'/Object/handle',(.10,.025,.025),(0,0,0),(1.,.28,.025));box(s,'/Object/guard',(.02,.03,.020),(.052,0,.002),(.10,.12,.14));box(s,'/Object/blade',(.014,.0005,.008),(.062,0,-.016),(.72,.75,.78))
 else:
  p=parts[name];box(s,'/Object/board',p['size'],(0,0,0),(.58,.36,.17))
  if name in ('north_bottom','south_bottom'):
   depth=W/2-T
   box(s,'/Object/tape',(p['size'][0],.024,.0001),(0,-depth/2+.009,-T/2-.0001),(.36,.23,.11),False)
  if name in ('east','west'):
   box(s,'/Object/end_tape',(.048,.0001,.040),(0,T/2+.0001,-H/2+.020),(.36,.23,.11),False)
 s.GetRootLayer().Save();return str(Path(path).resolve())

def install_joints(stage,parent,sealed=True):
 _,hs=layout()
 for h in hs:
  j=UsdPhysics.RevoluteJoint.Define(stage,parent+'/'+h['b']+'/'+h['name'])
  j.CreateBody0Rel().SetTargets([parent+'/'+h['a']]);j.CreateBody1Rel().SetTargets([parent+'/'+h['b']]);j.CreateAxisAttr(h['axis']);j.CreateLocalPos0Attr(Gf.Vec3f(*h['anchor_a']));j.CreateLocalPos1Attr(Gf.Vec3f(*h['anchor_b']));lock=sealed and h['sealed'];j.CreateLowerLimitAttr(0. if lock else h['low']);j.CreateUpperLimitAttr(0. if lock else h['high']);j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
  d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateTargetPositionAttr(h['target']);d.CreateStiffnessAttr(h['spring']);d.CreateDampingAttr(.001)
