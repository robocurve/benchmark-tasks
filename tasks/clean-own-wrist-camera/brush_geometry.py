"""Small wooden flat cleaning brush, with a rounded grip and individual bristle fibres."""
from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
BRUSH='/World/envs/env_0/lens_brush'
ROOT_LOCAL_Z=.130
CREATED={}
def build_brush(stage):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf,Sdf
 from yam_arena import station
 cfg=json.loads(Path(__file__).with_name('pickup_plan.json').read_text());pos=cfg['brush_root_position'];rot=np.array(cfg['brush_rotation'])

 def vec(v):return Gf.Vec3f(*map(float,v))
 def quat(r):
  q=Rotation.from_matrix(r).as_quat();return Gf.Quatf(float(q[3]),vec(q[:3]))
 friction=UsdShade.Material.Define(stage,'/World/BrushGripFriction');m=UsdPhysics.MaterialAPI.Apply(friction.GetPrim());m.CreateStaticFrictionAttr(2.);m.CreateDynamicFrictionAttr(1.5);m.CreateRestitutionAttr(0.)
 def coll(o):
  prim=o.GetPrim();UsdPhysics.CollisionAPI.Apply(prim);ph=PhysxSchema.PhysxCollisionAPI.Apply(prim);ph.CreateContactOffsetAttr(.0002);ph.CreateRestOffsetAttr(0.);UsdShade.MaterialBindingAPI.Apply(prim).Bind(friction,UsdShade.Tokens.weakerThanDescendants,'physics')
 def surface(name,color,rough,metal=0):
  mat=UsdShade.Material.Define(stage,'/World/'+name);s=UsdShade.Shader.Define(stage,'/World/'+name+'/shader');s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(color);s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough);s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(float(metal));mat.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return mat
 wood=surface('BrushWood',(.57,.30,.105),.42);metal=surface('BrushFerrule',(.64,.66,.68),.26,1.);hair=surface('BrushHair',(.72,.53,.27),.8)
 root=UsdGeom.Xform.Define(stage,BRUSH);root.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));root.AddOrientOp().Set(quat(rot));UsdPhysics.RigidBodyAPI.Apply(root.GetPrim());UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.018)
 rb=PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim());rb.CreateSolverPositionIterationCountAttr(64);rb.CreateSolverVelocityIterationCountAttr(16);rb.CreateAngularDampingAttr(.2);PhysxSchema.PhysxContactReportAPI.Apply(root.GetPrim()).CreateThresholdAttr(0.)
 def rounded_profile(name,rings,material,power=.6):
  pts=[];theta=np.arange(48)*2*np.pi/48
  for z,rx,ry in rings:
   pts.extend([(float(rx*np.sign(np.cos(t))*abs(np.cos(t))**power),float(ry*np.sign(np.sin(t))*abs(np.sin(t))**power),float(z-ROOT_LOCAL_Z)) for t in theta])
  faces=[]
  for j in range(len(rings)-1):
   for k in range(48):faces.append([j*48+k,j*48+(k+1)%48,(j+1)*48+(k+1)%48,(j+1)*48+k])
  faces.extend([list(reversed(range(48))),list(range((len(rings)-1)*48,len(rings)*48))])
  mesh=UsdGeom.Mesh.Define(stage,BRUSH+'/'+name);mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([len(f) for f in faces]);mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);mesh.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material);return mesh
 # Rounded wooden paddle: broad at the pinch, narrowing naturally towards the ferrule.
 rounded_profile('rounded_wooden_handle',[(.090,.0005,.001),(.092,.003,.005),(.097,.005,.008),(.108,.0067,.0105),(.121,.007,.011),(.135,.0068,.0108),(.146,.0059,.0094),(.155,.0046,.0074),(.164,.0035,.0062),(.173,.0032,.006)],wood)
 def box(path,p,sz,c,collision=True,visible=True):
  o=UsdGeom.Cube.Define(stage,path);o.CreateSizeAttr(1);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));o.AddScaleOp().Set(vec(sz));o.CreateDisplayColorAttr([c])
  if collision:coll(o)
  if not visible:o.GetVisibilityAttr().Set('invisible')
  return o
 # Grasp collision follows the flat central grip, independent of visual fibre detail.
 box(BRUSH+'/grip_collision',[0,0,.126-ROOT_LOCAL_Z],[.0136,.0215,.036],[.5,.3,.1],visible=False)
 box(BRUSH+'/neck_collision',[0,0,.157-ROOT_LOCAL_Z],[.007,.012,.028],[.5,.3,.1],visible=False)
 rounded_profile('metal_ferrule',[(.170,.0036,.008),(.172,.0044,.0114),(.175,.0048,.0125),(.181,.0048,.0128),(.185,.0045,.0128)],metal,.35)
 for z in [.1732,.183]:
  rounded_profile('ferrule_rim_'+str(z).replace('.','_'),[(z-.00045,.0049,.013),(z,.0051,.0132),(z+.00045,.0049,.013)],metal,.35)
 box(BRUSH+'/ferrule_collision',[0,0,.178-ROOT_LOCAL_Z],[.009,.025,.014],[.6,.6,.6],visible=False)
 # A continuous tuft with a slight fan. Fibres taper and have uneven ends.
 rng=np.random.default_rng(731);vertices=[];counts=[];widths=[];colors=[]
 for i in range(7000):
  x=rng.uniform(-.004,.004);y=rng.uniform(-.012,.012);length=rng.uniform(.0205,.024);z0=.183;endx=x*.8+rng.normal(0,.00028);endy=y*1.075+rng.normal(0,.00024)
  for t in [0.,.35,.72,1.]:
   vertices.append([float(x*(1-t)+endx*t),float(y*(1-t)+endy*t),float(z0+length*t-ROOT_LOCAL_Z)]);widths.append(float(.00019*(1-t)+.00007*t));c=np.array([.42,.28,.13])*(1-t)+np.array([.83,.71,.48])*t;c+=rng.uniform(-.025,.025);colors.append(tuple(map(float,np.clip(c,0,1))))
  counts.append(4)
 curves=UsdGeom.BasisCurves.Define(stage,BRUSH+'/individual_soft_bristles');curves.CreateTypeAttr('linear');curves.CreateCurveVertexCountsAttr(counts);curves.CreatePointsAttr(vertices);curves.CreateWidthsAttr(widths);curves.SetWidthsInterpolation('vertex');curves.CreateDisplayColorPrimvar('vertex').Set(colors)
 # Bristle contact is represented by one aggregate patch at the tips.
 box(BRUSH+'/bristle_contact',[0,0,.1995-ROOT_LOCAL_Z],[.007,.025,.015],[.8,.7,.5],visible=False)
 # Clean low resting tray, with no enclosing walls to snag the fingers.
 X,Y,Z=pos
 box('/World/envs/env_0/brush_rest',[X,Y,station.TABLE_TOP+.002],[.042,.044,.004],[.26,.30,.32])
 # Low open upright holder: guide frame surrounds the ferrule well below the pinch.
 # Its opening clears the whole brush, and its top stays below the finger tips.
 z0=station.TABLE_TOP+.004;top=station.TABLE_TOP+.030
 for ix in [-1,1]:
  for iy in [-1,1]:
   rod=UsdGeom.Cylinder.Define(stage,f'/World/envs/env_0/brush_holder_post_{ix}_{iy}'.replace('-','m'));rod.CreateRadiusAttr(.002);rod.CreateHeightAttr(top-z0);rod.AddTranslateOp().Set(Gf.Vec3d(X+ix*.013,Y+iy*.019,(z0+top)/2));rod.CreateDisplayColorAttr([(.34,.40,.44)]);coll(rod)
 for side in [-1,1]:
  box('/World/envs/env_0/holder_rim_x_'+str(side).replace('-','m'),[X+side*.013,Y,top],[.004,.042,.004],[.34,.40,.44])
  box('/World/envs/env_0/holder_rim_y_'+str(side).replace('-','m'),[X,Y+side*.019,top],[.026,.004,.004],[.34,.40,.44])
 CREATED.update(root_position=pos,root_rotation=rot.tolist(),tip_local=[0,0,.077],mass_kg=.018,grasp_attachment=False)
