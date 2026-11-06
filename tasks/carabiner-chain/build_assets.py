"""Rebuild original analytic USD geometry; no third-party asset redistribution."""
from pathlib import Path
import hashlib,json
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf
from chain_task.geometry import *
ROOT=Path(__file__).resolve().parent

def stage(name,mass=None):
 s=Usd.Stage.CreateNew(str(ROOT/'assets'/(name+'.usda')));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');p=UsdGeom.Xform.Define(s,'/Asset').GetPrim();s.SetDefaultPrim(p)
 if mass is not None:
  UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass);p.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(list(p.GetAppliedSchemas())+['PhysxRigidBodyAPI']));p.CreateAttribute('physxRigidBody:enableCCD',Sdf.ValueTypeNames.Bool).Set(True)
 return s

def color(shape,value):shape.CreateDisplayColorAttr([Gf.Vec3f(*value)]);UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
def rod(s,name,a,b,radius,rgb):
 a,b=np.asarray(a),np.asarray(b);v=b-a;length=np.linalg.norm(v);v/=length;helper=np.array([0.,0.,1.]) if abs(v[2])<.9 else np.array([1.,0.,0.]);u=np.cross(helper,v);u/=np.linalg.norm(u);rotation=np.column_stack([u,np.cross(v,u),v]);m=Gf.Matrix3d(*map(float,rotation.T.reshape(-1)))
 c=UsdGeom.Capsule.Define(s,'/Asset/'+name);c.CreateAxisAttr('Z');c.CreateRadiusAttr(radius);c.CreateHeightAttr(float(length));c.AddTranslateOp().Set(Gf.Vec3d(*map(float,(a+b)/2)));c.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(m.ExtractRotation().GetQuat());color(c,rgb)
def box(s,name,p,size,rgb):
 c=UsdGeom.Cube.Define(s,'/Asset/'+name);c.CreateSizeAttr(1.);c.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));c.AddScaleOp().Set(Gf.Vec3f(*size));color(c,rgb)
def build():
 (ROOT/'assets').mkdir(exist_ok=True)
 for i,rgb in enumerate(COLORS):
  s=stage('frame'+str(i),FRAME_MASS)
  for j,(a,b) in enumerate(zip(FRAME_POINTS[:-1],FRAME_POINTS[1:])):rod(s,'Arc'+str(j),a,b,RADIUS,rgb)
  s.GetRootLayer().Save()
 s=stage('gate',GATE_MASS);c=UsdGeom.Cylinder.Define(s,'/Asset/Gate');c.CreateAxisAttr('Y');c.CreateHeightAttr(GATE_LENGTH);c.CreateRadiusAttr(RADIUS);color(c,(.63,.65,.67));s.GetRootLayer().Save()
 s=stage('stand');box(s,'Base',[0,0,.010],[.16,.12,.020],(.16,.18,.19));rod(s,'Pole',[0,0,.02],[0,0,.30],.012,(.31,.33,.35));eye=EYE_POINTS@EYE_ROT.T+EYE_OFFSET
 for i,(a,b) in enumerate(zip(eye[:-1],eye[1:])):rod(s,'Eye'+str(i),a,b,EYE_RADIUS,(.58,.60,.61))
 rod(s,'Mount',[0,0,EYE_OFFSET[2]+.025],EYE_OFFSET+[0,0,.025],.006,(.31,.33,.35));s.GetRootLayer().Save()
 for p in (ROOT/'assets').glob('*.usda'):p.write_text(p.read_text().rstrip()+'\n')
 hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'assets').glob('*.usda'))};(ROOT/'assets/hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
 (ROOT/'assets/provenance.json').write_text(json.dumps(dict(source='Original analytic geometry authored for this task',third_party_geometry_redistributed=False,discovery=dict(catalog='Isaac6.0 SimReady',keys_scanned=51463,pages=52,carabiner_matches=0,installed_arena_registry_match=False),dimensions_m=dict(frame_rod_radius=RADIUS,gate_length=GATE_LENGTH,eye_rod_radius=EYE_RADIUS),masses_kg=dict(frame=FRAME_MASS,gate=GATE_MASS),approximations=['Rigid accessory frames made from compound capsules, preserving open holes','Cylinder gate with passive spring and centered angular stops; no locking sleeve','Fixed D eye and stand; no completed-link freezing or attachments']),indent=2)+'\n')
if __name__=='__main__':build()
