"""Adapt a catalog marker barrel; author separate hollow, physically insertable caps."""
from pathlib import Path
import math,hashlib,json
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
HERE=Path(__file__).resolve().parent
SOURCE='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd'

def material(s,color):
 m=UsdShade.Material.Define(s,'/Asset/Material');sh=UsdShade.Shader.Define(s,'/Asset/Material/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.6);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m

def stage(name,mass):
 s=Usd.Stage.CreateNew(str(HERE/'assets'/name));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');r=UsdGeom.Xform.Define(s,'/Asset');s.SetDefaultPrim(r.GetPrim());UsdPhysics.RigidBodyAPI.Apply(r.GetPrim());UsdPhysics.MassAPI.Apply(r.GetPrim()).CreateMassAttr(mass);return s

def cylinder(s,name,x,r,h,mat,visible=True):
 c=UsdGeom.Cylinder.Define(s,'/Asset/'+name);c.CreateAxisAttr('X');c.CreateRadiusAttr(r);c.CreateHeightAttr(h);c.AddTranslateOp().Set(Gf.Vec3d(x,0,0));UsdPhysics.CollisionAPI.Apply(c.GetPrim());UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
 if not visible:c.CreateVisibilityAttr('invisible')

def build(source):
 (HERE/'assets').mkdir(exist_ok=True);src=Usd.Stage.Open(str(source));mesh=next(UsdGeom.Mesh(p) for p in src.Traverse() if p.IsA(UsdGeom.Mesh));xf=UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim());pts=np.array([xf.Transform(p) for p in mesh.GetPointsAttr().Get()])[:,[1,0,2]]
 counts=[];faces=[];offset=0;indices=list(mesh.GetFaceVertexIndicesAttr().Get())
 for n in mesh.GetFaceVertexCountsAttr().Get():
  f=indices[offset:offset+n];offset+=n
  if np.all(np.abs(pts[f,0])<.043):counts.append(n);faces.extend(f)
 used=sorted(set(faces));lookup={old:new for new,old in enumerate(used)};pts=pts[used];pts[:,0]*=1.4;faces=[lookup[i] for i in faces]
 for color,rgb in [('red',(.75,.045,.035)),('blue',(.035,.16,.80))]:
  s=stage('pen-'+color+'.usdc',.035);mat=material(s,rgb);m=UsdGeom.Mesh.Define(s,'/Asset/CatalogBarrel');m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);m.CreateFaceVertexCountsAttr(counts);m.CreateFaceVertexIndicesAttr(faces);m.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
  cylinder(s,'Barrel',0,.0097,.140,mat,False);cylinder(s,'Shoulder',-.0672,.007,.0168,mat);cylinder(s,'Nib',-.0826,.003,.0196,mat);cylinder(s,'End',.0686,.0097,.0084,mat)
  s.GetRootLayer().Save()
  s=stage('cap-'+color+'.usdc',.008);mat=material(s,rgb)
  # 16 independent convex wall boxes leave a continuous open insertion bore.
  # Inner apothem12.5 mm, outer radius about16.3 mm; no solid tube collider.
  for i in range(16):
   a=i*math.tau/16;b=UsdGeom.Cube.Define(s,f'/Asset/Wall{i:02}');b.CreateSizeAttr(1.);b.AddTranslateOp().Set(Gf.Vec3d(.0015,.01425*math.cos(a),.01425*math.sin(a)));b.AddRotateXOp().Set(math.degrees(a));b.AddScaleOp().Set(Gf.Vec3f(.032,.0035,.0064));UsdPhysics.CollisionAPI.Apply(b.GetPrim());UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(mat)
  cylinder(s,'ClosedEnd',-.016,.016,.003,mat);s.GetRootLayer().Save()
 data={'source_url':SOURCE,'source_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),'terms_url':'https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/','changes':'Reuse central catalog barrel mesh, trim scan ends and replace materials with red/blue. Add analytic uncapped pen ends/colliders,35g mass; axial scale1.4 to165.2mm overall length. Separate8g caps have sixteen convex wall segments and one closed end,35mm length and12.5mm inner apothem; the opening has no collision face. Physics retention uses a fixed joint created only after measured aligned insertion and seating, preserving the measured relative pose.'}
 (HERE/'assets/provenance.json').write_text(json.dumps(data,indent=2)+'\n')
def build_areas():
 from PIL import Image,ImageDraw,ImageFont
 for color,rgb,label in [('red',(249,219,215),'RED / LEFT'),('blue',(210,228,252),'BLUE / RIGHT')]:
  image=Image.new('RGB',(800,960),rgb);d=ImageDraw.Draw(image);d.rectangle((8,8,791,951),outline=(45,48,52),width=8);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',62);d.text((400,80),label,font=font,fill=(30,35,40),anchor='mm');image.save(HERE/'assets'/('area-'+color+'.png'))
  s=Usd.Stage.CreateNew(str(HERE/'assets'/('area-'+color+'.usdc')));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');root=UsdGeom.Xform.Define(s,'/Asset');s.SetDefaultPrim(root.GetPrim());m=UsdGeom.Mesh.Define(s,'/Asset/Print');m.CreatePointsAttr([Gf.Vec3f(-.12,-.10,0),Gf.Vec3f(.12,-.10,0),Gf.Vec3f(.12,.10,0),Gf.Vec3f(-.12,.10,0)]);m.CreateFaceVertexCountsAttr([4]);m.CreateFaceVertexIndicesAttr([0,1,2,3]);m.CreateSubdivisionSchemeAttr('none');UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(1,0),(1,1),(0,1),(0,0)])
  mat=material(s,(1,1,1));sh=UsdShade.Shader.Get(s,'/Asset/Material/Surface');tex=UsdShade.Shader.Define(s,'/Asset/Material/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('area-'+color+'.png'));reader=UsdShade.Shader.Define(s,'/Asset/Material/UV');reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result');sh.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat);s.GetRootLayer().Save()

if __name__=='__main__':
 import sys
 build(Path(sys.argv[1]));build_areas()
