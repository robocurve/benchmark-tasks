"""Adapt a catalog marker barrel; author separate hollow, physically insertable caps."""
from pathlib import Path
import math,hashlib,json
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
from pen_capping import retention as fit
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
  build_cap(color,rgb)
 data={'source_url':SOURCE,'source_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),'terms_url':'https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/','changes':'Reuse central catalog barrel mesh, trim scan ends, axial scale1.4 and red/blue materials. Analytic pen ends/colliders,35g. Caps8g with16 rigid wall segments split axially around a recessed bonded silicone liner seat. Liner19.0mm free ID,25.8mm outer across flats,6mm grip length plus1.5mm tapered entrance to21mm ID. Visible compliant zone has no collider; rigid backing21.2mm across flats bounds clearance. Geometry-gated pose-preserving joint approximates uncalibrated retention; no physical elasticity or contact-only retention is simulated.'}
 (HERE/'assets/provenance.json').write_text(json.dumps(data,indent=2)+'\n')
def ring_segment(s,name,x0,x1,inner0,inner1,outer,angle0,angle1,mat,collision=False):
 # A convex trapezoidal prism per backing segment; visible silicone is separate.
 points=[]
 for x,inner in ((x0,inner0),(x1,inner1)):
  for r,a in ((inner,angle0),(outer,angle0),(outer,angle1),(inner,angle1)):
   points.append(Gf.Vec3f(x,r*math.cos(a),r*math.sin(a)))
 m=UsdGeom.Mesh.Define(s,'/Asset/'+name);m.CreatePointsAttr(points)
 m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr([3,2,1,0,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]);m.CreateSubdivisionSchemeAttr('none')
 UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
 if collision:
  UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull');m.CreateVisibilityAttr('invisible')
 return m

def build_cap(color,rgb):
 s=stage('cap-'+color+'.usdc',.008);mat=material(s,rgb)
 # Preserve outer dimensions; recess the last 7.5mm by 0.4mm for bonded liner.
 for i in range(16):
  a=i*math.tau/16
  for suffix,x0,x1,inner in (('',-.0145,fit.LINER_START,fit.SHELL_APOTHEM),('Seat',fit.LINER_START,fit.MOUTH_X,fit.LINER_OUTER_APOTHEM)):
   b=UsdGeom.Cube.Define(s,f'/Asset/Wall{i:02}'+suffix);b.CreateSizeAttr(1.)
   center=(.016+inner)/2;b.AddTranslateOp().Set(Gf.Vec3d((x0+x1)/2,center*math.cos(a),center*math.sin(a)));b.AddRotateXOp().Set(math.degrees(a));b.AddScaleOp().Set(Gf.Vec3f(x1-x0,.016-inner,.0064));UsdPhysics.CollisionAPI.Apply(b.GetPrim());UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(mat)
 cylinder(s,'ClosedEnd',-.016,.016,.003,mat)
 # Neutral silicone contrasts with both shell colors; free aperture is visible.
 liner=UsdShade.Material.Define(s,'/Asset/Silicone');shader=UsdShade.Shader.Define(s,'/Asset/Silicone/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.72,.72,.65));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.8);liner.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 for i in range(64):
  a0=(i-.5)*math.tau/64;a1=(i+.5)*math.tau/64
  # Outer polygon shares the shell seat's 16 planes. Each endpoint is computed
  # independently so no silicone extends through the PP wall.
  for name,x0,x1,r0,r1 in (('Grip',fit.LINER_START,fit.LINER_END,fit.LINER_INNER_RADIUS,fit.LINER_INNER_RADIUS),('LeadIn',fit.LINER_END,fit.MOUTH_X,fit.LINER_INNER_RADIUS,fit.LEAD_IN_RADIUS)):
   m=ring_segment(s,f'Liner{name}{i:02}',x0,x1,r0,r1,fit.LINER_OUTER_APOTHEM,a0,a1,liner)
   points=list(m.GetPointsAttr().Get())
   for j in (1,2,5,6):
    a=a0 if j in (1,5) else a1;delta=(a+math.pi/16)%(math.tau/16)-math.pi/16;r=fit.LINER_OUTER_APOTHEM/math.cos(delta);points[j]=Gf.Vec3f(points[j][0],r*math.cos(a),r*math.sin(a))
   m.GetPointsAttr().Set(points)
 for i in range(16):
  a0=(i-.5)*math.tau/16;a1=(i+.5)*math.tau/16
  ring_segment(s,f'LinerBacking{i:02}',fit.LINER_START,fit.MOUTH_X,fit.BACKING_INNER_RADIUS/math.cos(math.pi/16),fit.BACKING_INNER_RADIUS/math.cos(math.pi/16),fit.LINER_OUTER_APOTHEM/math.cos(math.pi/16),a0,a1,liner,True)
 s.GetRootLayer().Save()

def build_areas():
 from PIL import Image,ImageDraw,ImageFont
 for color,rgb,label in [('red',(249,219,215),'RED / LEFT'),('blue',(210,228,252),'BLUE / RIGHT')]:
  image=Image.new('RGB',(800,960),rgb);d=ImageDraw.Draw(image);d.rectangle((8,8,791,951),outline=(45,48,52),width=8);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',62);d.text((400,80),label,font=font,fill=(30,35,40),anchor='mm');image.save(HERE/'assets'/('area-'+color+'.png'))
  s=Usd.Stage.CreateNew(str(HERE/'assets'/('area-'+color+'.usdc')));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');root=UsdGeom.Xform.Define(s,'/Asset');s.SetDefaultPrim(root.GetPrim());m=UsdGeom.Mesh.Define(s,'/Asset/Print');m.CreatePointsAttr([Gf.Vec3f(-.12,-.10,0),Gf.Vec3f(.12,-.10,0),Gf.Vec3f(.12,.10,0),Gf.Vec3f(-.12,.10,0)]);m.CreateFaceVertexCountsAttr([4]);m.CreateFaceVertexIndicesAttr([0,1,2,3]);m.CreateSubdivisionSchemeAttr('none');UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(1,0),(1,1),(0,1),(0,0)])
  mat=material(s,(1,1,1));sh=UsdShade.Shader.Get(s,'/Asset/Material/Surface');tex=UsdShade.Shader.Define(s,'/Asset/Material/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('area-'+color+'.png'));reader=UsdShade.Shader.Define(s,'/Asset/Material/UV');reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result');sh.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat);s.GetRootLayer().Save()

if __name__=='__main__':
 import sys
 if sys.argv[1:] == ['--caps-only']:
  for color,rgb in [('red',(.75,.045,.035)),('blue',(.035,.16,.80))]:build_cap(color,rgb)
  data=json.loads((HERE/'assets/provenance.json').read_text());data['changes']='Central catalog barrel adaptation unchanged. Caps now include recessed liner seats, visible silicone grip/lead-in and rigid outer backing; see retention_revision for dimensions and approximation.';data['retention_revision']={'liner_free_id_m':2*fit.LINER_INNER_RADIUS,'liner_outer_across_flats_m':2*fit.LINER_OUTER_APOTHEM,'grip_interval_m':[fit.LINER_START,fit.LINER_END],'lead_in_interval_m':[fit.LINER_END,fit.MOUTH_X],'lead_in_id_m':2*fit.LEAD_IN_RADIUS,'backing_clearance_across_flats_m':2*fit.BACKING_INNER_RADIUS,'approximation':'Visible silicone inner zone has no collision; outer backing is rigid. Geometry-gated fixed joint, uncalibrated compliance/forces.'};(HERE/'assets/provenance.json').write_text(json.dumps(data,indent=2)+'\n')
 else:build(Path(sys.argv[1]));build_areas()
