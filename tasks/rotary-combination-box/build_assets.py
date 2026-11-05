"""Reuse a catalog wheel hub; add numbering and a physical gripping bar."""
from pathlib import Path
import hashlib
import math
import urllib.request
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
SOURCE='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Robots/NVIDIA/Carter/Props/carter_wheel_center.usd'

def material(s,name,color):
    m=UsdShade.Material.Define(s,'/Asset/Materials/'+name);sh=UsdShade.Shader.Define(s,m.GetPath().AppendChild('Surface'));sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65);sh.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.1)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m

def box(s,name,pos,size,mat,collision=True):
    b=UsdGeom.Cube.Define(s,'/Asset/'+name);b.CreateSizeAttr(1.);b.AddTranslateOp().Set(Gf.Vec3d(*pos));b.AddScaleOp().Set(Gf.Vec3f(*size));UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(mat)
    if collision:UsdPhysics.CollisionAPI.Apply(b.GetPrim())
    return b

def stage(name,rigid=False):
    s=Usd.Stage.CreateNew(str(HERE/'assets'/name));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z')
    root=UsdGeom.Xform.Define(s,'/Asset');s.SetDefaultPrim(root.GetPrim())
    if rigid:UsdPhysics.RigidBodyAPI.Apply(root.GetPrim());UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.08)
    return s

def build(source):
    out=HERE/'assets';out.mkdir(exist_ok=True)
    src=Usd.Stage.Open(str(source));mesh=next(UsdGeom.Mesh(p) for p in src.Traverse() if p.IsA(UsdGeom.Mesh));xf=UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    pts=np.asarray([xf.Transform(p) for p in mesh.GetPointsAttr().Get()]);center=(pts.min(axis=0)+pts.max(axis=0))/2
    pts=(pts-center)*(.035/.076)
    s=stage('wheel.usdc',True);metal=material(s,'hub',(.35,.39,.43));grip=material(s,'grip',(.08,.18,.25))
    m=UsdGeom.Mesh.Define(s,'/Asset/CatalogHub');m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);m.CreateFaceVertexCountsAttr(mesh.GetFaceVertexCountsAttr().Get());m.CreateFaceVertexIndicesAttr(mesh.GetFaceVertexIndicesAttr().Get());m.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(metal)
    collider=UsdGeom.Cylinder.Define(s,'/Asset/HubCollision');collider.CreateRadiusAttr(.035);collider.CreateHeightAttr(.0265);collider.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    box(s,'GripBar',(0,0,.019),(.045,.018,.012),grip)
    disc=UsdGeom.Mesh.Define(s,'/Asset/Digits');points=[(0,0,.0136)]+[(.0348*math.cos(i*math.tau/64),.0348*math.sin(i*math.tau/64),.0136) for i in range(64)]
    disc.CreatePointsAttr([Gf.Vec3f(*p) for p in points]);disc.CreateFaceVertexCountsAttr([3]*64);disc.CreateFaceVertexIndicesAttr([j for i in range(64) for j in (0,1+i,1+(i+1)%64)]);disc.CreateSubdivisionSchemeAttr('none')
    uv=UsdGeom.PrimvarsAPI(disc).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex');uv.Set([(p[0]/.07+.5,p[1]/.07+.5) for p in points])
    mat=material(s,'numbers',(.04,.05,.06));sh=UsdShade.Shader.Get(s,mat.GetPath().AppendChild('Surface'));tex=UsdShade.Shader.Define(s,mat.GetPath().AppendChild('Texture'));tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('digits.png'))
    reader=UsdShade.Shader.Define(s,mat.GetPath().AppendChild('UV'));reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result');sh.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb');UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(mat)
    s.GetDefaultPrim().CreateAttribute('source:url',Sdf.ValueTypeNames.String).Set(SOURCE);s.GetRootLayer().Save()
    image=Image.new('RGB',(700,700),(18,25,32));font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',65)
    for digit in range(10):
        angle=-digit*math.tau/10;tile=Image.new('RGBA',(100,100));ImageDraw.Draw(tile).text((50,50),str(digit),font=font,fill=(248,248,242,255),anchor='mm');tile=tile.rotate(math.degrees(angle)-90,resample=Image.Resampling.BICUBIC)
        x=350+270*math.cos(angle);y=350-270*math.sin(angle);image.paste(tile,(round(x-50),round(y-50)),tile)
    image.save(out/'digits.png')
    s=stage('fixture.usdc');body=material(s,'fixture',(.32,.38,.42));orange=material(s,'index',(.85,.22,.025));box(s,'Base',(0,0,.005),(.14,.14,.010),body)
    axle=UsdGeom.Cylinder.Define(s,'/Asset/Axle');axle.CreateRadiusAttr(.006);axle.CreateHeightAttr(.016);axle.AddTranslateOp().Set(Gf.Vec3d(0,0,.018));UsdPhysics.CollisionAPI.Apply(axle.GetPrim());UsdShade.MaterialBindingAPI.Apply(axle.GetPrim()).Bind(body)
    box(s,'IndexPost',(.045,0,.030),(.012,.018,.050),body)
    arrow=UsdGeom.Mesh.Define(s,'/Asset/Index');arrow.CreatePointsAttr([Gf.Vec3f(.036,0,.0555),Gf.Vec3f(.050,-.006,.0555),Gf.Vec3f(.050,.006,.0555)]);arrow.CreateFaceVertexCountsAttr([3]);arrow.CreateFaceVertexIndicesAttr([0,1,2]);arrow.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(arrow.GetPrim()).Bind(orange)
    s.GetRootLayer().Save()
    (out/'provenance.json').write_text(__import__('json').dumps({'source_url':SOURCE,'source_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),'terms_url':'https://www.nvidia.com/en-us/agreements/enterprise-software/isaac-sim-additional-software-and-materials-license/','changes':'Reuse complete wheel hub mesh at70 mm diameter. Replace catalog materials; add digit face, gripping bar, analytic cylinder collider,80 g mass, world revolute joint and soft periodic detents. Catalog provides geometry, not a combination lock mechanism.'},indent=2)+'\n')

def build_box(source):
    src=Usd.Stage.Open(str(source));mesh=UsdGeom.Mesh(src.GetPrimAtPath('/Root/Visuals/FOF_Mesh_Magenta_Box'));xf=UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    pts=np.asarray([xf.Transform(p) for p in mesh.GetPointsAttr().Get()]);lo=pts.min(axis=0);hi=pts.max(axis=0)
    pts=(pts-lo)/(hi-lo)*np.array([.14,.40,.060])+np.array([.25,-.20,0])
    s=stage('box.usdc');blue=material(s,'blue',(.10,.24,.35));metal=material(s,'plate',(.32,.38,.42));orange=material(s,'index',(.92,.24,.02))
    m=UsdGeom.Mesh.Define(s,'/Asset/CatalogStorageBox');m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);m.CreateFaceVertexCountsAttr(mesh.GetFaceVertexCountsAttr().Get());m.CreateFaceVertexIndicesAttr(mesh.GetFaceVertexIndicesAttr().Get());m.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(blue)
    box(s,'Floor',(.32,0,.004),(.14,.40,.008),blue)
    for name,pos,size in [('Near',(.253,0,.030),(.006,.40,.060)),('Far',(.387,0,.030),(.006,.40,.060)),('Left',(.32,.197,.030),(.14,.006,.060)),('Right',(.32,-.197,.030),(.14,.006,.060))]:box(s,name,pos,size,blue)
    box(s,'WheelLedge',(.19,0,.005),(.12,.40,.010),metal)
    for i,y in enumerate((.12,0,-.12)):
        axle=UsdGeom.Cylinder.Define(s,f'/Asset/Axle{i}');axle.CreateRadiusAttr(.006);axle.CreateHeightAttr(.016);axle.AddTranslateOp().Set(Gf.Vec3d(.19,y,.018));UsdPhysics.CollisionAPI.Apply(axle.GetPrim());UsdShade.MaterialBindingAPI.Apply(axle.GetPrim()).Bind(metal)
        box(s,f'IndexPost{i}',(.235,y,.030),(.012,.018,.050),metal)
        arrow=UsdGeom.Mesh.Define(s,f'/Asset/Index{i}');arrow.CreatePointsAttr([Gf.Vec3f(.226,y,.0555),Gf.Vec3f(.240,y-.006,.0555),Gf.Vec3f(.240,y+.006,.0555)]);arrow.CreateFaceVertexCountsAttr([3]);arrow.CreateFaceVertexIndicesAttr([0,1,2]);arrow.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(arrow.GetPrim()).Bind(orange)
    s.GetRootLayer().Save()
    s=stage('lid.usdc',True);UsdPhysics.MassAPI(s.GetDefaultPrim()).GetMassAttr().Set(.12);blue=material(s,'blue',(.10,.24,.35));grip=material(s,'handle',(.07,.09,.11))
    box(s,'Panel',(0,0,0),(.14,.40,.008),blue)
    # A raised grasping bar parallel to X, with clearance under its center.
    for i,x in enumerate((.023,.057)):box(s,f'HandlePost{i}',(x,-.12,.014),(.008,.018,.020),grip)
    box(s,'Handle',(.040,-.12,.026),(.055,.018,.012),grip)
    s.GetRootLayer().Save()
    import json
    pp=HERE/'assets/provenance.json';data=json.loads(pp.read_text());data['storage_box']={'source_url':'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Props/KLT_Bin/small_KLT_visual.usd','source_sha256':hashlib.sha256(Path(source).read_bytes()).hexdigest(),'changes':'Reuse catalog open-bin mesh, resized to140 by400 by60 mm. Add matching static collision walls, wheel ledge, and a separate140 by400 mm hinged lid with raised handle. Catalog labels are omitted.'};pp.write_text(json.dumps(data,indent=2)+'\n')

if __name__=='__main__':
    import sys
    build(Path(sys.argv[1]));build_box(Path(sys.argv[2]))
