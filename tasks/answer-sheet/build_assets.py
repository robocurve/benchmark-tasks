"""Reuse a catalog marker's barrel; add an uncapped physical nib and collisions.

The source scan includes its cap as one mesh. Only its central barrel strip is
retained. No scan material, lettering or presumed removable cap is retained.
"""
from pathlib import Path
import hashlib
import urllib.request
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
from PIL import Image, ImageDraw, ImageFont

HERE=Path(__file__).resolve().parent
SOURCE='https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd'

def material(stage, name, color):
    m=UsdShade.Material.Define(stage,'/Asset/Materials/'+name)
    sh=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('Surface'))
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
    return m

def build(source=None):
    out=HERE/'assets';out.mkdir(exist_ok=True)
    if source is None:
        source=out/'catalog-marker.usd'
        if not source.exists():
            with urllib.request.urlopen(SOURCE,timeout=30) as response: source.write_bytes(response.read())
    src=Usd.Stage.Open(str(source))
    mesh=next(UsdGeom.Mesh(p) for p in src.Traverse() if p.IsA(UsdGeom.Mesh))
    xf=UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    pts=np.array([xf.Transform(p) for p in mesh.GetPointsAttr().Get()])
    # Put catalog Y along marker X. Trim away both scanned end assemblies.
    pts=pts[:,[1,0,2]]
    indices=list(mesh.GetFaceVertexIndicesAttr().Get()); faces=[]; counts=[];offset=0
    for n in mesh.GetFaceVertexCountsAttr().Get():
        face=indices[offset:offset+n];offset+=n
        if np.all(np.abs(pts[face,0]) < .043):faces.extend(face);counts.append(n)
    used=sorted(set(faces));lookup={old:new for new,old in enumerate(used)}
    pts=pts[used];faces=[lookup[i] for i in faces]
    s=Usd.Stage.CreateNew(str(out/'marker.usdc'))
    UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z')
    root=UsdGeom.Xform.Define(s,'/Asset');s.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(.025)
    white=material(s,'barrel',(.77,.78,.74));black=material(s,'nib',(.015,.025,.06));blue=material(s,'blue',(.025,.06,.4))
    barrel=UsdGeom.Mesh.Define(s,'/Asset/CatalogBarrel')
    barrel.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);barrel.CreateFaceVertexCountsAttr(counts);barrel.CreateFaceVertexIndicesAttr(faces)
    barrel.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(barrel.GetPrim()).Bind(white)
    body=UsdGeom.Cylinder.Define(s,'/Asset/BarrelCollision');body.CreateAxisAttr('X');body.CreateRadiusAttr(.0097);body.CreateHeightAttr(.100)
    UsdPhysics.CollisionAPI.Apply(body.GetPrim());body.CreateVisibilityAttr('invisible')
    for name,x,r,h,m in [('Shoulder',-.048,.007,.012,white),('Nib',-.059,.003,.014,black),('End',.049,.0097,.006,blue)]:
        c=UsdGeom.Cylinder.Define(s,'/Asset/'+name);c.CreateAxisAttr('X');c.CreateRadiusAttr(r);c.CreateHeightAttr(h);c.AddTranslateOp().Set(Gf.Vec3d(x,0,0));UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(m)
        # Only nib sphere is a writing contact; the shoulder is a real collider.
        if name!='Nib':UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    tip=UsdGeom.Sphere.Define(s,'/Asset/Tip');tip.CreateRadiusAttr(.002);tip.AddTranslateOp().Set(Gf.Vec3d(-.066,0,0));UsdPhysics.CollisionAPI.Apply(tip.GetPrim());UsdShade.MaterialBindingAPI.Apply(tip.GetPrim()).Bind(black)
    root.GetPrim().CreateAttribute('source:url',Sdf.ValueTypeNames.String).Set(SOURCE)
    root.GetPrim().CreateAttribute('source:sha256',Sdf.ValueTypeNames.String).Set(hashlib.sha256(Path(source).read_bytes()).hexdigest())
    s.GetRootLayer().Save()
    # Printed image is code-authored, in physical paper coordinates.
    from answer_sheet.layout import FORM_CENTER, FORM_SIZE, BOXES
    image=Image.new('RGB',(2350,2350),(248,248,241));d=ImageDraw.Draw(image)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',85)
    heading=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',95)
    def pixel(x,y):return (int((FORM_CENTER[1]+FORM_SIZE[1]/2-y)*10000),int((FORM_CENTER[0]+FORM_SIZE[0]/2-x)*10000))
    d.text(pixel(.390,.020),'ANSWER SHEET',font=heading,fill=(5,10,15),anchor='mm')
    for row,centers in enumerate(BOXES):
        d.text(pixel(centers[0][0],.118),str(row+1),font=heading,fill=(5,10,15),anchor='mm')
        for col,(x,y) in enumerate(centers):
            px,py=pixel(x,y)
            d.rectangle((px-110,py-110,px+110,py+110),outline=(5,10,15),width=8)
            d.text(pixel(x+.017,y),'ABCD'[col],font=font,fill=(5,10,15),anchor='mm')
    image.save(out/'form.png')
    print('Marker source SHA256:',hashlib.sha256(Path(source).read_bytes()).hexdigest())

if __name__=='__main__':
    import sys
    build(Path(sys.argv[1]) if len(sys.argv)>1 else None)
