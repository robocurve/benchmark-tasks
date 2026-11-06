"""Build the static recovery hood using self-contained USD meshes.

Visual edge chamfers are 0.7 mm; simple panel collision envelopes retain the
original 28 mm opening and 20 mm observation slot. Requires usd-core, numpy,
and scipy only to regenerate; the committed USD needs none of these at run time.
"""
from pathlib import Path
import itertools
import numpy as np
from scipy.spatial import ConvexHull
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf

PANELS = {
 'back': ((.56,.055,.02),(.01,.18,.04)),
 'left': ((.45,.15,.02),(.21,.01,.04)),
 'right': ((.45,-.04,.02),(.21,.01,.04)),
 'roof_left': ((.45,.1175,.032),(.21,.065,.008)),
 'roof_right': ((.45,.01,.032),(.21,.11,.008)),
}
ORIGIN = np.array((.45,.055,0.))


def build(destination):
    stage=Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    root=UsdGeom.Xform.Define(stage,'/RecoveryHood')
    stage.SetDefaultPrim(root.GetPrim())
    def material(name,color,metal=.0,rough=.4):
        m=UsdShade.Material.Define(stage,'/RecoveryHood/Materials/'+name)
        shader=UsdShade.Shader.Define(stage,m.GetPath().AppendChild('Surface'))
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough)
        m.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        return m
    blue=material('powder_coat',(.20,.29,.36),.25,.36)
    metal=material('machined_steel',(.49,.54,.58),.82,.25)
    black=material('rubber',(.025,.033,.038),0,.7)
    yellow=material('safety_yellow',(.93,.62,.055),.1,.45)
    def panel(name,center,size,mat,collide=True,bevel=.0007):
        center=np.array(center)-ORIGIN
        half=np.array(size)/2
        b=min(bevel,float(min(half))*.35)
        verts=[]
        for signs in itertools.product((-1,1),repeat=3):
            corner=half*np.array(signs)
            for axis in range(3):
                point=corner.copy();point[axis]-=signs[axis]*b
                verts.append(point+center)
        verts=np.array(verts);hull=ConvexHull(verts)
        triangles=[]
        for tri,eq in zip(hull.simplices,hull.equations):
            tri=tri.copy()
            a,c,d=verts[tri]
            if np.dot(np.cross(c-a,d-a),eq[:3])<0:tri[[1,2]]=tri[[2,1]]
            triangles.append(tri)
        mesh=UsdGeom.Mesh.Define(stage,'/RecoveryHood/'+name)
        mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
        mesh.CreateFaceVertexCountsAttr([3]*len(triangles))
        mesh.CreateFaceVertexIndicesAttr(np.array(triangles).flatten().tolist())
        mesh.CreateSubdivisionSchemeAttr('none')
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        if collide:
            cube=UsdGeom.Cube.Define(stage,'/RecoveryHood/Colliders/'+name)
            cube.CreateSizeAttr(1.);cube.AddTranslateOp().Set(Gf.Vec3d(*center));cube.AddScaleOp().Set(Gf.Vec3f(*size))
            cube.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    for name,(center,size) in PANELS.items():panel(name,center,size,blue)
    # Four external mounting lugs and visible steel fasteners stay outside the cavity.
    for i,(x,y) in enumerate(itertools.product((.38,.525),(-.055,.165))):
        panel('mount_'+str(i),(x,y,.003),(.032,.021,.006),metal)
        bolt=UsdGeom.Cylinder.Define(stage,'/RecoveryHood/bolt_'+str(i))
        bolt.CreateRadiusAttr(.003);bolt.CreateHeightAttr(.0025)
        bolt.AddTranslateOp().Set(Gf.Vec3d(*(np.array((x,y,.00725))-ORIGIN)))
        UsdShade.MaterialBindingAPI.Apply(bolt.GetPrim()).Bind(black)
    # Slot trim explains the observation opening; trim remains on the roof itself.
    for i,y in enumerate((.063,.087)):
        panel('slot_trim_'+str(i),(.45,y,.0363),(.19,.002,.0006),metal,False,.0001)
    # Front edge warning strip shows the low access mouth, with no branding/text.
    for i in range(18):
        y=-.04+i*.010
        if .062<y<.088:continue
        panel('warning_'+str(i),(.3448,y,.032),(.0006,.009,.005),yellow if i%2 else black,False,.0001)
    # Fold seams and service-panel fasteners give the cover a readable scale.
    for i,(x,y) in enumerate(itertools.product((.365,.535),(-.025,.13))):
        bolt=UsdGeom.Cylinder.Define(stage,'/RecoveryHood/cover_screw_'+str(i))
        bolt.CreateRadiusAttr(.0023);bolt.CreateHeightAttr(.001)
        bolt.AddTranslateOp().Set(Gf.Vec3d(*(np.array((x,y,.0365))-ORIGIN)))
        UsdShade.MaterialBindingAPI.Apply(bolt.GetPrim()).Bind(metal)
    stage.GetRootLayer().customLayerData={'description':'Static unpowered low-profile machine recovery hood. Five panel collision envelopes; visual chamfers, mounting lugs, fasteners and slot trim.'}
    stage.GetRootLayer().Save()

if __name__=='__main__':
    build(Path(__file__).resolve().parent/'assets'/'recovery_hood.usdc')
