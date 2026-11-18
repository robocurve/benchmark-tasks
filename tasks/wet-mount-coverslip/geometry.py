"""Native real-size slide,22mm coverslip and passive pickup supports.

Alpha transparency is an optical approximation, not refractive glass. The
launcher installs the conservative liquid/air response separately. Optional
optical diagnostics below are disabled in the fixed submitted scene.
"""
import numpy as np
from surface_runtime import box,body,WORLD
BODIES=['wet_coverslip']
FIXED=['slide_support','wet_slide','cover_support']
SLIDE=np.array([.33,.025,.777]);COVER=np.array([.33,-.065,.784075])
TARGETS=[('slide',SLIDE),('coverslip',COVER)]
OPTICAL_AIR_RADIUS=None
WHITE_BACKING=False
NEEDLE_MODE=False
NEEDLE_X=False
ROTATE_SLIDE=False
NEEDLE_START=np.array([.34,.18,.817])

def needle(stage):
    """140mm/4g dissecting-needle proxy with actual free rigid contact.

    Length/mass follow Usbeck5470.100mm handle and40mm needle,6mm handle
    diameter and1mm-to-point taper are engineering approximations pending CAD.
    No hinge, actuator, tool/plate attachment or post-reset pose writes.
    """
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
    from scipy.spatial.transform import Rotation
    body(stage,WORLD+'/mounting_needle',NEEDLE_START,.004)
    q=Rotation.from_euler('z',0 if NEEDLE_X else -90,degrees=True).as_quat()
    UsdGeom.Xformable(stage.GetPrimAtPath(WORLD+'/mounting_needle')).AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))))
    for name,centre,length,radius,cone,color in [('handle',0.,.100,.003,False,(.15,.35,.55)),('steel',.070,.040,.0005,True,(.65,.68,.72))]:
        path=WORLD+'/mounting_needle/'+name
        sh=(UsdGeom.Cone if cone else UsdGeom.Cylinder).Define(stage,path);sh.CreateAxisAttr('X');sh.CreateHeightAttr(length);sh.CreateRadiusAttr(radius);sh.AddTranslateOp().Set(Gf.Vec3d(centre,0,0));sh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(sh.GetPrim());pc=PhysxSchema.PhysxCollisionAPI.Apply(sh.GetPrim());pc.CreateContactOffsetAttr(.00002);pc.CreateRestOffsetAttr(0.)
    for i,dy in enumerate([-.03,.03]):
        name=f'needle_rest{i}';box(stage,WORLD+'/'+name,[NEEDLE_START[0]+dy,NEEDLE_START[1],.78075] if NEEDLE_X else [NEEDLE_START[0],NEEDLE_START[1]+dy,.78075],[.006,.008,.0665] if NEEDLE_X else [.008,.006,.0665],[.25,.3,.34])
        if name not in FIXED:FIXED.append(name)
    if 'mounting_needle' not in BODIES:BODIES.append('mounting_needle')

def glass(stage,prim,path,ior):
    from isaaclab.sim.spawners.materials.visual_materials_cfg import GlassMdlCfg
    from isaaclab.sim.spawners.materials.visual_materials import spawn_from_mdl_file
    from pxr import UsdShade
    cfg=GlassMdlCfg(glass_color=(1.,1.,1.),frosting_roughness=0.,thin_walled=False,glass_ior=ior)
    spawn_from_mdl_file(path,cfg)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material.Get(stage,path))

def optical_film(stage,radius):
    """Frozen air cavity is a known optical specimen, never a scored bubble event."""
    from shapely.geometry import box as polygon_box,Point
    from shapely.geometry.polygon import orient
    from shapely.ops import triangulate
    from pxr import UsdGeom,UsdPhysics,Gf
    outer=polygon_box(-.011,-.011,.011,.011)
    # Boolean subtraction may reverse exterior/interior ring winding. Keep all
    # boundary normals outward so the refractive volume has a consistent medium.
    shape=orient(outer if radius==0 else outer.difference(Point(0,0).buffer(radius,quad_segs=32)),sign=1.)
    vertices=[];faces=[];z0=SLIDE[2]+.0005;z1=z0+.00002
    def face(points):
        start=len(vertices);vertices.extend(points);faces.append(list(range(start,len(vertices))))
    for t in triangulate(shape):
        if not shape.covers(t):continue
        xy=list(t.exterior.coords)[:-1]
        face([[SLIDE[0]+x,SLIDE[1]+y,z1] for x,y in xy]);face([[SLIDE[0]+x,SLIDE[1]+y,z0] for x,y in xy[::-1]])
    for boundary in [shape.exterior,*shape.interiors]:
        coords=list(boundary.coords)
        for (x,y),(xx,yy) in zip(coords[:-1],coords[1:]):
            face([[SLIDE[0]+x,SLIDE[1]+y,z0],[SLIDE[0]+xx,SLIDE[1]+yy,z0],[SLIDE[0]+xx,SLIDE[1]+yy,z1],[SLIDE[0]+x,SLIDE[1]+y,z1]])
    m=UsdGeom.Mesh.Define(stage,WORLD+'/known_water_film');m.CreatePointsAttr(vertices);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDoubleSidedAttr(True)
    glass(stage,m.GetPrim(),WORLD+'/water_optics',1.333)
    j=UsdPhysics.FixedJoint.Define(stage,WORLD+'/optical_cover_holder');j.CreateBody1Rel().SetTargets([WORLD+'/wet_coverslip'])
    pos=SLIDE+[0,0,.0005+.00002+.000075]
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,pos)));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))

def build(stage):
    if NEEDLE_MODE:needle(stage)
    box(stage,WORLD+'/slide_support',[SLIDE[0],SLIDE[1],.762],[.030,.08,.029] if ROTATE_SLIDE else [.08,.030,.029],[.88,.88,.88] if WHITE_BACKING else [.12,.17,.23])
    slide=box(stage,WORLD+'/wet_slide',SLIDE,[.025,.075,.001] if ROTATE_SLIDE else [.075,.025,.001],[.74,.83,.88],opacity=.5)
    if OPTICAL_AIR_RADIUS is not None:glass(stage,slide.GetPrim(),WORLD+'/slide_optics',1.52)
    # Optical test specimen, NOT simulated tissue and NOT a scored substitute.
    box(stage,WORLD+'/optical_sample',SLIDE+[0,0,.000501],[.006,.006,.000002],[.7,.28,.4],False)
    box(stage,WORLD+'/sample_stripe',SLIDE+[0,0,.0005023],[.0005,.006,.000001],[.22,.13,.17],False)
    box(stage,WORLD+'/cover_support',[COVER[0],COVER[1],.76575],[.010,.010,.0365],[.15,.19,.25])
    # Glass rho=2500 kg/m^3, 22x22x0.15 mm -> 0.1815 g.
    pos=COVER if OPTICAL_AIR_RADIUS is None else SLIDE+[0,0,.0005+.00002+.000075]
    body(stage,WORLD+'/wet_coverslip',pos,.022*.022*.00015*2500)
    c=box(stage,WORLD+'/wet_coverslip/glass',[0,0,0],[.022,.022,.00015],[.8,.88,.92],opacity=.35)
    if OPTICAL_AIR_RADIUS is not None:
        glass(stage,c.GetPrim(),WORLD+'/cover_optics',1.52);optical_film(stage,OPTICAL_AIR_RADIUS)
