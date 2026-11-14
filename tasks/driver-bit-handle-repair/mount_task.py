"""Repair-scene prototype with real-scale hex insertion and spring retention.
Drawer is a static context asset: no motor, threaded fastener or repair score yet.
"""
import math
import pathlib
import sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import partkit
from partkit import part, Slider
from yam_arena.task_file import TABLE_TOP

TOP=TABLE_TOP['station']
DX,DY=.18,.035
BX,BY=.10,-.13
SCENE='station'
INSTRUCTION='Insert the hex bit into the holder, release it, then lift the holder.'
SCORE_OVER='end'
GREY=(.6,.62,.66)
BLUE=(.1,.25,.42)
DARK=(.045,.05,.055)
ORANGE=(.92,.38,.06)

old_spawn=partkit._spawn_part
def spawn(prim_path,cfg,*args,**kwargs):
    result=old_spawn(prim_path,cfg,*args,**kwargs)
    if cfg.part_name=='driver':
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        from isaaclab.sim.utils.stage import get_current_stage
        stage=get_current_stage()
        # Hex socket lead-in: tangible tapered walls, not pose snapping.
        for k in range(6):
            a=k*math.pi/3
            n=(math.cos(a),math.sin(a)); tangent=(-n[1],n[0])
            pts=[]
            for z,inner in ((.120,.0035),(.130,.0065)):
                for radius,t in ((inner,-.0045),(inner,.0045),(.010,.0045),(.010,-.0045)):
                    pts.append(Gf.Vec3f(radius*n[0]+t*tangent[0],radius*n[1]+t*tangent[1],z))
            faces=[[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
            m=UsdGeom.Mesh.Define(stage,prim_path+f'/body/lead_in_{k}')
            m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([4]*6)
            m.CreateFaceVertexIndicesAttr([v for f in faces for v in f])
            m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(*GREY)])
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
            PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim()).CreateContactOffsetAttr(.0001)
    if cfg.part_name in ('bit','bit_hex','bit_flat'):
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        from isaaclab.sim.utils.stage import get_current_stage
        stage=get_current_stage()
        stage.RemovePrim(prim_path+'/body/box_0')
        mesh=UsdGeom.Mesh.Define(stage,prim_path+'/body/hex_shank')
        # 1/4 inch across flats; small lead-in bevel. Lower end enters socket.
        rings=[(0,.0024),(.003,.003175),(.060,.003175)]
        pts=[Gf.Vec3f(a/math.cos(math.pi/6)*math.cos(math.pi/6+k*math.pi/3),
                     a/math.cos(math.pi/6)*math.sin(math.pi/6+k*math.pi/3),z)
             for z,a in rings for k in range(6)]
        faces=[[5,4,3,2,1,0],[12,13,14,15,16,17]]
        faces += [[j*6+k,j*6+(k+1)%6,(j+1)*6+(k+1)%6,(j+1)*6+k] for j in range(2) for k in range(6)]
        mesh.CreatePointsAttr(pts)
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*GREY)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        pc=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        pc.CreateContactOffsetAttr(.0001)
        pc.CreateRestOffsetAttr(0.)
        from isaaclab.sim.utils import bind_physics_material
        bind_physics_material(str(mesh.GetPath()),prim_path+'/Looks/physics',stage=stage)
    return result
partkit._spawn_part=spawn

walls=[]
for k in (1,2,4,5):
    angle=k*math.pi/3
    walls.append(((.0045*math.cos(angle),.0045*math.sin(angle),.110),
                  (.002,.006,.020),GREY,angle))
sliders=[]
for sign,label in ((1,'jaw_plus'),(-1,'jaw_minus')):
    rest=(sign*.004,0,.109)
    sliders.append(Slider(name=label,rest=rest,
                   boxes=((rest,(.002,.004,.006),GREY),),
                   travel=.002,rate=1000.,damping=1.,mass=.002,axis='X',sign=sign))
driver_details=[((-.019,0,.077),(.002,.013,.024),DARK),
                ((-.021,0,.077),(.002,.009,.009),ORANGE)]
driver_cylinders=[((0,0,.05),.019,.10,ORANGE),((0,0,.009),.021,.018,DARK),
                  ((0,0,.086),.020,.014,DARK),((0,0,.098),.017,.004,GREY)]
# Separated rubber ribs and a molded housing seam read as a hand tool at close range.
for z in (.026,.035,.044,.053,.062):
    driver_cylinders.append(((0,0,z),.0196,.003,DARK))
tip_cross=[((0,0,.064),(.0045,.0014,.008),GREY),((0,0,.064),(.0014,.0045,.008),GREY)]
OBJECTS=[
    part('driver',(DX,DY,TOP),walls+driver_details,.32,cylinders=driver_cylinders,sliders=sliders),
    part('bit',(BX,BY,TOP+.026),[((0,0,.030),(.00635,.00635,.060),GREY)]+tip_cross,.014),
    part('bit_hex',(BX,-.04,TOP+.026),[((0,0,.030),(.00635,.00635,.060),GREY),((0,0,.064),(.003,.003,.008),GREY)],.014),
    part('bit_flat',(BX,.05,TOP+.026),[((0,0,.030),(.00635,.00635,.060),GREY),((0,0,.064),(.005,.001,.008),GREY)],.014),
    part('bit_rack',(BX,BY,TOP),[((0,0,.010),(.050,.050,.020),DARK)]+
         [((s*.010,0,.028),(.012,.025,.016),DARK) for s in (-1,1)]+
         [((0,s*.010,.028),(.008,.012,.016),DARK) for s in (-1,1)],.2,fixed_to_world=True),
    part('cradle',(DX,DY,TOP),[((s*.029,0,.006),(.010,.068,.012),DARK) for s in (-1,1)]+
         [((0,s*.029,.006),(.048,.010,.012),DARK) for s in (-1,1)],.2,fixed_to_world=True)
]
for name,by in [('hex',-.04),('flat',.05)]:
    OBJECTS.append(part('rack_'+name,(BX,by,TOP),[((0,0,.010),(.050,.050,.020),DARK)]+
       [((s*.010,0,.028),(.012,.025,.016),DARK) for s in (-1,1)]+
       [((0,s*.010,.028),(.008,.012,.016),DARK) for s in (-1,1)],.15,fixed_to_world=True))

# Removed wooden drawer: accessible inside of front panel faces toward arms.
# Static context only; the screw heads are visible recesses, not yet threaded assets.
wood=(.57,.33,.17); edge=(.73,.50,.28); steel=(.55,.57,.61)
drawer=[((.325,.035,.008),(.17,.32,.016),wood),
        ((.402,.035,.073),(.016,.34,.146),edge),
        ((.325,-.122,.038),(.17,.014,.06),wood),
        ((.325,.192,.038),(.17,.014,.06),wood),
        ((.247,.035,.038),(.014,.30,.06),wood),
        ((.438,.035,.085),(.018,.146,.018),DARK),
        ((.420,-.025,.085),(.030,.017,.017),DARK),
        ((.420,.095,.085),(.030,.017,.017),DARK)]
# Subtle longitudinal wood strips; solid furniture geometry remains visible.
for z in (.018,.041,.067,.102,.126):
    drawer.append(((.3937,.035,z),(.0005,.32,.001),wood))
for yy in (-.025,.095):
    for sy in (-1,1):
        for sz in (-1,1):
            drawer.append(((.3915,yy+sy*.0032,.085+sz*.0032),(.004,.0045,.0045),steel))
    drawer.append(((.393,yy,.085),(.001,.010,.010),DARK))
# Lay the front panel outside-down on two supports. Its inner screw faces point +Z.
# Transform old X to -Z, old Z to +X; no fixed joint attaches the drawer.
flat_drawer=[]
for b in drawer:
    (x,y,z),(sx,sy,sz),color=b[:3]
    flat_drawer.append(((.40-z-.327,.035-y,.49-x),(sz,sy,sx),color))
OBJECTS.append(part('drawer_context',(.327,.035,TOP),flat_drawer,1.4,friction=.6))
for index,yy in enumerate((-.12,.19)):
    OBJECTS.append(part('drawer_support_'+str(index),(.327,yy,TOP),
        [((0,0,.038),(.146,.023,.076),DARK)],.3,fixed_to_world=True))
def score(state): return 0.
def success(state): return False
