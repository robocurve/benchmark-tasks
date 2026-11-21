"""Bayonet retention and original-camera seal inspection for a dry training module.

The station fixture exposes the mechanism vertically for component testing.
These estimates are not production drawings of the MA7203.
"""
import os
import math
import json
import numpy as np
from surface_process_probe import WORLD, body, box

MODE = "spent_locked"
if MODE not in {"spent_locked", "unlocked", "seal_pierced", "seal_pierced_1mm", "intact_mounted", "intact_unlocked", "empty_socket"}:
    raise ValueError(MODE)
BODIES = ["installed", "spare"]
FIXTURE = "service_lever"
if FIXTURE not in {"original", "ring_sensor", "lever", "service_lever"}:
    raise ValueError(FIXTURE)
SENSOR = FIXTURE != "original"
LEVER = FIXTURE in {"lever", "service_lever"}
STOP_ANGLE = 64.5 if FIXTURE == "service_lever" else (61 if SENSOR else 70)
TOP = .7475
SEAT = (.31,-.07,.810)
LAYOUT = "disposition"
if LAYOUT not in {"inspection_stand", "service_tray", "parts_cup", "offset_cup", "disposition"}:
    raise ValueError(LAYOUT)
SPARE = (.340,-.190,TOP+.009) if LAYOUT in {"service_tray", "parts_cup", "offset_cup", "disposition"} else (.19,.075,TOP+.009)
CUP = (.203,.014) if LAYOUT in {"offset_cup","disposition"} else (.239,-.116)
HEIGHT = .139
BORE = .002
TARGETS = [("spare seal", np.array(SPARE)+[0,0,HEIGHT])]
FIXED = ["trainer_base", "trainer_post", "socket_back", "parts_stand"]
if LAYOUT in {"parts_cup", "offset_cup","disposition"}:
    FIXED.extend(["cup_front", "cup_back", "cup_left", "cup_right"])
RING_SECTORS = [i for i in range(32) if min(abs((i*11.25+90)%180-90),90)>38]
FIXED += [f"bay_shelf_{i}" for i in RING_SECTORS] + ["stop_a", "stop_b"]

if SENSOR:
    BODIES.append("follower")
    FIXED.extend(f"socket_wall_{i}" for i in range(32))

if LEVER:
    BODIES.append("lever")
    FIXED.extend(["lever_mount", "lever_pivot"])

if LAYOUT == "disposition":
    from disposition_layout import STOCK_B, HOLD, HOLD_HALF, INDEX_DEG, INDEX_RADIUS, witness
    STOCK_ORDER = "intact-first"
    if STOCK_ORDER not in {"intact-first","pierced-first"}:
        raise ValueError(STOCK_ORDER)
    BODIES.append("reserve")
    FIXED.extend(["reserve_stand","hold_front","hold_back","hold_left","hold_right"])

def save_probe(out):
    spec=dict(initial=MODE,ring_sensor=SENSOR,layout=LAYOUT,lever=LEVER,stop_center_deg=STOP_ANGLE)
    if LAYOUT=="disposition":spec["stock_order"]=STOCK_ORDER
    (out/"fixture-variant.json").write_text(json.dumps(spec)+"\n")

def revolve(stage, path, rings, color):
    """Continuous closed lathe surface. An inner descending ring forms the recess."""
    from pxr import UsdGeom,Gf,UsdShade,Sdf
    from yam_arena.station import linear_rgb
    points=[];faces=[];counts=[]
    n=48
    for r,z in rings:
        for k in range(n):
            a=k*2*math.pi/n;points.append((r*math.cos(a),r*math.sin(a),z))
    for j in range(len(rings)-1):
        for k in range(n):
            a=j*n+k;b=j*n+(k+1)%n
            faces.extend([a,b,b+n,a+n]);counts.append(4)
    mesh=UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr(counts);mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateSubdivisionSchemeAttr("none");mesh.CreateDoubleSidedAttr(True)
    mat=UsdShade.Material.Define(stage,path+"/mat")
    shader=UsdShade.Shader.Define(stage,path+"/mat/surface");shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput("roughness",Sdf.ValueTypeNames.Float).Set(.33)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh

def rotated_box(stage,path,center,size,color,angle):
    from pxr import UsdGeom
    tile=box(stage,path,center,size,color)
    xf=UsdGeom.Xformable(tile);translate,scale=xf.GetOrderedXformOps()
    rotate=xf.AddRotateZOp();rotate.Set(angle);xf.SetXformOpOrder([translate,rotate,scale])
    return tile

def cylinder_collider(stage,path,z,r,h):
    from pxr import UsdGeom,Gf,UsdPhysics,PhysxSchema
    shape=UsdGeom.Cylinder.Define(stage,path);shape.CreateRadiusAttr(r);shape.CreateHeightAttr(h)
    shape.AddTranslateOp().Set(Gf.Vec3d(0,0,z));shape.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim()).CreateContactOffsetAttr(.00002)
    return shape

def cartridge(stage,name,pos,yaw,pierced,bore=BORE):
    from pxr import UsdGeom
    path=WORLD+"/"+name
    prim=body(stage,path,pos,.102)
    UsdGeom.Xformable(prim).AddRotateZOp().Set(yaw)
    metal=(.76,.77,.75)
    rings=[(0,0),(.010,0),(.0127,.004),(.0127,.103),(.0122,.109),(.010,.116),
           (.006,.122),(.0052,.124),(.0052,HEIGHT),(.0033,HEIGHT)]
    rings += [(bore/2,HEIGHT),(bore/2,HEIGHT-.0028),(0,HEIGHT-.0028)] if pierced else [(0,HEIGHT)]
    revolve(stage,path+"/metal",rings,metal)
    if LAYOUT=="disposition":
        # The symmetric bayonet has two valid orientations. Both index lines
        # are painted on every part, independent of the condition of its seal.
        for sign in [-1,1]:
            box(stage,path+("/index_a" if sign==1 else "/index_b"),
                (sign*INDEX_RADIUS,0,.095),(.00045,.002,.015),(.95,.95,.93),False)
    revolve(stage,path+"/bayonet",[(.0053,.124),(.0073,.125),(.0073,.133),(.0053,.134)],(.17,.18,.19))
    revolve(stage,path+"/seal_ring",[(.0052,.134),(.0058,.1345),(.0058,.1355),(.0052,.136)],(.15,.16,.16))
    cylinder_collider(stage,path+"/can_collider",.052,.0127,.104)
    cylinder_collider(stage,path+"/neck_collider",.1215,.0052,.035)
    if SENSOR:
        # Collision envelopes for the visible shoulder, collar, seal and sensing ring.
        for i,(z0,z1,r0,r1) in enumerate([
            (.103,.109,.0127,.0122),(.109,.116,.0122,.010),(.116,.122,.010,.006),
            (.122,.124,.006,.0052),(.124,.125,.0053,.0073),(.125,.133,.0073,.0073),
            (.133,.134,.0073,.0053),(.134,.1355,.0057,.0058)]):
            convex_frustum(stage,path+f"/profile_{i}",z0,z1,r0,r1)
        ring_z=.137 if pierced else .143
        # Three struts support the undischarged ring on the free training part.
        for i in range(3):
            angle=2*math.pi*i/3
            px=.00625*math.cos(angle);py=.00625*math.sin(angle)
            box(stage,path+f"/ring_web_{i}",(px,py,(.132+ring_z)/2),
                (.0014,.0014,ring_z-.132),(.28,.29,.28))
        for i in range(24):
            a=i*math.pi/12
            rotated_box(stage,path+f"/ring_land_{i}",(.0065*math.cos(a),.0065*math.sin(a),ring_z),
                        (.0016,.00179,.0015),(.36,.36,.35),math.degrees(a))
    for i,sign in enumerate([-1,1]):
        box(stage,path+f"/lug_{i}",(sign*.0088,0,.130),(.0058,.004,.003),(.25,.26,.25))

def convex_frustum(stage,path,z0,z1,r0,r1):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema
    mesh=revolve(stage,path,[(0,z0),(r0,z0),(r1,z1),(0,z1)],(.5,.5,.5))
    mesh.MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim()).CreateContactOffsetAttr(.00002)

def indicator(stage):
    from pxr import Gf,UsdPhysics
    x,y,z=SEAT
    path=WORLD+"/follower"
    rest=.9496
    initial_z=rest+.006 if MODE in {"intact_mounted","intact_unlocked"} else rest
    body(stage,path,(x,y,initial_z),.006)
    # Twin sensing feet straddle the metal seal and bear on the annular land.
    for i,sign in enumerate([-1,1]):
        box(stage,path+f"/foot_{i}",(0,sign*.00645,.002),(.0015,.0014,.004),(.84,.84,.82))
    box(stage,path+"/bridge",(0,0,.0048),(.002,.0144,.0016),(.65,.66,.65))
    box(stage,path+"/arm",(.010,0,.005),(.020,.003,.0016),(.66,.66,.65))
    # One fixed window; two bands travel with the real follower on its slide.
    box(stage,path+"/red",(.020,0,.0066),(.002,.012,.0036),(.84,.08,.06),False)
    box(stage,path+"/green",(.020,0,.0029),(.002,.012,.0037),(.11,.68,.25),False)
    for label,z_offset in [("lower",.0004),("upper",.0128)]:
        box(stage,WORLD+"/window_"+label,(x+.0178,y,rest+z_offset),(.001,.017,.009),(.13,.16,.17),False)
    for i,sign in enumerate([-1,1]):
        box(stage,WORLD+f"/window_side_{i}",(x+.0178,y+sign*.0076,rest+.0066),
            (.001,.0025,.013),(.13,.16,.17),False)
    joint=UsdPhysics.PrismaticJoint.Define(stage,WORLD+"/shutter_slide")
    joint.CreateBody1Rel().SetTargets([path]);joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(x,y,rest));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    joint.CreateLowerLimitAttr(0);joint.CreateUpperLimitAttr(.012)
    joint.CreateExcludeFromArticulationAttr(True)

def reset_lever(stage):
    """Dry trainer rocker. Its blunt toe clears the fresh ring when raised."""
    from pxr import Gf, UsdPhysics
    path = WORLD + "/lever"
    pivot = (.287, -.070, .952)
    box(stage, WORLD + "/lever_mount", (.287, .024, .8455),
        (.012,.010,.196), (.26,.31,.32))
    box(stage, WORLD + "/lever_pivot", (.287, .009, .9445),
        (.010,.040,.006), (.32,.36,.36))
    body(stage, path, pivot, .022)
    box(stage, path + "/toe", (.0165,0,-.0005), (.0022,.0034,.0026), (.65,.65,.63))
    box(stage, path + "/nose_arm", (.0085,0,.0011), (.017,.0034,.002), (.47,.49,.49))
    box(stage, path + "/crossbar", (0,.031,.001), (.004,.065,.004), (.47,.49,.49))
    box(stage, path + "/handle", (.027,.061,.001), (.055,.007,.004), (.78,.38,.08))
    box(stage, path + "/paddle", (.053,.061,.001), (.020,.014,.004), (.90,.48,.11))
    joint=UsdPhysics.RevoluteJoint.Define(stage,WORLD+"/lever_hinge")
    joint.CreateBody1Rel().SetTargets([path])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*pivot))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    joint.CreateAxisAttr("Y")
    joint.CreateLowerLimitAttr(-32)
    joint.CreateUpperLimitAttr(0)
    joint.CreateExcludeFromArticulationAttr(True)
    spring=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),"angular")
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(.028)
    spring.CreateDampingAttr(.0015)
    spring.CreateTargetPositionAttr(0.)
    spring.CreateMaxForceAttr(.48)


def build(stage):
    x,y,z=SEAT
    # Open PFD service panel represented by an immobilised fabric work surface.
    box(stage,WORLD+"/service_panel",(x,y,TOP+.002),(.24,.195,.004),(.91,.49,.08),False)
    box(stage,WORLD+"/waist_webbing",(x,y,TOP+.0048),(.050,.29,.0015),(.15,.17,.17),False)
    box(stage,WORLD+"/trainer_base",(x+.036,y,TOP+.010),(.070,.074,.010),(.22,.25,.25))
    socket_z=z+.125
    box(stage,WORLD+"/trainer_post",(x+.048,y,(socket_z+TOP+.015)/2),
        (.014,.043,socket_z-TOP-.015),(.33,.36,.36))
    box(stage,WORLD+"/socket_back",(x+.031,y,socket_z+.001),(.032,.037,.014),(.21,.22,.22))
    for i in RING_SECTORS:
        a=math.radians(i*11.25);r=.0124
        center=(x+r*math.cos(a),y+r*math.sin(a),socket_z)
        rotated_box(stage,WORLD+f"/bay_shelf_{i}",center,(.009,.0027,.006),
                    (.20,.21,.21),math.degrees(a))
    if SENSOR:
        for i in range(32):
            a=i*math.pi/16;r=.0158
            rotated_box(stage,WORLD+f"/socket_wall_{i}",
                        (x+r*math.cos(a),y+r*math.sin(a),z+.126),
                        (.0034,.0032,.021),(.24,.25,.25),math.degrees(a))
        indicator(stage)
    if LEVER:
        reset_lever(stage)
    for name,offset in [("stop_a",0),("stop_b",180)]:
        a=math.radians(STOP_ANGLE+offset)
        stop_r=.0108 if SENSOR else .009
        rotated_box(stage,WORLD+"/"+name,(x+stop_r*math.cos(a),y+stop_r*math.sin(a),z+.130),
                    (.0048 if SENSOR else .006,.002,.008),(.25,.26,.25),math.degrees(a))
    sx,sy,sz=SPARE
    box(stage,WORLD+"/parts_stand",(sx,sy,sz-.0045),(.060,.050,.009),(.33,.37,.39))
    if LAYOUT in {"parts_cup", "offset_cup","disposition"}:
        # Low dividers catch the retired cartridge at the actual drop-off; the
        # table supports its weight. Keep the gripper approach clear above it.
        for name,dx,dy,span in [
            ("cup_front",-.025,0,(.004,.054,.018)),("cup_back",.025,0,(.004,.054,.018)),
            ("cup_left",0,-.025,(.046,.004,.018)),("cup_right",0,.025,(.046,.004,.018))]:
            box(stage,WORLD+"/"+name,(CUP[0]+dx,CUP[1]+dy,TOP+.009),span,(.32,.36,.37))
    old_pos=(.24,-.117,TOP) if MODE=="empty_socket" else SEAT
    cartridge(stage,"installed",old_pos,0 if MODE in {"unlocked","intact_unlocked"} else 45,
              MODE not in {"intact_mounted","intact_unlocked"})
    if LAYOUT=="disposition":
        box(stage,WORLD+"/reserve_stand",(*STOCK_B,TOP+.0045),(.060,.050,.009),(.33,.37,.39))
        hx,hy=HOLD_HALF
        for name,dx,dy,span in [
            ("hold_front",-hx-.002,0,(.004,2*hy+.008,.018)),("hold_back",hx+.002,0,(.004,2*hy+.008,.018)),
            ("hold_left",0,-hy-.002,(2*hx,.004,.018)),("hold_right",0,hy+.002,(2*hx,.004,.018))]:
            box(stage,WORLD+"/"+name,(HOLD[0]+dx,HOLD[1]+dy,TOP+.009),span,(.78,.12,.08))
        for i,angle in enumerate([INDEX_DEG,INDEX_DEG+180]):
            dx,dy=witness(angle)
            box(stage,WORLD+f"/seat_index_back_{i}",(x+.0165*dx,y+.0165*dy,z+.1135),
                (.003,.003,.019),(.19,.20,.21),False)
            box(stage,WORLD+f"/seat_index_{i}",(x+.018*dx,y+.018*dy,z+.106),
                (.002,.002,.010),(.98,.91,.08),False)
        cartridge(stage,"spare",SPARE,0,STOCK_ORDER=="pierced-first")
        cartridge(stage,"reserve",(*STOCK_B,TOP+.009),0,STOCK_ORDER=="intact-first")
    else:
        cartridge(stage,"spare",SPARE,0,MODE.startswith("seal_pierced"),
                  bore=.001 if MODE=="seal_pierced_1mm" else BORE)
