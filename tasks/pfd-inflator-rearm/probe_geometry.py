"""Bayonet retention and original-camera seal inspection for a dry training module.

The station fixture exposes the mechanism vertically for component testing.
These estimates are not production drawings of the MA7203.
"""
import math
from surface_process_probe import WORLD, body, box

from disposition_layout import (STOCK_A, STOCK_B, HOLD, HOLD_HALF, INDEX_DEG, INDEX_RADIUS,
                                TABLE, SEAT_XY, USED, witness)

BODIES = ["installed", "spare", "reserve", "follower", "lever"]
TOP = TABLE
SEAT = (*SEAT_XY, .810)
SPARE = (*STOCK_A, TOP + .009)
CUP = USED
HEIGHT = .139
BORE = .002
STOP_ANGLE = 64.5
RING_SECTORS = [i for i in range(32) if abs((i*11.25+90)%180-90)>38]
FIXED = ["trainer_base", "trainer_post", "socket_back", "parts_stand",
         "cup_front", "cup_back", "cup_left", "cup_right", "stop_a", "stop_b",
         "lever_mount", "lever_pivot", "reserve_stand", "hold_front", "hold_back",
         "hold_left", "hold_right", "housing_back", "housing_lip_a", "housing_lip_b"]
FIXED += [f"bay_shelf_{i}" for i in RING_SECTORS] + [f"socket_wall_{i}" for i in range(32)]

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

def cartridge(stage,name,pos,yaw,pierced,ring_attached,bore=BORE):
    from pxr import UsdGeom
    path=WORLD+"/"+name
    prim=body(stage,path,pos,.102)
    UsdGeom.Xformable(prim).AddRotateZOp().Set(yaw)
    metal=(.76,.77,.75)
    rings=[(0,0),(.010,0),(.0127,.004),(.0127,.103),(.0122,.109),(.010,.116),
           (.006,.122),(.0052,.124),(.0052,HEIGHT),(.0033,HEIGHT)]
    rings += [(bore/2,HEIGHT),(bore/2,HEIGHT-.0028),(0,HEIGHT-.0028)] if pierced else [(0,HEIGHT)]
    revolve(stage,path+"/metal",rings,metal)
    # The symmetric bayonet has two valid orientations. Both index lines
    # are painted on every part, independent of the condition of its seal.
    for sign in [-1,1]:
        box(stage,path+("/index_a" if sign==1 else "/index_b"),
            (sign*INDEX_RADIUS,0,.095),(.00045,.002,.015),(.95,.95,.93),False)
    revolve(stage,path+"/bayonet",[(.0053,.124),(.0073,.125),(.0073,.133),(.0053,.134)],(.17,.18,.19))
    revolve(stage,path+"/seal_ring",[(.0052,.134),(.0058,.1345),(.0058,.1355),(.0052,.136)],(.15,.16,.16))
    cylinder_collider(stage,path+"/can_collider",.052,.0127,.104)
    cylinder_collider(stage,path+"/neck_collider",.1215,.0052,.035)
    # Collision envelopes for the metal shoulder, bayonet and O-ring.
    for i,(z0,z1,r0,r1) in enumerate([
        (.103,.109,.0127,.0122),(.109,.116,.0122,.010),(.116,.122,.010,.006),
        (.122,.124,.006,.0052),(.124,.125,.0053,.0073),(.125,.133,.0073,.0073),
        (.133,.134,.0073,.0053),(.134,.1355,.0057,.0058)]):
        convex_frustum(stage,path+f"/profile_{i}",z0,z1,r0,r1)
    # MA7203 Fig. 1/3: the breakaway ring surrounds the root of the bayonet,
    # behind its locking tabs. The metal face and the O-ring are separate parts.
    # Both loose training adapters have the same attached ring, even the reject.
    if ring_attached:
        revolve(stage,path+"/sensor_ring",
                [(.009,.114),(.0133,.114),(.0133,.1165),(.009,.1165),(.009,.114)],
                (.27,.28,.28))
        for i in range(32):
            angle=i*360/32
            dx,dy=witness(angle)
            tile=rotated_box(stage,path+f"/sensor_ring_contact_{i}",
                             (.01115*dx,.01115*dy,.11525),(.0043,.00220,.0025),
                             (.27,.28,.28),angle)
            tile.MakeInvisible()
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
    body(stage,path,(x,y,rest),.006)
    # Two axial probes in the entry slots bear on the OUTER adapter flange.
    # Neither reaches the central metal face. The bridge clears the dummy neck.
    dx,dy=witness(-24)
    for i,sign in enumerate([-1,1]):
        box(stage,path+f"/probe_{i}",(sign*.01265*dx,sign*.01265*dy,-.01065),
            (.0015,.0015,.0333),(.63,.64,.62))
    # The fork is routed behind the rocker. The reset lever can swing through
    # the centre of the head without lifting the flag in an empty socket.
    tip_x, tip_y = .01265*dx, .01265*dy
    for i,sign in enumerate((-1,1)):
        end_y = sign*tip_y
        box(stage,path+f"/fork_side_{i}",(sign*tip_x,(.010+end_y)/2,.0068),
            (.0016,.010-end_y,.0016),(.48,.49,.49))
    box(stage,path+"/fork_back",(0,.010,.0068),(2*tip_x+.0016,.0016,.0016),(.48,.49,.49))
    # The opaque head has a front-facing indicator bezel at its near corner.
    # The flag travels just behind its aperture on a lightweight carrier.
    face_x, face_y = -.013, -.015
    box(stage,path+"/flag_carrier",(-.0123,-.0045,.005),(.002,.022,.001),
        (.37,.38,.37),False)
    box(stage,path+"/red",(face_x,face_y,.0066),(.001,.012,.0036),(.84,.08,.06),False)
    box(stage,path+"/green",(face_x,face_y,.0029),(.001,.012,.0037),(.11,.68,.25),False)
    # The bezel and rear plate are fixed to the receiver. Its four edges leave
    # the aperture open to the normal wrist view.
    box(stage,WORLD+"/window_back",(x+face_x+.0046,y+face_y,rest+.0066),
        (.008,.017,.018),(.24,.26,.26),False)
    for label,z_offset in [("lower",.0004),("upper",.0128)]:
        box(stage,WORLD+"/window_"+label,(x+face_x-.0012,y+face_y,rest+z_offset),
            (.001,.017,.009),(.13,.16,.17),False)
    for i,sign in enumerate([-1,1]):
        box(stage,WORLD+f"/window_side_{i}",(x+face_x-.0012,y+face_y+sign*.0076,rest+.0066),
            (.001,.0025,.013),(.13,.16,.17),False)
    joint=UsdPhysics.PrismaticJoint.Define(stage,WORLD+"/shutter_slide")
    joint.CreateBody1Rel().SetTargets([path]);joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(x,y,rest));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
    joint.CreateLowerLimitAttr(0);joint.CreateUpperLimitAttr(.012)
    joint.CreateExcludeFromArticulationAttr(True)

def reset_lever(stage):
    """Manual reset rocker with a blunt actuator beside the dummy seal."""
    from pxr import Gf, UsdPhysics
    path = WORLD + "/lever"
    pivot = (.287, -.070, .952)
    box(stage, WORLD + "/lever_mount", (.287, .024, .8455),
        (.012,.010,.196), (.26,.31,.32))
    box(stage, WORLD + "/lever_pivot", (.287, .009, .9445),
        (.010,.040,.006), (.32,.36,.36))
    body(stage, path, pivot, .022)
    box(stage, path + "/toe", (.0200,0,-.00195), (.0022,.0034,.0017), (.65,.65,.63))
    box(stage, path + "/nose_arm", (.0100,0,.0000), (.020,.0034,.0024), (.47,.49,.49))
    box(stage, path + "/crossbar", (0,.031,.001), (.004,.065,.004), (.47,.49,.49))
    box(stage, path + "/handle", (.027,.061,.001), (.055,.007,.004), (.83,.68,.13))
    box(stage, path + "/paddle", (.053,.061,.001), (.020,.014,.004), (.95,.78,.13))
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


def build(stage, stock_order):
    if stock_order not in ("intact-first", "pierced-first"):
        raise ValueError(stock_order)
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
    for i in range(32):
        a=i*math.pi/16;r=.0158
        rotated_box(stage,WORLD+f"/socket_wall_{i}",
                    (x+r*math.cos(a),y+r*math.sin(a),z+.126),
                    (.0034,.0032,.021),(.24,.25,.25),math.degrees(a))
    indicator(stage)
    reset_lever(stage)
    # Cutaway three-sided head behind the socket, rectangular face aperture
    # in the front mask and a separate mounting post for the inverted dry bench.
    for name,px,py,pz,size in [
        ("housing_back",.023,0,.149,(.004,.031,.028)),
        ("housing_lip_a",.009,.0137,.149,(.027,.003,.028)),
        ("housing_lip_b",.009,-.0137,.149,(.027,.003,.028))]:
        box(stage,WORLD+"/"+name,(x+px,y+py,z+pz),size,(.32,.34,.33))
    for name,offset in [("stop_a",0),("stop_b",180)]:
        a=math.radians(STOP_ANGLE+offset)
        stop_r=.0108
        rotated_box(stage,WORLD+"/"+name,(x+stop_r*math.cos(a),y+stop_r*math.sin(a),z+.130),
                    (.0048,.002,.008),(.25,.26,.25),math.degrees(a))
    sx,sy,sz=SPARE
    box(stage,WORLD+"/parts_stand",(sx,sy,sz-.0045),(.060,.050,.009),(.33,.37,.39))
    # Low dividers catch the retired cartridge at the actual drop-off; the
    # table supports its weight. Keep the gripper approach clear above it.
    for name,dx,dy,span in [
        ("cup_front",-.025,0,(.004,.054,.018)),("cup_back",.025,0,(.004,.054,.018)),
        ("cup_left",0,-.025,(.046,.004,.018)),("cup_right",0,.025,(.046,.004,.018))]:
        box(stage,WORLD+"/"+name,(CUP[0]+dx,CUP[1]+dy,TOP+.009),span,(.32,.36,.37))
    cartridge(stage,"installed",SEAT,45,pierced=True,ring_attached=False)
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
    cartridge(stage,"spare",SPARE,0,stock_order=="pierced-first",ring_attached=True)
    cartridge(stage,"reserve",(*STOCK_B,TOP+.009),0,stock_order=="intact-first",ring_attached=True)
