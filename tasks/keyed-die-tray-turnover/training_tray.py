"""Machinable inspection coupons and keyed trays. Dimensions in metres."""
import numpy as np
from layout import KEY_CENTER
from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
from native import body,box,collider
from yam_arena.station import linear_rgb

POCKET=.010
DIE=.009
THICK=.0005
DEPTH=.001
BEVEL=.00045
HALF=.0254
PITCH=.013
FLANGE_TOP=-.00145
EDGE_STEP=.0032
def prism(stage,path,levels,colour):
    # A straight or bevelled convex piece; points in each ring are counter-clockwise.
    vertices=[[x,y,z] for z,ring in levels for x,y in ring]
    n=len(levels[0][1])
    faces=[list(range(n-1,-1,-1)),list(range(len(vertices)-n,len(vertices)))]
    for j in range(len(levels)-1):
        for i in range(n):
            k=(i+1)%n
            faces.append([j*n+i,j*n+k,(j+1)*n+k,(j+1)*n+i])
    mesh=UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr(vertices);mesh.CreateFaceVertexCountsAttr(list(map(len,faces)))
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb(colour))])
    collider(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
def polygon(x0,x1,y0,y1,cut=False):
    points=[(x0,y0),(x1,y0)]
    points+=([(x1,y1-.003),(x1-.003,y1)] if cut else [(x1,y1)])
    return points+[(x0,y1)]
def tray(stage,path,pos,rot):
    body(stage,path,pos,.0102,rot)
    colour=(.22,.23,.25)
    outline=polygon(-HALF,HALF,-HALF,HALF,True)
    # The full-width lower flange slides under the receiver's retaining rails.
    flange_top=FLANGE_TOP
    inner=HALF-EDGE_STEP
    prism(stage,path+'/flange',[(-.0035,outline),(flange_top,outline)],colour)
    # The pocket floor is rectangular. A primitive box preserves its horizontal
    # top surface even at the 0.45 mm thickness of this step.
    box(stage,path+'/floor',[0,0,(flange_top-DEPTH)/2],[2*inner,2*inner,-DEPTH-flange_top],colour)
    def boundaries(width):
        result=[-inner]
        for pos in [-PITCH,0,PITCH]:
            result.extend([pos-width/2,pos+width/2])
        return result+[inner]
    low=boundaries(POCKET); high=boundaries(POCKET+2*BEVEL)
    for i in range(7):
        for j in range(7):
            if i%2 and j%2:continue
            cut=False
            bottom=polygon(*low[i:i+2],*low[j:j+2],cut)
            top=polygon(*high[i:i+2],*high[j:j+2],cut)
            prism(stage,path+f'/wall{i}_{j}',[(-DEPTH,bottom),(-BEVEL,bottom),(0.,top)],colour)
    # Paint on the outside rim at the clipped corner. This physical index is visible in the policy RGB.
    box(stage,path+'/corner_ink',[KEY_CENTER,KEY_CENTER,.000009],[.003,.003,.000018],(.99,.03,.72),collision=False)
def chip(stage,path,pos,rot):
    prim = body(stage,path,pos,2330*DIE**2*THICK,rot)
    # Keep per-step contact reporting active after the loose coupon comes to rest.
    PhysxSchema.PhysxRigidBodyAPI(prim).CreateSleepThresholdAttr(0.)
    box(stage,path+'/silicon',[0,0,0],[DIE,DIE,THICK],(.62,.63,.65))
    box(stage,path+'/front',[0,0,THICK/2+.000002],[DIE*.99,DIE*.99,.000004],(.32,.37,.49),collision=False)
    for i in (-1,0,1):
        box(stage,path+f'/lane{i+1}',[i*.0025,0,THICK/2+.000005],[.00035,.008,.000002],(.65,.59,.43),collision=False)
