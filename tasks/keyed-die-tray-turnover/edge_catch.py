"""Optional spring-loaded roller catch on the far edge of the two-leaf jig.

A fixed headed striker contacts two spring-backed sliders. Positive retention and
release result entirely from rigid-body contact with the inclined surfaces.
"""
import numpy as np
from pxr import Gf,UsdGeom,UsdPhysics
from layout import BASIS,world
from native import body,box,collider
from yam_arena.station import linear_rgb

def striker_piece(stage,path,h0,w0,h1,w1):
    y=.055
    vertex=[[.050+x,y+dy,z] for z,w in [(h0,w0),(h1,w1)] for x,dy in [(-w,-.003),(w,-.003),(w,.003),(-w,.003)]]
    faces=[[3,2,1,0],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
    geom=UsdGeom.Mesh.Define(stage,path)
    geom.CreatePointsAttr(vertex);geom.CreateFaceVertexCountsAttr([4]*6)
    geom.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);geom.CreateSubdivisionSchemeAttr('none')
    geom.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb((.8,.7,.4)))])
    collider(geom.GetPrim(),.18);UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr('convexHull')
def add(stage,root,stiffness=420.):
    source=root+'/source_frame'; receiver=root+'/receiver_frame'
    y=.055
    box(stage,source+'/catch_bridge',[.0375,y,-.0055],[.027,.010,.0034],(.42,.45,.48))
    # Join both socket walls from behind. At full closure the tip enters 6.8 mm.
    box(stage,receiver+'/catch_bridge',[.0425,-y+.0085,-.0055],[.037,.005,.0034],(.42,.45,.48))
    box(stage,source+'/catch_mount',[.050,y-.003,-.0035],[.014,.013,.004],(.34,.45,.52))
    box(stage,source+'/neck',[.050,y,-.0002],[.0028,.006,.0044],(.68,.69,.7))
    striker_piece(stage,source+'/ramp_out',.0018,.0014,.0043,.0035)
    striker_piece(stage,source+'/ramp_in',.0043,.0035,.0068,.0007)
    for s in [-1,1]:
        # Housing lands on either side of the striker; the central socket is open.
        box(stage,receiver+f'/catch_wall{s+1}',[.050+s*.009,-y+.002,-.0045],[.008,.013,.006],(.38,.38,.40))
        anchor=[.050+s*.0031,-y,-.00265]
        path=root+f'/roller{s+1}'
        body(stage,path,world(anchor),.000372,BASIS)
        ball=UsdGeom.Sphere.Define(stage,path+'/ball')
        ball.CreateRadiusAttr(.0014);ball.CreateDisplayColorAttr([Gf.Vec3f(.5,.5,.53)])
        collider(ball.GetPrim(),.10)
        joint=UsdPhysics.PrismaticJoint.Define(stage,path+'_guide')
        joint.CreateBody0Rel().SetTargets([receiver]);joint.CreateBody1Rel().SetTargets([path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,anchor)));joint.CreateAxisAttr('X')
        joint.CreateCollisionEnabledAttr(False);joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateLowerLimitAttr(-.00020 if s==1 else -.0036)
        joint.CreateUpperLimitAttr(.0036 if s==1 else .00020)
        spring=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear')
        spring.CreateTypeAttr('force');spring.CreateStiffnessAttr(stiffness);spring.CreateDampingAttr(.6)
        spring.CreateTargetPositionAttr(-s*.001);spring.CreateMaxForceAttr(6.)
