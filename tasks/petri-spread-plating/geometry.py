"""Normal-size Petri lid/spreader mechanical feasibility, not a biological model.

92x16mm dish and140x35mm T-spreader use nominal catalogue dimensions.
Wall thickness, mass split and rests are engineering approximations. The fixed
scene enables native compliant gel contact; finite liquid and damage responses
are installed separately by the launcher. No microbial growth is modeled.
"""
import numpy as np
from surface_runtime import box,body,WORLD
BODIES=['petri_base','petri_lid','cell_spreader']
FIXED=['tool_rest0','tool_rest1','tool_head_rest','petri_base/agar_rigid_witness']
T_SPREADER=False
SOFT_AGAR=False
BASE=np.array([.33,.04,.7475]);LID=np.array([.33,.04,.7635]);TOOL=np.array([.43,-.12,.8035])
TARGETS=[('dish',BASE+[0,0,.007]),('spreader',TOOL)]

def cylinder(stage,path,center,radius,height,axis='Z',color=(.75,.83,.87),opacity=.6):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdShade
    c=UsdGeom.Cylinder.Define(stage,path);c.CreateAxisAttr(axis);c.CreateHeightAttr(height);c.CreateRadiusAttr(radius)
    c.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)));c.CreateDisplayColorAttr([Gf.Vec3f(*color)]);c.CreateDisplayOpacityAttr([opacity])
    UsdPhysics.CollisionAPI.Apply(c.GetPrim());a=PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim());a.CreateContactOffsetAttr(.00002);a.CreateRestOffsetAttr(0.)
    mat=UsdShade.Material.Define(stage,path+'/friction');m=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());m.CreateStaticFrictionAttr(.7);m.CreateDynamicFrictionAttr(.6);m.CreateRestitutionAttr(0.);UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat,materialPurpose='physics')
    return c

def ring(stage,path,inner,outer,z0,z1):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
    from scipy.spatial import ConvexHull
    for i in range(96):
        ang=np.array([i,i+1])*2*np.pi/96
        v=np.array([[r*np.cos(t),r*np.sin(t),z] for z in [z0,z1] for r in [inner,outer] for t in ang])
        hull=ConvexHull(v);m=UsdGeom.Mesh.Define(stage,path+f'/wall_{i}')
        m.CreatePointsAttr(v.tolist());m.CreateFaceVertexCountsAttr([3]*len(hull.simplices));m.CreateFaceVertexIndicesAttr(hull.simplices.reshape(-1).tolist());m.CreateSubdivisionSchemeAttr('none');m.CreateDoubleSidedAttr(True);m.CreateDisplayColorAttr([Gf.Vec3f(.72,.82,.88)]);m.CreateDisplayOpacityAttr([.45])
        UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull');a=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());a.CreateContactOffsetAttr(.00002);a.CreateRestOffsetAttr(0.)

def build(stage):
    body(stage,WORLD+'/petri_base',BASE,.028)
    cylinder(stage,WORLD+'/petri_base/bottom',[0,0,.0005],.0445,.001)
    ring(stage,WORLD+'/petri_base',.0435,.0445,.001,.015)
    # Ordinary ink ticks make the grading quadrants observable after rotation.
    for i,(x,y) in enumerate([(1,0),(-1,0),(0,1),(0,-1)]):
        size=[.0008,.003,.0001] if x else [.003,.0008,.0001]
        box(stage,WORLD+f'/petri_base/grading_tick_{i}',[x*.044,y*.044,.01505],size,[.06,.06,.06],collision=False)
    cylinder(stage,WORLD+'/petri_base/agar_rigid_witness',[0,0,.0035],.0434,.005,color=(.83,.68,.36),opacity=.85)
    if SOFT_AGAR:
        from pxr import PhysxSchema
        material=stage.GetPrimAtPath(WORLD+'/petri_base/agar_rigid_witness/friction')
        soft=PhysxSchema.PhysxMaterialAPI.Apply(material)
        soft.CreateCompliantContactStiffnessAttr(2500.)
        # Engineering critical-damping reference for a free2.1g tool against
        # the2500N/m witness:2*sqrt(k*m)=4.58Ns/m, not a fitted agar parameter.
        soft.CreateCompliantContactDampingAttr(4.6)
        soft.CreateRestitutionCombineModeAttr('max')
    body(stage,WORLD+'/petri_lid',LID,.008)
    cylinder(stage,WORLD+'/petri_lid/roof',[0,0,0],.046,.001)
    ring(stage,WORLD+'/petri_lid',.045,.046,-.012,-.0005)
    body(stage,WORLD+'/cell_spreader',TOOL,.0021)
    # Shaft146mm long, bent leg38mm wide; rounded cylinders, not a broad paddle.
    cylinder(stage,WORLD+'/cell_spreader/shaft',[0,0,0],.002,.136 if T_SPREADER else .142,axis='Y',color=(.35,.65,.38),opacity=1.)
    cylinder(stage,WORLD+'/cell_spreader/contact_bar',[0,.068,0] if T_SPREADER else [.017,.071,0],.002,.035 if T_SPREADER else .038,axis='X',color=(.35,.65,.38),opacity=1.)
    for i,y in enumerate([-.06,.04]):box(stage,WORLD+f'/tool_rest{i}',[TOOL[0],TOOL[1]+y,.7745],[.012,.006,.054],[.3,.33,.35])
    # Support the offset L-head so the round shaft cannot roll90deg on its
    # two rests before pickup. This is a visible passive rack, not a weld.
    box(stage,WORLD+'/tool_head_rest',TOOL+([0,.068,-.029] if T_SPREADER else [.017,.071,-.029]),[.04,.008,.054],[.3,.33,.35])
