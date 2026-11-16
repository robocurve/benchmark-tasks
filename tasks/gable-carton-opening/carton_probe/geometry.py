"""Native folded facets and force-breakable seals; no commanded object motion.

This is a crease/topology prototype, not a validated paperboard material model.
Flat facets and the carton body do not yet buckle, crush or tear. Panel thickness,
mass, crease damping and bond thresholds are provisional, not measured fits.
"""
import math
import numpy as np

ROOT = '/World/envs/env_0'
ORIGIN = np.array([.18, .05, .7475 + .110])  # centre of shoulder plane
WIDTH = .070
A = WIDTH / 2
HEIGHT = .110
THICKNESS = .00045
FIN = .012
INNER_FIN = .024  # extended paper pouring edge; prototype dimensions, not a measured SKU
AREAL_MASS = .35
FACET_STRIPS = 64
SEALED = True
MECHANICAL_ONLY = False
INITIAL_OUTER_DEG = 120.
CREASE_RELIEF = .0005  # explicit finite-thickness crease-zone approximation
BODY_MASS = .520  # lumped contents + lower carton; no fluid simulation

B_MINUS = np.array([-A, -A, 0.])
B_PLUS = np.array([-A, A, 0.])
TIP = np.array([-A, 0., A])
INNER = np.array([0., 0., A])
REAR = np.array([A, 0., A])

FACETS = {
    'carton_spout': np.array([B_MINUS, B_PLUS, INNER]),
    'carton_inner_minus': np.array([B_MINUS, TIP, INNER]),
    'carton_inner_plus': np.array([B_PLUS, INNER, TIP]),
    'carton_roof_minus': np.array([B_MINUS, INNER, TIP]),
    'carton_roof_plus': np.array([B_PLUS, TIP, INNER]),
    'carton_fin_minus': np.array([TIP, INNER, INNER+[0,0,FIN], TIP+[0,0,FIN]]),
    'carton_fin_plus': np.array([INNER, TIP, TIP+[0,0,FIN], INNER+[0,0,FIN]]),
    'carton_inner_fin_minus': np.array([TIP, INNER, INNER+[0,0,INNER_FIN], TIP+[0,0,INNER_FIN]]),
    'carton_inner_fin_plus': np.array([INNER, TIP, TIP+[0,0,INNER_FIN], INNER+[0,0,INNER_FIN]]),
}
CENTRES = {n: v.mean(axis=0) for n, v in FACETS.items()}
CENTRES['carton_body'] = np.array([0., 0., -HEIGHT/2])

# Each physical crease joins its two material edges. INNER and the stationary
# roof ridge occupy the same place only while closed; they are NOT one node.
HINGES = [
    ('spout_base', 'carton_body', 'carton_spout', B_MINUS, B_PLUS),
    ('spout_minus', 'carton_spout', 'carton_inner_minus', B_MINUS, INNER),
    ('spout_plus', 'carton_spout', 'carton_inner_plus', B_PLUS, INNER),
    ('corner_minus', 'carton_inner_minus', 'carton_roof_minus', B_MINUS, TIP),
    ('corner_plus', 'carton_inner_plus', 'carton_roof_plus', B_PLUS, TIP),
    ('roof_minus', 'carton_body', 'carton_roof_minus', B_MINUS, INNER),
    ('roof_plus', 'carton_body', 'carton_roof_plus', B_PLUS, INNER),
    ('fin_minus', 'carton_roof_minus', 'carton_fin_minus', TIP, INNER),
    ('fin_plus', 'carton_roof_plus', 'carton_fin_plus', TIP, INNER),
    ('inner_fin_minus', 'carton_inner_minus', 'carton_inner_fin_minus', TIP, INNER),
    ('inner_fin_plus', 'carton_inner_plus', 'carton_inner_fin_plus', TIP, INNER),
    ('inner_fin_centre', 'carton_inner_fin_minus', 'carton_inner_fin_plus', INNER, INNER+[0,0,INNER_FIN]),
]


def area(vertices):
    return sum(np.linalg.norm(np.cross(vertices[i]-vertices[0], vertices[i+1]-vertices[0]))/2
               for i in range(1, len(vertices)-1))


def objects():
    from yam_arena.task_file import Box
    initial = initial_poses()
    return [Box('carton_body', pos=tuple((ORIGIN+CENTRES['carton_body']).tolist()),
                size=(.001,)*3, mass=BODY_MASS)] + [
        Box(n, pos=tuple((ORIGIN+initial[n]['center']).tolist()), size=(.001,)*3,
            mass=float(area(v)*AREAL_MASS)) for n,v in FACETS.items()]


def initial_poses():
    if MECHANICAL_ONLY:
        from .folding import poses
        return poses(INITIAL_OUTER_DEG)
    return {n:dict(center=c.tolist(),quat_xyzw=[0.,0.,0.,1.]) for n,c in CENTRES.items()}


def material(stage, path, color):
    from pxr import Gf, Sdf, UsdShade
    from yam_arena.station import linear_rgb
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+'/shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.7)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def physical(prim):
    from pxr import Sdf, UsdPhysics, UsdShade
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema('PhysxCollisionAPI')
    prim.CreateAttribute('physxCollision:contactOffset', Sdf.ValueTypeNames.Float).Set(.0001)
    prim.CreateAttribute('physxCollision:restOffset', Sdf.ValueTypeNames.Float).Set(0.)
    mat = UsdShade.Material.Define(prim.GetStage(), str(prim.GetPath())+'/friction')
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(.5); api.CreateDynamicFrictionAttr(.4)
    api.CreateRestitutionAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose='physics')


def panel(stage, path, points, color, collide=True, relief=0.):
    """Native thin boxes; identical rendered and collision surfaces."""
    from pxr import Gf, UsdGeom, UsdShade
    v = np.asarray(points, dtype=float)
    normal = np.cross(v[1]-v[0], v[2]-v[0]); normal /= np.linalg.norm(normal)
    n = len(v)
    if relief:
        c=v.mean(axis=0)
        edges=np.roll(v,-1,axis=0)-v
        distances=np.abs(np.cross(edges,c-v)@normal)/np.linalg.norm(edges,axis=1)
        # Short omitted crease zones represented by the native hinges; BOTH
        # rendered and collision facets are trimmed, no invisible enlargement.
        factor=1-relief/float(distances.min())
        if not 0<factor<1:raise ValueError('Relief exceeds facet inradius')
        v=c+(v-c)*factor
    # Native boxes avoid convex cooking degeneracy for very thin rectangular
    # walls. Their dimensions and visible surfaces are exactly the same prism.
    if n==4:
        x,y=v[1]-v[0],v[3]-v[0]
        lx,ly=np.linalg.norm(x),np.linalg.norm(y)
        if abs(x@y)<1e-12 and np.linalg.norm(v[2]-v[0]-x-y)<1e-10:
            from scipy.spatial.transform import Rotation
            matrix=np.column_stack((x/lx,y/ly,np.cross(x/lx,y/ly)))
            q=Rotation.from_matrix(matrix).as_quat()
            cube=UsdGeom.Cube.Define(stage,path);cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*v.mean(axis=0).tolist()))
            cube.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3].tolist())))
            cube.AddScaleOp().Set(Gf.Vec3f(float(lx),float(ly),THICKNESS))
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material(stage,path+'/look',color))
            if collide:physical(cube.GetPrim())
            return cube
    if collide:
        # Native box strips form an inscribed approximation to a thin convex
        # polygon. The SDK reduced the 0.45 mm triangular prism to a 1.15 mm
        # tetrahedron even with minThickness overridden. Avoid that cook path.
        # The displayed strips ARE the colliders; no hidden thickening.
        from scipy.spatial.transform import Rotation
        origin=v.mean(axis=0);ex=(v[1]-v[0])/np.linalg.norm(v[1]-v[0])
        ey=np.cross(normal,ex);basis=np.column_stack((ex,ey,normal))
        uv=(v-origin)@basis[:,:2]
        def interval(y):
            xs=[]
            for start,end in zip(uv,np.roll(uv,-1,axis=0)):
                if abs(end[1]-start[1])<1e-12:
                    if abs(y-start[1])<1e-10:xs.extend([start[0],end[0]])
                elif min(start[1],end[1])-1e-12<=y<=max(start[1],end[1])+1e-12:
                    u=(y-start[1])/(end[1]-start[1]);xs.append(start[0]+u*(end[0]-start[0]))
            if not xs:raise ValueError('Invalid convex polygon section')
            return min(xs),max(xs)
        # Include every vertex ordinate so no strip crosses a slope change.
        ys=np.unique(np.r_[np.linspace(uv[:,1].min(),uv[:,1].max(),FACET_STRIPS+1),uv[:,1]])
        root=UsdGeom.Xform.Define(stage,path)
        q=Rotation.from_matrix(basis).as_quat()
        look=material(stage,path+'/look',color)
        for i,(lo,hi) in enumerate(zip(ys[:-1],ys[1:])):
            if hi-lo<1e-9:continue
            a,b=interval(lo),interval(hi);left=max(a[0],b[0]);right=min(a[1],b[1])
            if right-left<1e-8:continue
            centre=origin+ex*((left+right)/2)+ey*((lo+hi)/2)
            cube=UsdGeom.Cube.Define(stage,path+f'/strip_{i:03d}');cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre.tolist()))
            cube.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3].tolist())))
            cube.AddScaleOp().Set(Gf.Vec3f(float(right-left),float(hi-lo),THICKNESS))
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(look)
            physical(cube.GetPrim())
        return root
    points = np.concatenate([v-normal*THICKNESS/2, v+normal*THICKNESS/2])
    faces = [list(range(n-1,-1,-1)), list(range(n,2*n))]
    faces += [[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    mesh = UsdGeom.Mesh.Define(stage,path)
    mesh.CreatePointsAttr(points.tolist())
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr('none')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(stage,path+'/look',color))
    return mesh


def joint(stage, name, kind, b0, b1, point, end=None, threshold=None):
    from pxr import Gf, UsdPhysics
    cls = UsdPhysics.RevoluteJoint if kind=='hinge' else UsdPhysics.SphericalJoint
    j = cls.Define(stage, ROOT+'/carton_body/joints/'+name)
    j.CreateBody0Rel().SetTargets([ROOT+'/'+b0])
    j.CreateBody1Rel().SetTargets([ROOT+'/'+b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*(point-CENTRES[b0]).tolist()))
    j.CreateLocalPos1Attr(Gf.Vec3f(*(point-CENTRES[b1]).tolist()))
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateCollisionEnabledAttr(False)
    if end is not None:
        direction = (end-point)/np.linalg.norm(end-point)
        q = Gf.Rotation(Gf.Vec3d(1.,0.,0.), Gf.Vec3d(*direction.tolist())).GetQuat()
        q = Gf.Quatf(q)
        j.CreateLocalRot0Attr(q); j.CreateLocalRot1Attr(q)
        j.CreateAxisAttr('X')
        if MECHANICAL_ONLY:
            bounds={'spout_base':(-90.,0.),'roof_minus':(0.,120.),'roof_plus':(-120.,0.)}
            if name.startswith('fin_'):bounds[name]=(0.,0.)
            if name in bounds:
                lo,hi=bounds[name];j.CreateLowerLimitAttr(lo);j.CreateUpperLimitAttr(hi)
            # Constitutive drives are initialized after all bodies exist, in
            # finish_filters. Their rest angles evolve only with actual strain.
        else:
            drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular')
            drive.CreateTypeAttr('force'); drive.CreateStiffnessAttr(.000005 if name.startswith('fin_') else 0.)
            drive.CreateTargetPositionAttr(0.)
            drive.CreateDampingAttr(.000001)
            drive.CreateTargetVelocityAttr(0.)
    if threshold is not None:
        j.CreateBreakForceAttr(threshold)
    return j


def decorate(prim, name):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema
    stage=prim.GetStage(); path=str(prim.GetPath())
    # The original Arena object remains registered and owns reset/state handling.
    for child in list(prim.GetChildren()): stage.RemovePrim(child.GetPath())
    prim.AddAppliedSchema('PhysxRigidBodyAPI')
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    for key, value in [('solverPositionIterationCount',192 if MECHANICAL_ONLY else 16),('solverVelocityIterationCount',16 if MECHANICAL_ONLY else 4)]:
        prim.CreateAttribute('physxRigidBody:'+key,Sdf.ValueTypeNames.Int).Set(value)
    if name in FACETS:
        color = (.94,.94,.87) if 'inner' in name or 'spout' in name else (.78,.9,.95)
        if 'fin' in name: color=(.90,.88,.72) if 'inner_fin' in name else (.1,.38,.55)
        panel(stage,path+'/paper',FACETS[name]-CENTRES[name],color,relief=CREASE_RELIEF)
        return
    c=CENTRES[name]
    # Four real walls, bottom, rear gable and stationary parts of the roofs.
    lo=-HEIGHT
    surfaces = {
        'front': [[-A,-A,lo],[-A,A,lo],[-A,A,0],[-A,-A,0]],
        'back': [[A,A,lo],[A,-A,lo],[A,-A,0],[A,A,0]],
        'left': [[-A,A,lo],[A,A,lo],[A,A,0],[-A,A,0]],
        'right': [[A,-A,lo],[-A,-A,lo],[-A,-A,0],[A,-A,0]],
        'bottom': [[-A,-A,lo],[A,-A,lo],[A,A,lo],[-A,A,lo]],
        'rear_gable': [[A,-A,0],[A,A,0],[A,0,A]],
        'roof_left': [[-A,A,0],[A,A,0],[A,0,A],[0,0,A]],
        'roof_right': [[A,-A,0],[-A,-A,0],[0,0,A],[A,0,A]],
        'rear_fin': [[0,0,A],[A,0,A],[A,0,A+FIN],[0,0,A+FIN]],
    }
    for n,v in surfaces.items():
        panel(stage,path+'/'+n,np.array(v)-c,(.72,.87,.95) if n in ('front','back','left','right') else (.87,.93,.95),
              relief=CREASE_RELIEF if n.startswith('roof_') or n in ('rear_gable','rear_fin') else 0.)
    # Printed opening indicator, a surface marking without an extra sensor.
    panel(stage,path+'/opening_arrow',np.array([[-A-.0003,-.008,-.018],[-A-.0003,.008,-.018],
                                               [-A-.0003,0.,-.005]])-c,(.06,.22,.4),False)
    for n,b0,b1,p,e in HINGES: joint(stage,n,'hinge',b0,b1,p,e)
    if SEALED and not MECHANICAL_ONLY:
        # Concentrated bond approximation, deliberately uncalibrated. Unlike a
        # pose trigger, the solver must develop the specified reaction force.
        for k,u in enumerate((.25,.75)):
            joint(stage,f'outer_seal_{k}','bond','carton_fin_minus','carton_fin_plus',
                  (1-u)*TIP+u*INNER+[0,0,FIN/2],threshold=7.5)
        joint(stage,'inner_seal','bond','carton_body','carton_spout',INNER,threshold=10.)
    # Facets start laminated. Filter only these initial paper-paper coincidences;
    # there is no blanket robot/carton or table/carton collision suppression.


def finish_filters(stage):
    from pxr import UsdPhysics
    if MECHANICAL_ONLY:
        # Ideal linked folds: omit sheet self-contact forces, while retaining
        # every collider for real robot/table contact. No runtime pose commands.
        for name in CENTRES:
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(ROOT+'/'+name)).CreateFilteredPairsRel().SetTargets(
                [ROOT+'/'+other for other in CENTRES if other!=name])
        crease_memory().native_drives(stage,ROOT+'/carton_body/joints')
        return
    pairs=[('carton_inner_minus','carton_roof_minus'),('carton_inner_plus','carton_roof_plus'),
           ('carton_fin_minus','carton_fin_plus')]
    for left,right in pairs:
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(ROOT+'/'+left)).CreateFilteredPairsRel().AddTarget(ROOT+'/'+right)


def crease_memory():
    from .plasticity import CreaseMemory
    names=list(CENTRES);initial=initial_poses()
    q=np.array([np.r_[ORIGIN+initial[n]['center'],initial[n]['quat_xyzw']] for n in names])
    return CreaseMemory(HINGES,names,q)


def contract():
    from .plasticity import STIFFNESS_NM_PER_DEG,YIELD_TORQUE_NM,DAMPING
    return dict(origin=ORIGIN.tolist(),width_m=WIDTH,body_height_m=HEIGHT,panel_thickness_m=THICKNESS,
                filled_body_mass_kg=BODY_MASS,areal_mass_kg_m2=AREAL_MASS,sealed=SEALED and not MECHANICAL_ONLY,
                mechanical_only=MECHANICAL_ONLY,initial_outer_deg=INITIAL_OUTER_DEG if MECHANICAL_ONLY else 0.,
                paper_self_contact=not MECHANICAL_ONLY,initial_poses=initial_poses(),
                crease_material_model='elastic-perfectly-plastic angular memory; uncalibrated' if MECHANICAL_ONLY else None,
                crease_material_parameters=dict(stiffness_Nm_per_deg=STIFFNESS_NM_PER_DEG,
                    yield_torque_Nm=YIELD_TORQUE_NM,damping=DAMPING,
                    rest_update='actual relative angle beyond elastic range only; no task goal or success input'),
                inner_pouring_fin_height_m=INNER_FIN,outer_fin_height_m=FIN,
                inner_pouring_fin_central_crease=True,
                carton_solver_iterations=[192,16] if MECHANICAL_ONLY else [16,4],
                geometry_reference='US5816487A, projecting inner sealing panel33; dimensions are provisional',
                local_solver_position_iterations=192 if MECHANICAL_ONLY else 16,
                local_solver_velocity_iterations=16 if MECHANICAL_ONLY else 4,
                facets={n:v.tolist() for n,v in FACETS.items()},centres={n:v.tolist() for n,v in CENTRES.items()},
                hinges=[dict(name=n,bodies=[b0,b1],edge=[p.tolist(),e.tolist()]) for n,b0,b1,p,e in HINGES],
                native_bond_limits_N={} if MECHANICAL_ONLY else {'outer_two_sites_each':7.5,'inner':10.},crease_relief_m=CREASE_RELIEF,
                fin_crease_stiffness=0. if MECHANICAL_ONLY else .000005,facet_strips=FACET_STRIPS,
                facet_representation='inscribed native box strips; rendered exactly as colliders',calibrated=False,
                limitation='Rigid facets/lower body; no bulk paper buckling, tearing, cohesive peel front or fluids. Not a finished task.')
