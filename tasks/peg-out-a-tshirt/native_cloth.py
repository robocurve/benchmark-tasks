"""Native PhysX surface FEM authoring. Import USD only after SimulationApp.

No runtime vertex targets, attachment APIs, kinematic cloth or pose writes.
Sources: NVIDIA Omni Physics surface-deformable documentation and IsaacLab
revision 91a0906f93802eb859192dcb0bca879f891e92d6. Parameters are provisional;
the probe must establish stability/contact before any task evaluation.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Fabric:
    thickness: float = .0006
    density: float = 300.0  # 180 g/m2 at the nominal 0.6 mm thickness
    youngs_modulus: float = 2e6
    poissons_ratio: float = .3
    dynamic_friction: float = .55
    bend_modulus: float = 1.8e5  # physical bend coefficient is proportional to t**3
    elasticity_damping: float = .005
    bend_damping: float = .005
    contact_offset: float = .0008
    rest_offset: float = .0003
    solver_iterations: int = 32


def _set(prim, name, value):
    attr = prim.GetAttribute(name)
    if not attr:
        raise RuntimeError(f'Installed simulator lacks required attribute {name}')
    if not attr.Set(value):
        raise RuntimeError(f'Cannot author {name}')


def author_cloth(stage, path, mesh, material=Fabric(), points=None, color=(.08,.35,.72),vertex_colors=None):
    """Create one triangle mesh that is both simulated and rendered.

    ``points`` may differ from the unstressed rest mesh for initial draping only.
    Caller must inspect strain and collision at that reset pose. This authoring
    function is never called after the simulation starts.
    """
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Vt, PhysxSchema
    mesh.audit()
    p = mesh.rest if points is None else np.asarray(points)
    if p.shape != mesh.rest.shape or not np.isfinite(p).all():
        raise ValueError('initial cloth points do not match rest mesh')
    tri = UsdGeom.Mesh.Define(stage, path)
    tri.CreatePointsAttr(Vt.Vec3fArray(p.tolist()))
    tri.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
    tri.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
    tri.CreateSubdivisionSchemeAttr('none')
    tri.CreateDoubleSidedAttr(True)
    tri.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if vertex_colors is not None:
        colors=np.asarray(vertex_colors,dtype=float)
        if colors.shape!=p.shape or not np.isfinite(colors).all() or np.any(colors<0) or np.any(colors>1):
            raise ValueError('invalid cloth material-coordinate colours')
        pv=tri.GetDisplayColorPrimvar()
        pv.SetInterpolation('vertex')
        pv.Set(Vt.Vec3fArray(colors.tolist()))
    prim = tri.GetPrim()
    for schema in ('OmniPhysicsDeformableBodyAPI','OmniPhysicsSurfaceDeformableSimAPI',
                   'PhysxSurfaceDeformableBodyAPI'):
        if not prim.ApplyAPI(schema):
            raise RuntimeError(f'Native surface cloth API unavailable: {schema}')
    _set(prim,'omniphysics:restShapePoints',Vt.Vec3fArray(mesh.rest.tolist()))
    _set(prim,'omniphysics:restTriVtxIndices',[Gf.Vec3i(*map(int,t)) for t in mesh.faces])
    # The sewn side strips have non-flat rest bends. Native flatDefault would
    # try to unfold those seams even though restShapePoints describes them.
    # This sets their physical rest angles, with unchanged finite stiffness.
    _set(prim,'omniphysics:restBendAnglesDefault','restShapeDefault')
    _set(prim,'omniphysics:mass',mesh.audit()['surface_area_m2']*material.thickness*material.density)
    _set(prim,'physxDeformableBody:selfCollision',True)
    import os
    if os.environ.get('ROBOCURVE_SHIRT_SURFACE_CONTACT_DIAGNOSTIC') == '1':
        _set(prim,'physxDeformableBody:enableSpeculativeCCD',True)
        _set(prim,'physxDeformableBody:collisionPairUpdateFrequency',8)
    _set(prim,'physxDeformableBody:disableGravity',False)
    _set(prim,'physxDeformableBody:solverPositionIterationCount',material.solver_iterations)
    UsdPhysics.CollisionAPI.Apply(prim)
    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr(material.contact_offset)
    collision.CreateRestOffsetAttr(material.rest_offset)
    mat = UsdShade.Material.Define(stage,path+'/Fabric')
    m=mat.GetPrim()
    for api in ('OmniPhysicsBaseMaterialAPI','OmniPhysicsDeformableMaterialAPI',
                'OmniPhysicsSurfaceDeformableMaterialAPI','PhysxSurfaceDeformableMaterialAPI'):
        if not m.ApplyAPI(api):
            raise RuntimeError(f'Missing material schema {api}')
    for attr,val in {'omniphysics:density':material.density,
                     'omniphysics:dynamicFriction':material.dynamic_friction,
                     'omniphysics:youngsModulus':material.youngs_modulus,
                     'omniphysics:poissonsRatio':material.poissons_ratio,
                     'omniphysics:surfaceThickness':material.thickness,
                     'omniphysics:surfaceBendStiffness':material.bend_modulus,
                     'physxDeformableMaterial:elasticityDamping':material.elasticity_damping,
                     'physxDeformableMaterial:bendDamping':material.bend_damping}.items():
        _set(m,attr,val)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
    return tri


def author_box(stage,path,size,pos,color=(.5,.5,.5),dynamic=False,mass=.005,friction=.55):
    from pxr import Gf,UsdGeom,UsdPhysics,UsdShade,PhysxSchema
    root=UsdGeom.Xform.Define(stage,path)
    root.AddTranslateOp().Set(Gf.Vec3d(*pos))
    prim=root.GetPrim()
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(8)
        rb.CreateEnableCCDAttr(True)
    shape=UsdGeom.Cube.Define(stage,path+'/Shape')
    shape.CreateSizeAttr(1.)
    shape.AddScaleOp().Set(Gf.Vec3d(*size))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    cp=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
    cp.CreateContactOffsetAttr(.0005)
    cp.CreateRestOffsetAttr(0.)
    mat=UsdShade.Material.Define(stage,path+'/Material')
    pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(friction)
    pm.CreateDynamicFrictionAttr(friction)
    pm.CreateRestitutionAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
    return root


def author_peg_jaw(stage,path,pos,*,upper=False,angle=0.):
    """One ordinary rigid jaw; called only by scene construction."""
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    body=UsdGeom.Xform.Define(stage,path)
    body.AddTranslateOp().Set(Gf.Vec3d(*pos))
    body.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(math.cos(angle/2),0.,math.sin(angle/2),0.))
    p=body.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr(.004)
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(8)
    rb.CreateEnableCCDAttr(True)
    sign=1 if upper else -1
    author_box(stage,path+'/jaw',(.035,.010,.006),(.0175,0.,sign*.0035),(.68,.48,.25))
    author_box(stage,path+'/handle',(.035,.010,.006),(-.0175,0.,sign*.008),(.68,.48,.25))
    return p


def author_peg_hinge(stage,path,pos=(0.,0.,0.),*,fixed_lower=False,spring=True):
    """Constant closing spring and revolute constraint, not an actuator policy."""
    from pxr import Gf,Sdf,UsdPhysics
    hinge=UsdPhysics.RevoluteJoint.Define(stage,path+'/hinge')
    hinge.CreateBody0Rel().SetTargets([Sdf.Path(path+'/lower')])
    hinge.CreateBody1Rel().SetTargets([Sdf.Path(path+'/upper')])
    hinge.CreateAxisAttr('Y')
    hinge.CreateLocalPos0Attr(Gf.Vec3f(0.))
    hinge.CreateLocalPos1Attr(Gf.Vec3f(0.))
    hinge.CreateLowerLimitAttr(-45.)
    hinge.CreateUpperLimitAttr(15.)
    hinge.CreateCollisionEnabledAttr(True)
    drive=UsdPhysics.DriveAPI.Apply(hinge.GetPrim(),'angular')
    drive.CreateTypeAttr('force')
    # USD angular drives use degrees. 0.4 Nm/rad * 0.1 rad preload = .04 Nm.
    drive.CreateStiffnessAttr(.4*math.pi/180 if spring else 0.)
    drive.CreateDampingAttr(.003*math.pi/180 if spring else 0.)
    drive.CreateTargetPositionAttr(math.degrees(.1))
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateMaxForceAttr(.35)
    if fixed_lower:
        clamp=UsdPhysics.FixedJoint.Define(stage,path+'/diagnostic_fixture')
        clamp.CreateBody1Rel().SetTargets([Sdf.Path(path+'/lower')])
        clamp.CreateLocalPos0Attr(Gf.Vec3f(*pos))
    return hinge


def author_peg(stage,path,pos=(0.,0.,0.),*,fixed_lower=False,spring=True,open_angle=-.35,base_angle=0.):
    """Two rigid jaws with a passive spring. Only a diagnostic may fix one jaw.

    Nominal 70 x 10 x 22 mm open-envelope wooden peg, mass 8 g. Jaw contact
    remains active. No controller changes the spring or attaches any fabric.
    Closing torque/friction require physical calibration.
    """
    author_peg_jaw(stage,path+'/lower',pos,upper=False,angle=base_angle)
    author_peg_jaw(stage,path+'/upper',pos,upper=True,angle=base_angle+open_angle)
    hinge=author_peg_hinge(stage,path,pos,fixed_lower=fixed_lower,spring=spring)
    if fixed_lower and base_angle:
        from pxr import Gf,UsdPhysics
        fixed=UsdPhysics.FixedJoint(stage.GetPrimAtPath(path+'/diagnostic_fixture'))
        fixed.CreateLocalRot0Attr(Gf.Quatf(math.cos(base_angle/2),0.,math.sin(base_angle/2),0.))
    return hinge


def ensure_gpu_scene(stage):
    from pxr import UsdPhysics,PhysxSchema
    scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:
        raise RuntimeError('Create the simulation context before cloth')
    for prim in scenes:
        api=PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateEnableGPUDynamicsAttr(True)
        api.CreateBroadphaseTypeAttr('GPU')
        api.CreateEnableCCDAttr(False)  # Unsupported together with the GPU tensor pipeline
        api.CreateSolverTypeAttr('TGS')
