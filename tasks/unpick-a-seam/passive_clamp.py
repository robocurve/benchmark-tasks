"""Force-limited passive sewing jaw on an ideal linear guide.

The guide is an ordinary mechanical constraint of the fixture, never a cloth
attachment. Its fixed spring target is authored once. At the cloth's 0.4 mm
contact thickness the nominal closing load is 0.5 N; the native solver must
demonstrate actual load, retention and stability before this fixture is used.
"""
from dataclasses import asdict,dataclass
from geometry import TABLE_TOP
from clamp_layout import PLANE_X,TOP,SUPPORT_SHIFT


@dataclass(frozen=True)
class SpringJaw:
    mass_kg:float=.012
    initial_gap_m:float=.0006
    stiffness_n_m:float=1250.
    damping_n_s_m:float=8.
    force_limit_n:float=.6
    opening_travel_m:float=.001


PARAMETERS=SpringJaw()
FRONT_FACE=PLANE_X-.0006
MOVING_NAME='clamp_moving_jaw'
MOVING_SIZE=(.006,.085,.008)
MOVING_POSITION=(FRONT_FACE+PARAMETERS.initial_gap_m+.003,0.,TOP-.004)


def fixture_boxes(*,lower_edge_supported=False):
    """Fixed front jaw and overhead frame leave the rear jaw's travel clear."""
    shift=SUPPORT_SHIFT if lower_edge_supported else 0.
    top=TOP-shift
    moving=(MOVING_POSITION[0],0.,MOVING_POSITION[2]-shift)
    return [
        ('clamp_front_jaw',(.006,.085,.016),(FRONT_FACE-.003,0.,top)),
        (MOVING_NAME,MOVING_SIZE,moving),
        # Move the bolted foot behind the lower fabric when its edge rests on
        # the table; otherwise the old foot would penetrate its bottom 10 mm.
        ('clamp_base',(.080,.110,.010),(.350 if lower_edge_supported else .320,0.,TABLE_TOP+.005)),
        ('clamp_post',(.012,.012,.148-shift),(.320,0.,TABLE_TOP+.084-shift/2)),
        ('clamp_top_bridge',(.0326,.012,.004),(.3097,0.,top+.006)),
    ]


def description():
    return {**asdict(PARAMETERS),'guide':'world-fixed X-axis prismatic bearing',
            'target_displacement_m':-PARAMETERS.initial_gap_m,
            'lower_limit_m':-PARAMETERS.initial_gap_m,
            'upper_limit_m':PARAMETERS.opening_travel_m,
            'nominal_seated_load_n':PARAMETERS.stiffness_n_m*.0004,
            'contact_thickness_m':.0004,'cloth_attachments':False,
            'runtime_target_changes':False}


def author_guide(stage,path,position=MOVING_POSITION):
    from pxr import Gf,Sdf,UsdPhysics,PhysxSchema
    prim=stage.GetPrimAtPath(path)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError('The moving sewing jaw must be an ordinary dynamic rigid body')
    body=UsdPhysics.RigidBodyAPI(prim)
    body.CreateKinematicEnabledAttr(False)
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(64)
    rb.CreateSolverVelocityIterationCountAttr(4)
    rb.CreateMaxDepenetrationVelocityAttr(.1)
    joint=UsdPhysics.PrismaticJoint.Define(stage,path+'/ClosingGuide')
    # Empty body0 means the fixed world. The physical stand is bolted down;
    # its guide removes five jaw DOFs while allowing spring-driven X motion.
    joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
    joint.CreateAxisAttr('X')
    joint.CreateLocalPos0Attr(Gf.Vec3f(*position))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.))
    joint.CreateLowerLimitAttr(-PARAMETERS.initial_gap_m)
    joint.CreateUpperLimitAttr(PARAMETERS.opening_travel_m)
    drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear')
    drive.CreateTypeAttr('force')
    drive.CreateTargetPositionAttr(-PARAMETERS.initial_gap_m)
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateStiffnessAttr(PARAMETERS.stiffness_n_m)
    drive.CreateDampingAttr(PARAMETERS.damping_n_s_m)
    drive.CreateMaxForceAttr(PARAMETERS.force_limit_n)
    return joint
