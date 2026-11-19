"""CQ-inspired native crank/rocking-cylinder geometry and stroke metering.

The dimensions below are estimates for the original reconstruction, not
manufacturer internal measurements. The 1/4 imperial pint rating is sourced.
No helper here commands a body pose or creates score from handle movement.
"""
import math

import numpy as np


LEVER_PIVOT = np.array([0., 0., -.035])
CYLINDER_PIVOT = np.array([0., 0., -.230])
CRANK_PIN = np.array([.055, 0., 0.])
PULL_ANGLE = math.radians(60)
NOMINAL_STROKE_ML = 568.26125 / 4
PISTON_REST = LEVER_PIVOT + CRANK_PIN
REST_LENGTH = float(np.linalg.norm(PISTON_REST-CYLINDER_PIVOT))
CYLINDER_ANGLE = math.atan2(*(PISTON_REST-CYLINDER_PIVOT)[[0, 2]])
CYLINDER_ROTATION = (math.cos(CYLINDER_ANGLE/2), 0., math.sin(CYLINDER_ANGLE/2), 0.)


def crank_pin(angle):
    """Reference kinematics for authoring/robot waypoints, not fluid readback."""
    c, s = math.cos(angle), math.sin(angle)
    return LEVER_PIVOT + np.array([c*CRANK_PIN[0], 0., -s*CRANK_PIN[0]])


STROKE = float(np.linalg.norm(crank_pin(-PULL_ANGLE)-CYLINDER_PIVOT)-REST_LENGTH)
EFFECTIVE_AREA = NOMINAL_STROKE_ML*1e-6/STROKE
# Native world transforms are float32; resolve motion at 1 micrometre.
DISPLACEMENT_RESOLUTION_M = 1e-6


def body_frames():
    """Product-local rest transforms; cylinder/piston mesh axes are local Z."""
    rotation = list(CYLINDER_ROTATION)
    return {
        'lever': dict(position=LEVER_PIVOT.tolist(), orientation=[1., 0., 0., 0.]),
        'cylinder': dict(position=CYLINDER_PIVOT.tolist(), orientation=rotation),
        'piston': dict(position=PISTON_REST.tolist(), orientation=rotation),
    }


def joint_definitions():
    """Native articulation tree; add_loop_joint closes its mechanical loop.

    The effective return spring and hydraulic damping are explicit estimates.
    They apply passive forces; neither drive has a moving position target.
    """
    return [
        dict(name='handle_hinge', type='revolute', parent='frame', child='lever',
             position_parent=LEVER_PIVOT.tolist(), axis='Y', lower=-PULL_ANGLE,
             upper=0., stiffness=2., damping=.08, target=math.radians(2)),
        dict(name='cylinder_hinge', type='revolute', parent='frame', child='cylinder',
             position_parent=CYLINDER_PIVOT.tolist(), orientation_parent=list(CYLINDER_ROTATION),
             axis='Y', lower=-.5, upper=.5,
             damping=.002),
        dict(name='piston_slide', type='prismatic', parent='cylinder', child='piston',
             position_parent=[0., 0., REST_LENGTH], axis='Z', lower=0., upper=STROKE,
             damping=120.),
    ]


def add_loop_joint(stage, root):
    """Connect the actual rod end to the crank with a native excluded hinge."""
    from pxr import Gf, Sdf, UsdPhysics

    path = str(root).rstrip('/')
    hinge = UsdPhysics.RevoluteJoint.Define(stage, path+'/crank_pin')
    hinge.CreateBody0Rel().SetTargets([Sdf.Path(path+'/lever')])
    hinge.CreateBody1Rel().SetTargets([Sdf.Path(path+'/piston')])
    hinge.CreateLocalPos0Attr(Gf.Vec3f(*CRANK_PIN))
    hinge.CreateLocalPos1Attr(Gf.Vec3f(0.))
    hinge.CreateLocalRot0Attr(Gf.Quatf(1.))
    hinge.CreateLocalRot1Attr(Gf.Quatf(1.))
    hinge.CreateAxisAttr('Y')
    hinge.CreateCollisionEnabledAttr(False)
    hinge.CreateExcludeFromArticulationAttr(True)
    return hinge


def measured_extension(cylinder_position, cylinder_rotation, piston_position):
    """Read native body transforms, projecting rod-end separation on barrel Z."""
    c = np.asarray(cylinder_position, dtype=float)
    r = np.asarray(cylinder_rotation, dtype=float)
    p = np.asarray(piston_position, dtype=float)
    if c.shape != (3,) or p.shape != (3,) or r.shape != (3, 3) or not np.isfinite(np.r_[c, p, r.ravel()]).all():
        raise ValueError('Invalid native pump transform')
    if not np.allclose(r.T@r, np.eye(3), atol=1e-4) or np.linalg.det(r)<.999:
        raise ValueError('Invalid native cylinder rotation')
    return float(np.dot(p-c, r[:, 2])-REST_LENGTH)


class StrokeMeter:
    """Ideal single-acting primed pump boundary, from measured rod movement.

    Positive extension dispenses; shortening refills without outlet delivery.
    Partial reversals remain possible. There is no full-home rearming gate,
    rate cap, angle-open timer, or dependence on the receiving glass.
    """
    def __init__(self, initial_extension):
        self.previous = float(initial_extension)
        if not math.isfinite(self.previous):
            raise ValueError('Invalid initial native piston extension')
        self.supplied_ml = 0.

    def update(self, extension):
        current = float(extension)
        if not math.isfinite(current):
            raise ValueError('Invalid native piston extension')
        displacement=current-self.previous
        # Keep sub-resolution movement cumulative; ignore only readback jitter.
        if abs(displacement)<DISPLACEMENT_RESOLUTION_M:
            return 0.
        volume = max(displacement, 0.)*EFFECTIVE_AREA*1e6
        self.previous = current
        self.supplied_ml += volume
        return volume


if __name__=='__main__':
    assert STROKE > 0
    c, s = math.cos(CYLINDER_ANGLE), math.sin(CYLINDER_ANGLE)
    rotation=np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    assert abs(measured_extension(CYLINDER_PIVOT, rotation, PISTON_REST))<1e-12
    # Mount translation/yaw must not change a measured rod extension.
    yaw=.4; c, s = math.cos(yaw), math.sin(yaw)
    mount=np.array([[c,-s,0.], [s,c,0.], [0.,0.,1.]])
    shift=np.array([.3,-.1,.77])
    piston=PISTON_REST+rotation[:,2]*STROKE
    assert math.isclose(measured_extension(mount@CYLINDER_PIVOT+shift,
        mount@rotation, mount@piston+shift), STROKE, rel_tol=1e-12)
    totals=[]
    for samples in (8, 160):
        meter=StrokeMeter(0.)
        for value in np.linspace(0., STROKE, samples):meter.update(value)
        totals.append(meter.supplied_ml)
        assert meter.update(STROKE)==0.
        assert meter.update(0.)==0.
        assert math.isclose(meter.update(STROKE/2), NOMINAL_STROKE_ML/2, rel_tol=1e-12)
    assert all(math.isclose(v, NOMINAL_STROKE_ML, rel_tol=1e-12) for v in totals)
    # The native stationary trace has <60 nm world-pose readback variation.
    for rest in (0.,.000123456,STROKE):
        meter=StrokeMeter(rest)
        for noise in np.tile([0.,-80e-9,-20e-9],1000):
            assert meter.update(rest+noise)==0.
    meter=StrokeMeter(0.)
    for value in np.linspace(0.,5e-6,101):meter.update(value)
    assert 0<=5e-6*EFFECTIVE_AREA*1e6-meter.supplied_ml<=DISPLACEMENT_RESOLUTION_M*EFFECTIVE_AREA*1e6+1e-12
    print('Stroke arithmetic passed; native linkage and fluid behavior are not validated.')
