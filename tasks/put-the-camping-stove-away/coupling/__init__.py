"""A separable threaded connection between two real rigid bodies.

The native six-axis joint supplies lateral/tilt retention and axial resistance.
Its axial window follows measured relative rotation at the declared thread lead.
After the declared engagement turns it is disabled: neither body is copied,
teleported, pinned or moved by this module. This is an uncalibrated thread proxy,
not resolved helical contact or a model of the fuel seal.
"""
import math
import numpy as np

PATH = None
LEAD = 0.0254 / 28
TURNS = 2.0  # estimate; not a measured PocketRocket engagement
DRAG = 0.004  # N m s/rad, estimate
_angle = 0.0
_last = None
_released = False
_known = False


def reset():
    global _angle, _last, _released, _known
    _angle, _last, _released, _known = 0.0, None, False, False


def create(stage, path, canister_path, stove_path):
    global PATH
    from pxr import Gf, Sdf, UsdPhysics

    PATH = path
    j = UsdPhysics.Joint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(canister_path)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(stove_path)])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.062))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    # Put the native D6 twist axis (local X) along the vertical fitting axis.
    frame = Gf.Quatf(math.sqrt(.5), 0, -math.sqrt(.5), 0)
    j.CreateLocalRot0Attr().Set(frame)
    j.CreateLocalRot1Attr().Set(frame)
    j.CreateExcludeFromArticulationAttr().Set(True)
    j.CreateCollisionEnabledAttr().Set(False)
    for axis in ('transY', 'transZ', 'rotY', 'rotZ'):
        lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.)
        lim.CreateHighAttr().Set(-1.)
    axial = UsdPhysics.LimitAPI.Apply(j.GetPrim(), 'transX')
    axial.CreateLowAttr().Set(-0.0001)
    axial.CreateHighAttr().Set(0.0001)
    drag = UsdPhysics.DriveAPI.Apply(j.GetPrim(), 'rotX')
    drag.CreateTypeAttr().Set('force')
    drag.CreateStiffnessAttr().Set(0.)
    drag.CreateDampingAttr().Set(DRAG * math.pi / 180)
    drag.CreateTargetVelocityAttr().Set(0.)
    drag.CreateMaxForceAttr().Set(0.5)
    j.CreateJointEnabledAttr().Set(True)
    return j


def step(sim, can, stove):
    global _angle, _last, _released, _known
    import parts
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdPhysics

    rc = parts._quat_to_matrix(can.quat_xyzw)
    rs = parts._quat_to_matrix(stove.quat_xyzw)
    rel = rc.T @ rs
    raw = math.atan2(rel[1, 0], rel[0, 0])
    if not np.isfinite(np.r_[can.pos, stove.pos, raw]).all():
        raise ValueError('unknown coupling state')
    if _last is not None:
        _angle += math.atan2(math.sin(raw - _last), math.cos(raw - _last))
    _last, _known = raw, True
    j = UsdPhysics.Joint.Get(get_current_stage(), PATH)
    if not j:
        raise ValueError('native coupling missing')
    if not _released:
        travel = max(0., _angle) * LEAD / (2 * math.pi)
        axial = UsdPhysics.LimitAPI(j.GetPrim(), 'transX')
        axial.GetLowAttr().Set(max(-0.0001, travel - 0.0001))
        axial.GetHighAttr().Set(travel + 0.0001)
        if _angle >= TURNS * 2 * math.pi:
            j.GetJointEnabledAttr().Set(False)
            _released = True
    return _released


def restore(sim):
    from isaaclab.sim.utils import get_current_stage
    from pxr import UsdPhysics

    reset()
    j = UsdPhysics.Joint.Get(get_current_stage(), PATH)
    if not j:
        raise ValueError('coupling was not authored')
    axial = UsdPhysics.LimitAPI(j.GetPrim(), 'transX')
    axial.GetLowAttr().Set(-0.0001)
    axial.GetHighAttr().Set(0.0001)
    j.GetJointEnabledAttr().Set(True)
