"""USD joints whose local frames are computed from each body's spawn pose."""

from dataclasses import dataclass

import numpy as np

import bodykit
from yam_arena.ik import matrix_to_quat_xyzw

_connections = {}
_fastener_path = None
_fastener_paths = {}
_row_paths = {}
_active_rows = set()
_partners = {}
_pending_disable = set()
_break_subscription = None
_broken = False

def fastener_broken():
    if _pending_disable:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics
        for path in tuple(_pending_disable):
            UsdPhysics.Joint.Get(get_current_stage(),path).GetJointEnabledAttr().Set(False)
            _pending_disable.discard(path)
    return _broken



@dataclass(frozen=True)
class Connection:
    name: str
    parent: object
    child: object
    world_anchor: np.ndarray
    world_frame: np.ndarray
    travel: float | None = None
    stiffness: float = 42.0


def register(connection):
    _connections[connection.child.name] = connection


def _quat(frame):
    from pxr import Gf

    x, y, z, w = matrix_to_quat_xyzw(frame)
    return Gf.Quatf(float(w), float(x), float(y), float(z))


def _vec(v):
    from pxr import Gf

    return Gf.Vec3f(*map(float, v))


def _local(spec, point, frame):
    basis = bodykit.rotation(spec.start_xyzw) if spec.start_xyzw is not None else np.eye(3)
    return _vec(basis.T @ (point - spec.pos)), _quat(basis.T @ frame)


def _author_spawn_joints(stage, path, spec):
    from pxr import UsdPhysics

    _base_author_balls(stage, path, spec)
    connection = _connections.get(spec.name)
    if connection is None:
        return
    parent_path = path.rsplit("/", 1)[0]
    joint_path = f"{path}/{connection.name}"
    if connection.name == "seam":
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        drive = None
    elif connection.travel is None:
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
        joint.CreateAxisAttr("Y")
        joint.CreateLowerLimitAttr(-105.0)
        joint.CreateUpperLimitAttr(105.0)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateDampingAttr(0.00004)

    else:
        joint = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
        joint.CreateAxisAttr("X")
        joint.CreateLowerLimitAttr(-connection.travel)
        joint.CreateUpperLimitAttr(0.0)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateStiffnessAttr(connection.stiffness)
        drive.CreateDampingAttr(0.30)
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateMaxForceAttr(5.0)
    if drive is not None:
        drive.CreateTypeAttr("force")
    joint.CreateBody0Rel().SetTargets([f"{parent_path}/{connection.parent.name}/body"])
    joint.CreateBody1Rel().SetTargets([f"{path}/body"])
    p0, r0 = _local(connection.parent, connection.world_anchor, connection.world_frame)
    p1, r1 = _local(spec, connection.world_anchor, connection.world_frame)
    joint.CreateLocalPos0Attr(p0)
    joint.CreateLocalPos1Attr(p1)
    joint.CreateLocalRot0Attr(r0)
    joint.CreateLocalRot1Attr(r1)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)

    hook_names = ("strap_tip","tail_fabric_0","tail_fabric_1")
    if spec.name in hook_names:
        hook = hook_names.index(spec.name)
        for loop in range(4):
            pair = hook*4+loop
            _author_fastener(stage,parent_path,path,pair)
            _author_rows(stage,parent_path,path,pair)


def _author_rows(stage, parent, tip, index):
    from pxr import UsdPhysics
    for row in range(8):
        paths=[]
        for mode,axes in (("shear",("transX","transY")),("peel",("transZ",))):
            path=f"{tip}/engaged_{index}_{row}_{mode}"
            joint=UsdPhysics.Joint.Define(stage,path)
            joint.CreateBody0Rel().SetTargets([f"{parent}/strap_{index%4:02d}/body"])
            joint.CreateBody1Rel().SetTargets([f"{tip}/body"])
            for axis in axes:
                limit=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis)
                limit.CreateLowAttr(-.10 if axis=="transZ" else -.0012)
                limit.CreateHighAttr(.0012)
            joint.CreateJointEnabledAttr(False)
            joint.CreateCollisionEnabledAttr(True)
            joint.CreateExcludeFromArticulationAttr(True)
            _row_paths[index,row,mode]=path
            paths.append(path)
        _partners[paths[0]],_partners[paths[1]]=paths[1],paths[0]


def _seat_rows(stage, base, tip, receiver, points, index, first_row=0):
    from pxr import UsdPhysics
    from geometry import CELL_AREA
    positions = (points - receiver.pos) @ receiver.rotation
    mid = np.median(positions[:, :2], axis=0)
    quadrant = (positions[:, 0] >= mid[0]).astype(int) + 2 * (positions[:, 1] >= mid[1])
    for row in range(4):
        group = points[quadrant == row]
        if not len(group):
            continue
        anchor = group.mean(axis=0)
        width = np.ptp(group @ receiver.rotation[:,1]) + np.sqrt(CELL_AREA)
        capacities={"shear":103000.*len(group)*CELL_AREA, "peel":200.*width}
        for mode,capacity in capacities.items():
            path=_row_paths[index,first_row+row,mode]
            joint=UsdPhysics.Joint.Get(stage,path)
            joint.CreateLocalPos0Attr(_vec(base.rotation.T@(anchor-base.pos)))
            joint.CreateLocalPos1Attr(_vec(tip.rotation.T@(anchor-tip.pos)))
            joint.CreateLocalRot0Attr(_quat(base.rotation.T@receiver.rotation))
            joint.CreateLocalRot1Attr(_quat(tip.rotation.T@receiver.rotation))
            joint.CreateBreakForceAttr(float(capacity))
            joint.GetJointEnabledAttr().Set(True)
            _active_rows.add(path)


def recruit(base,tip,receiver,points,index,first_row):
    from isaaclab.sim.utils.stage import get_current_stage
    _seat_rows(get_current_stage(),base,tip,receiver,points,index,first_row)


def row_group_intact(index,first_row):
    return any(_row_paths[index,r,"peel"] in _active_rows for r in range(first_row,first_row+4))


def _author_fastener(stage, parent_path, tip_path, index, seated=False):
    global _fastener_path
    from pxr import UsdPhysics

    _fastener_path = f"{tip_path}/hook_and_loop_{index}"
    if seated:
        _fastener_path += "_seated"
    _fastener_paths[index, seated] = _fastener_path
    schema = UsdPhysics.FixedJoint if seated else UsdPhysics.Joint
    joint = schema.Define(stage, _fastener_path)
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        if seated:
            break
        translation = axis.startswith("trans")
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateStiffnessAttr(3000.0 if translation else 0.001)
        drive.CreateDampingAttr(0.6 if translation else 0.0001)
        drive.CreateMaxForceAttr(4.0 if translation else 0.010)
    joint.CreateBody0Rel().SetTargets([f"{parent_path}/strap_{index%4:02d}/body"])
    joint.CreateBody1Rel().SetTargets([f"{tip_path}/body"])
    joint.CreateJointEnabledAttr(False)
    joint.CreateCollisionEnabledAttr(True)
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateBreakForceAttr(7.0)
    joint.CreateBreakTorqueAttr(0.10)


def _fastener():
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    return UsdPhysics.Joint(get_current_stage().GetPrimAtPath(_fastener_path))


def set_fastener(base_pad=None, tip_pad=None, receiver=None, tip=None, index=0, points=None, seated=False):
    global _break_subscription, _broken, _fastener_path
    if _fastener_path is None:
        return
    _broken = False
    if _break_subscription is None:
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import SimulationEvent
        from pxr import PhysicsSchemaTools
        def on_event(event):
            global _broken
            if event.type == int(SimulationEvent.JOINT_BREAK):
                path = PhysicsSchemaTools.decodeSdfPath(*event.payload["jointPath"])
                if str(path) in _active_rows:
                    _active_rows.discard(str(path))
                    partner=_partners[str(path)]
                    _active_rows.discard(partner)
                    _pending_disable.add(partner)
                    _broken = not _active_rows
                elif str(path) == _fastener_path and not _active_rows:
                    _broken = True
        events = get_physx_interface().get_simulation_event_stream_v2()
        _break_subscription = events.create_subscription_to_pop(on_event)
    if base_pad is None:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics
        for path in _row_paths.values():
            UsdPhysics.Joint.Get(get_current_stage(),path).GetJointEnabledAttr().Set(False)
        _active_rows.clear()
        _pending_disable.clear()
        for path in _fastener_paths.values():
            _fastener_path = path
            joint = _fastener()
            joint.GetJointEnabledAttr().Set(False)
            joint.GetCollisionEnabledAttr().Set(True)
        return
    if seated:
        _fastener_path = _fastener_paths[index, False]
        _fastener().GetJointEnabledAttr().Set(False)
        from isaaclab.sim.utils.stage import get_current_stage
        _seat_rows(get_current_stage(),base_pad,tip_pad,receiver,points,index)
        return
    _fastener_path = _fastener_paths[index, seated]
    joint = _fastener()
    # Preserve the observed mating pose. The joint applies load without snapping either link.
    from geometry import PAD_LENGTH, STRAP_WIDTH
    contact = (receiver.pos + tip.pos) / 2
    delta = receiver.rotation.T @ (tip.pos - receiver.pos)
    length = max(0.0, PAD_LENGTH - abs(delta[0]))
    width = max(0.0, STRAP_WIDTH - abs(delta[1]))
    if points is not None:
        from geometry import CELL_AREA
        contact = np.mean(points,axis=0)
        width = min(width, np.ptp(points @ receiver.rotation[:,1])+np.sqrt(CELL_AREA))
        length = min(length, len(points)*CELL_AREA/max(width,1e-6))
    # Hook 88 / Loop 1000 datasheet averages, in SI units.
    shear_force = 103000.0 * length * width
    peel_moment = 200.0 * width * length / 2
    from pxr import UsdPhysics
    normal_force = 200.0 * width
    # Compliant hook-and-loop transmits load through the pile until it
    # separates. Every drive has a finite material-derived force limit.
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        if seated:
            break
        drive = UsdPhysics.DriveAPI.Get(joint.GetPrim(), axis)
        if axis.startswith("trans"):
            capacity = normal_force if axis == "transZ" else shear_force
            stiffness = capacity / 0.0012
            damping = 2 * np.sqrt(stiffness * 0.00125)
        else:
            capacity = shear_force * min(width, length) / 2 if axis == "rotZ" else peel_moment
            stiffness = capacity / 6.0
            damping = 0.00007
        drive.CreateMaxForceAttr(float(capacity))
        drive.CreateStiffnessAttr(float(stiffness))
        drive.CreateDampingAttr(float(damping))
    joint.CreateBreakForceAttr(float(np.hypot(np.sqrt(2) * shear_force, normal_force)))
    joint.CreateBreakTorqueAttr(float(np.hypot(np.sqrt(2) * peel_moment, shear_force * min(width, length) / 2)))
    print(f"[fastener] calibrated force={shear_force:.3f} moment={peel_moment:.4f}", flush=True)
    joint.CreateLocalPos0Attr(_vec(base_pad.rotation.T @ (contact - base_pad.pos)))
    joint.CreateLocalRot0Attr(_quat(base_pad.rotation.T @ receiver.rotation))
    joint.CreateLocalPos1Attr(_vec(tip_pad.rotation.T @ (contact - tip_pad.pos)))
    joint.CreateLocalRot1Attr(_quat(tip_pad.rotation.T @ receiver.rotation))
    # Once coupled, the joint carries contact load for this pair.
    joint.GetCollisionEnabledAttr().Set(not seated)
    joint.GetJointEnabledAttr().Set(True)


_base_author_balls = bodykit._author_balls
bodykit._author_balls = _author_spawn_joints
