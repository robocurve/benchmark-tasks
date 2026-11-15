"""Contact-gated ideal dry pinch for native flexible cloth.

PhysX surface cloth has dynamic but no static friction. This approximation
holds a small patch already between nearly closed pads in the measured hand
frame. It does not attract cloth, select task landmarks, or move unheld nodes.
Opening the hand releases the patch. Native hard vertex attachments resolve
cloth and hand together; a >20 mm residual releases a failed binding. Finite
static friction and force-limited grasp slip are not calibrated.
The same material rule applies to every policy, including Dream.
"""
import numpy as np


class Pinch:
    def __init__(self):
        self.indices = np.empty(0, dtype=int)
        self.local = None
        self.events = []
        self.time = 0.
        self.last_correction = 0.
        self.blocked_until_open = False

    def update(self, points, velocities, tcp, rotation, gap, dt):
        self.time += dt
        p = np.asarray(points)
        if gap > .008:
            if len(self.indices):
                self.events.append(dict(time_s=self.time, event='opened', count=len(self.indices)))
            self.indices = np.empty(0, dtype=int)
            self.blocked_until_open = False
            return
        if not len(self.indices):
            if gap > .004 or self.blocked_until_open:
                return
            local = (p - tcp) @ rotation
            inside = ((abs(local[:, 0]) <= gap / 2 + .0012) &
                      (abs(local[:, 1]) <= .009) &
                      (local[:, 2] >= -.029) & (local[:, 2] <= .011))
            indices = np.flatnonzero(inside)
            if len(indices) < 3:
                return
            # Limit the area to a pad-sized patch around the nearest contact.
            centre = indices[np.argmin(np.linalg.norm(local[indices]-[0, 0, -.009], axis=1))]
            indices = indices[np.linalg.norm(p[indices]-p[centre], axis=1) <= .012]
            if len(indices) < 3:
                return
            self.indices = indices
            self.local = local[indices].copy()
            self.events.append(dict(time_s=self.time, event='pinched', count=len(indices),
                                    indices=indices.tolist(), gap_m=float(gap)))
        targets = self.local @ rotation.T + tcp
        correction = targets - p[self.indices]
        self.last_correction = float(np.max(np.linalg.norm(correction, axis=1)))
        if self.last_correction > .020:
            self.events.append(dict(time_s=self.time, event='slipped', correction_m=self.last_correction))
            self.indices = np.empty(0, dtype=int)
            self.blocked_until_open = True
            return
        # Position projection only at the contact patch. Keep all other native
        # state exactly unchanged; native FEM solves the remaining garment.
        velocities[self.indices] += correction / dt
        p[self.indices] = targets



def install(base, cloth_name):
    """Native hard static-friction approximation, with two-way reaction.

    A vertex attachment is created only after measured contact qualifies.
    PhysX then resolves the cloth and hand together; opening disables it.
    Neither vertex positions nor robot poses are written by this backend.
    NVIDIA's native attachment ignores finite stiffness, so force-limited
    grasp slip is not claimed. The20mm check detects a failed native binding.
    """
    import omni.usd
    from pxr import Sdf,UsdPhysics,Vt
    from interpret_motion import rotation_xyzw
    from yam_arena.constants import FINGER_JOINTS,TCP_OFFSET
    stage=omni.usd.get_context().get_stage()
    cloth=base.scene[cloth_name]
    candidates=[str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).endswith('/'+cloth_name)
                and 'OmniPhysicsDeformableBodyAPI' in p.GetAppliedSchemas()]
    if len(candidates)!=1:raise RuntimeError('Expected one deformable body for pinch')
    cloth_path=candidates[0];hands=[]
    for side in ('left','right'):
        robot=base.scene[side+'_robot']
        from yam_arena.embodiment import SIDES
        arm_prim=SIDES[side]['prim']
        paths=[str(p.GetPath()) for p in stage.Traverse() if '/'+arm_prim+'/' in str(p.GetPath())
               and str(p.GetPath()).endswith('/gripper') and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(paths)!=1:raise RuntimeError('Expected one measured gripper link for '+side)
        path=cloth_path+'_'+side+'_dry_pinch'
        prim=stage.DefinePrim(path,'OmniPhysicsVtxXformAttachment')
        prim.CreateRelationship('omniphysics:src0').SetTargets([Sdf.Path(cloth_path)])
        prim.CreateRelationship('omniphysics:src1').SetTargets([Sdf.Path(paths[0])])
        enabled=prim.CreateAttribute('omniphysics:attachmentEnabled',Sdf.ValueTypeNames.Bool);enabled.Set(False)
        indices=prim.CreateAttribute('omniphysics:vtxIndicesSrc0',Sdf.ValueTypeNames.IntArray);indices.Set([])
        positions=prim.CreateAttribute('omniphysics:localPositionsSrc1',Sdf.ValueTypeNames.Point3fArray);positions.Set([])
        hands.append((side,robot,list(robot.body_names).index('gripper'),
            robot.find_joints(list(FINGER_JOINTS),preserve_order=True)[0],Pinch(),enabled,indices,positions))
    original=base.scene.write_data_to_sim
    def array(value):return getattr(value,'torch',value).detach().cpu().numpy()
    def apply(*args,**kwargs):
        result=original(*args,**kwargs)
        points=array(cloth.data.nodal_pos_w)[0]
        for side,robot,body,fingers,pinch,enabled,indices,positions in hands:
            pose=array(robot.data.body_link_pose_w)[0,body];R=rotation_xyzw(pose[3:])
            tcp=pose[:3]+R@np.asarray(TCP_OFFSET)
            gap=float(abs(array(robot.data.joint_pos)[0,fingers]).sum())
            was=bool(len(pinch.indices))
            # Selection and failure checks run on copies. Native state remains
            # untouched; the native vertex attachment enforces the constraint.
            pinch.update(points.copy(),np.zeros_like(points),tcp,R,gap,float(base.physics_dt))
            now=bool(len(pinch.indices))
            if now and not was:
                indices.Set(pinch.indices.tolist())
                positions.Set(Vt.Vec3fArray((pinch.local+np.asarray(TCP_OFFSET)).tolist()))
                enabled.Set(True)
            elif was and not now:enabled.Set(False)
        return result
    base.scene.write_data_to_sim=apply
    return {side:p for side,_,_,_,p,_,_,_ in hands}
