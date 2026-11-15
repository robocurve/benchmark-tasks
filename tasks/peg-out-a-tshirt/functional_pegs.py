"""Contact-driven spring pegs with ideal static grip and native flexible cloth.

Measured finger compression opens the jaws. A peg follows a hand only after
its handles are between the pads. Releasing closes it; it stays on the line
only when the line is inside the mouth. Actual cloth vertices in that mouth
are constrained to the jaw until it is opened. No target hem or task score
selects captured material. Spring force, frictional slip and free-peg tumbling
are approximated; garment gravity, bending and self-contact remain native FEM.
"""
import math
import numpy as np


def ry(a):
    c,s=math.cos(a),math.sin(a)
    return np.array(((c,0,s),(0,1,0),(-s,0,c)))


def configure(stage, root):
    from pxr import UsdPhysics,PhysxSchema,Sdf
    from yam_arena.embodiment import SIDES
    # Hand/peg contact is resolved by the measured handle-grip law below.
    # Also solving those contacts against a prescribed kinematic peg injects
    # a second, incompatible constraint and pushes the fingers apart.
    robot_parts=[p.GetPath() for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and
        any('/'+SIDES[s]['prim']+'/' in str(p.GetPath()) for s in ('left','right'))]
    if not robot_parts:
        # Props are spawned before the robot. USD relationships may target
        # these known YAM rigid links before their prims are authored.
        parent=root.rsplit('/',1)[0]
        chain=['Geometry/base']
        for name in ('link1','link2','link3','link4','link5','gripper'):
            chain.append(chain[-1]+'/'+name)
        chain += [chain[-1]+'/tip_left',chain[-1]+'/tip_right']
        robot_parts=[Sdf.Path(parent+'/'+SIDES[s]['prim']+'/'+name)
                     for s in ('left','right') for name in chain]
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(root+'/'):
            if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(False)
                pairs=UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
                pairs.SetTargets(list(dict.fromkeys(list(pairs.GetTargets())+robot_parts)))


class Peg:
    def __init__(self,position,angle):
        self.p=np.array(position,float);self.R=ry(angle);self.opening=-.075
        self.held=None;self.clipped=True;self.indices=[];self.local=None
        self.events=[];self.time=0.;self.v=np.zeros(3)

    def update(self,hands,points,dt):
        self.time+=dt;previous=self.held
        if self.held is None:
            for side,tcp,R,gap in hands:
                handle=(np.asarray(tcp)-self.p)@self.R
                # The18mm pads can enclose a section of the35mm handles;
                # their centres need not coincide. Require the handle span,
                # lateral overlap and matching compression direction.
                if (-.033<handle[0]<-.003 and abs(handle[1])<.012 and abs(handle[2])<.006 and
                    abs(float(R[:,0]@self.R[:,2]))>.80 and gap<.024):
                    self.held=side;self.grip_p=R.T@(self.p-tcp);self.grip_R=R.T@self.R
                    self.events.append(dict(time_s=self.time,event='handles_grasped',side=side));break
        if self.held is not None:
            side,tcp,R,gap=next(h for h in hands if h[0]==self.held)
            self.p=np.asarray(tcp)+R@self.grip_p;self.R=R@self.grip_R
            if gap<.027:
                self.opening=-float(np.clip((.022-gap)/.0175,.075,.45))
                if self.opening<-.15:
                    if self.clipped:self.events.append(dict(time_s=self.time,event='opened'))
                    self.clipped=False;self.indices=[]
            else:
                self.held=None;self.opening=-.075
                point=np.array((.52,self.p[1],1.09));local=(point-self.p)@self.R
                # The3mm line lies within the30mm gripping mouth; jaw rotation
                # closes onto it. This geometric fit replaces submillimetre friction.
                self.clipped=bool(abs(self.p[1])<.235 and .020<local[0]<.036 and
                                  abs(local[1])<.004 and -.003<local[2]<.004)
                self.indices=[]
                if self.clipped:
                    q=(points-self.p)@self.R
                    inside=(q[:,0]>.015)&(q[:,0]<.036)&(abs(q[:,1])<.005)&(abs(q[:,2])<.006)
                    self.indices=np.flatnonzero(inside).tolist();self.local=q[self.indices]@ry(-self.opening/2)
                self.events.append(dict(time_s=self.time,event='released',clipped=self.clipped,captured_vertices=len(self.indices)))
        elif not self.clipped:
            self.v[2]-=9.81*dt;self.p+=self.v*dt
            if self.p[2]<.7825:self.p[2]=.7825;self.v[:]=0.
        return previous!=self.held


def install(base,cloth_name='tshirt'):
    import torch,omni.usd,omni.physics.tensors as tensors
    from pxr import UsdUtils,Sdf,Vt,Gf,UsdGeom
    from scipy.spatial.transform import Rotation
    from interpret_motion import rotation_xyzw
    from yam_arena.constants import FINGER_JOINTS,TCP_OFFSET
    from arena_props import PEG_SPECS
    stage=omni.usd.get_context().get_stage();cache=UsdUtils.StageCache.Get()
    # Confirm that every pre-authored pair resolves after robot construction.
    for name in PEG_SPECS:
        for part in ('lower','upper'):
            prim=stage.GetPrimAtPath('/World/envs/env_0/'+name+'/'+part)
            from pxr import UsdPhysics
            pairs=UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()
            if any(not stage.GetPrimAtPath(p).IsValid() for p in pairs):
                raise RuntimeError('Peg collision filter has an unresolved native body')
    stage_id=cache.GetId(stage).ToLongInt()
    if stage_id<0:cache.Insert(stage);stage_id=cache.GetId(stage).ToLongInt()
    sim=tensors.create_simulation_view('torch',stage_id);sim.set_subspace_roots('/')
    cloth=base.scene[cloth_name];root='/World/envs/env_0/';cloth_path=root+cloth_name
    hands=[]
    for side in ('left','right'):
        r=base.scene[side+'_robot'];hands.append((side,r,list(r.body_names).index('gripper'),r.find_joints(list(FINGER_JOINTS),preserve_order=True)[0]))
    items={}
    for name,spec in PEG_SPECS.items():
        peg=Peg(spec.pos,spec.base_angle)
        view=sim.create_rigid_body_view([root+name+'/lower',root+name+'/upper'])
        paths=list(view.prim_paths);order=[paths.index(root+name+'/'+s) for s in ('lower','upper')]
        # A closed peg is stationary on the fixed line. Use an identity world
        # anchor for its static jaw contact: this avoids this Isaac version's
        # stale attachment transform cache for tensor-driven kinematic bodies.
        # Only material actually inside the jaw at closure is retained.
        anchor=UsdGeom.Xform.Define(stage,'/World/'+name+'_closed_jaw_anchor')
        attachment=stage.DefinePrim(root+name+'_cloth_grip','OmniPhysicsVtxXformAttachment')
        attachment.CreateRelationship('omniphysics:src0').SetTargets([Sdf.Path(cloth_path)])
        attachment.CreateRelationship('omniphysics:src1').SetTargets([anchor.GetPath()])
        enabled=attachment.CreateAttribute('omniphysics:attachmentEnabled',Sdf.ValueTypeNames.Bool);enabled.Set(False)
        indices=attachment.CreateAttribute('omniphysics:vtxIndicesSrc0',Sdf.ValueTypeNames.IntArray);indices.Set([])
        positions=attachment.CreateAttribute('omniphysics:localPositionsSrc1',Sdf.ValueTypeNames.Point3fArray);positions.Set([])
        ops=[]
        for side in ('lower','upper'):
            xform=UsdGeom.Xformable(stage.GetPrimAtPath(root+name+'/'+side))
            by_type={op.GetOpType():op for op in xform.GetOrderedXformOps()}
            translation=by_type.get(UsdGeom.XformOp.TypeTranslate)
            orient=by_type.get(UsdGeom.XformOp.TypeOrient)
            if translation is None:translation=xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            if orient is None:orient=xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            xform.SetXformOpOrder([translation,orient]);ops.append((translation,orient))
        items[name]=(peg,view,order,enabled,indices,positions,ops)
    original=base.scene.write_data_to_sim
    def array(x):return getattr(x,'torch',x).detach().cpu().numpy()
    def apply(*args,**kwargs):
        result=original(*args,**kwargs);observed=[];points=array(cloth.data.nodal_pos_w)[0]
        for side,r,body,fingers in hands:
            pose=array(r.data.body_link_pose_w)[0,body];R=rotation_xyzw(pose[3:]);gap=float(abs(array(r.data.joint_pos)[0,fingers]).sum())
            observed.append((side,pose[:3]+R@np.asarray(TCP_OFFSET),R,gap))
        for name,(peg,view,order,enabled,indices,positions,ops) in items.items():
            old=tuple(peg.indices)
            if old:
                expected=peg.local@(peg.R@ry(-peg.opening/2)).T+peg.p
                peg.attachment_error_m=float(np.max(np.linalg.norm(points[peg.indices]-expected,axis=1)))
                if peg.attachment_error_m>.020:
                    enabled.Set(False);peg.indices=[]
                    peg.events.append(dict(time_s=peg.time,event='failed_cloth_binding',residual_m=peg.attachment_error_m))
            if hasattr(peg,'restore_indices'):
                peg.indices=peg.restore_indices;peg.local=peg.restore_local
                del peg.restore_indices;del peg.restore_local
            peg.update(observed,points,float(base.physics_dt))
            if tuple(peg.indices)!=old:
                enabled.Set(False)
                if peg.indices:
                    target=peg.local@(peg.R@ry(-peg.opening/2)).T+peg.p
                    indices.Set(peg.indices);positions.Set(Vt.Vec3fArray(target.tolist()));enabled.Set(True)
            # GPU kinematic targets and runtime USD transform edits are not
            # supported by this build. Apply the explicitly kinematic material
            # poses through the native rigid-body API and verify the readback.
            targets=view.get_transforms().clone()
            old=array(targets).copy()
            for index,R,(translate,orient) in zip(order,(peg.R@ry(-peg.opening/2),peg.R@ry(peg.opening/2)),ops):
                q=Rotation.from_matrix(R).as_quat()
                # The native attachment API also reads the target Xform. Keep
                # it coherent with the explicitly kinematic native body pose.
                translate.Set(Gf.Vec3d(*peg.p))
                orient.Set(Gf.Quatd(float(q[3]),Gf.Vec3d(*q[:3])))
                targets[index]=torch.as_tensor(np.r_[peg.p,q],device=targets.device,dtype=targets.dtype)
            indices_all=torch.arange(view.count,device=targets.device,dtype=torch.int32)
            view.set_transforms(targets,indices_all)
            actual=array(view.get_transforms())
            if not np.allclose(actual[:,:3],array(targets)[:,:3],rtol=0.,atol=2e-6):
                raise RuntimeError('Native peg pose update was not applied')
        return result
    base.scene.write_data_to_sim=apply
    # Keep the parent tensor view alive for the complete simulation.
    return dict(simulation_view=sim,pegs={name:item[0] for name,item in items.items()})


def restore_checkpoint(mechanisms, state, provenance, base):
    """Reset-only reconstruction for a labelled, unscored continuation probe."""
    from scipy.spatial.transform import Rotation
    from yam_arena.constants import TCP_OFFSET
    if not provenance.get('functional_pegs'):return
    for name,record in provenance['functional_pegs'].items():
        peg=mechanisms['pegs'][name];pose=state[name+'_body_poses'][0]
        peg.p=pose[:3].copy();peg.opening=float(record['opening_rad'])
        peg.R=Rotation.from_quat(pose[3:]).as_matrix()@ry(peg.opening/2)
        peg.clipped=bool(record['clipped']);peg.held=record['held'];peg.indices=[]
        # If a clip held fabric, the normal attachment callback initializes it
        # from the recorded native material on the first physics step.
        ids=list(record['captured_vertices'])
        if ids:
            peg.restore_indices=ids
            peg.restore_local=(state['nodes'][ids]-peg.p)@(peg.R@ry(-peg.opening/2))
        if peg.held:
            r=base.scene[peg.held+'_robot'];body=list(r.body_names).index('gripper')
            value=getattr(r.data.body_link_pose_w,'torch',r.data.body_link_pose_w)
            hand=value.detach().cpu().numpy()[0,body];R=Rotation.from_quat(hand[3:]).as_matrix()
            tcp=hand[:3]+R@np.asarray(TCP_OFFSET)
            peg.grip_p=R.T@(peg.p-tcp);peg.grip_R=R.T@peg.R
        peg.events=[dict(event='checkpoint_reconstruction',source_step=provenance['source_step'])]
