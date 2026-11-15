"""Quasistatic tacking stitches on native flexible fabric.

A closed tack keeps its loop shape relative to the adjacent upper fabric
panel. Two native spherical connectors retain the layers at its needle holes.
A measured inward blade contact severs the tack, releases those connectors,
and lets the open native thread fall. Thread sliding through individual fibres
and knot friction are approximated. Fabric bending, tool contact and the
blade-through-fabric damage check remain physical. No demonstration phase,
target stitch order or scoring state controls the material law.
"""
import numpy as np


def prepare_thread(stage, item):
    from pxr import UsdPhysics
    cloth=[p.GetPath() for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)
           and any('/'+name+'/' in str(p.GetPath()) for name in ('cloth_left','cloth_right'))]
    if not cloth:raise RuntimeError('Tacking model requires native fabric panels')
    item['approximated_cloth_pairs']=[str(p) for p in cloth]
    for path in item['bodies']:
        prim=stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
        pairs=UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
        pairs.SetTargets(list(dict.fromkeys(list(pairs.GetTargets())+cloth)))
    for path in item['joints']:
        UsdPhysics.Joint(stage.GetPrimAtPath(path)).CreateJointEnabledAttr(False)


class Tacks:
    def __init__(self, stage, base, threads, shells, views, thread_view, order):
        from pxr import Gf,Sdf,UsdPhysics
        from scipy.spatial.transform import Rotation
        self.stage=stage;self.threads=threads;self.views=views
        self.view=thread_view;self.order=order;self.closed=[True]*len(threads)
        self.connections=[];self.anchors=[];self.events=[]
        def arr(v):return v.detach().cpu().numpy()
        poses=arr(thread_view.get_transforms())[order]
        start=0
        for index,t in enumerate(threads):
            n=len(t['bodies']);part=poses[start:start+n];centre=part[:,:3].mean(0)
            panel_ids=[int(np.argmin(np.linalg.norm(s['model']['centres']-centre,axis=1))) for s in shells]
            upper=views[0].native_state()[0][panel_ids[0]]
            R=Rotation.from_quat(upper[3:]).as_matrix()
            self.anchors.append((panel_ids[0],(part[:,:3]-upper[:3])@R,
                np.einsum('ij,njk->nik',R.T,Rotation.from_quat(part[:,3:]).as_matrix()),start,n))
            # Two positions along the original10mm tack constrain translation
            # but leave the flexible panels free to bend about the seam.
            paths=[]
            for k,z in enumerate((float(t['nodes'][:,2].min()),float(t['nodes'][:,2].max()))):
                point=np.array((.300,centre[1],z))
                ids=[int(np.argmin(np.linalg.norm(s['model']['centres']-point,axis=1))) for s in shells]
                j=UsdPhysics.SphericalJoint.Define(stage,f'/World/tack_{index}_binding_{k}')
                for side in (0,1):
                    path=shells[side]['bodies'][ids[side]]
                    getattr(j,f'CreateBody{side}Rel')().SetTargets([Sdf.Path(path)])
                    pose=views[side].native_state()[0][ids[side]]
                    local=Rotation.from_quat(pose[3:]).as_matrix().T@(point-pose[:3])
                    getattr(j,f'CreateLocalPos{side}Attr')(Gf.Vec3f(*local))
                j.CreateJointEnabledAttr(True);j.CreateCollisionEnabledAttr(False)
                paths.append(str(j.GetPath()))
            self.connections.append(paths);start+=n
        original=base.scene.write_data_to_sim
        def apply(*args,**kwargs):
            result=original(*args,**kwargs);self.follow();return result
        base.scene.write_data_to_sim=apply

    def follow(self):
        import torch
        from scipy.spatial.transform import Rotation
        if not any(self.closed):return
        poses=self.view.get_transforms().clone();panels=self.views[0].native_state()[0]
        changed=[]
        for closed,(panel,local,rotation,start,n) in zip(self.closed,self.anchors):
            if not closed:continue
            p=panels[panel];R=Rotation.from_quat(p[3:]).as_matrix()
            target=np.c_[local@R.T+p[:3],Rotation.from_matrix(np.einsum('ij,njk->nik',R,rotation)).as_quat()]
            ids=self.order[start:start+n];poses[ids]=torch.as_tensor(target,dtype=poses.dtype,device=poses.device)
            changed.extend(ids.tolist())
        self.view.set_transforms(poses,torch.tensor(changed,dtype=torch.int32,device=poses.device))

    def cut(self, global_joints, time_s):
        from pxr import UsdPhysics
        from native_stitches import set_joint_enabled
        boundaries=np.cumsum([len(t['bodies']) for t in self.threads])
        for joint in global_joints:
            index=int(np.searchsorted(boundaries,int(joint),side='right'))
            if not self.closed[index]:continue
            self.closed[index]=False;t=self.threads[index]
            start=0 if index==0 else int(boundaries[index-1]);cut=int(joint)-start
            for path in self.connections[index]:
                UsdPhysics.Joint(self.stage.GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)
            for path in t['bodies']:
                prim=self.stage.GetPrimAtPath(path)
                UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Set(False)
                # The coarse tack model does not resolve fibre-scale sliding
                # through the needle holes. Keep thread/fabric contact omitted
                # after severing as well: restoring overlapping tiny contacts
                # injects nonphysical impulses into the canvas. Open fragments
                # remain native dynamic chains with tool/table contact.
            for j in range(len(t['joints'])):
                set_joint_enabled(self.stage,t,j,j!=cut,restore_contact=False)
            self.events.append(dict(time_s=float(time_s),stitch=index,cut_joint=cut))
