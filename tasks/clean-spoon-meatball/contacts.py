"""PhysX contact tensors sampled after every physics step, including substeps."""
import re
import numpy as np


def array(value):
    if hasattr(value,"detach"):return value.detach().cpu().numpy()
    if hasattr(value,"numpy"):return value.numpy()
    return np.asarray(value)


class ContactMonitor:
    def __init__(self,env,names):
        from isaaclab.app.settings_manager import get_settings_manager
        from isaaclab_physx.physics.physx_manager import PhysxManager,IsaacEvents
        import omni.usd
        from pxr import Usd,UsdPhysics
        get_settings_manager().set_bool("/physics/disableContactProcessing",False)
        self.names=list(names);base=env.unwrapped
        def path(name):
            obj=base.scene[name]
            cfg=getattr(obj,"cfg",obj)
            value=cfg.prim_path.replace("env_[^/]+","env_0").replace("env_.*","env_0")
            return value
        paths={n:path(n) for n in self.names}
        self.sensors=[n for n in self.names if n in ("CleanSpoon","UsedSpoon") or n.startswith(("Meatball_","RawChicken_"))]
        stage=omni.usd.get_context().get_stage()
        self.filters=[];filter_paths=[]
        for name,value in paths.items():
            prim=stage.GetPrimAtPath(value)
            if not prim.IsValid():
                raise RuntimeError(f"No authored contact prim for {name}: {value}")
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                filter_paths.append(value);self.filters.append(name)
            else:
                colliders=[p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]
                if not colliders:raise RuntimeError("No static contact shapes for "+name)
                for collider in colliders:
                    filter_paths.append(str(collider.GetPath()));self.filters.append(name)
        for side in ("left","right"):
            prefix=path(side+"_robot")
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith(prefix+"/") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    filter_paths.append(str(prim.GetPath()));self.filters.append(side+"_robot")
            if side+"_robot" not in self.filters:
                raise RuntimeError("No robot contact filter bodies for "+side)
        self.view=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            [paths[n] for n in self.sensors],filter_patterns=[filter_paths]*len(self.sensors),max_contact_data_count=4096)
        if self.view.sensor_count!=len(self.sensors) or self.view.filter_count!=len(self.filters):
            raise RuntimeError(f"Contact view dimensions disagree: {self.view.sensor_count}, {self.view.filter_count}")
        self.pending=set();self.seen=set();self.callback_count=0;self.contact_count=0;self.error=None
        self._totals=None
        self.subscription=PhysxManager.register_callback(self._sample,IsaacEvents.POST_PHYSICS_STEP,
            name="clean_spoon_contact_history",wrap_weak_ref=False)

    def _sample(self,*args):
        self.callback_count+=1
        try:
            import torch
            import warp as wp
            from contact_accumulator import accumulate
            _,_,_,distance,count,start=self.view.get_contact_data(dt=1/240)
            def tensor(value,dtype):
                return (value if isinstance(value,torch.Tensor) else wp.to_torch(value)).to(dtype=dtype)
            counts=tensor(count,torch.int32).reshape(len(self.sensors),len(self.filters))
            starts=tensor(start,torch.int32).reshape(counts.shape)
            distances=tensor(distance,torch.float32).reshape(-1)
            if self._totals is None:
                self._totals=torch.zeros((len(self.sensors),len(self.filters)+1),dtype=torch.int32,device=counts.device)
            wp.launch(accumulate,dim=tuple(counts.shape),inputs=[wp.from_torch(counts),wp.from_torch(starts),
                wp.from_torch(distances),wp.from_torch(self._totals)],
                stream=wp.stream_from_torch(torch.cuda.current_stream(counts.device)))
        except Exception as exc:self.error=repr(exc)

    def _collect(self):
        if self._totals is None:return
        totals=self._totals.cpu().numpy()
        self._totals.zero_()
        if np.any(totals[:,-1]):self.error='Contact data buffer overflow'
        for i,j in np.argwhere(totals[:,:-1]>0):
            a,b=self.sensors[i],self.filters[j]
            if a==b:continue
            pair=tuple(sorted((a,b)));self.pending.add(pair);self.seen.add(pair)
            self.contact_count+=int(totals[i,j])

    def drain(self):
        self._collect()
        if self.error:raise RuntimeError("PhysX contact tensor failed: "+self.error)
        result,self.pending=self.pending,set()
        return result

    def reset(self):
        self.pending.clear();self.seen.clear();self.callback_count=0;self.contact_count=0
        if self._totals is not None:self._totals.zero_()

    def report(self):
        self._collect()
        return {"source":"PhysX contact tensors sampled after every 240 Hz physics step",
            "callback_count":self.callback_count,"contact_count":self.contact_count,
            "pairs_seen":[list(p) for p in sorted(self.seen)],"maximum_contact_separation_m":.0005,
            "sensor_count":len(self.sensors),"filter_count":len(self.filters),"error":self.error}

    def close(self):
        if self.subscription is not None:
            self.subscription.deregister()
            self.subscription=None
