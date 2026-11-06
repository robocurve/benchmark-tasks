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
            if name in base.scene['ice'].body_names:
                cfg=base.scene['ice'].cfg.rigid_objects[name]
            else:
                obj=base.scene[name]
                cfg=getattr(obj,"cfg",obj)
            value=cfg.prim_path.replace("env_[^/]+","env_0").replace("env_.*","env_0")
            return value
        paths={n:path(n) for n in self.names}
        self.sensors=[n for n in self.names if n in ("Scoop","CleanLid") or n.startswith(("CleanIce_","FishIce_"))]
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
        self.subscription=PhysxManager.register_callback(self._sample,IsaacEvents.POST_PHYSICS_STEP,
            name="ice_fish_b1_contact_history",wrap_weak_ref=False)

    def _sample(self,*args):
        self.callback_count+=1
        try:
            _,_,_,distance,count,start=self.view.get_contact_data(dt=1/240)
            counts=array(count).reshape(len(self.sensors),len(self.filters))
            starts=array(start).reshape(counts.shape)
            distances=array(distance).reshape(-1)
            for i,j in np.argwhere(counts>0):
                a,b=self.sensors[i],self.filters[j]
                if a==b:continue
                first=int(starts[i,j]);last=first+int(counts[i,j])
                if last>len(distances):raise RuntimeError("Contact data buffer overflow")
                if np.any(distances[first:last]<=.0005):
                    pair=tuple(sorted((a,b)));self.pending.add(pair);self.seen.add(pair);self.contact_count+=1
        except Exception as exc:self.error=repr(exc)

    def drain(self):
        if self.error:raise RuntimeError("PhysX contact tensor failed: "+self.error)
        result,self.pending=self.pending,set()
        return result

    def reset(self):
        self.pending.clear();self.seen.clear();self.callback_count=0;self.contact_count=0

    def report(self):
        return {"source":"PhysX contact tensors sampled after every 240 Hz physics step",
            "callback_count":self.callback_count,"contact_count":self.contact_count,
            "pairs_seen":[list(p) for p in sorted(self.seen)],"maximum_contact_separation_m":.0005,
            "sensor_count":len(self.sensors),"filter_count":len(self.filters),"error":self.error}

    def close(self):
        if self.subscription is not None:
            self.subscription.deregister()
            self.subscription=None
