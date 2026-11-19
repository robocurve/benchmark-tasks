"""Read-only native rigid contacts for diagnosing glass grip and tap actuation."""
import json
import numpy as np


class NativeContacts:
    def __init__(self, base, stage, directory):
        from pxr import Usd, UsdPhysics
        self.stage=stage
        self.file=(directory/'native-rigid-contacts.jsonl').open('w')
        paths=[]
        for suffix in ('/glass/glass','/tap/lever','/tap/frame','/tap/cylinder','/tap/piston'):
            matches=[str(p.GetPath()) for p in stage.Traverse(Usd.TraverseInstanceProxies())
                     if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).endswith(suffix)]
            if len(matches)!=1:raise RuntimeError(f'Ambiguous native contact body: {suffix}')
            paths.extend(matches)
        self.view=base.sim.physics_manager.get_physics_sim_view().create_rigid_contact_view(
            paths,filter_patterns=[],max_contact_data_count=8192)
        self.sensors=list(self.view.sensor_paths)
        if set(self.sensors)!=set(paths):raise RuntimeError('Native contact sensors did not bind')
        self.cache={}
        self.file.write(json.dumps(dict(event='binding',sensors=self.sensors))+'\n')

    def sample(self, step):
        import warp as wp
        values=[np.array(a.numpy(),copy=True) for a in self.view.get_raw_contact_data(dt=1.)]
        if len(values)!=7:raise RuntimeError('Unexpected native contact buffers')
        impulse,point,normal,separation,count,start,ids=values
        active=[]
        used=set()
        for sensor,n,first in zip(self.sensors,count,start):
            n,first=int(n),int(first)
            if n<0 or (n and (first<0 or first+n>len(ids))):
                raise RuntimeError('Invalid native contact range')
            for i in range(first,first+n):
                if i in used:raise RuntimeError('Overlapping native contact ranges')
                used.add(i);active.append((sensor,i))
        if len(used)>=len(ids):raise RuntimeError('Native contact buffer saturated')
        unknown=sorted({int(ids[i]) for _,i in active}-self.cache.keys())
        paths=self.view.get_other_actor_paths_from_ids(wp.array(unknown,dtype=wp.uint64,device='cpu')) if unknown else []
        if len(paths)!=len(unknown):raise RuntimeError('Native contact counterpart unresolved')
        for identity,path in zip(unknown,paths):
            path=str(path)
            if not self.stage.GetPrimAtPath(path).IsValid():raise RuntimeError('Unknown native contact actor')
            self.cache[identity]=path
        rows=[]
        for sensor,i in active:
            if not np.isfinite(np.r_[impulse[i],point[i],normal[i],separation[i]]).all():
                raise RuntimeError('Nonfinite native contact')
            rows.append(dict(body=sensor,other=self.cache[int(ids[i])],
                impulse_Ns=float(impulse[i,0]),point_m=point[i].tolist(),
                normal=normal[i].tolist(),separation_m=float(separation[i,0])))
        self.file.write(json.dumps(dict(physics_step=step,rows=rows))+'\n')

    def close(self):
        self.file.close()
