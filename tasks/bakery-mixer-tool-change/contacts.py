"""Native contact evidence for support scoring and diagnostic snapshots."""
import json
import numpy as np


class NativeContacts:
    def __init__(self,base):
        base.sim.set_setting('/physics/disableContactProcessing',False)
        if bool(base.sim.get_setting('/physics/disableContactProcessing')):
            raise RuntimeError('Native contact processing is disabled')
        paths=['/World/envs/env_0/'+p for p in ['beater/beater','guard/guard','mixer/chassis','mixer/bowl','mixer/lift_handle']]
        self.view=base.sim.physics_manager.get_physics_sim_view().create_rigid_contact_view(
            paths,filter_patterns=[],max_contact_data_count=4096)
        self.paths=list(self.view.sensor_paths)
        if set(self.paths)!=set(paths):raise RuntimeError('Contact observer bound incorrect bodies')

    def external_beater_contact(self):
        import warp as wp
        impulse,point,normal,separation,count,start,actors=[np.array(v.numpy(),copy=True)
            for v in self.view.get_raw_contact_data(dt=1.)]
        sensor=self.paths.index('/World/envs/env_0/beater/beater')
        indices=range(int(start[sensor]),int(start[sensor])+int(count[sensor]))
        ids=sorted({int(actors[i]) for i in indices if abs(float(impulse[i,0]))>1e-7})
        paths=self.view.get_other_actor_paths_from_ids(wp.array(ids,dtype=wp.uint64,device='cpu')) if ids else []
        allowed={'/World/envs/env_0/mixer/shaft','/World/envs/env_0/mixer/retaining_collar'}
        return any(str(path) not in allowed for path in paths)

    def sample(self,control):
        import warp as wp
        impulse,point,normal,separation,count,start,actors=[np.array(v.numpy(),copy=True)
            for v in self.view.get_raw_contact_data(dt=1.)]
        if sum(map(int,count))>=len(impulse):raise RuntimeError('Native contact diagnostic capacity exhausted')
        indices=[i for n,offset in zip(count,start) for i in range(int(offset),int(offset)+int(n))]
        ids=sorted({int(actors[i]) for i in indices})
        paths=self.view.get_other_actor_paths_from_ids(wp.array(ids,dtype=wp.uint64,device='cpu')) if ids else []
        resolved=dict(zip(ids,map(str,paths)))
        pairs={}
        for path,n,offset in zip(self.paths,count,start):
            for i in range(int(offset),int(offset)+int(n)):
                amount=abs(float(impulse[i,0]))
                if amount<1e-7:continue
                key=' | '.join(sorted([path,resolved[int(actors[i])]])).replace('/World/envs/env_0/','')
                entry=pairs.setdefault(key,[0.,np.zeros(3),np.zeros(3)])
                entry[0]+=amount;entry[1]+=amount*point[i];entry[2]+=float(impulse[i,0])*normal[i]
        result={key:dict(normal_impulse_magnitude_Ns=value[0],position_m=(value[1]/value[0]).tolist(),
                         reported_normal_impulse_vector_Ns=value[2].tolist())
                for key,value in pairs.items() if value[0]>.00001}
        print('MIXER_CONTACTS '+json.dumps(dict(control=control,sampling='last physics frame',pairs=result)),flush=True)
