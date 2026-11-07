"""Private per-physics-step GPU contacts, separate from policy observations."""
import numpy as np
from . import layout as L
def arm(path):
    return next((s for s in ('LeftArm','RightArm') if '/'+s+'/' in path),None)
def task_object(path):
    parts=path.split('/')
    return next((n for n in ['collar',*(o.name for o in L.FIXED)] if n in parts),None)
class Monitor:
    def __init__(self): self.ready=False; self.reset()
    def reset(self):
        self.events=[]; self.violations=[]; self.samples=0; self.active_grasp=False
        self.error=None; self.step=0; self.peak_grasp_force=0.; self.contact_pairs=set()
    def prohibited(self,a,b):
        aa,ab=arm(a),arm(b)
        if aa and ab: return aa!=ab
        for robot,other in ((a,b),(b,a)):
            if arm(robot):
                name=task_object(other)
                if name=='collar':return robot.rsplit('/',1)[-1] not in ('tip_left','tip_right')
                if name is not None:return True
                raise RuntimeError('Unclassified robot contact')
        names={task_object(a),task_object(b)}
        if None in names or 'collar' not in names:raise RuntimeError('Unclassified object contact')
        return any(n in L.PROTECTED for n in names)
    def bind(self,stage):
        from pxr import Usd,UsdPhysics
        from isaaclab.sim.utils.stage import get_current_stage_id
        import omni.physics.tensors as tensors
        from isaaclab_physx.physics.physx_manager import PhysxManager
        root='/World/envs/env_0/'
        sensors=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root[:-1]))
          if arm(str(p.GetPath())) and 'PhysxContactReportAPI' in p.GetAppliedSchemas()]
        assert len(sensors)==18, sensors
        body_names=('base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right')
        assert {(arm(p),p.rsplit('/',1)[-1]) for p in sensors}=={(s,n) for s in ('LeftArm','RightArm') for n in body_names}
        sensors += [root+'collar']
        coverage={obj.name:[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root+obj.name))
                           if p.HasAPI(UsdPhysics.CollisionAPI)] for obj in L.FIXED}
        assert all(coverage.values()),coverage
        fixed=[p for paths in coverage.values() for p in paths]
        self.sim=tensors.create_simulation_view('torch',stage_id=get_current_stage_id())
        self.sim.set_subspace_roots('/')
        self.view=self.sim.create_rigid_contact_view(sensors,[fixed+sensors]*len(sensors),max_contact_data_count=4096)
        view=self.view
        sensor_paths=list(view.sensor_paths);filter_paths=list(view.filter_paths)
        assert view.sensor_count==19
        flat=[p for group in filter_paths for p in group] if isinstance(filter_paths[0],(list,tuple)) else filter_paths
        assert set(flat)==set(fixed+sensors),'Contact filter coverage mismatch'
        self.contract=dict(sensors=list(view.sensor_paths),filters=list(view.filter_paths),physics_hz=240,
                           capacity=view.max_contact_data_count,fixed_coverage=coverage)
        original=PhysxManager.step
        def sample():
            self.samples+=1
            forces,points,normals,sep,counts,starts=[v.detach().cpu().numpy() for v in view.get_contact_data(dt=1/240)]
            if int(counts.sum())>=view.max_contact_data_count: raise RuntimeError('Contact capacity exhausted')
            self.active_grasp=False
            seen=set()
            for i,j in np.argwhere(counts>0):
                a=sensor_paths[i]
                b=filter_paths[i][j] if isinstance(filter_paths[i],(tuple,list)) else filter_paths[j]
                key=tuple(sorted((a,b)))
                if a==b or key in seen or (arm(a) and arm(a)==arm(b)): continue
                seen.add(key)
                start,n=int(starts[i,j]),int(counts[i,j]); sl=slice(start,start+n)
                if start<0 or n<1 or start+n>len(forces):raise RuntimeError('Invalid contact buffer range')
                if not all(np.isfinite(v[sl]).all() for v in (forces,points,sep)): raise RuntimeError('Nonfinite contact')
                force=float(np.abs(forces[sl]).sum()); separation=float(sep[sl].min())
                # Discard zero-force speculative manifold points, not real contacts.
                if force<=1e-6: continue
                grasp=(a.endswith('/collar') and arm(b)) or (b.endswith('/collar') and arm(a))
                self.active_grasp |= bool(grasp)
                if grasp:self.peak_grasp_force=max(self.peak_grasp_force,force)
                bad=bool(self.prohibited(a,b))
                self.contact_pairs.add(key)
                event=dict(step=self.step,physics_sample=self.samples,a=a,b=b,force_n=force,
                           separation_m=separation,prohibited=bad)
                self.events.append(event)
                if bad:self.violations.append(event)
        def step(cls):
            original()
            try: sample()
            except Exception as exc:
                self.error=repr(exc);self.ready=False;raise
        PhysxManager.step=classmethod(step)
        self.ready=True
