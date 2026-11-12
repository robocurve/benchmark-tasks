"""Private all-substep contact diagnostics, never supplied to the probe policy."""
import numpy as np
from . import layout as L

def arm(path):
    return next((s for s in ('LeftArm','RightArm') if '/'+s+'/' in path),None)

class Monitor:
    def __init__(self):
        self.ready=False; self.error=None; self.samples=0; self.events=[]; self.step=0
        self.pairs={}
        self.interval_samples=0;self.interval_robot_sleeve=False;self.interval_supported=True

    def consume_interval(self):
        result=dict(valid=self.ready and self.error is None, samples=self.interval_samples,
                    robot_sleeve_contact=self.interval_robot_sleeve,
                    supported_all_substeps=self.interval_supported)
        self.interval_samples=0;self.interval_robot_sleeve=False;self.interval_supported=True
        return result

    def bind(self,stage):
        from pxr import Usd,UsdPhysics
        from isaaclab.sim.utils.stage import get_current_stage_id
        from isaaclab_physx.physics.physx_manager import PhysxManager
        import omni.physics.tensors as tensors
        root='/World/envs/env_0/'
        sensors=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root[:-1]))
                 if arm(str(p.GetPath())) and 'PhysxContactReportAPI' in p.GetAppliedSchemas()]
        names=('base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right')
        assert len(sensors)==18 and {(arm(p),p.rsplit('/',1)[-1]) for p in sensors}=={
            (s,n) for s in ('LeftArm','RightArm') for n in names}
        sensors += [root+n for n in L.MOVING]
        fixed=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(root[:-1]))
               if not arm(str(p.GetPath())) and not any('/'+n+'/' in str(p.GetPath()) or
               str(p.GetPath())==root+n for n in L.MOVING) and p.HasAPI(UsdPhysics.CollisionAPI)]
        assert any('/table/' in p or p.endswith('/table') for p in fixed),fixed
        self.sim=tensors.create_simulation_view('torch',stage_id=get_current_stage_id())
        self.sim.set_subspace_roots('/')
        self.view=self.sim.create_rigid_contact_view(sensors,[fixed+sensors]*len(sensors),max_contact_data_count=4096)
        view=self.view; sp=list(view.sensor_paths); fp=list(view.filter_paths)
        assert view.sensor_count==len(sensors)
        flat=[p for row in fp for p in row] if isinstance(fp[0],(list,tuple)) else fp
        assert set(flat)==set(fixed+sensors)
        self.contract=dict(sensors=sp,filters=fp,physics_hz=240,capacity=view.max_contact_data_count,
                           diagnostic_only=True)
        original=PhysxManager.step
        def sample():
            self.samples+=1
            self.interval_samples+=1;supported=False
            forces,points,normals,sep,counts,starts=[v.detach().cpu().numpy() for v in view.get_contact_data(dt=1/240)]
            if int(counts.sum())>=view.max_contact_data_count:raise RuntimeError('Contact capacity exhausted')
            seen=set()
            for i,j in np.argwhere(counts>0):
                a,b=sp[i],fp[i][j] if isinstance(fp[i],(list,tuple)) else fp[j]
                key=tuple(sorted((a,b)))
                if a==b or key in seen:continue
                seen.add(key)
                start,n=int(starts[i,j]),int(counts[i,j]);sl=slice(start,start+n)
                if start<0 or n<1 or start+n>len(forces):raise RuntimeError('Invalid contact range')
                if not all(np.isfinite(v[sl]).all() for v in (forces,points,sep)):raise RuntimeError('Nonfinite contact')
                force=float(np.abs(forces[sl]).sum())
                sleeve=root+'sleeve'
                if sleeve in key:
                    other=b if a==sleeve else a
                    if arm(other):self.interval_robot_sleeve=True
                    if other.startswith(root+'housing/seat_'):supported=True
                if force<=1e-6:continue
                summary=self.pairs.setdefault('|'.join(key),dict(samples=0,peak_n=0.))
                summary['samples']+=1;summary['peak_n']=max(summary['peak_n'],force)
                # Static coupon/table support is summarized; every robot contact
                # is retained, including same-arm contacts, without a safety verdict.
                if arm(a) or arm(b):
                    self.events.append(dict(step=self.step,sample=self.samples,a=a,b=b,force_n=force,
                                            separation_m=float(sep[sl].min())))
            self.interval_supported=self.interval_supported and supported
        def step(cls):
            original()
            try:sample()
            except Exception as exc:self.error=repr(exc);self.ready=False;raise
        PhysxManager.step=classmethod(step);self.ready=True

    def report(self):
        return dict(ready=self.ready,error=self.error,samples=self.samples,robot_contacts=self.events,pairs=self.pairs)
