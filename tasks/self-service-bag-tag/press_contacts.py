"""GPU-native PhysX contact samples between the two tag end bodies only."""
import numpy as np
_VIEW = None

def reset():
    global _VIEW
    _VIEW=None

def array(value):
    if hasattr(value,'detach'):return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value,'numpy') else value)

def read():
    global _VIEW
    if _VIEW is None:
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics
        paths={}
        for p in get_current_stage().Traverse():
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                for i in (0,9):
                    if f'/tag_panel_{i:02d}/body' in str(p.GetPath()):paths[i]=str(p.GetPath())
        if set(paths)!={0,9}:raise RuntimeError('Missing tag-end contact bodies')
        _VIEW=PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            [paths[0]],filter_patterns=[[paths[9]]],max_contact_data_count=128)
    # dt=1 returns impulses; caller divides by the actual physics dt.
    force,point,normal,separation,count,start=map(array,_VIEW.get_contact_data(dt=1.))
    contacts=[]
    for n,s in zip(count.reshape(-1),start.reshape(-1)):
        for i in range(int(s),int(s+n)):
            if not np.isfinite(force[i]).all():raise RuntimeError('Nonfinite patch contact')
            contacts.append((point[i],normal[i],normal[i]*float(force[i].reshape(-1)[0])))
    return contacts
