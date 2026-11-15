"""Read-only native reset observations; never modifies a body or advances physics."""
from pathlib import Path
import json
import numpy as np


def array(value):
    value=getattr(value,'torch',value)
    if hasattr(value,'detach'):return value.detach().cpu().numpy().copy()
    return np.asarray(value.numpy() if hasattr(value,'numpy') else value).copy()


class ResetDiagnostic:
    def __init__(self,emb,out):
        self.emb,self.out=emb,Path(out)
        self.samples=[]
        self.frames=[]
        self.metadata={}

    def __call__(self,label,elapsed):
        import arena_cloth
        base=self.emb._base
        spec=arena_cloth.SPECS['tshirt']
        view=base.scene['tshirt'].root_view
        row=dict(label=label,time_s=float(elapsed),physics_step_count=int(base.sim._physics_step_count))
        try:
            points=array(view.get_simulation_nodal_positions())[0,:len(spec.mesh.rest)].reshape(-1,3)
            velocity=array(view.get_simulation_nodal_velocities())[0,:len(spec.mesh.rest)].reshape(-1,3)
            self.frames.append(points)
            row.update(frame=len(self.frames)-1,max_speed_m_s=float(np.linalg.norm(velocity,axis=1).max()),
                       max_from_authored_initial_m=float(np.linalg.norm(points-spec.initial,axis=1).max()))
            if 'topology' not in self.metadata:
                faces=array(view.get_simulation_element_indices())[0,:len(spec.mesh.faces)].reshape(-1,3)
                rest_faces=array(view.get_rest_element_indices())[0,:len(spec.mesh.faces)].reshape(-1,3)
                rest=array(view.get_rest_nodal_positions())[0,:len(spec.mesh.rest)].reshape(-1,3)
                np.savez_compressed(self.out/'native-topology.npz',simulation_faces=faces,rest_faces=rest_faces,
                    rest_positions=rest,authored_faces=spec.mesh.faces,authored_rest=spec.mesh.rest,
                    authored_initial=spec.initial)
                def canonical(value):
                    value=np.sort(value,axis=1)
                    return value[np.lexsort(value.T[::-1])]
                self.metadata['topology']=dict(simulation_faces_equal=bool(np.array_equal(canonical(faces),canonical(spec.mesh.faces))),
                    rest_faces_equal=bool(np.array_equal(canonical(rest_faces),canonical(spec.mesh.faces))),
                    max_rest_node_difference_m=float(np.linalg.norm(rest-spec.mesh.rest,axis=1).max()))
        except Exception as exc:
            row['read_error']=str(exc)
        self.samples.append(row)
        if self.frames:
            np.savez_compressed(self.out/'reset-native-nodes.npz',positions=np.asarray(self.frames),
                faces=spec.mesh.faces,rest=spec.mesh.rest,
                times_s=np.asarray([s['time_s'] for s in self.samples if 'frame' in s]))
        self.metadata['samples']=self.samples
        (self.out/'reset-native-observations.json').write_text(json.dumps(self.metadata,indent=2)+'\n')
