"""Diagnostic-only state snapshots. Resumed rollouts are never full-demo evidence."""
import json
from pathlib import Path
import numpy as np

def save(path,env,commands):
    import partkit
    scene=env.unwrapped.scene
    data={'diagnostic_only':True,'commands':commands,'assets':{}}
    for kind in ('articulations','rigid_objects'):
        for name,a in getattr(scene,kind).items():
            row={'root':partkit._np(a.data.root_state_w).tolist()}
            if kind=='articulations':row.update(q=partkit._np(a.data.joint_pos).tolist(),qd=partkit._np(a.data.joint_vel).tolist())
            data['assets'][name]=row
    Path(path).write_text(json.dumps(data,indent=2))

def restore(path,env):
    import torch
    data=json.loads(Path(path).read_text());scene=env.unwrapped.scene
    for name,row in data['assets'].items():
        a=scene[name]
        a.write_root_state_to_sim(torch.tensor(row['root'],device=a.device))
        if 'q' in row:
            q=torch.tensor(row['q'],device=a.device);qd=torch.tensor(row['qd'],device=a.device)
            a.write_joint_state_to_sim(q,qd)
            if name.endswith('_robot'):a.set_joint_position_target(q)
    scene.write_data_to_sim();env.unwrapped.sim.forward();scene.update(0.)
    return data['commands']
