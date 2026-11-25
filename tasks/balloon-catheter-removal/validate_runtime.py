"""Bounded passive integration/reset check. No demo/controller/model or recordings."""
import argparse
import json
from pathlib import Path
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True)
AppLauncher.add_app_launcher_args(p)
a=p.parse_args();a.enable_cameras=False
app=AppLauncher(a).app
status={'complete':False,'scope':'passive runtime and reset only','paid_calls':0,'robot_completion_claimed':False}
try:
    import sys
    sys.path.insert(0,str(Path(__file__).parent))
    import numpy as np
    import torch
    import task
    from yam_arena import station,ik
    from yam_arena.constants import READY
    from yam_arena.embodiment import BimanualYamEmbodiment
    from yam_arena.scene import object_states
    emb=BimanualYamEmbodiment(enable_cameras=False,contact_sensors=False,rig=station.rig(),overview_cam=False)
    env=task.make_environment(emb,a.device)
    dynamics=task.make_dynamics(env)
    action=torch.tensor([[*READY,1,*READY,1]],dtype=torch.float32,device=env.unwrapped.device)
    resets=[];reset_errors=[]
    for episode in range(2):
        env.reset(seed=0);dynamics.reset();scorer=task.make_scorer()
        _,initial_poses=task.read_object_states(env)
        from scoring import INITIAL
        from layout import VALVE_QUATERNION
        initial_error=max(float(np.linalg.norm(initial_poses[n][:3]-INITIAL[n])) for n in task.OBJECT_NAMES)
        expected={n:np.array(VALVE_QUATERNION if n=='valve'else[0,0,0,1])for n in task.OBJECT_NAMES}
        quaternion_error=max(float(min(np.linalg.norm(initial_poses[n][3:]-expected[n]),
                                         np.linalg.norm(initial_poses[n][3:]+expected[n]))) for n in task.OBJECT_NAMES)
        reset_errors.append({'position_m':initial_error,'quaternion_sign_invariant':quaternion_error})
        if initial_error>1e-5 or quaternion_error>1e-5:raise RuntimeError('Reset did not restore declared initial state')
        for step in range(-30,0):dynamics.before_step(step,action[0].cpu().numpy());env.step(action)
        rows=[]
        for step in range(1,41):
            dynamics.before_step(step,action[0].cpu().numpy());env.step(action)
            objects,poses=task.read_object_states(env)
            robot={}
            for side in ('left','right'):
                r=env.unwrapped.scene[side+'_robot'];data=r.data.joint_pos
                data=data.torch if hasattr(data,'torch')else data;q=data[0].cpu().numpy();names=r.joint_names
                joints=np.array([q[names.index('joint'+str(i))] for i in range(1,7)])
                tcp,_=ik.tcp_pose(joints,station.BASES[side])
                robot[side]={'joints':joints.tolist(),'fingers':[float(q[names.index('joint'+str(i))]) for i in (7,8)],'tcp':tcp.tolist()}
            m=dynamics.after_step(step,objects,poses,robot)
            result=scorer.update(step=step,objects=objects,root_poses=poses,robot=robot,mechanics=m)
            if result['score']!=0 or result.get('failure'):raise RuntimeError('Passive state invalid or rewarded: '+str(result))
            rows.append({'step':step,'poses':{k:v.tolist() for k,v in poses.items()},'mechanics':m,'metrics':result})
        resets.append(rows)
    # Reset reproduces the initial configuration, not the exact trajectory of
    # freely moving joints. Equivalent q and -q must not count as pose drift.
    settled_position_variation=max(float(np.linalg.norm(np.array(resets[0][-1]['poses'][n])[:3]-
                                      np.array(resets[1][-1]['poses'][n])[:3])) for n in task.OBJECT_NAMES)
    status.update(complete=True,passive_score_zero=True,reset_errors=reset_errors,
                  settled_position_variation_m=settled_position_variation,episodes=resets)
    env.close()
except Exception as exc:
    import traceback
    traceback.print_exc();status['error']=repr(exc)
    if 'resets' in locals():status['episodes']=resets
    if 'reset_errors' in locals():status['reset_errors']=reset_errors
finally:
    a.output_dir.mkdir(parents=True,exist_ok=True)
    (a.output_dir/'runtime-validation.json').write_text(json.dumps(status,indent=2)+'\n')
    app.close()
