"""Lift the freely suspended bottom knot with two real left fingers and let it fall back."""
import argparse
import json
from pathlib import Path

import numpy as np
from inspect_robots import Action, Scene
from yam_arena import ik
from .arena import make_embodiment, array
from .fixed_scene import INSTRUCTION

PAD=np.array([0.,-.004,.008])
BASE=np.array([0.,.305,.75])

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--condition',choices=['bottom_knot'],required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    rig=make_embodiment(args.out)
    poses=json.loads(Path(__file__).with_name('knot-grip.json').read_text())
    r=np.array(poses['rot']);offset=np.array(poses['knot_in_pad'])
    target_q=np.array(poses['seed']);trace=[]
    try:
        obs=rig.reset(Scene(id='bottom-knot-contact',instruction=INSTRUCTION),seed=0)
        cmd=np.array(obs.state['joint_pos']).copy()
        from yam_arena.robot import asset_link_paths
        links=asset_link_paths()
        knot='/World/envs/env_0/training_fixture/parts/bottom_knot'
        loads=[rig.physics.create_rigid_contact_view(['/World/envs/env_0/LeftArm'+links[tip]],[[knot]])
               for tip in ('tip_left','tip_right')]
        knot_at=lambda:rig.poses['bottom_knot'][0].copy()
        pre=knot_at()-r@offset+[0,.034,.019]
        plan=[('prepare above and behind',90,.3),('move open pads around knot',65,.30),
              ('close',30,0.),('raise knot',70,0.),('hold',35,0.),('open where held',35,.45),
              ('move clear',55,.60)]
        for name,n,jaw in plan:
            q=np.array(obs.state['joint_pos']);tcp,actual=ik.tcp_pose(q[:6],BASE)
            start=tcp+actual@PAD;jaw0=cmd[6];q0=q[:6].copy()
            if name=='prepare above and behind':end=pre
            elif name=='move open pads around knot':end=knot_at()-r@offset
            elif name=='raise knot':
                knot0=knot_at();pad0=start.copy();end=start+[.010,0,.018]
            elif name=='move clear':end=start+[0,.055,.025]
            else:end=start
            unit=ik.matrix_to_quat_xyzw(r)
            approach,_,_=ik.solve(end-r@PAD,unit,target_q,BASE,iters=100,rot_weight=.2)
            for i in range(1,n+1):
                t=ik.smoothstep(i/n)
                if name=='prepare above and behind': goal=q0+t*(approach-q0)
                elif name in ('close','open where held'): goal=q0
                else:
                    dest=start+t*(end-start)
                    current=np.asarray(obs.state['joint_pos'])[:6]
                    goal,_,_=ik.solve(dest-r@PAD,unit,current,BASE,iters=60,rot_weight=.2)
                cmd[:6]+=np.clip(goal-cmd[:6],-.085,.085)
                cmd[6]=jaw0+t*(jaw-jaw0)
                obs=rig.step(Action(data=cmd.copy(),meta={})).observation
                q=np.asarray(obs.state['joint_pos'])
                tcp,ar=ik.tcp_pose(q[:6],BASE);pad=tcp+ar@PAD
                force=[float(np.linalg.norm(array(v.get_contact_force_matrix(dt=float(rig._base.sim.get_physics_dt()))).reshape(-1,3),axis=1).max()) for v in loads]
                row=dict(tick=rig.tick,phase=name,knot_m=knot_at().tolist(),pad_m=pad.tolist(),normal_n=force)
                if name in ('raise knot','hold','open where held','move clear'):
                    row.update(rise_mm=float(1000*(knot_at()-knot0)[2]),slip_mm=float(1000*np.linalg.norm((knot_at()-knot0)-(pad-pad0))))
                trace.append(row)
        hold=[v for v in trace if v['phase']=='hold'][-20:]
        passed=all(v['rise_mm']>12 and v['slip_mm']<6 and min(v['normal_n'])>.05 for v in hold)
        (args.out/'CONTROL.json').write_text(json.dumps(dict(condition='bottom_knot',passed=passed,full_reference=False,trace=trace)))
    finally:rig.close()

if __name__=='__main__':main()
