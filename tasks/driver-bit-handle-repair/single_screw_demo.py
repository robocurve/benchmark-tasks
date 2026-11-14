"""YAM-only pick, hex insert, release and lift probe. No object pose writes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import imageio.v2 as imageio

# Reachable left-wrist orientation that points the flipped tool's bit down.
# Left-side approach selected offline from the measured tool-in-gripper transform;
# its physical validation is recorded separately from the earlier frontal demo.
TOOL_DOWN_R=np.array([[0.8707521687219243, 0.08070189984349643, 0.485054495935989], [0.49107990302090704, -0.19312205542810912, -0.8494376966889328], [0.025123485328323186, 0.9778502314101444, -0.2077925297408089]])
TOOL_CLEAR_TCP=np.array([0.38098805992753104, 0.2850635676018516, 1.120466219374101])
PICKUP_WRIST_ROLL_RAD=np.radians(-19.)
if TOOL_DOWN_R.shape!=(3,3) or not np.allclose(TOOL_DOWN_R.T@TOOL_DOWN_R,np.eye(3)) or not np.isclose(np.linalg.det(TOOL_DOWN_R),1.):
    raise ValueError('Tool-down orientation must be a proper 3x3 rotation')

def run(out,lift_only=False,preview_only=False,assembly_only=False,hover_only=False):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena import ik,station
    from yam_arena.constants import READY, ARM_JOINTS, FINGER_JOINTS
    from yam_arena.demo import grip_rotation
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import yam_arena.inspect_embodiment as ie
    from isaaclab.app import AppLauncher
    if ie._APP is None:
        ie._APP=AppLauncher(headless=True,device='cuda:0',enable_cameras=True).app
        __import__('atexit').register(ie._close_app_at_exit)
    from yam_arena.embodiment import BimanualYamEmbodiment
    previous_init=BimanualYamEmbodiment.__init__
    def instrumented_init(self,*args,**kwargs):
        kwargs['contact_sensors']=True
        previous_init(self,*args,**kwargs)
    BimanualYamEmbodiment.__init__=instrumented_init
    out.mkdir(parents=True,exist_ok=False)
    emb=yam_arena_embodiment(task=str(Path(__file__).with_name('single_screw_task.py').resolve()),object_jitter=0,control_hz=20,cam_width=640,cam_height=352,top_cam='demo')
    # Same official station camera poses as the model run; only output resolution differs.
    from scipy.spatial.transform import Rotation
    obs=emb.reset(Scene(id='single_screw_repair',instruction='Inspect the loose drawer-knob screw. Fit the matching bit into the stopped powered screwdriver, tighten the screw with that tool, then release the stopped tool in its stand.'))
    from yam_arena.scene import object_states
    task=emb.task.module
    imageio.imwrite(out/'scene_overview.png',np.asarray(obs.images['top_cam']))
    for cam,img in obs.images.items(): imageio.imwrite(out/f'initial_{cam}.png',np.asarray(img))
    if preview_only:
        (out/'result.json').write_text(json.dumps({'scope':'scene preview only','physics_pass':None})+'\n')
        return
    q={s:np.array(READY,float) for s in ('left','right')}
    pos,rot,grip={},{},{s:1. for s in q}
    for s in q: pos[s],rot[s]=ik.tcp_pose(q[s],np.array(station.BASES[s]))
    rows=[]; frames=[]; actions=[]; step=0; seated=False; tool_grip=False; operating_grip=False
    from pxr import PhysxSchema, UsdPhysics, PhysicsSchemaTools
    from isaaclab.sim.utils.stage import get_current_stage
    from omni.physx import get_physx_simulation_interface
    contact_totals={}
    def report(headers,data):
        for h in headers:
            if not h.num_contact_data:continue
            a=str(PhysicsSchemaTools.intToSdfPath(h.collider0));b=str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            key=a+' | '+b
            entry=contact_totals.setdefault(key,{'impulse_Ns':0.,'max_impulse_Ns':0.,'count':0})
            for ix in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                mag=float(np.linalg.norm(data[ix].impulse));entry['impulse_Ns']+=mag;entry['max_impulse_Ns']=max(entry['max_impulse_Ns'],mag);entry['count']+=1
    subscription=get_physx_simulation_interface().subscribe_contact_report_events(report)
    def move(phase,side,target,opening,n,orientation=None,joint_space=False,q_target=None):
        nonlocal obs,step,seated
        if side=='left' and opening==0:
            opening=(float(os.environ.get('REPAIR_OPERATING_GRIP','.02')) if operating_grip
                     else .10 if tool_grip else .38)
        a=pos[side].copy(); r=rot[side].copy(); g=grip[side]
        rr=grip_rotation(0) if orientation is None else orientation
        qstart=q[side].copy()
        if joint_space:
            if q_target is None:
                candidates=[ik.solve(target,ik.matrix_to_quat_xyzw(rr),q_init=seed,base_pos=np.array(station.BASES[side]),iters=200) for seed in ([qstart,np.array(READY),np.array(selected['waypoints'][0]['q'])] if phase=='turn_bit' else [qstart,np.array(READY)])]
                qend,ee,re=min(candidates,key=lambda x:x[1]+x[2])
                if ee>.002 or re>np.radians(2):raise RuntimeError('Joint transit endpoint invalid')
            else:
                qend=np.asarray(q_target,float)
        for i in range(1,n+1):
            u=ik.smoothstep(i/n)
            pp=a+u*(np.array(target)-a)
            rnow=ik.slerp_matrix(r,rr,u)
            if joint_space:
                q[side]=qstart+u*(qend-qstart);err=0.;rot_err=0.
                pp,rnow=ik.tcp_pose(q[side],np.array(station.BASES[side]))
            else:
                q[side],err,rot_err=ik.solve(pp,ik.matrix_to_quat_xyzw(rnow),q_init=q[side],base_pos=np.array(station.BASES[side]),iters=80)
            if err>.005 or rot_err>np.radians(3):
                (out/'abort.json').write_text(json.dumps({'phase':phase,'step':step,'position_error_m':err,'angle_error_deg':float(np.degrees(rot_err)),'mount_pass':False}))
                if frames:imageio.mimsave(out/'aborted.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
                raise RuntimeError(f'Unreachable path at {phase}: {err:.4f} m, {np.degrees(rot_err):.2f} deg')
            if step>=6000:raise RuntimeError('6000-step task limit exceeded')
            grip[side]=g+u*(opening-g)
            command=pack(np.r_[q['left'],grip['left']],np.r_[q['right'],grip['right']])
            actions.append(np.asarray(command,dtype=np.float32).copy())
            obs=emb.step(Action(data=command)).observation
            step+=1
            if phase in ('direct_120','direct_90'):
                crossing=object_states(emb._env,['bit','drawer_context'])
                bit_tip=crossing['bit'].pos+crossing['bit'].rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])
                screw=crossing['drawer_context'].pos+crossing['drawer_context'].rotation()@np.array([task.FASTENER_X,-.06,.099])
                clearance=float(bit_tip[2]-screw[2])
                if clearance<.060:
                    (out/'abort.json').write_text(json.dumps({'phase':phase,'step':step,'bit_tip_clearance_m':clearance,
                                                             'minimum_clearance_m':.060,'mount_pass':False})+'\n')
                    raise RuntimeError(f'Unsafe drawer crossing clearance: {clearance:.4f} m')
            if step%3==0: frames.append(np.asarray(obs.images['top_cam']))
            if step%5==0 or i==n:
                st=object_states(emb._env,['bit','driver','drawer_context'])
                row={'step':step,'phase':phase,'joint_space_transit':joint_space,'ik_error':float(err),'ik_rotation_error_deg':float(np.degrees(rot_err)),
                     'commanded_tcp':{side:pp.tolist()},
                     'positions':{k:v.pos.tolist() for k,v in st.items()},
                     'quaternions':{k:v.quat_xyzw.tolist() for k,v in st.items()},
                     'bit_in_driver':st['driver'].rotation().T.dot(st['bit'].pos-st['driver'].pos).tolist()}
                row['measured_robot']={}
                for arm in ('left','right'):
                    robot=emb._env.unwrapped.scene[arm+'_robot']
                    actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                    measured_tcp,_=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],np.array(station.BASES[arm]))
                    row['measured_robot'][arm]={'tcp':measured_tcp.tolist(),'finger_joints':actual[[robot.joint_names.index(j) for j in FINGER_JOINTS]].tolist(),'joint_names':robot.joint_names,'actual_q':actual.tolist(),'command_q':q[arm].tolist()}
                    for attr in ['applied_torque','computed_torque','joint_vel']:
                        try:row['measured_robot'][arm][attr]=getattr(robot.data,attr).torch[0].detach().cpu().numpy().tolist()
                        except (AttributeError,TypeError):pass
                row['contact_net']={}
                for arm in ('left','right'):
                    sensor=emb._env.unwrapped.scene[arm+'_contacts']
                    forces=sensor.data.net_forces_w
                    if hasattr(forces,'torch'):forces=forces.torch
                    row['contact_net'][arm]={'bodies':sensor.body_names,'force_N':forces[0].detach().cpu().numpy().tolist()}
                driver=emb._env.unwrapped.scene['driver']
                row['mechanism']=dict(task.MECHANISM)
                row['jaw_positions_m']=driver.data.joint_pos.torch[0].detach().cpu().numpy().tolist()
                row['contacts_since_last_sample']=dict(contact_totals)
                contact_totals.clear()
                rows.append(row)
                with (out/'trace.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
            if phase.startswith('insert_'):
                contact=object_states(emb._env,['bit','driver'])
                rel=contact['driver'].rotation().T@(contact['bit'].pos-contact['driver'].pos)
                rel[2]+=task.mechanism_state(emb._env)['pressure_m']
                if .1005<rel[2]<.1030 and np.linalg.norm(rel[:2])<.001:
                    robot=emb._env.unwrapped.scene[side+'_robot'];actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                    target,rr=ik.tcp_pose(actual[[robot.joint_names.index(n) for n in ARM_JOINTS]],np.array(station.BASES[side]))
                    seated=True
                    break
        pos[side]=np.array(target); rot[side]=rr
        if not phase.startswith('insert_') or phase.endswith('_5'):
            for cam,img in obs.images.items(): imageio.imwrite(out/f'{phase}_{cam}.png',np.asarray(img))
            imageio.imwrite(out/f'{phase}_overview.png',np.asarray(obs.images['top_cam']))
    def clear_right_for_pickup():
        # Clear the operating grip corridor without returning to ready.
        rr=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        move('right_tool_clear','right',np.array([.25,-.25,1.05]),1,50,rr,joint_space=True)
    def brace_workpiece():
        rr=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        drawer=object_states(emb._env,['drawer_context'])['drawer_context']
        # Pinch the near wooden wall (14 mm thick), rather than closing in
        # front of it. The contact target follows the measured drawer pose.
        near_wall=drawer.pos+drawer.rotation()@np.array([.035,-.157,.135])
        move('brace_approach','right',np.array([.295+task.WORKPIECE_SHIFT_X,
             -.23+task.WORKPIECE_SHIFT_Y-.045,task.TOP+.20]),1,45,rr,joint_space=True)
        brace_y=near_wall[1]+float(os.environ.get('REPAIR_BRACE_Y_BIAS','0'))
        # Cross above the lip, then descend with both jaws open so neither
        # fingertip pushes the near wall away before the wood is between them.
        move('brace_above','right',np.array([.395+task.WORKPIECE_SHIFT_X,brace_y,task.TOP+.25]),1,55,rr)
        move('brace_contact','right',np.array([.395+task.WORKPIECE_SHIFT_X,brace_y,task.TOP+.135]),1,55,rr)
        move('brace_close','right',pos['right'],0,25,rr)
        move('brace_hold','right',pos['right'],0,10,rr)
        sensor=emb._env.unwrapped.scene['right_contacts']
        forces=sensor.data.net_forces_w
        if hasattr(forces,'torch'):forces=forces.torch
        tip_forces=[float(np.linalg.norm(forces[0,sensor.body_names.index(name)].detach().cpu().numpy()))
                    for name in ('tip_left','tip_right')]
        robot=emb._env.unwrapped.scene['right_robot']
        body_pos=robot.data.body_pos_w
        if hasattr(body_pos,'torch'):body_pos=body_pos.torch
        fingertip_centers={name:body_pos[0,robot.body_names.index(name)].detach().cpu().numpy().tolist()
                           for name in ('tip_left','tip_right')}
        (out/'brace_contact_check.json').write_text(json.dumps({'near_wall_m':near_wall.tolist(),
            'tcp_target_m':pos['right'].tolist(),'tip_forces_N':tip_forces,
            'fingertip_body_centers_m':fingertip_centers},indent=2)+'\n')
        if min(tip_forces)<.2:
            raise RuntimeError(f'Right jaw did not pinch wooden wall: tip forces {tip_forces} N')
    def return_tool():
        rr=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        move('disengage_lower','left',pos['left']+[0,0,.040],0,50,rot['left'])
        box_before=object_states(emb._env,['drawer_context'])['drawer_context'].pos.copy()
        # Open while withdrawing from the pinched wall, then go around its
        # near side at a safe height before returning to the ready posture.
        move('brace_release','right',pos['right']+[-.08,0,.04],1,50,rr)
        move('right_near_clear','right',np.array([.25,-.18,task.TOP+.27]),1,55,rr,joint_space=True)
        ready_p,ready_R=ik.tcp_pose(np.array(READY),np.array(station.BASES['right']))
        move('right_clear','right',ready_p,1,60,ready_R,joint_space=True)
        box_after=object_states(emb._env,['drawer_context'])['drawer_context'].pos.copy()
        (out/'brace_release_check.json').write_text(json.dumps({'box_before_m':box_before.tolist(),
            'box_after_m':box_after.tolist(),'box_displacement_m':float(np.linalg.norm(box_after-box_before))},indent=2)+'\n')
        plan=json.loads(Path(__file__).with_name('horizontal_plan.json').read_text())['candidates'][1]
        start=np.array(plan['start'])+np.array([task.TOOL_SHIFT_X,0,0]);start[1]=-.045;Ri=np.array(plan['initial_R'])
        # The workpiece shift made the former Cartesian return target unreachable.
        # Clear the screw first, then rotate toward the stand in joint space.
        move('tool_clear','left',TOOL_CLEAR_TCP.copy(),0,80,TOOL_DOWN_R,joint_space=True)
        move('tool_horizontal','left',start+[0,0,.10],0,80,Ri,joint_space=True)
        # Align the held *body* while clear of the stand, then lower it at
        # fixed x/y and orientation. Contact-level corrections caused grip slip.
        home=np.array([.225,-.015,task.TOP+.067])
        from scipy.optimize import least_squares
        dock_bias=np.zeros(6)
        def dock_at(label,body_target,limit=10):
            nonlocal dock_bias
            for j in range(limit):
                st=object_states(emb._env,['driver'])['driver']
                rob=emb._env.unwrapped.scene['left_robot']
                aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
                measured_q=aq[[rob.joint_names.index(n) for n in ARM_JOINTS]]
                tcp,Ractual=ik.tcp_pose(measured_q,station.BASES['left'])
                body_offset=Ractual.T@(st.pos-tcp)
                body_rotation_in_tcp=Ractual.T@st.rotation()
                home_rotation=Rotation.from_quat([0.,-2**-.5,2**-.5,0.]).as_matrix()
                def metrics(joints):
                    planned_tcp,planned_R=ik.tcp_pose(joints,station.BASES['left'])
                    body_error=planned_tcp+planned_R@body_offset-body_target
                    rotation_error=Rotation.from_matrix(
                        (planned_R@body_rotation_in_tcp)@home_rotation.T).as_rotvec()
                    return body_error,rotation_error
                def residual(joints):
                    p_error,rotation_error=metrics(joints)
                    return np.r_[p_error,.1*rotation_error]
                fit=least_squares(residual,np.clip(measured_q,ik.LOWER+1e-6,ik.UPPER-1e-6),
                                  bounds=(ik.LOWER,ik.UPPER),max_nfev=150)
                qgoal=fit.x
                planned_error,planned_rotation=metrics(qgoal)
                if np.linalg.norm(planned_error)>.020 or np.linalg.norm(planned_rotation)>np.radians(16.):
                    raise RuntimeError(f'Dock body target unreachable: {np.linalg.norm(planned_error):.4f} m, '
                                       f'{np.degrees(np.linalg.norm(planned_rotation)):.1f} deg')
                target_p,target_R=ik.tcp_pose(qgoal,station.BASES['left'])
                if j:dock_bias=np.clip(dock_bias+.5*(qgoal-measured_q),-.20,.20)
                move(f'{label}_{j}','left',target_p,0,20,target_R,joint_space=True,
                     q_target=np.clip(qgoal+dock_bias,ik.LOWER,ik.UPPER))
                body=object_states(emb._env,['driver'])['driver']
                p_err=np.linalg.norm(body.pos-body_target)
                a_err=Rotation.from_matrix(body.rotation()@home_rotation.T).magnitude()
                if p_err<.018 and a_err<np.radians(7 if body_target[2]<=home[2]+.001 else 13):break
            return p_err,a_err
        for h in (.10,.05,.02,0.):
            err,ang=dock_at(f'dock_{int(h*1000)}',home+[0,0,h])
            if h>=.02 and (err>.025 or ang>np.radians(15)):
                raise RuntimeError(f'High-clearance docking failed at {h:.3f} m: {err:.4f} m, {np.degrees(ang):.1f} deg')
        if err>.025 or ang>np.radians(8):
            raise RuntimeError(f'Physical tool placement missed stand: {err:.4f} m, {np.degrees(ang):.1f} deg')
        move('tool_release','left',pos['left'],1,30,rot['left'])
        # Retreat from the measured release pose with a shoulder lift. A
        # fixed-orientation Cartesian lift is outside this wrist's IK range,
        # and a horizontal sweep can wedge the open jaws against the stand.
        rob=emb._env.unwrapped.scene['left_robot']
        aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
        released_q=aq[[rob.joint_names.index(j) for j in ARM_JOINTS]]
        qclear=released_q.copy();qclear[1]-=.30
        clear_p,clear_R=ik.tcp_pose(qclear,np.array(station.BASES['left']))
        move('left_shoulder_clear','left',clear_p,1,90,clear_R,joint_space=True,q_target=qclear)
        ready_p,ready_R=ik.tcp_pose(np.array(READY),np.array(station.BASES['left']))
        move('left_ready','left',ready_p,1,70,ready_R,joint_space=True,q_target=np.array(READY))
        move('final_hold','left',ready_p,1,80,ready_R,joint_space=True,q_target=np.array(READY))
    import os
    if os.environ.get('REPAIR_RESET_ONLY')=='1':
        move('reset_settle','left',pos['left'],1,50,rot['left'])
        body=object_states(emb._env,['driver'])['driver']
        expected_R=Rotation.from_quat([0.,-2**-.5,2**-.5,0.]).as_matrix()
        angle=float(np.degrees(np.linalg.norm(Rotation.from_matrix(body.rotation()@expected_R.T).as_rotvec())))
        result={'scope':'Unforced reset settling only','tool_angle_error_deg':angle,
                'tool_position':body.pos.tolist(),'passed':bool(angle<2.),'steps':step}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        if not result['passed']:raise RuntimeError('Tool does not settle level in stand')
        return
    if os.environ.get('REPAIR_PICKUP_ONLY')=='1':
        plan=json.loads(Path(__file__).with_name('horizontal_plan.json').read_text())['candidates'][1]
        start=np.array(plan['start'])+np.array([task.TOOL_SHIFT_X,0,0]);start[1]=-.045
        Ri=np.array(plan['initial_R'])
        initial=object_states(emb._env,['driver'])['driver']
        tool_grip=True;operating_grip=True
        for name,target,opening,n in [('approach_tool',start+[0,0,.10],1,45),
                                      ('lower_tool',start,1,45),('grasp_tool',start,0,30),
                                      ('lift_tool',start+[0,0,.10],0,50)]:
            move(name,'left',target,opening,n,Ri)
            if name=='grasp_tool':
                robot=emb._env.unwrapped.scene['left_robot']
                actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
                grasp_R=tcp_R.T@object_states(emb._env,['driver'])['driver'].rotation()
        final=object_states(emb._env,['driver'])['driver']
        robot=emb._env.unwrapped.scene['left_robot']
        actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
        _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
        slip=float(np.degrees(np.linalg.norm(Rotation.from_matrix((tcp_R.T@final.rotation())@grasp_R.T).as_rotvec())))
        result={'scope':'Pickup-only physical diagnostic; no bit installation or tightening',
                'tool_lift_m':float(final.pos[2]-initial.pos[2]),'tool_rotation_in_grip_deg':slip,
                'passed':bool(final.pos[2]-initial.pos[2]>.06 and slip<12.),'steps':step}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        np.savez_compressed(out/'actions.npz',actions=np.asarray(actions,dtype=np.float32))
        imageio.mimsave(out/'pickup.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
        print(json.dumps(result),flush=True)
        if not result['passed']:raise RuntimeError('Pickup-only probe failed')
        return
    resume=os.environ.get('REPAIR_CHECKPOINT')
    at_assembly=bool(resume and Path(resume).name=='assembly_checkpoint.json')
    at_prehover=bool(resume and Path(resume).name=='prehover_checkpoint.json')
    at_hover=bool(resume and Path(resume).name=='hover_checkpoint.json')
    at_powered=bool(resume and Path(resume).name=='powered_checkpoint.json')
    if resume:
        from checkpoint import restore
        cmds=restore(resume,emb._env)
        for name,dest in [('q',q),('pos',pos),('rot',rot)]:
            dest.update({arm:np.array(v) for arm,v in cmds[name].items()})
        grip.update(cmds['grip'])
        if at_hover or at_prehover or at_assembly:
            insertion=json.loads(Path(resume).with_name('insertion_result.json').read_text())
            if insertion.get('mount_pass') is not True:
                raise RuntimeError('Diagnostic hover checkpoint lacks prior physical assembly evidence')
            task.MECHANISM['tool_assembled']=True
            (out/'insertion_result.json').write_text(json.dumps(insertion,indent=2)+'\n')
        if at_assembly:
            R=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
            bit_pick=np.array([.107,-.13,task.TOP+.070])
            move('withdraw_transit','right',pos['right']+np.array([0,0,.10]),1,60,rot['right'])
            clear_right_for_pickup()
            plan=json.loads(Path(__file__).with_name('horizontal_plan.json').read_text())['candidates'][1]
            initial=object_states(emb._env,['driver'])['driver']
            # Roll the pickup wrist around the observed tool axis. This changes
            # the bit-in-gripper transform without exceeding the YAM joint limits.
            Ri=Rotation.from_rotvec(initial.rotation()[:,2]*PICKUP_WRIST_ROLL_RAD).as_matrix()@np.array(plan['initial_R'])
            goal=np.array([.39,0.,.99])
            grip_local=np.array([0.,0.,.030])
            start=initial.pos+initial.rotation()@grip_local;start[2]-=.002
            tool_grip=True;operating_grip=True
            slips={}
            for name,target,opening,n,orientation in [
                ('approach_tool',start+[0,0,.04],1,45,Ri),
                ('lower_tool',start,1,45,Ri),('grasp_tool',start,0,30,Ri),
                ('lift_tool',start+[0,0,.04],0,50,Ri),
                ('turn_down',TOOL_CLEAR_TCP.copy(),0,100,TOOL_DOWN_R)]:
                move(name,'left',target,opening,n,orientation,joint_space=(name in ('approach_tool','turn_down')))
                if name in ('grasp_tool','lift_tool','turn_down'):
                    robot=emb._env.unwrapped.scene['left_robot']
                    actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                    _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
                    tool_R=object_states(emb._env,['driver'])['driver'].rotation()
                    if name=='grasp_tool':grasp_R=tcp_R.T@tool_R
                    else:slips[name]=float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                        (tcp_R.T@tool_R)@grasp_R.T).as_rotvec())))
            move('turn_settle','left',pos['left'],0,30,rot['left'],joint_space=True,q_target=q['left'])
            robot=emb._env.unwrapped.scene['left_robot']
            actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
            _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
            settled=object_states(emb._env,['driver','bit','drawer_context'])
            slips['turn_settle']=float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                (tcp_R.T@settled['driver'].rotation())@grasp_R.T).as_rotvec())))
            move('clearance_reposition','left',TOOL_CLEAR_TCP.copy(),0,60,rot['left'],joint_space=True)
            move('clearance_settle','left',pos['left'],0,30,rot['left'],joint_space=True,q_target=q['left'])
            robot=emb._env.unwrapped.scene['left_robot']
            actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
            _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
            settled=object_states(emb._env,['driver','bit','drawer_context'])
            slips['clearance_settle']=float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                (tcp_R.T@settled['driver'].rotation())@grasp_R.T).as_rotvec())))
            tip=settled['bit'].pos+settled['bit'].rotation()@np.array([0,0,task.BIT_TIP_LOCAL_Z])
            screw=settled['drawer_context'].pos+settled['drawer_context'].rotation()@np.array([task.FASTENER_X,-.06,.099])
            result={'scope':'Assembly-checkpoint pickup and turn diagnostic; not full-demo evidence',
                    'steps':step,'grip_opening':float(os.environ.get('REPAIR_OPERATING_GRIP','.02')),
                    'slip_deg':slips,'bit_vertical_clearance_m':float(tip[2]-screw[2]),
                    'passed':bool(max(slips.values())<12. and tip[2]-screw[2]>.06)}
            if result['passed']:
                from checkpoint import save
                save(out/'prehover_checkpoint.json',emb._env,
                     {'q':{k:v.tolist() for k,v in q.items()},
                      'pos':{k:v.tolist() for k,v in pos.items()},
                      'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
                (out/'insertion_result.json').write_text(json.dumps(insertion,indent=2)+'\n')
            np.savez_compressed(out/'actions.npz',actions=np.asarray(actions,dtype=np.float32))
            (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            imageio.mimsave(out/'pickup_turn.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
            print(json.dumps(result),flush=True)
            if not result['passed']:raise RuntimeError('Pickup-turn diagnostic failed')
            return
    else:
        move('idle_control','left',pos['left'],1,20,rot['left'])
        # Hold the rigid rear sleeve while the other hand approaches the chuck.
        # The side-by-side grippers stay separated by the tool's body length.
        rear_R=np.diag([-1.,1.,-1.])
        rear_p=np.array([.225,.035,task.TOP+.067])
        move('rear_approach','left',rear_p+[0,0,.08],1,45,rear_R,joint_space=True)
        move('rear_lower','left',rear_p,1,45,rear_R)
        tool_grip=True
        move('rear_grasp','left',rear_p,0,30,rear_R)
        braced_pos=object_states(emb._env,['driver'])['driver'].pos.copy()
        # Side-grip the upright bit, turn it to the horizontal socket, then insert.
        pick=json.loads(Path(__file__).with_name('bit_side_pick_ik.json').read_text())[0]
        R=np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]);target=np.array([.107,-.13,task.TOP+.070])
        selected=json.loads(Path(__file__).with_name('selected_insertion_plan.json').read_text())['candidates'][0]
        mount=json.loads(Path(__file__).with_name('horizontal_side_insertion_ik.json').read_text())['solutions'][0]
        Rm=np.array(mount['rotation']);pm=np.array(mount['position'])+np.array([task.TOOL_SHIFT_X,0,0])
        move('approach_bit_side','right',target-R[:,2]*.08,1,40,R)
        move('grasp_position','right',target,1,35,R)
        move('close_bit','right',target,0,25,R)
        move('lift_bit','right',target+[0,0,.10],0,40,R)
        initial=object_states(emb._env,['bit','driver'])
        bit_lift=float(initial['bit'].pos[2]-(task.TOP+.067))
        if bit_lift<.04:
            result={'scope':'Side bit grasp only; stopped before insertion','bit_lift_m':bit_lift,'mount_pass':False}
        else:
            opposite_side=Rotation.from_euler('y',240,degrees=True).as_matrix()@Rotation.from_euler('z',180,degrees=True).as_matrix()
            def mirrored_tcp(w):
                p=np.array(w['tcp']);p[1]=-.030-p[1]
                return p
            move('turn_bit','right',mirrored_tcp(selected['waypoints'][0]),0,80,opposite_side@np.array(selected['waypoints'][0]['R']),joint_space=True)
            if np.linalg.norm(object_states(emb._env,['driver'])['driver'].pos-braced_pos)>.008:
                raise RuntimeError('Tool shifted before bit reached the socket')
            move('near_socket','right',mirrored_tcp(selected['waypoints'][1])+np.array([0,-.017,0]),0,40,opposite_side@np.array(selected['waypoints'][1]['R']))
            seated=False
            def insertion_pass(attempt):
                nonlocal seated
                for k,h in enumerate([.160,.152,.145,.138,.132,.128,.124,.120,.116,.112,.108,.104,.101,.098,.094,.090,.086,.082,.078,.074,.070,.066,.062]):
                    for j in range(6):
                        st=object_states(emb._env,['bit','driver'])
                        rob=emb._env.unwrapped.scene['right_robot'];actual=rob.data.joint_pos.torch[0].detach().cpu().numpy()
                        tcp,Ractual=ik.tcp_pose(actual[[rob.joint_names.index(n) for n in ARM_JOINTS]],np.array(station.BASES['right']))
                        desired_bit_R=st['driver'].rotation()@Rotation.from_euler('z',120,degrees=True).as_matrix()
                        delta=desired_bit_R@st['bit'].rotation().T
                        rv=Rotation.from_matrix(delta).as_rotvec();angle=np.linalg.norm(rv)
                        # Bound rotation feedback; hold axial position until orientation converges.
                        if angle>np.radians(2):delta=Rotation.from_rotvec(rv*np.radians(2)/angle).as_matrix()
                        raw_rel=st['driver'].rotation().T@(st['bit'].pos-st['driver'].pos)
                        rel=raw_rel.copy()
                        rel[2]+=task.mechanism_state(emb._env)['pressure_m']
                        # Release only after the rigid bit reaches the same
                        # mounted range used by the scorer. The spring-loaded
                        # nose may still be compressed under the gripper; the
                        # subsequent hands-off hold verifies actual retention.
                        # The nose spring unloads by up to 4 mm when the
                        # gripper releases, so seat the rigid bit near the
                        # socket floor before testing hands-off retention.
                        if h<=.082 and .0995<raw_rel[2]<.1015 and np.linalg.norm(raw_rel[:2])<.001:
                            seated=True
                            # Unload elastic tracking error using a robot target at its measured pose.
                            pos['right']=tcp.copy();rot['right']=Ractual.copy()
                            return
                        depth=max(h,rel[2]) if angle>np.radians(.3) else h
                        desired_tip=st['driver'].pos+st['driver'].rotation()@np.array([0,0,depth-task.mechanism_state(emb._env)['pressure_m']])
                        offset=Ractual.T@(st['bit'].pos-tcp)
                        desired_actual_R=delta@Ractual
                        desired_R=delta@rot['right']
                        desired_p=pos['right']+desired_tip-desired_actual_R@offset-tcp
                        d=desired_p-pos['right'];length=np.linalg.norm(d)
                        if length>.004:desired_p=pos['right']+d*.004/length
                        move(f'insert_{attempt}_{k}_{j}','right',desired_p,0,10,desired_R)
            insertion_pass(0)
            if not seated:
                (out/'abort.json').write_text(json.dumps({'phase':'insertion_not_seated','step':step,'mount_pass':False}))
                imageio.mimsave(out/'aborted.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
                raise RuntimeError('First insertion did not seat; stop before release')
            move('release_bit','right',pos['right'],1,25,rot['right'])
            move('withdraw_axial','right',pos['right']+np.array([0,-.06,0]),1,35,rot['right'])
            move('rear_release','left',pos['left'],1,25,rot['left'])
            tool_grip=False
            move('rear_clear','left',pos['left']+[0,0,.08],1,45,rot['left'])
            move('hold_after_release','right',pos['right'],1,40,rot['right'])
            st=object_states(emb._env,['bit','driver']);rel=st['driver'].rotation().T@(st['bit'].pos-st['driver'].pos)
            result={'scope':'Actual robot pick, insertion, release, withdrawal and 2 s hold; no tool lift or tightening','bit_lift_m':bit_lift,'bit_in_driver_m':rel.tolist(),'mount_pass':bool(np.linalg.norm(rel[:2])<.002 and .097<rel[2]<.106)}
        (out/'insertion_result.json').write_text(json.dumps(result,indent=2)+'\n')
        if not result['mount_pass']:raise RuntimeError('Insertion gate failed')
        # Preserve the successful physical assembly state for later local
        # diagnostics. Resuming it never counts as a from-reset full demo.
        from checkpoint import save
        save(out/'assembly_checkpoint.json',emb._env,
             {'q':{k:v.tolist() for k,v in q.items()},
              'pos':{k:v.tolist() for k,v in pos.items()},
              'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
        if assembly_only:
            partial={'scope':'bit installation only; no tool pickup or screw tightening',
                     'steps':step,'mount_pass':result['mount_pass'],
                     'assembly_pass':bool(task.MECHANISM.get('assembly_pass')),
                     'score':float(task.score(None)),
                     'mechanism':dict(task.MECHANISM)}
            (out/'result.json').write_text(json.dumps(partial,indent=2)+'\n')
            imageio.mimsave(out/'assembly.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
            if partial['score']!=.2:raise RuntimeError('Assembly-only probe did not earn expected 0.2; see result.json')
            print(json.dumps(partial),flush=True)
            return
        move('withdraw_transit','right',pos['right']+np.array([0,0,.10]),1,60,rot['right'])
        clear_right_for_pickup()
        plan=json.loads(Path(__file__).with_name('horizontal_plan.json').read_text())['candidates'][1]
        start=np.array(plan['start'])+np.array([task.TOOL_SHIFT_X,0,0]);start[1]=-.045;goal=np.array([.39,0.,.99]);Rf=TOOL_DOWN_R
        pickup_initial_state=object_states(emb._env,['driver'])['driver']
        Ri=Rotation.from_rotvec(pickup_initial_state.rotation()[:,2]*PICKUP_WRIST_ROLL_RAD).as_matrix()@np.array(plan['initial_R'])
        pickup_initial_z=pickup_initial_state.pos[2]
        # After two-arm bit fitting, grasp the main body near its mass centre.
        # This operating grip shortens the lever arm during tool return.
        grip_local=np.array([0.,0.,.030])
        # Assembly shifts the tool by up to a few millimetres in its stand.
        # Center the jaws on the observed rigid-body grip point, keeping the
        # validated 2 mm low TCP offset, instead of using the reset nominal.
        start=pickup_initial_state.pos+pickup_initial_state.rotation()@grip_local
        start[2]-=.002
        pickup_grip_initial_z=float((pickup_initial_state.pos+
            pickup_initial_state.rotation()@grip_local)[2])
        tool_grip=True;operating_grip=True
        for name,target,opening,n,R in [('approach_tool',start+[0,0,.04],1,45,Ri),('lower_tool',start,1,45,Ri),('grasp_tool',start,0,30,Ri),('lift_tool',start+[0,0,.04],0,50,Ri),('turn_down',TOOL_CLEAR_TCP.copy(),0,100,Rf)]:
            move(name,'left',target,opening,n,R,joint_space=(name in ('approach_tool','turn_down')))
            if name=='grasp_tool':
                robot=emb._env.unwrapped.scene['left_robot']
                actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
                grasp_relative_R=tcp_R.T@object_states(emb._env,['driver'])['driver'].rotation()
            if name=='turn_down':
                driver_state=object_states(emb._env,['driver'])['driver']
                lift=float(driver_state.pos[2]-pickup_initial_z)
                grip_lift=float((driver_state.pos+
                    driver_state.rotation()@grip_local)[2]-pickup_grip_initial_z)
                robot=emb._env.unwrapped.scene['left_robot']
                actual=robot.data.joint_pos.torch[0].detach().cpu().numpy()
                _,tcp_R=ik.tcp_pose(actual[[robot.joint_names.index(j) for j in ARM_JOINTS]],station.BASES['left'])
                relative_rotation_deg=float(np.degrees(np.linalg.norm(Rotation.from_matrix(
                    (tcp_R.T@driver_state.rotation())@grasp_relative_R.T).as_rotvec())))
                (out/'pickup_gate.json').write_text(json.dumps({'phase':'pickup_gate',
                    'tool_center_lift_m':lift,'gripped_body_lift_m':grip_lift,
                    'tool_rotation_in_grip_deg':relative_rotation_deg,
                    'passed':grip_lift>=.06 and relative_rotation_deg<12.},indent=2)+'\n')
                if grip_lift<.06 or relative_rotation_deg>=12.:
                    (out/'abort.json').write_text(json.dumps({'phase':'pickup_gate','tool_center_lift_m':lift,
                        'gripped_body_lift_m':grip_lift,'tool_rotation_in_grip_deg':relative_rotation_deg,'passed':False}))
                    imageio.mimsave(out/'aborted.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
                    raise RuntimeError('Actual tool pickup failed')
        move('turn_settle','left',pos['left'],0,30,rot['left'],joint_space=True,q_target=q['left'])
        # The old side-arc wrist rotations were planned for the tool before
        # its socket was flipped; carrying them forward would point the bit up.
        from checkpoint import save
        save(out/'prehover_checkpoint.json',emb._env,
             {'q':{k:v.tolist() for k,v in q.items()},
              'pos':{k:v.tolist() for k,v in pos.items()},
              'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
    if at_powered:
        tool_grip=True;operating_grip=grip['left']<=.05
        move('grip_settle','left',pos['left'],0,40,rot['left'],
             joint_space=True,q_target=q['left'])
        return_tool()
        m=dict(task.MECHANISM)
        result={'scope':'Physical tool-return diagnostic from powered checkpoint; not full-demo evidence',
                'steps':step,'tool_in_stand':bool(m.get('tool_in_stand')),
                'motor_stopped':not m.get('motor_enabled',True),
                'hands_clear':bool(m.get('hands_clear')),'mechanism':m}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='mechanism'}),flush=True)
        if not (result['tool_in_stand'] and result['motor_stopped'] and result['hands_clear']):
            raise RuntimeError('Return-only diagnostic failed')
        return
    if resume:
        tool_grip=True
        # Preserve the grip aperture used when the diagnostic checkpoint was
        # recorded. Full reset rollouts retain their original operating grip.
        operating_grip=bool(at_prehover or (at_hover and grip['left']<=.05))
        if at_hover:
            # A restored joint pose is authoritative; the cached Cartesian
            # target can differ slightly after the final feedback update.
            robot=emb._env.unwrapped.scene['left_robot']
            measured=robot.data.joint_pos.torch[0].detach().cpu().numpy()
            q['left']=measured[[robot.joint_names.index(j) for j in ARM_JOINTS]]
            pos['left'],rot['left']=ik.tcp_pose(q['left'],station.BASES['left'])
        move('grip_settle','left',pos['left'],0 if operating_grip else .10,
             20 if at_hover else 80,rot['left'],joint_space=at_hover,
             q_target=q['left'] if at_hover else None)
    # Reach a measured high-clearance pose before crossing to the screw.
    if not at_hover:
        move('clearance_reposition','left',TOOL_CLEAR_TCP.copy(),0,60,rot['left'],joint_space=True)
        move('clearance_settle','left',pos['left'],0,30,rot['left'],joint_space=True,q_target=q['left'])
        if not resume or at_assembly or at_prehover:brace_workpiece()
        clearance_states=object_states(emb._env,['bit','drawer_context'])
        clearance_bit=clearance_states['bit'];clearance_drawer=clearance_states['drawer_context']
        clearance_tip=clearance_bit.pos+clearance_bit.rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])
        screw_center=clearance_drawer.pos+clearance_drawer.rotation()@np.array([task.FASTENER_X,-.06,.099])
        if clearance_tip[2]<screw_center[2]+.060:
            raise RuntimeError(f'Insufficient clearance before drawer crossing: {clearance_tip[2]-screw_center[2]:.4f} m')
    left_robot=emb._env.unwrapped.scene['left_robot']
    from scipy.optimize import least_squares
    # Joint 5 tracks about 58 mrad short when commanded to its hard upper
    # stop. Preserve 80 mrad of command margin; offline IK confirms that the
    # measured bit/TCP transform reaches every hover and engagement height.
    approach_upper=ik.UPPER.copy();approach_upper[4]-=.08
    def fit_bit_target(target_tip,bit_state,drawer_state,current_q):
        tcp_now,R_now=ik.tcp_pose(current_q,station.BASES['left'])
        bit_R_now=bit_state.rotation();drawer_R_now=drawer_state.rotation()
        bit_in_tcp=R_now.T@bit_R_now
        tip_now=bit_state.pos+bit_R_now@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])
        offset_in_tcp=R_now.T@(tip_now-tcp_now)
        def residual(joints):
            planned_tcp,planned_R=ik.tcp_pose(joints,station.BASES['left'])
            planned_bit_R=planned_R@bit_in_tcp
            planned_tip=planned_tcp+planned_R@offset_in_tcp
            axis=np.arccos(np.clip(planned_bit_R[:,2]@(-drawer_R_now[:,2]),-1.,1.))
            yaw=np.arctan2((drawer_R_now.T@planned_bit_R)[1,0],
                           (drawer_R_now.T@planned_bit_R)[0,0])
            cross=(yaw+np.pi/4)%(np.pi/2)-np.pi/4
            return np.r_[planned_tip-target_tip,
                         .3*max(0.,axis-np.radians(6.)),
                         .08*max(0.,abs(cross)-np.radians(12.))]
        fitted=least_squares(residual,
                             np.clip(current_q,ik.LOWER+1e-6,approach_upper-1e-6),
                             bounds=(ik.LOWER,approach_upper),max_nfev=120)
        planned_tcp,planned_R=ik.tcp_pose(fitted.x,station.BASES['left'])
        planned_bit_R=planned_R@bit_in_tcp
        axis_deg=float(np.degrees(np.arccos(np.clip(planned_bit_R[:,2]@(-drawer_R_now[:,2]),-1.,1.))))
        yaw=np.arctan2((drawer_R_now.T@planned_bit_R)[1,0],
                       (drawer_R_now.T@planned_bit_R)[0,0])
        cross_deg=float(np.degrees((yaw+np.pi/4)%(np.pi/2)-np.pi/4))
        return fitted.x,float(np.linalg.norm(fitted.fun[:3])),axis_deg,cross_deg
    alignment_states=object_states(emb._env,['bit','drawer_context'])
    measured_q=left_robot.data.joint_pos.torch[0].detach().cpu().numpy()
    measured_q=measured_q[[left_robot.joint_names.index(j) for j in ARM_JOINTS]]
    reach=[]
    reach_heights=(.015,0.) if at_hover else (.120,.090,.035,.015,0.)
    for candidate_height in reach_heights:
        drawer=alignment_states['drawer_context']
        target_tip=drawer.pos+drawer.rotation()@np.array([task.FASTENER_X,-.06,.099+candidate_height])
        _,e,axis_deg,cross_deg=fit_bit_target(target_tip,alignment_states['bit'],drawer,measured_q)
        reach.append({'height_m':candidate_height,'tip_error_m':e,
                      'axis_error_deg':axis_deg,'cross_error_deg':cross_deg})
    (out/'pose_reachability.json').write_text(json.dumps(reach,indent=2)+'\n')
    if any(r['tip_error_m']>.001 or r['axis_error_deg']>7.5 or abs(r['cross_error_deg'])>14. for r in reach):
        (out/'abort.json').write_text(json.dumps({'phase':'pose_reachability','step':step,'candidates':reach})+'\n')
        raise RuntimeError('Held-bit target unreachable within screw-engagement tolerances')
    errors=[];servo_bias=np.zeros(6)
    height=.015;converged=False
    # Restoring a PhysX snapshot does not restore contact warm-start caches.
    # Re-align the tip after grip settling before using a hover checkpoint.
    for height in ([.015] if at_hover else [.120,.090,.035,.015] if not resume or 'prehover' in resume else [.015]):
        st=object_states(emb._env,['bit','drawer_context']);bit=st['bit'];drawer=st['drawer_context']
        target_tip=drawer.pos+drawer.rotation()@np.array([task.FASTENER_X,-.06,.099+height])
        actual_tip=bit.pos+bit.rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])
        rob=emb._env.unwrapped.scene['left_robot'];aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
        measured_q=aq[[rob.joint_names.index(j) for j in ARM_JOINTS]]
        qgoal,ee,axis_deg,cross_deg=fit_bit_target(target_tip,bit,drawer,measured_q)
        if ee>.001 or axis_deg>7.5 or abs(cross_deg)>14.:
            (out/'abort.json').write_text(json.dumps({'phase':f'direct_{int(height*1000)}','step':step,
                                                     'position_error_m':ee,'axis_error_deg':axis_deg,
                                                     'cross_error_deg':cross_deg,
                                                     'mount_pass':False})+'\n')
            raise RuntimeError(f'Direct hover target unreachable: {ee:.4f} m, {axis_deg:.2f} deg axis')
        _,target_tcp_R=ik.tcp_pose(qgoal,station.BASES['left'])
        move(f'direct_{int(height*1000)}','left',target_tip,0,100,target_tcp_R,
             joint_space=True,q_target=np.clip(qgoal+servo_bias,ik.LOWER,ik.UPPER))
        if height in (.120,.090):
            crossed=object_states(emb._env,['bit','drawer_context'])
            crossed_tip=crossed['bit'].pos+crossed['bit'].rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])
            crossed_screw=crossed['drawer_context'].pos+crossed['drawer_context'].rotation()@np.array([task.FASTENER_X,-.06,.099])
            if crossed_tip[2]<crossed_screw[2]+.060:
                raise RuntimeError(f'Unsafe drawer crossing clearance: {crossed_tip[2]-crossed_screw[2]:.4f} m')
        converged=False
        for k in range(30):
            st=object_states(emb._env,['bit','drawer_context']);bit=st['bit'];drawer=st['drawer_context']
            target_tip=drawer.pos+drawer.rotation()@np.array([task.FASTENER_X,-.06,.099+height])
            actual_tip=bit.pos+bit.rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])
            dp=target_tip-actual_tip
            drawer_R_now=drawer.rotation();bit_R_now=bit.rotation()
            axis_error=float(np.degrees(np.arccos(np.clip(bit_R_now[:,2]@(-drawer_R_now[:,2]),-1.,1.))))
            bit_yaw_now=np.arctan2((drawer_R_now.T@bit_R_now)[1,0],
                                   (drawer_R_now.T@bit_R_now)[0,0])
            cross_error=float(np.degrees((bit_yaw_now+np.pi/4)%(np.pi/2)-np.pi/4))
            errors.append({'height_m':height,'iteration':k,'tip_error_m':float(np.linalg.norm(dp)),
                           'axis_error_deg':axis_error,'cross_error_deg':cross_error})
            tolerance=.0025 if height>=.090 else .0015
            if np.linalg.norm(dp)<tolerance and axis_error<7.5 and abs(cross_error)<14.:
                converged=True;break
            aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
            measured_q=aq[[rob.joint_names.index(j) for j in ARM_JOINTS]]
            tcp,Ractual=ik.tcp_pose(measured_q,station.BASES['left'])
            bit_in_tcp=Ractual.T@bit.rotation()
            offset_in_tcp=Ractual.T@(actual_tip-tcp)
            new_qgoal,position_residual,planned_axis,planned_cross=fit_bit_target(
                target_tip,bit,drawer,measured_q)
            if position_residual>.001 or planned_axis>7.5 or abs(planned_cross)>14.:
                raise RuntimeError(f'Physical bit pose unreachable within engagement tolerances: {position_residual:.4f} m')
            # The loaded wrist persistently under-tracks its command. Integrate
            # measured joint error while keeping commands inside official limits.
            servo_bias=np.clip(servo_bias+.4*(qgoal-measured_q),-.20,.20)
            qgoal=new_qgoal
            move(f'servo_{int(height*1000)}_{k}','left',target_tip,0,20,Ractual,
                 joint_space=True,q_target=np.clip(qgoal+servo_bias,ik.LOWER,ik.UPPER))
        if not converged and errors[-1]['tip_error_m']>.003:break
    # Verify the physical tip remains close while the arm holds position.
    hold_errors=[]
    for k in range(10):
        move(f'hold_feedback_{k}','left',pos['left'],0,20,rot['left'],joint_space=True,q_target=q['left'])
        st=object_states(emb._env,['bit','drawer_context'])
        target_tip=st['drawer_context'].pos+st['drawer_context'].rotation()@np.array([task.FASTENER_X,task.SCREW_SITES['lower'],.099+height])
        bs=st['bit']
        hold_errors.append(float(np.linalg.norm(bs.pos+bs.rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z])-target_tip)))

    from checkpoint import save
    save(out/'hover_checkpoint.json',emb._env,{'q':{k:v.tolist() for k,v in q.items()},'pos':{k:v.tolist() for k,v in pos.items()},'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
    st=object_states(emb._env,['bit','driver','drawer_context']);tip=st['bit'].pos+st['bit'].rotation()@np.array([0.,0.,task.BIT_TIP_LOCAL_Z]);rel=st['driver'].rotation().T@(st['bit'].pos-st['driver'].pos)
    result={'scope':'Actual robot bit installation, tool pickup and oblique hover; no engagement or tightening','converged':converged,'tip_error_m':float(np.linalg.norm(tip-target_tip)),'final_target_height_m':height,'bit_in_driver_m':rel.tolist(),'steps':step,'tool_tilt_deg':float(np.degrees(np.arccos(-st['bit'].rotation()[2,2]))),'errors':errors,'hold_mode':'Ten quasistatic updates at 1 Hz; approach gate: 3 mm position error and 2 mm lateral error at 15 mm nominal clearance','hold_errors_m':hold_errors}
    result['verification_max_error_m']=max(hold_errors[-3:])
    final_bit_R=st['bit'].rotation();final_drawer_R=st['drawer_context'].rotation()
    final_axis_deg=float(np.degrees(np.arccos(np.clip(final_bit_R[:,2]@(-final_drawer_R[:,2]),-1.,1.))))
    final_yaw=np.arctan2((final_drawer_R.T@final_bit_R)[1,0],(final_drawer_R.T@final_bit_R)[0,0])
    final_cross_deg=float(np.degrees((final_yaw+np.pi/4)%(np.pi/2)-np.pi/4))
    result.update(axis_error_deg=final_axis_deg,cross_error_deg=final_cross_deg)
    result['approach_clearance_pass']=bool(height==.015 and result['tip_error_m']<.003 and result['verification_max_error_m']<.003 and np.linalg.norm((tip-target_tip)[:2])<.002 and final_axis_deg<8. and abs(final_cross_deg)<15. and np.linalg.norm(rel[:2])<.0025 and .0965<rel[2]<.107)
    (out/'hover_result.json').write_text(json.dumps(result,indent=2)+'\n')
    imageio.mimsave(out/'alignment.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
    print(json.dumps({k:v for k,v in result.items() if k!='errors'}),flush=True)
    if hover_only:return
    # Only robot joint/gripper actions below; nose compression and tip engagement own motor activation.
    if not result['approach_clearance_pass']:
        raise RuntimeError('Hover gate failed; no tightening attempted')
    # Hover snapshots already include the right-hand brace. Pre-hover resumes
    # brace before alignment above; never shift the workpiece after alignment.
    # Lower to the recess through measured-tip targets. Refit the arm joints
    # within the task's engagement-angle tolerances at every stage, as in the
    # verified side hover; exact tool-body yaw is unreachable at this wrist pose.
    for h in [.010,.005,.002,0.0]:
        for k in range(15):
            st=object_states(emb._env,['bit','drawer_context'])
            bit=st['bit'];drawer=st['drawer_context']
            target=drawer.pos+drawer.rotation()@np.array([task.FASTENER_X,task.SCREW_SITES['lower'],.099+h])
            tip=bit.pos+bit.rotation()@np.array([0,0,task.BIT_TIP_LOCAL_Z]);err=target-tip
            axis_error=np.degrees(np.arccos(np.clip(np.dot(bit.rotation()[:,2],-drawer.rotation()[:,2]),-1.,1.)))
            if np.linalg.norm(err)<(.0004 if h==0 else .0008) and axis_error<8:break
            rob=emb._env.unwrapped.scene['left_robot'];aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
            measured_q=aq[[rob.joint_names.index(j) for j in ARM_JOINTS]]
            qgoal,ee,planned_axis,planned_cross=fit_bit_target(target,bit,drawer,measured_q)
            if ee>.001 or planned_axis>7.5 or abs(planned_cross)>14.:
                raise RuntimeError(f'Engagement target unreachable at {h:.3f} m: '
                                   f'{ee:.4f} m, {planned_axis:.1f} deg axis')
            _,target_R=ik.tcp_pose(qgoal,station.BASES['left'])
            if k:servo_bias=np.clip(servo_bias+.5*(qgoal-measured_q),-.20,.20)
            move(f'engage_{h}_{k}','left',target,0,15,target_R,joint_space=True,
                 q_target=np.clip(qgoal+servo_bias,ik.LOWER,ik.UPPER))
        st=object_states(emb._env,['bit','drawer_context']);bit=st['bit'];drawer=st['drawer_context']
        target=drawer.pos+drawer.rotation()@np.array([task.FASTENER_X,task.SCREW_SITES['lower'],.099+h])
        tip=bit.pos+bit.rotation()@np.array([0,0,task.BIT_TIP_LOCAL_Z]);err=target-tip
        axis_error=np.degrees(np.arccos(np.clip(np.dot(bit.rotation()[:,2],-drawer.rotation()[:,2]),-1.,1.)))
        if np.linalg.norm(err)>.002 or axis_error>=8:
            raise RuntimeError(f'Physical bit missed screw at {h:.3f} m: {np.linalg.norm(err):.4f} m, {axis_error:.1f} deg')
    powered=[]
    motor_started=False
    axial_follow_m=0.
    for k in range(200):
        st=object_states(emb._env,['bit','drawer_context'])
        m=task.mechanism_state(emb._env)
        motor_started |= bool(task.MECHANISM.get('motor_enabled',False) and
                              task.MECHANISM.get('tip_engaged',{}).get('lower',False))
        if motor_started:
            if not task.MECHANISM.get('tip_engaged',{}).get('lower',False):
                raise RuntimeError(f'Bit disengaged after motor start at step {k}')
            if m['pressure_m']<.00125:
                # The screw advances away from the nose as it turns. Follow
                # by a few tenths of a millimetre using a local differential
                # joint target, preserving the physically established contact.
                advance=min(.0005,max(.00015,.5*(.00165-m['pressure_m'])))
                axial_follow_m+=advance
                if axial_follow_m>.002:
                    raise RuntimeError(f'Pressure follow exceeded 2 mm at step {k}')
                bit,drawer=st['bit'],st['drawer_context']
                rob=emb._env.unwrapped.scene['left_robot']
                aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
                measured_q=aq[[rob.joint_names.index(j) for j in ARM_JOINTS]]
                tcp_now,R_now=ik.tcp_pose(measured_q,station.BASES['left'])
                tip_now=bit.pos+bit.rotation()@np.array([0,0,task.BIT_TIP_LOCAL_Z])
                tip_in_tcp=R_now.T@(tip_now-tcp_now)
                target_tip=tip_now-drawer.rotation()[:,2]*advance
                def follow_residual(joints):
                    pp,RR=ik.tcp_pose(joints,station.BASES['left'])
                    return np.r_[pp+RR@tip_in_tcp-target_tip,.1*(joints-measured_q)]
                fit=least_squares(follow_residual,
                                  np.clip(measured_q,ik.LOWER+1e-6,ik.UPPER-1e-6),
                                  bounds=(ik.LOWER,ik.UPPER),max_nfev=50)
                if np.linalg.norm(fit.fun[:3])>.0002:
                    raise RuntimeError(f'Incremental pressure follow unreachable at step {k}')
                qfollow=np.clip(q['left']+2*(fit.x-measured_q),ik.LOWER,ik.UPPER)
                pp,RR=ik.tcp_pose(qfollow,station.BASES['left'])
                move(f'pressure_follow_{k}','left',pp,0,4,RR,
                     joint_space=True,q_target=qfollow)
            else:
                move(f'powered_hold_{k}','left',pos['left'],0,4,rot['left'],
                     joint_space=True,q_target=q['left'])
            powered.append(dict(task.MECHANISM))
            if k>15 and max(x['handle_gap_m'] for x in powered[-10:])<.0002 and task.MECHANISM['powered_lower_m']>=task.POWERED_FEED_REQUIRED_M and m['screw_feed_lower_m']>=task.POWERED_FEED_REQUIRED_M:break
            continue
        # Command a known bit-tip goal directly, with a shallow axial preload.
        # Contact determines spring compression and motor start; no object pose
        # or screw state is set by this trajectory.
        press_depth=min(.003,.00025+.00015*k)
        target=st['drawer_context'].pos+st['drawer_context'].rotation()@np.array(
            [task.FASTENER_X,task.SCREW_SITES['lower'],.099-m['screw_feed_lower_m']-press_depth])
        rob=emb._env.unwrapped.scene['left_robot']
        aq=rob.data.joint_pos.torch[0].detach().cpu().numpy()
        measured_q=aq[[rob.joint_names.index(j) for j in ARM_JOINTS]]
        qgoal,ee,planned_axis,planned_cross=fit_bit_target(
            target,st['bit'],st['drawer_context'],measured_q)
        if ee>.0015 or planned_axis>7.5 or abs(planned_cross)>14.:
            raise RuntimeError(f'Powered bit goal unreachable at step {k}: '
                               f'{ee:.4f} m, {planned_axis:.1f} deg axis')
        servo_bias=np.clip(servo_bias+.3*(qgoal-measured_q),-.20,.20)
        _,target_R=ik.tcp_pose(qgoal,station.BASES['left'])
        move(f'powered_{k}','left',target,0,4,target_R,joint_space=True,
             q_target=np.clip(qgoal+servo_bias,ik.LOWER,ik.UPPER))
        powered.append(dict(task.MECHANISM))
        if k==30 and not any(x['motor_enabled'] for x in powered[-10:]):
            (out/'drive_abort.json').write_text(json.dumps({'reason':'push-start motor did not engage',
                'pressure_m':task.MECHANISM['pressure_m'],
                'dock_interlock':task.MECHANISM['dock_interlock'],'step':step},indent=2)+'\n')
            raise RuntimeError('Push-start motor did not engage; stop unproductive powered loop')
        if k>15 and max(x['handle_gap_m'] for x in powered[-10:])<.0002 and task.MECHANISM['powered_lower_m']>=task.POWERED_FEED_REQUIRED_M and m['screw_feed_lower_m']>=task.POWERED_FEED_REQUIRED_M:break
    (out/'powered_trace.json').write_text(json.dumps(powered,indent=2))
    save(out/'powered_checkpoint.json',emb._env,{'q':{k:v.tolist() for k,v in q.items()},'pos':{k:v.tolist() for k,v in pos.items()},'rot':{k:v.tolist() for k,v in rot.items()},'grip':grip})
    if os.environ.get('REPAIR_DRIVE_ONLY')=='1':
        result={'scope':'Drive-only diagnostic from checkpoint; not full-demo evidence',
                'steps':step,'powered_lower_m':task.MECHANISM['powered_lower_m'],
                'motor_enabled':task.MECHANISM['motor_enabled'],'score':task.score(None)}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        return
    if task.mechanism_state(emb._env)['handle_gap_m']>.0002:
        imageio.mimsave(out/'tightening_failed.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
        raise RuntimeError('Handle did not close; no full-score claim')
    return_tool()
    result={'scope':'Full robot sequence' if not resume else 'Diagnostic checkpoint continuation; not full-demo evidence',
            'score':task.score(None),'mechanism':dict(task.MECHANISM),'steps':step,
            'full_demo':bool(not resume and task.score(None)==1.),'success':bool(task.success(None))}
    (out/'result.json').write_text(json.dumps(result,indent=2))
    if result['full_demo']:
        np.savez_compressed(out/'actions.npz',actions=np.asarray(actions,dtype=np.float32))
        source_dir=Path(__file__).parent
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source_dir.iterdir()
                if p.is_file() and p.suffix in ('.py','.json')}
        (out/'replay_manifest.json').write_text(json.dumps({'task':'single_screw_repair',
            'control_hz':20,'object_jitter':0,'steps':len(actions),'score':result['score'],
            'source_sha256':hashes},indent=2)+'\n')
    imageio.mimsave(out/'demo.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
    print(json.dumps(result),flush=True)
    if not result['success']:raise RuntimeError('Single-screw sequence did not meet full score; see result.json and demo.mp4')

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('--preview-only',action='store_true',help='Render initial scene and stop before robot actions')
    parser.add_argument('--assembly-only',action='store_true',help='Stop after physical bit installation and score check')
    parser.add_argument('--hover-only',action='store_true',help='Diagnostic: stop after hover alignment; checkpoints are not full-demo evidence')
    args=parser.parse_args()
    run(args.output,preview_only=args.preview_only,assembly_only=args.assembly_only,hover_only=args.hover_only)
