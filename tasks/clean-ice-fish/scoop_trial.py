"""Physical shovel pickup and pour using the held scoop; no object pose writes."""
import json
import math
import hashlib
import numpy as np


def rotation_xyzw(q):
    x,y,z,w=map(float,q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def object_rotation(yaw,pitch):
    a,b=np.radians(yaw),np.radians(pitch)
    rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
    ry=np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    return rz@ry


def run_scoop_trial(env,layout,out,initial_command=None,render_enabled=False):
    import torch
    # This is a physics-only test; Lab's documented switch skips Kit frame updates.
    # It leaves physics stepping and the original actuator configuration unchanged.
    print('PHYSICS_ONLY_RENDER_DISABLED',bool(env.unwrapped.render_enabled),flush=True)
    env.unwrapped.render_enabled=render_enabled
    from yam_arena import ik,station
    from yam_arena.constants import READY
    from yam_arena.runtime import tcp_world
    from task import pose_reader
    robot=env.unwrapped.scene['right_robot'];base=np.array(station.BASES['right'])
    arm_ids=robot.find_joints([f'joint{i}' for i in range(1,7)],preserve_order=True)[0]
    gripper_id=robot.body_names.index('gripper')
    names=[n for n,r in layout['objects'].items() if 'ice_source' in r]
    pose=pose_reader(env)
    (out/'held_checkpoint.json').write_text(json.dumps({
        'layout_sha256':hashlib.sha256(json.dumps(layout,sort_keys=True).encode()).hexdigest(),
        'objects':{name:pose(name).tolist() for name,row in layout['objects'].items() if row['dynamic']},
        'robots':{side:{'joint_names':env.unwrapped.scene[f'{side}_robot'].joint_names,
            'joint_positions':env.unwrapped.scene[f'{side}_robot'].data.joint_pos.torch[0].cpu().tolist()}
            for side in ('left','right')},
        'initial_command':None if initial_command is None else np.asarray(initial_command).tolist(),
        'scope':'Read-only checkpoint after real lid removal and scoop pickup, before any shovel stroke.'},indent=2)+'\n')
    def tool_pose():
        actual_q=robot.data.joint_pos.torch[0,arm_ids].cpu().numpy().copy()
        # Calibrate against the same FK frame used by the inverse-kinematics solver.
        return ik.tcp_pose(actual_q,base)
    def bucket_pose():
        p=pose('Scoop');r=rotation_xyzw(p[3:]);return p[:3]+r@np.array([.045,0,.0025]),r
    def inside_bucket():
        p=pose('Scoop');r=rotation_xyzw(p[3:]);result=[]
        for name in names:
            local=r.T@(pose(name)[:3]-p[:3])
            if -.008<local[0]<.113 and abs(local[1])<layout['scoop_bucket_inner_width_m']/2+.003 and .005<local[2]<.060:result.append(name)
        return result
    ee,re=tool_pose();sp=pose('Scoop');ro=rotation_xyzw(sp[3:])
    relative_rotation=re.T@ro
    root_offset=re.T@(sp[:3]-ee)
    bucket_offset=root_offset+relative_rotation@np.array([.045,0,.0025])
    (out/'held_scoop_calibration.json').write_text(json.dumps({'relative_rotation':relative_rotation.tolist(),
        'root_offset':root_offset.tolist(),'bucket_offset':bucket_offset.tolist()},indent=2)+'\n')
    trajectory=[];max_error=0.;phase_counts={};watched=set();q=robot.data.joint_pos.torch[0,arm_ids].cpu().numpy().copy()
    tracking_bias=np.zeros(6) if initial_command is None else np.asarray(initial_command)-q
    initial_bias=tracking_bias.copy();previous_command=q+tracking_bias
    def move(point,rotation,count,label):
        nonlocal q,max_error,relative_rotation,bucket_offset,tracking_bias,previous_command
        env.phase=label
        ee,re=tool_pose();current_scoop=pose('Scoop');current_ro=rotation_xyzw(current_scoop[3:])
        grip=current_scoop[:3]+current_ro@np.array(layout['scoop_grip_local'])
        if np.linalg.norm(grip-ee)>.030:
            raise RuntimeError(f'{label}: scoop slipped out of the gripper')
        # Start from the arm's calibrated tool frame so contact deflection does not
        # become an instantaneous wrist command at the next motion segment.
        start=ee+re@bucket_offset;rotation_start=re@relative_rotation
        point=np.array(point,float)
        print('SCOOP_PHASE',label,'bucket',bucket_pose()[0].tolist(),'target',point.tolist(),flush=True)
        for i in range(1,count+9):
            t=ik.smoothstep(min(i/count,1));b=start+t*(point-start)
            rb=ik.slerp_matrix(rotation_start,rotation,t);rt=rb@relative_rotation.T
            # Preserve the calibrated grasp transform. Chasing contact-induced bucket
            # rotation drives the wrist into the pile instead of letting it settle.
            p=b-rt@bucket_offset
            actual_q=robot.data.joint_pos.torch[0,arm_ids].cpu().numpy().copy()
            q,pe,re_error=ik.solve(p,ik.matrix_to_quat_xyzw(rt),actual_q,base_pos=base,iters=150,damping=.025)
            max_error=max(max_error,float(pe))
            if pe>.018 or re_error>np.radians(12):
                raise RuntimeError(f'{label}: IK position error {pe:.4f} m, rotation {np.degrees(re_error):.2f} degrees')
            tracking_error=q-actual_q
            tracking_bias=np.clip(tracking_bias+1.2*.05*tracking_error,-.18,.18)
            command=np.clip(q+tracking_bias+.25*tracking_error,ik.LOWER,ik.UPPER)
            command=np.clip(command,previous_command-.12,previous_command+.12)
            previous_command=command
            action=torch.tensor([[*READY,1.,*command.tolist(),0.]],dtype=torch.float32,device='cuda:0')
            env.step(action)
            actual,ar=bucket_pose()
            trajectory.append({'phase':label,'bucket_center':actual.tolist(),'target_bucket_center':b.tolist(),'scoop_pose':pose('Scoop').tolist(),
                               'watched_ice_poses':{name:pose(name).tolist() for name in watched},
                               'tcp':np.array(tcp_world(robot)).tolist(),'right_joints':robot.data.joint_pos.torch[0].cpu().tolist()})
        phase_counts[label]=inside_bucket()
        watched.update(phase_counts[label])
        (out/'scoop_progress.json').write_text(json.dumps({'phase':label,'bucket_ice':phase_counts[label],
            'max_ik_error_m':max_error,'trajectory':trajectory},indent=2)+'\n')
        print('BUCKET_ICE',label,phase_counts[label],flush=True)
    z=layout['table_top_z'];clean=np.array(layout['objects']['CleanBin']['position']);bowl=np.array(layout['objects']['Bowl']['position'])
    carried=[];attempts=[]
    try:
        current,_=bucket_pose()
        move([current[0],current[1],z+.130],object_rotation(layout['grasps']['Scoop']['long_yaw_degrees'],0),45,'raise_clear')
        for attempt in range(1,3):
            move([clean[0],clean[1]-.044,z+.130],object_rotation(90,0),65,f'entry_hover_{attempt}')
            move([clean[0],clean[1]-.044,z+.0435],object_rotation(90,30),75,f'enter_loose_ice_{attempt}')
            move([clean[0],clean[1]+.055,z+.0435],object_rotation(90,30),65,f'push_under_ice_{attempt}')
            move([clean[0],clean[1]+.046,z+.125],object_rotation(90,-10),8,f'curl_mouth_up_{attempt}')
            move([clean[0],clean[1]+.046,z+.175],object_rotation(90,-15),25,f'curl_and_lift_{attempt}')
            carried=inside_bucket();attempts.append({'attempt':attempt,'carried':carried})
            if carried:break
        if not carried:raise RuntimeError('No ice retained after two physical scoop attempts')
        move([clean[0]-.070,clean[1]+.015,z+.140],object_rotation(-30,-15),50,'clear_source_bin')
        move([bowl[0],bowl[1],z+.135],object_rotation(-30,-5),70,'carry_to_bowl')
        move([bowl[0],bowl[1],z+.095],object_rotation(-30,35),45,'pour_into_bowl')
        move([bowl[0],bowl[1],z+.095],object_rotation(-30,35),50,'wait_for_ice_to_settle')
        move([clean[0]-.070,clean[1]+.015,z+.140],object_rotation(-30,-15),50,'withdraw_to_show_served_ice')
        clean_in=[];bad_in=[];positions={}
        for name in names:
            p=pose(name);positions[name]=p.tolist()
            inside=np.linalg.norm(p[:2]-bowl[:2])<.086 and z+.003<p[2]<z+.065
            if inside:(clean_in if layout['objects'][name]['ice_source']=='clean' else bad_in).append(name)
        delivered_carried=sorted(set(clean_in)&set(carried))
        result={'passed':bool(delivered_carried and not bad_in),'clean_ice_in_bowl':clean_in,'fish_ice_in_bowl':bad_in,
            'carried_clean_ice_delivered':delivered_carried,
            'carried_ice_evidence':carried,'attempts':attempts,'max_ik_error_m':max_error,
            'trajectory':trajectory,'ice_final_poses':positions,
            'final_scoop_pose':pose('Scoop').tolist(),'final_lid_pose':pose('CleanLid').tolist(),
            'robot_link_poses':{side:{name:body.tolist() for name,body in zip(env.unwrapped.scene[f'{side}_robot'].body_names,
                env.unwrapped.scene[f'{side}_robot'].data.body_link_pose_w.torch[0].cpu().numpy())} for side in ('left','right')},
            'initial_joint_tracking_bias_rad':initial_bias.tolist(),'final_joint_tracking_bias_rad':tracking_bias.tolist(),
            'method':'Dynamic source cubes lifted and poured by the physically grasped scoop; only robot joint actions are commanded.'}
    except BaseException as error:
        result={'passed':False,'error':str(error),'attempts':attempts,'carried_ice_evidence':carried,
                'max_ik_error_m':max_error,'trajectory':trajectory,
                'ice_final_poses':{name:pose(name).tolist() for name in names},
                'final_scoop_pose':pose('Scoop').tolist(),'final_lid_pose':pose('CleanLid').tolist(),
                'robot_link_poses':{side:{name:body.tolist() for name,body in zip(env.unwrapped.scene[f'{side}_robot'].body_names,
                    env.unwrapped.scene[f'{side}_robot'].data.body_link_pose_w.torch[0].cpu().numpy())} for side in ('left','right')}}
    (out/'scoop_transfer.json').write_text(json.dumps(result,indent=2)+'\n')
    print('SCOOP_TRANSFER',{k:v for k,v in result.items() if k not in ('trajectory','ice_final_poses','robot_link_poses')},flush=True)
    return result
