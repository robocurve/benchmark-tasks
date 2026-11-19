"""Privileged native-state demonstration: robot joint/gripper commands only."""
import math
import numpy as np
from .geometry import GLASS_START, GLASS_YAW, NOZZLE, MOUNT, ROTATION, HANDLE_POINT
from .pump import LEVER_PIVOT
from .scoring import RIM, INNER_RADIUS, BASE, REST_OFFSET


def scripted_actions(task,base,policy,max_steps):
    from tools.run_recovery_pilot import planned_actions
    from yam_arena import ik,station
    from yam_arena.runtime import arm_q
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.urdf import quat_xyzw_to_matrix

    if max_steps!=task.policy_actions+40:raise ValueError('Reference budget must include the fixed 40-control tail')
    action=task.initial_action().copy()
    for side,offset in [('left',0),('right',7)]:
        action[offset:offset+6]=arm_q(base.scene[side+'_robot'])
    if policy=='hold':
        for _ in range(60):yield action.copy(),'native stationary pump diagnostic',{}
        return
    if policy not in ('serve','one_pull','acquire_glass'):
        raise ValueError(f'Unknown hand-pull policy {policy!r}')
    c,s=math.cos(.8),math.sin(.8)
    # Pinch across the handle's Y thickness so gravity cannot roll it between pads.
    grip_frame=np.array([[0.,-s,c],[-1.,0.,0.],[0.,-c,-s]])
    partial_open=.015/abs(FINGER_OPEN)
    glass_grip=.06
    pump_grip=.42
    c,s=math.cos(GLASS_YAW),math.sin(GLASS_YAW)
    glass_rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    grip=np.array([-.006,.074,.0785])
    pickup=np.asarray(GLASS_START)+grip
    receiving_base=np.asarray(NOZZLE)-[0.,0.,RIM-.005]
    destination=receiving_base-[0.,0.,.015]
    side=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
    c,s=math.cos(-.7),math.sin(-.7)
    grasp_rotation=np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])@side
    steps=0
    feedback=False
    lower_with_fill=False
    handle_offset=0.
    delivery_angle=math.radians(45.)
    receiving_raise=.098

    def target(position,rotation,grip):
        return dict(position=np.asarray(position).tolist(),
                    rotation_xyzw=ik.matrix_to_quat_xyzw(rotation).tolist(),grip=grip)

    def glass_error():
        position,quaternion,_=task.pose_state(base,'glass')
        position=position.cpu().numpy()
        rotation=quat_xyzw_to_matrix(quaternion.cpu().numpy())
        correction=glass_rotation@rotation.T
        angle=float(math.acos(np.clip((np.trace(correction)-1.)/2,-1.,1.)))
        return position,correction,angle

    def correct_glass():
        nonlocal destination
        position,correction,angle=glass_error()
        if lower_with_fill:
            points=task.fluid.positions()-position
            local=points@(correction.T@glass_rotation)
            radial=np.linalg.norm(local[:,:2],axis=1)
            pool=(radial>.012)&(radial<INNER_RADIUS-REST_OFFSET)&(local[:,2]>=BASE)&(local[:,2]<RIM)
            # Observe the bulk pool outside the central incoming jet.
            surface=float(np.quantile(points[pool,2],.99)+REST_OFFSET) if np.count_nonzero(pool)>20 else BASE
            height=NOZZLE[2]-surface-.010-receiving_base[2]
            destination=receiving_base+[0.,0.,float(np.clip(height,0.,receiving_raise))]
        error=destination-position
        actual_tcp,_=ik.tcp_pose(arm_q(base.scene['left_robot']),station.BASES['left'])
        command_tcp,command_rotation=ik.tcp_pose(action[:6],station.BASES['left'])
        fraction=min(.25,.01/angle) if angle>1e-9 else 0.
        turn=ik.slerp_matrix(np.eye(3),correction,fraction)
        delta=.25*error+(turn-np.eye(3))@(actual_tcp-position)
        distance=np.linalg.norm(delta)
        if distance>.002:delta*=.002/distance
        command,pe,re=ik.solve(command_tcp+delta,
            ik.matrix_to_quat_xyzw(turn@command_rotation),action[:6],
            base_pos=station.BASES['left'],iters=100)
        if pe>.003 or re>.04:raise RuntimeError(f'Unreachable native glass correction: {pe}, {re}')
        action[:6]=command
        return float(np.linalg.norm(error)),angle,dict(position_m=pe,rotation_rad=re)

    def require_glass():
        position,_,angle=glass_error()
        if np.linalg.norm(destination-position)>.008 or angle>math.radians(3):
            raise RuntimeError('Native mug is not held at the receiving pose')

    def move(phase,seconds,**arms):
        nonlocal action,steps
        plan=[dict(phase=phase,seconds=seconds,**arms)]
        for command,label,errors in planned_actions(action,plan,task.policy_actions-steps):
            if feedback and 'left' not in arms:command[:7]=action[:7]
            action=command.copy()
            if feedback and 'left' not in arms:_,_,errors['left']=correct_glass()
            steps+=1
            yield action.copy(),label,errors

    def stabilize(label,controls=60):
        nonlocal steps
        stable=0
        for _ in range(controls):
            distance,angle,errors=correct_glass()
            steps+=1
            yield action.copy(),label,dict(left=errors)
            stable=stable+1 if distance<.002 and angle<math.radians(1) else 0
            if stable>=10:break
        require_glass()

    def handle_pose(angle):
        c,s=math.cos(angle),math.sin(angle)
        turn=np.array([[c,0.,s],[0.,1.,0.],[-s,0.,c]])
        point=MOUNT+ROTATION@(LEVER_PIVOT+turn@HANDLE_POINT)
        # The round handle permits an independent grasp azimuth at rest.
        rotation=ROTATION@turn@ROTATION.T@grasp_rotation
        return point,rotation

    def stroke(start,end,label,seconds,stop_when_filled=False):
        nonlocal handle_offset
        for angle in np.linspace(start,end,round(seconds*20)+1)[1:]:
            if stop_when_filled and (task.metrics['retained_ml']>=420. or task.metrics['spill_latched']):break
            error=angle-task.pump_state()['handle_angle_rad']
            handle_offset=float(np.clip(handle_offset+np.clip(.15*error,-.01,.01),-.20,.20))
            point,rotation=handle_pose(angle+handle_offset)
            yield from move(label,.05,right=target(point,rotation,pump_grip))

    yield from move('approach empty mug handle',3.,left=target(pickup-.10*grip_frame[:,2],grip_frame,partial_open))
    yield from move('place fingers around mug handle',2.,left=target(pickup,grip_frame,partial_open))
    yield from move('grasp mug through native contact',1.,left=target(pickup,grip_frame,glass_grip))
    yield from move('lift empty mug',2.,left=target(pickup+[0.,0.,.060],grip_frame,glass_grip))
    position,quaternion,_=task.pose_state(base,'glass')
    held_rotation=quat_xyzw_to_matrix(quaternion.cpu().numpy())
    lowest=float(position[2])-.040*np.linalg.norm(held_rotation[2,:2])
    if lowest<GLASS_START[2]+.005 or held_rotation[2,2]<math.cos(math.radians(20)):
        raise RuntimeError('Native mug is not lifted clear and upright')
    # Infer the held transform including native arm sag before approaching the spout.
    command_tcp,command_rotation=ik.tcp_pose(action[:6],station.BASES['left'])
    grip=held_rotation.T@(command_tcp-position.cpu().numpy())
    carry_rotation=held_rotation.T@command_rotation
    yield from move('place upright mug below swan neck',3.,
                    left=target(destination+glass_rotation@grip,glass_rotation@carry_rotation,glass_grip))
    yield from stabilize('settle mug below outlet')
    feedback=True
    destination=receiving_base+[0.,0.,receiving_raise]
    yield from stabilize('raise mug around the clear outlet tip',controls=120)
    if policy=='acquire_glass':
        yield from move('check native mug hold before pumping',3.)
        require_glass()
        return
    point,rotation=handle_pose(0.)
    yield from move('approach upper cream hand-pull grip',3.,
                    right=target(point-.10*rotation[:,2],rotation,1.))
    yield from move('place fingers around hand-pull grip',2.,right=target(point,rotation,1.))
    yield from move('grasp hand-pull through native contact',1.,right=target(point,rotation,pump_grip))
    require_glass()
    lower_with_fill=True
    for cycle in range(1 if policy=='one_pull' else 4):
        yield from stroke(0.,-delivery_angle,'pull the real lever through delivery stroke',
                          6. if cycle<2 else 12.,stop_when_filled=policy=='serve')
        yield from move('hold end of pull; no time-based supply',.5)
        actual_angle=task.pump_state()['handle_angle_rad']
        if (actual_angle>-delivery_angle+.06 and task.metrics['retained_ml']<420.
                and not task.metrics['spill_latched']):
            raise RuntimeError('Robot did not complete the requested native partial stroke')
        yield from stroke(actual_angle,0.,'return hand-pull for cylinder refill',2.)
        yield from move('hold returned handle',.5)
        if task.metrics['spill_latched']:
            raise RuntimeError('Scripted diagnostic stopped after irreversible spill; not a full-horizon demo')
        if task.metrics['retained_ml']>=420.:break
    if policy=='one_pull':
        yield from move('settle after one native cycle diagnostic',2.)
        return
    while steps<max_steps:
        errors={}
        if steps<task.policy_actions:_,_,errors['left']=correct_glass()
        steps+=1
        phase='unactuated observation tail' if steps>task.policy_actions else 'hold glass and returned handle'
        yield action.copy(),phase,errors
