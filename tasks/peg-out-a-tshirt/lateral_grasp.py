"""Unvalidated lateral hem pickup on the explicitly raised laundry support.

Native reset coordinates select ordinary bounded robot targets. No material,
prop, collision or force writer is used. Actual contact remains unproven.
"""
import copy,hashlib
import numpy as np
from fixed_motion import ROTATIONS


def blend_lift(plan):
    """Join a measured pickup to the fixed carry with ordinary smooth joints."""
    if not plan.get('blend_measured_pickup') or len(plan['frames'])<260:return
    start=np.asarray(plan['frames'][199]['action14'],float)
    end=np.asarray(plan['frames'][259]['action14'],float)
    for i in range(60):
        t=(i+1)/60;t=t*t*(3-2*t)
        plan['frames'][200+i]=dict(action14=(start+t*(end-start)).tolist(),
            stage='lift measured held hem into planned carry')


def calibrate(reference,rest,nodes,*,edge_patch=False,clear_height=None,clear_x=.15,left_angle=-10.,right_angle=-10.,flip_left=False,left_patch_x=-.13,left_closing_tilt=0.):
    from yam_arena import ik,station
    from yam_arena.constants import READY,SHOULDER,WRIST,GRIPPER,FINGER_OPEN
    rest=np.asarray(rest);nodes=np.asarray(nodes)
    if rest.shape!=nodes.shape or not np.isfinite(nodes).all():raise ValueError('Invalid cloth observation')
    if not -.14<=left_patch_x<=-.10:raise ValueError('Left patch must stay on exposed hem')
    arms={};records=[];patches={}
    for side,hx,sign in [('left',left_patch_x if edge_patch else -.12,1.),('right',.12,-1.)]:
        ids=(abs(rest[:,0]-hx)<.011)&(rest[:,1]<.011);front=ids&(rest[:,2]>0)
        uv=rest[front,:2];points=nodes[front]
        fit=np.linalg.lstsq(np.c_[uv-uv.mean(0),np.ones(len(uv))],points-points.mean(0),rcond=None)[0]
        width=fit[0]/np.linalg.norm(fit[0]);normal=np.cross(width,fit[1]);normal/=np.linalg.norm(normal)
        approach=sign*width;closing=sign*normal;R=np.column_stack((closing,np.cross(approach,closing),approach))
        if not -35.<=left_angle<=35. or not -35.<=right_angle<=35.:raise ValueError('Bounded in-plane grasp angle required')
        angle=np.deg2rad((left_angle if side=='left' else right_angle) if edge_patch else (-10. if side=='right' else 0.))
        R=R@np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
        if not -15.<=left_closing_tilt<=15.:raise ValueError('Closing-axis tilt is bounded to15degrees')
        if edge_patch and side=='left':
            tilt=np.deg2rad(left_closing_tilt)
            R=R@np.array(((np.cos(tilt),0.,np.sin(tilt)),(0.,1.,0.),(-np.sin(tilt),0.,np.cos(tilt))))
        if edge_patch and side=='left' and flip_left:R=R@np.diag((-1.,-1.,1.))
        center=nodes[ids].mean(0)
        target=center-R@np.array((0.,-.008 if edge_patch else 0.,-.010))
        pre=target-(.020 if edge_patch else .060)*R[:,2]
        if not (.17<center[0]<.27 and .84<center[2]<.94 and abs(center[1]-sign*.12)<.04):
            raise ValueError('Actual hem is outside the raised-support planning region')
        q=np.array(READY,float);opening=1.;frames=[]
        schedule=[(3.,np.array((.15,sign*.12,1.00)),.2,ROTATIONS[side],'orient above and outside the support','joint'),
            (1.,np.array((.15,pre[1],pre[2])),.2,R,'descend and turn in front of support','cartesian'),
            (1.,pre,.2,R,'align laterally outside hem edge','cartesian'),
            (2.,target,.2,R,'insert along exposed hem edge','cartesian'),
            (1.,target,.018,R,'pinch both cloth layers','cartesian'),
            (1.,np.array((.1494,target[1],target[2])),.018,R,'pull forward clear of support','cartesian'),
            (1.,np.array((.1494,sign*.115,.898 if side=='left' else .902)),.018,ROTATIONS[side],'turn held hem in clear space','cartesian')]
        if edge_patch:
            schedule[1:3]=[(2.,pre,.2,R,'align outside the measured hem patch','joint')]
            if clear_height is not None:
                if not 0.<=clear_height<=.10:raise ValueError('Bounded front approach height required')
                if not .13<=clear_x<=.22:raise ValueError('Front approach must remain outside board')
                schedule[1:2]=[(1.,np.array((clear_x,pre[1],pre[2]+clear_height)),.2,R,'orient in front of board','joint'),
                    (1.,pre,.2,R,'enter clear hem approach','cartesian')]
        for seconds,position,grip,rotation,label,mode in schedule:
            q0=q.copy();p0,R0=ik.tcp_pose(q0,station.BASES[side]);opening0=opening;goal=None;em=er=0.
            if mode=='joint':
                goal,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(rotation),q0,base_pos=station.BASES[side],iters=800)
                if edge_patch and (pe>.004 or re>.04):
                    from scipy.optimize import least_squares
                    def error(candidate):
                        p,r=ik.tcp_pose(candidate,station.BASES[side])
                        return np.r_[p-position,.3*ik._rot_error(rotation,r)]
                    seeds=[q0,(ik.LOWER+ik.UPPER)/2,*np.random.default_rng(0).uniform(ik.LOWER,ik.UPPER,(8,6))]
                    solutions=[]
                    for seed in seeds:
                        result=least_squares(error,seed,bounds=(ik.LOWER,ik.UPPER),max_nfev=200,
                            ftol=1e-10,xtol=1e-10,gtol=1e-10)
                        solutions.append(result)
                        if np.linalg.norm(result.fun[:3])<=.004 and np.linalg.norm(result.fun[3:])/.3<=.04:break
                    result=min(solutions,key=lambda s:(np.linalg.norm(s.fun[:3])>.004 or np.linalg.norm(s.fun[3:])/.3>.04,np.linalg.norm(s.fun)))
                    goal=result.x;pe=np.linalg.norm(result.fun[:3]);re=np.linalg.norm(result.fun[3:])/.3
                if pe>.004 or re>.04:raise ValueError(f'Raised-support clear approach is unreachable: {side} {label}; {pe:.6g}m {re:.6g}rad')
            for k in range(1,round(seconds*20)+1):
                t=ik.smoothstep(k/(seconds*20))
                if goal is not None:q=q0+t*(goal-q0);pe=re=0.
                else:q,pe,re=ik.solve(p0+t*(position-p0),ik.matrix_to_quat_xyzw(ik.slerp_matrix(R0,rotation,t)),q,base_pos=station.BASES[side],iters=120)
                em=max(em,float(pe));er=max(er,float(re));opening=opening0+t*(grip-opening0)
                frames.append(dict(action=np.r_[q,opening].tolist(),stage=label))
            records.append(dict(side=side,stage=label,max_position_error_m=em,max_rotation_error_rad=er))
            if em>.004 or er>.04:raise ValueError('Actual lateral-grasp IK is unreachable: '+repr(records[-1]))
        if edge_patch and clear_height is None:
            # Bow the clear-space shoulder path away from the raised board.
            # Endpoints are unchanged; this is a bounded ordinary joint path.
            for i in range(60,100):frames[i]['action'][1]-=.60*np.sin(np.pi*(i-59)/40)**2
        arms[side]=frames;patches[side]=dict(material_node_ids=np.flatnonzero(ids).tolist(),observed_center_m=center.tolist(),target_tcp_m=target.tolist(),rotation=R.tolist(),desired_patch_local_m=[0.,-.008 if edge_patch else 0.,-.010])
    result=copy.deepcopy(reference)
    if edge_patch:
        result['edge_grasp_angles_degrees']=dict(left=float(left_angle),right=float(right_angle))
        result['edge_grasp_flip_left']=bool(flip_left)
        result['edge_grasp_material_x']=dict(left=float(left_patch_x),right=.12)
        result['edge_grasp_left_closing_tilt']=float(left_closing_tilt)
    result['frames'][:200]=[dict(action14=a['action']+b['action'],stage=a['stage']) for a,b in zip(arms['left'],arms['right'])]
    blend_lift(result)
    actions=np.array([np.r_[READY,1.,READY,1.]]+[f['action14'] for f in result['frames']])
    limits=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
    if np.any(abs(np.diff(actions,axis=0))>limits*1.001):raise ValueError('Lateral grasp exceeds ordinary action speed')
    for offset in (0,7):
        if np.any(actions[:,offset:offset+6]<ik.LOWER) or np.any(actions[:,offset:offset+6]>ik.UPPER):
            raise ValueError('Lateral grasp exceeds ordinary joint bounds')
    result['kinematic_audit']=records
    return result,dict(patches=patches,input_native_nodes_sha256=hashlib.sha256(np.ascontiguousarray(nodes).tobytes()).hexdigest(),state_writes=False,method='Measured lateral hem insertion on the declared180mm support')
