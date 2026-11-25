"""Choose a real hub end-face pinch from complete native-joint IK paths.

This plans robot commands only; no simulator object, joint or score is changed.
"""
import numpy as np
from yam_arena import ik,station
from mechanics import rotation,point
from layout import PLACEMENT_ROOT_TARGET,PLACEMENT_CARRY,PLACEMENT_APPROACH,PLACEMENT_LOWER_START


def choose_hub_grasp(p,q0):
    pose=p['catheter'];r=rotation(pose[3:]);axis=r[:,1]
    c=np.sqrt(1-.025**2)
    aligned=np.array([[1.,0.,0.],[0.,c,.025],[0.,-.025,c]])
    ap=np.array([.375,pose[1],.8543]);reports=[];valid=[]
    # Preserve the contact-proven end-face grasp while checking the full new path.
    for sign in (-1,):
        # Keep the shallow oblique end-face pinch that clears the loose branch
        # supports. A steeper palm can seat the valve between the rest arms.
        for angle in (30.,):
            for lateral in (0.,):
                closing=sign*axis
                vertical=np.array([0.,0.,-1.]);vertical-=closing*np.dot(closing,vertical);vertical/=np.linalg.norm(vertical)
                a=np.deg2rad(angle)
                approach=np.cos(a)*vertical+np.sin(a)*np.cross(closing,vertical)
                tool=np.column_stack([closing,np.cross(approach,closing),approach])
                pick=point(pose,[lateral,-.087,.008])-approach*.013
                offset=r.T@(pick-pose[:3]);relative=r.T@tool
                targets=[('hover',pick+[0,0,.08],tool),('pick',pick,tool)]
                for i in range(1,7):
                    f=i/6;rr=ik.slerp_matrix(r,aligned,f)
                    targets.append(('align',pose[:3]+f*(ap-pose[:3])+rr@offset,rr@relative))
                for i in range(1,7):
                    targets.append(('withdraw',ap+np.array([0.,i/6*(-.180-pose[1]),0.])+aligned@offset,aligned@relative))
                targets += [('lift',np.array([.375,-.180,.930])+offset,relative),
                            ('carry',np.array(PLACEMENT_CARRY)+offset,relative),
                            ('approach',np.array(PLACEMENT_APPROACH)+offset,relative)]
                for origin,destination in ((PLACEMENT_CARRY,PLACEMENT_APPROACH),(PLACEMENT_APPROACH,PLACEMENT_LOWER_START),(PLACEMENT_LOWER_START,PLACEMENT_ROOT_TARGET)):
                    for fraction in np.linspace(.1,1.,10):
                        pos=np.array(origin)+fraction*(np.array(destination)-origin)
                        targets.append(('placement',pos+offset,relative))
                # After opening, the controller uses a joint-space READY
                # retreat, whose joint limits are native robot limits.
                q=q0.copy();max_pe=0.;max_re=0.;margin=float('inf');passed=True
                for name,pt,rt in targets:
                    q,pe,re=ik.solve(pt,ik.matrix_to_quat_xyzw(rt),q,base_pos=station.BASES['right'],iters=140)
                    max_pe=max(max_pe,pe);max_re=max(max_re,re)
                    margin=min(margin,float(np.min(np.minimum(q-ik.LOWER,ik.UPPER-q))))
                    if pe>.0015 or re>.025:passed=False;break
                report={'closing_sign':sign,'approach_angle_deg':angle,'pad_lateral_m':lateral,
                        'all_waypoints_reachable':passed,'max_ik_position_error_m':max_pe,
                        'max_ik_rotation_error_rad':max_re,'minimum_joint_margin_rad':margin}
                reports.append(report)
                if passed:valid.append((margin-.01*abs(lateral/.004),pick,tool,report))
    if not valid:raise RuntimeError('No reachable hub grasp for the complete physical path')
    _,pick,tool,chosen=max(valid,key=lambda x:x[0])
    return pick,tool,{'chosen':chosen,'candidates':reports}
