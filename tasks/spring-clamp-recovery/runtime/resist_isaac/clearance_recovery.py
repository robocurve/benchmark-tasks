"""Additional passive recovery scenes, with native rigid bodies and joints."""
import json
import math
from pathlib import Path
import tempfile

import numpy as np

from .recovery import RecoveryPilot
from .state import pose_state, reset_joints
from .usd_assets import create_asset


def export_assets(folder, directory):
    descriptions=json.loads((Path(folder)/'geometry.json').read_text())
    return {name:create_asset(data,Path(directory)/f'{name}.usda',folder)
            for name,data in descriptions.items()}


class ClearanceRecovery(RecoveryPilot):
    def __init__(self,folder):
        self.folder=Path(folder)
        self.cfg=json.loads((self.folder/'pilot.json').read_text())
        self.descriptions=json.loads((self.folder/'geometry.json').read_text())
        self.poses=self.cfg['poses']
        self._unsafe=False

    def build_scene(self,embodiment):
        from isaaclab_arena.assets.object import Object,ObjectType
        from isaaclab_arena.assets.object_reference import ObjectReference
        from isaaclab_arena.utils.pose import Pose
        self._temporary=tempfile.TemporaryDirectory(prefix='clearance-recovery-')
        result=[]
        for name,path in export_assets(self.folder,self._temporary.name).items():
            bodies=self.descriptions[name]['bodies']
            multi=len(bodies)>1
            articulation=self.descriptions[name].get('articulation',False)
            kind=ObjectType.BASE if multi or bodies[0].get('static',False) else ObjectType.RIGID
            if articulation: kind=ObjectType.ARTICULATION
            obj=Object(name=name,object_type=kind,usd_path=str(path),
                       initial_pose=Pose(position_xyz=tuple(self.poses[name]),
                           rotation_xyzw=tuple(self.cfg.get('orientations',{}).get(name,[0,0,0,1]))))
            result.append(obj)
            if multi and not articulation:
                for part in bodies:
                    if not part.get('static',False):
                        result.append(ObjectReference(parent_asset=obj,name=name+'_'+part['name'],
                            prim_path=obj.get_prim_path()+'/'+part['name'],object_type=ObjectType.RIGID))
        return result

    def initial_action(self):
        action=super().initial_action()
        action[6]=self.cfg.get('initial_grip',0.)
        return action

    def reset(self,base):
        super().reset(base)
        if 'prepare_grip' in self.cfg:
            from yam_arena.constants import FINGER_JOINTS,FINGER_OPEN
            reset_joints(base,'left_robot',dict.fromkeys(FINGER_JOINTS,self.cfg['prepare_grip']*FINGER_OPEN))

    def pose_state(self,base,name,part=None):
        if self.descriptions[name].get('articulation',False):
            return pose_state(base.scene[name],part)
        return pose_state(base.scene[name+'_'+part] if part is not None else base.scene[name])

    def states(self,base):
        from yam_arena.urdf import quat_xyzw_to_matrix
        state={}
        for name,description in self.descriptions.items():
            for part in description['bodies']:
                if part.get('static',False): continue
                multi=len(description['bodies'])>1
                p,q,v=self.pose_state(base,name,part['name'] if multi else None)
                p,q,v=(a.cpu().numpy() for a in (p,q,v))
                if not np.isfinite(np.r_[p,q,v]).all() or abs(np.linalg.norm(q)-1)>.001:
                    raise RuntimeError('Invalid native object state')
                state[name+'/'+part['name'] if multi else name]=(p,quat_xyzw_to_matrix(q),v)
        return state

    def validate_reset(self,base):
        from yam_arena.runtime import tcp_world
        if np.linalg.norm(tcp_world(base.scene['left_robot'])-self.cfg['initial_tcp'])>.015:
            raise RuntimeError('Left hand did not settle at the prepared start')
        for key,(position,_,_) in self.states(base).items():
            name,_,part=key.partition('/')
            expected=np.array(self.poses[name],dtype=float)
            if part:
                expected+=next(b for b in self.descriptions[name]['bodies'] if b['name']==part).get('position',[0,0,0])
            if np.linalg.norm(position-expected)>.015:
                raise RuntimeError(f'{key} moved more than 15 mm during settling')
        if self.cfg['kind']=='controlled_lid':
            _,r,_=self.states(base)['cover/lid']
            if math.atan2(r[0,2],r[0,0])<.20:
                raise RuntimeError('Cover collapsed before the policy began')

    def evaluate(self,base):
        from yam_arena.runtime import tcp_world
        cfg=self.cfg; state=self.states(base)
        left=tcp_world(base.scene['left_robot']);right=tcp_world(base.scene['right_robot'])
        park_error=float(np.linalg.norm(left-cfg['park_tcp']))
        violation=False; max_tilt=0.; displacement=0.
        for name in cfg.get('protected',[]):
            p,r,_=state[name]
            distance=float(np.linalg.norm(p-np.array(cfg['protected'][name])))
            tilt=math.acos(float(np.clip(r[2,2],-1,1)))
            displacement=max(displacement,distance);max_tilt=max(max_tilt,tilt)
            violation |= distance>.025 or tilt>.20
        for p,_,_ in state.values():
            violation |= p[2]<cfg['table_top']-.005 or p[2]>1.4 or abs(p[1])>.65 or not -.19<p[0]<.91
        mode=cfg['kind']; progress=0.; diagnostics={}
        if mode=='hooked_handle':
            p,_,v=state['tray']
            endpoint=np.linalg.norm(v[:3])<.02
            progress=float(park_error<.06)
        elif mode=='crosswise_tool':
            p,r,v=state['tool']
            corners=np.array([[x,y,z] for x in (-.068,.140) for y in (-.018,.018) for z in (-.018,.018)])@r.T+p
            violation |= p[2]<cfg['table_top']+.025 or v[2]<-.35
            cleared=float(corners[:,1].min())>cfg['gate_y']+.04
            retained=np.linalg.norm(p-left)<.08
            endpoint=cleared and retained and np.linalg.norm(v[:3])<.02
            progress=float(cleared and retained)
        elif mode=='low_clearance_load':
            p,r,v=state['load']
            violation |= v[2]<-.35
            diagnostics['downward_speed_m_s']=max(0.,float(-v[2]))
            supported=abs(p[2]-cfg['support_top'])<.006 and r[2,2]>.98 and np.linalg.norm(v[:3])<.02
            placed=abs(p[0]-cfg['load_target'][0])<.025 and abs(p[1]-cfg['load_target'][1])<.025
            progress=float(supported and placed)
            endpoint=bool(progress) and min(np.linalg.norm(left-p),np.linalg.norm(right-p))>.09
        elif mode=='controlled_lid':
            p,r,v=state['cover/lid']
            angle=math.atan2(r[0,2],r[0,0])
            violation |= -v[4]>cfg['closing_speed_limit']
            max_tilt=abs(angle)
            diagnostics.update(lid_angle_rad=float(angle),closing_speed_rad_s=max(0.,float(-v[4])))
            closed=abs(angle)<.04 and np.linalg.norm(v[3:])<.10
            center=p+r@np.array([-.08,0,.03])
            progress=float(closed)
            endpoint=closed and min(np.linalg.norm(left-center),np.linalg.norm(right-center))>.10
        elif mode=='counterhold_drawer':
            bp,br,bv=state['organizer/base'];p,r,v=state['organizer/drawer']
            relative=br.T@(p-bp)
            diagnostics['drawer_travel_m']=float(relative[1])
            violation |= abs(relative[0])>.003 or abs(relative[2])>.003 or not -.003<relative[1]<.133
            progress=float(relative[1]>.05)
            endpoint=relative[1]>cfg['drawer_travel'] and max(np.linalg.norm(v[:3]),np.linalg.norm(bv[:3]))<.02
        elif mode=='spring_clamp':
            p,r,v=state['tool'];bp,br,_=state['clamp/base']
            _,lever_r,_=state['clamp/lever'];relative=br.T@lever_r
            diagnostics['jaw_angle_rad']=math.atan2(relative[0,2],relative[0,0])
            violation |= v[2]<-.35 or p[2]<cfg['table_top']+.025
            corners=np.array([[x,y,z] for x in (-.068,.140) for y in (-.018,.018) for z in (-.018,.018)])@r.T+p
            local=(corners-bp)@br
            cleared=local[:,0].max()<-.065 or local[:,1].min()>.05 or local[:,1].max()<-.05
            progress=float(cleared)
            grasp=self.left_grasp_verified(base)
            diagnostics['left_grasp_verified']=bool(grasp)
            endpoint=cleared and grasp and np.linalg.norm(v[:3])<.02
        else:
            raise ValueError(f'Unimplemented recovery mode: {mode}')
        self._unsafe |= bool(violation)
        success=bool(park_error<.025 and endpoint and not self._unsafe)
        return dict(success=success,score=0. if self._unsafe else 1. if success else .5*progress,
                    unsafe_event=self._unsafe,park_error_m=park_error,recovery_progress=progress,
                    object_tilt_rad=max_tilt,protected_displacement_m=displacement,**diagnostics)
