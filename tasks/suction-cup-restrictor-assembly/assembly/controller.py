"""Privileged known-layout feasibility script; outputs ordinary joint commands."""
import numpy as np
BODY_EVIDENCE=None
DIAGNOSTICS=dict(max_tcp_fit_error_mm=0.,clipped_corrections=0)
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from . import geometry as G


def install(plan):
    from yam_arena.scripted import YamScriptedPolicy
    from yam_arena import ik,station
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import pack,STATE_KEY
    original=YamScriptedPolicy.reset
    def reset(self,scene):
        original(self,scene);self._i=0;self._bias=np.zeros(14);self._previous=None;self._grasps={}
        DIAGNOSTICS.update(max_tcp_fit_error_mm=0.,clipped_corrections=0)
    def act(self,observation):
        row=np.array(plan[min(self._i,len(plan)-1)]['action'],float)
        measured=np.asarray(observation.state[STATE_KEY],float)
        # Privileged object poses are allowed only for this known-layout
        # feasibility script. Commands still act through the original joints;
        # no object is attached, teleported, driven, or given a scripted force.
        goals=plan[min(self._i,len(plan)-1)].get('scripted_body_goals',{})
        if goals:
            if BODY_EVIDENCE is None or BODY_EVIDENCE() is None:raise RuntimeError('Missing scripted physical evidence')
            paths,poses=BODY_EVIDENCE()
            for side,start in [('left',0),('right',7)]:
                if side not in goals:continue
                goal=goals[side];idx=next(k for k,p in enumerate(paths) if p.rsplit('/',1)[-1]==G.PREFIX+goal['name'])
                body=poses[idx];ro=Rotation.from_quat(body[3:]).as_matrix()
                tcp,rg=ik.tcp_pose(measured[start:start+6],station.BASES[side])
                # Capture the grasp once. A slipping/blocked part must not make
                # the controller continually advance the desired TCP through it.
                key=(side,goal['name'])
                may_refine=goal.get('maintain_seated_pose',False) and np.linalg.norm(body[:3]-np.asarray(goal['position']))<.002
                if key not in self._grasps or may_refine:self._grasps[key]=(ro.T@(tcp-body[:3]),ro.T@rg)
                offset,relative=self._grasps[key]
                if side=='right' and goal.get('centre_neck',False):
                    # Slip transverse to the insertion axis must not leave the
                    # neck pressing against the bore edge. Use measured grasp
                    # feedback only in X/Z; preserve the captured axial offset
                    # so a blocked cup cannot make the fingers creep through it.
                    fresh=ro.T@(tcp-body[:3])
                    offset=offset.copy()
                    offset[[0,2]]+=np.clip(fresh[[0,2]]-offset[[0,2]],-.002,.002)
                    change=Rotation.from_matrix(relative.T@(ro.T@rg)).as_rotvec()
                    change*=min(1.,np.deg2rad(5)/max(np.linalg.norm(change),1e-9))
                    relative=relative@Rotation.from_rotvec(change).as_matrix()
                target_r=np.asarray(goal['rotation'])@relative
                target_origin=np.asarray(goal['position']).copy()
                target_p=target_origin+np.asarray(goal['rotation'])@offset
                def residual(q):
                    pp,rr=ik.tcp_pose(q,station.BASES[side])
                    return np.r_[5*(pp-target_p),Rotation.from_matrix(target_r.T@rr).as_rotvec()]
                # Bound each correction from the actual arm configuration,
                # not cumulative deviation from a stale nominal grasp path.
                seed=measured[start:start+6].copy()
                fit=least_squares(residual,np.clip(seed,ik.LOWER+1e-7,ik.UPPER-1e-7),bounds=(ik.LOWER+1e-7,ik.UPPER-1e-7),max_nfev=100)
                if not np.isfinite(fit.x).all():raise RuntimeError('Nonfinite scripted joint target')
                DIAGNOSTICS['max_tcp_fit_error_mm']=max(DIAGNOSTICS['max_tcp_fit_error_mm'],float(np.linalg.norm(residual(fit.x)[:3])/5*1000))
                # Contact can rotate a held part without making its target
                # unreachable. Approach that target in bounded joint increments
                # instead of aborting on a large IK correction near a singularity.
                correction=fit.x-seed
                if np.max(abs(correction))>.08:
                    self._bias[start:start+6]=0.
                    DIAGNOSTICS['clipped_corrections']+=1
                row[start:start+6]=seed+np.clip(correction,-.08,.08)

        pressing_core=plan[min(self._i,len(plan)-1)].get('no_integral_left',False)
        if pressing_core:self._bias[:6]=0.
        if self._previous is not None:
            for start in (0,7):
                if start==0 and pressing_core:continue
                s=slice(start,start+6)
                if np.max(abs(row[s]-self._previous[s]))<.01:
                    self._bias[s]=np.clip(self._bias[s]+.15*(self._previous[s]-measured[s]),-.25,.25)
        command=row+self._bias
        for start in (0,7):command[start:start+6]=np.clip(command[start:start+6],ik.LOWER,ik.UPPER)
        self._previous=row;self._i+=1
        return ActionChunk(actions=[Action(data=pack(command[:7],command[7:]))],control_hz=20)
    YamScriptedPolicy.reset=reset;YamScriptedPolicy.act=act
