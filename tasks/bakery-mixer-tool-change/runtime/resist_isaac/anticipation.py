"""Passive anticipation prototypes. Native feasibility is not established by this code."""
import json
import math
from pathlib import Path

import numpy as np

from .clearance_recovery import ClearanceRecovery


def body_bounds(description, position, rotation):
    """World AABB of the authored boxes, spheres and axial cylinders, not just origins."""
    limits=[]
    for shape in description['shapes']:
        if not shape.get('collision',True): continue
        if 'orientation' in shape or 'fromto' in shape:
            raise ValueError('These prototypes use body-aligned collision primitives')
        centre=position+rotation@np.asarray(shape.get('position',[0,0,0]))
        size=np.asarray(shape['size']);kind=shape['type']
        if kind=='box': extent=np.abs(rotation)@size
        elif kind=='sphere': extent=np.full(3,size[0])
        elif kind=='cylinder': extent=size[0]*np.sqrt(rotation[:,0]**2+rotation[:,1]**2)+size[1]*np.abs(rotation[:,2])
        else: raise ValueError(kind)
        limits.append((centre-extent,centre+extent))
    return np.min([lo for lo,_ in limits],axis=0),np.max([hi for _,hi in limits],axis=0)


def point_box_distance(point, low, high):
    return float(np.linalg.norm(np.maximum(np.maximum(low-point,point-high),0)))


class AnticipationTask(ClearanceRecovery):
    """Reuse native asset/reset/state access, with the station's unchanged top camera."""
    def __init__(self,folder):
        # Deliberately do not call the legacy constructor that installs an oblique camera.
        self.folder=Path(folder)
        self.cfg=json.loads((self.folder/'pilot.json').read_text())
        self.descriptions=json.loads((self.folder/'geometry.json').read_text())
        self.poses=self.cfg['poses']
        self._unsafe=False
        self._previous=None
        self.hold_s=1.0

    def initial_action(self):
        from yam_arena.constants import READY
        return np.r_[READY,1.,READY,1.]

    def _part(self,key):
        name,_,part=key.partition('/')
        bodies=self.descriptions[name]['bodies']
        return next(b for b in bodies if b['name']==part) if part else bodies[0]

    def _bounds(self,state,key,frame=None):
        p,r,_=state[key]
        if frame is not None:
            fp,fr,_=state[frame];p=fr.T@(p-fp);r=fr.T@r
        return body_bounds(self._part(key),p,r)

    def _guide(self,state):
        cfg=self.cfg;key=cfg['guide_body'];p,r,_=state[key]
        displacement=p-np.asarray(cfg['guide_origin']);axis=cfg['guide_axis']
        off=np.delete(displacement,axis)
        angle=math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
        q=float(displacement[axis]);lower,upper=cfg['guide_limits']
        if np.linalg.norm(off)>.002 or angle>.025 or not lower-.002<=q<=upper+.002:
            raise RuntimeError('Native prismatic guide is invalid; do not count this as a policy trial')
        return q

    def validate_reset(self,base):
        state=self.states(base);self._guide(state)
        for key,(p,r,v) in state.items():
            name,_,part=key.partition('/')
            expected=np.asarray(self.poses[name])+np.asarray(self._part(key).get('position',[0,0,0]))
            if np.linalg.norm(p-expected)>.01 or np.linalg.norm(v[:3])>.03:
                raise RuntimeError(f'Unstable unassisted reset: {key}; position={p.tolist()}, expected={expected.tolist()}, velocity={v.tolist()}')
        if self._unsafe: raise RuntimeError('Reset already violates safety conditions')

    def begin_trial(self,base):
        """Read the settled starting state before the first action; never move objects."""
        self.validate_reset(base)
        self._previous=self.states(base)

    def evaluate(self,base):
        from yam_arena.runtime import tcp_world
        if abs(base.step_dt-1/self.cfg['control_hz'])>1e-8:
            raise ValueError('These anticipation scorers require 20 Hz observations')
        return self.score_states(self.states(base),[tcp_world(base.scene[s+'_robot']) for s in ('left','right')])

    def score_states(self,state,tcps):
        """Pure measurement logic; unit tests of this function are not physics trials."""
        cfg=self.cfg;top=cfg['table_top'];q=self._guide(state)
        for p,r,v in state.values():
            if not np.isfinite(np.r_[p,r.ravel(),v]).all():raise RuntimeError('Invalid native state')
            self._unsafe |= bool(p[2]<top-.01 or p[2]>1.45 or abs(p[1])>.64 or not -.18<p[0]<.90)
        if cfg['kind']=='capture_discharge':
            return self._capture(state,tcps,q)
        if cfg['kind']=='tool_balancer':
            return self._balancer(state,tcps,q)
        raise ValueError(cfg['kind'])

    def _clear(self,state,tcps,keys):
        return all(point_box_distance(np.asarray(tcp),*self._bounds(state,key))>.04 for key in keys for tcp in tcps)

    def _capture(self,state,tcps,q):
        cfg=self.cfg;cp,cr,cv=state['cup'];radius=cfg['ball_radius'];contained=0
        for name in cfg['balls']:
            p,_,_=state[name];local=cr.T@(p-cp)
            inside=(np.all(np.abs(local[:2])+radius<=np.asarray(cfg['cup_inner_half_xy'])+.0001)
                    and cfg['cup_floor_top']-.0001<=local[2]-radius
                    and local[2]+radius<=cfg['cup_rim_height']+.0001)
            contained+=int(inside)
            # A ball touching bare tabletop has escaped; later recovery cannot erase it.
            self._unsafe |= bool(p[2]-radius<=cfg['table_top']+.0005 and not inside)
        still=np.linalg.norm(cv[:3])<.02 and np.linalg.norm(cv[3:])<.15
        cup_low,_=self._bounds(state,'cup')
        parked=(np.linalg.norm(cp[:2]-np.asarray(cfg['return_xy']))<=.02
                and abs(cup_low[2]-cfg['pad_top'])<=.003 and cr[2,2]>=math.cos(math.radians(10)) and still)
        balls_still=all(np.linalg.norm(state[n][2][:3])<.03 for n in cfg['balls'])
        success=(contained==len(cfg['balls']) and parked and balls_still and abs(q)<.004
                 and self._clear(state,tcps,['cup',cfg['guide_body']]) and not self._unsafe)
        score=0. if self._unsafe else 1. if success else .5*contained/len(cfg['balls'])
        return dict(success=bool(success),score=score,unsafe_event=self._unsafe,
                    contained_balls=contained,gate_travel_m=-q,cup_parked=bool(parked))

    def _balancer(self,state,tcps,q):
        cfg=self.cfg;hp,_,hv=state[cfg['guide_body']];tp,tr,tv=state['tool']
        upward=downward=0.
        # Score observable 50 ms motion; native contact impulses can report velocity
        # even for a clamped guide whose actual position does not change.

        if self._previous is not None:
            upward=max(upward,float(hp[2]-self._previous[cfg['guide_body']][0][2])*cfg['control_hz'])
            downward=max(downward,float(self._previous['tool'][0][2]-tp[2])*cfg['control_hz'])
        self._previous={key:tuple(value.copy() for value in values) for key,values in state.items()}
        self._unsafe |= upward>cfg['max_hook_upward_speed'] or downward>cfg['max_tool_downward_speed']
        lo,hi=body_bounds(self._part('tool'),tp-np.asarray(self.poses['fixture']),tr)
        stored=(np.all(lo>=np.asarray(cfg['tray_inner_low'])-.0002)
                and np.all(hi<=np.asarray(cfg['tray_inner_high'])+.0002)
                and np.linalg.norm(tv[:3])<.02 and np.linalg.norm(tv[3:])<.15)
        # Minimum tool-to-hook AABB gap rules out a still-attached hanging tool.
        tlo,thi=self._bounds(state,'tool');hlo,hhi=self._bounds(state,cfg['guide_body'])
        gap=float(np.linalg.norm(np.maximum(np.maximum(tlo-hhi,hlo-thi),0)))
        detached=gap>.06
        retracted=q>=-.006 and np.linalg.norm(hv[:3])<.02
        success=(stored and detached and retracted and self._clear(state,tcps,['tool',cfg['guide_body']]) and not self._unsafe)
        progress=.5 if stored and detached else .25 if detached and retracted else 0.
        score=0. if self._unsafe else 1. if success else progress
        return dict(success=bool(success),score=score,unsafe_event=self._unsafe,
                    hook_extension_m=-q,hook_upward_speed_m_s=upward,
                    native_hook_velocity_m_s=float(hv[2]),native_tool_velocity_m_s=float(tv[2]),
                    tool_downward_speed_m_s=downward,tool_stored=bool(stored),tool_detached=bool(detached))
