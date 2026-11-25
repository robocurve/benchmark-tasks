"""Primary 20 Hz scorer. Uses recorded measurements; no policy or object control."""
import math
import numpy as np
from mechanics import connector_metrics, point, rotation, BODY_PATHS, RECEIVER_STROKE

from layout import VALVE_WORLD_POSITION,RECEIVER_WORLD_POSITION,RECOVERY_DISH_CENTER,RECOVERY_SHAFT_Z_BOUNDS

INITIAL = {'catheter':(.375,-.045,.851),'valve':VALVE_WORLD_POSITION,
           'barrel':RECEIVER_WORLD_POSITION,'plunger':RECEIVER_WORLD_POSITION}

class TrialScorer:
    def __init__(self):
        self.last=0;self.failed=False;self.reason=None;self.connection=False
        self.deflated=False;self.withdrawn=False;self.dwell=0;self.previous_balloon=None
        self.disconnected_after_deflation=False

    def fail(self, reason):
        self.failed=True
        if self.reason is None:self.reason=reason

    def update(self, *, step, objects, root_poses, robot, mechanics=None):
        if step!=self.last+1:raise ValueError('Exactly one consecutive measurement per 20 Hz step required')
        self.last=step
        try:
            p={name:np.asarray(root_poses[name],float) for name in BODY_PATHS}
            if any(v.shape!=(7,) or not np.isfinite(v).all() or abs(np.linalg.norm(v[3:])-1)>.02 for v in p.values()):
                raise ValueError('Finite measured unit-quaternion poses required')
            hands=[]
            for side in ('left','right'):
                tcp=np.asarray(robot[side]['tcp'],float);f=np.asarray(robot[side]['fingers'],float)
                if tcp.shape!=(3,) or f.shape!=(2,) or not np.isfinite(np.r_[tcp,f]).all():raise ValueError('Measured hands required')
                hands.append(tcp)
            m=mechanics or {}
            for key in ('connected','drained_fraction','balloon_radius_m','collapsed','failed'):
                if key not in m:raise ValueError('Missing measured mechanics: '+key)
            if not np.isfinite([m['drained_fraction'],m['balloon_radius_m']]).all():raise ValueError('Nonfinite mechanics')
        except (KeyError,TypeError) as exc:
            raise ValueError('Complete measured state required') from exc
        if step==1 and (np.linalg.norm(p['catheter'][:3]-INITIAL['catheter'])>.025 or
                        abs(connector_metrics(p)['stroke_m'])>.005 or m['connected'] or m['collapsed']):
            self.fail('invalid_initial_configuration')
        if any(v[2]<.710 for v in p.values()):self.fail('dropped_component')
        if m['failed']:self.fail(m.get('failure') or 'mechanism_failure')
        cm=connector_metrics(p)
        forces=m.get('tip_device_forces_N',{})
        readings=[np.asarray(forces.get(side,[]),float) for side in ('left','right')]
        if any(x.shape!=(2,2) or not np.isfinite(x).all() for x in readings):
            raise ValueError('Measured tip forces against valve and receiver are required')
        left,right=readings
        supported=(min(left[:,0])>=.05 and min(right[:,1])>=.05) or (min(right[:,0])>=.05 and min(left[:,1])>=.05)
        if m['connected'] and not self.deflated and not supported:
            self.fail('connection_without_opposing_device_grasps')
        if m['connected'] and cm['aligned'] and abs(cm['stroke_m'])<=.005 and supported:self.connection=True
        collapsed=bool(m['collapsed'] and m['drained_fraction']>=.995 and m['balloon_radius_m']<=.005)
        if collapsed and not self.connection:self.fail('deflation_without_valid_connection')
        if collapsed and self.connection:self.deflated=True
        if not collapsed and self.withdrawn:self.fail('reinflated_after_extraction')
        balloon=point(p['catheter'],np.array([0.,.132,0.]))
        tip=point(p['catheter'],np.array([0.,.168,0.]))
        if balloon[1]<.035 and not collapsed:self.fail('inflated_extraction')
        # The rear neck plane is the first boundary on the outward path.
        # A joined assembly must not enter it, even if disconnected later.
        if balloon[1]<=.062 and (m['connected'] or not self.disconnected_after_deflation):
            self.fail('withdrawal_before_disconnection')
        if self.deflated and collapsed and not m['connected'] and m.get('ever_disconnected'):
            self.disconnected_after_deflation=True
        # Detect a sweep through the neck even if a single frame crosses it.
        if self.previous_balloon is not None:
            old=self.previous_balloon
            for plane in (.016,.062):
                if (old[1]-plane)*(balloon[1]-plane)<=0 and abs(old[1]-balloon[1])>1e-9:
                    t=(plane-old[1])/(balloon[1]-old[1]);cross=old+t*(balloon-old)
                    if np.linalg.norm(cross[[0,2]]-[.375,.851])>.003:self.fail('extraction_outside_neck')
        self.previous_balloon=balloon.copy()
        if tip[1]<=.006:
            if not self.deflated or not collapsed:self.fail('withdrawal_before_deflation')
            else:self.withdrawn=True
        shaft_mid=point(p['catheter'],np.array([0.,.045,0.]))
        axis=rotation(p['catheter'][3:])[:,1]
        # Rigid-shaft approximation rests across the dish; end overhang is
        # allowed explicitly. The core objective is safe clearance, not packing.
        catheter_parked=bool(abs(shaft_mid[0]-RECOVERY_DISH_CENTER[0])<=.025 and abs(shaft_mid[1]-RECOVERY_DISH_CENTER[1])<=.035
                             and RECOVERY_SHAFT_Z_BOUNDS[0]<=shaft_mid[2]<=RECOVERY_SHAFT_Z_BOUNDS[1] and abs(axis[2])<=math.sin(math.radians(10)))
        barrel_parked=bool(np.linalg.norm((p['barrel'][:3]-INITIAL['barrel'])[:2])<=.025
                           and .805<=p['barrel'][2]<=.835)
        points=[point(p['catheter'],np.array([0.,y,0.])) for y in (-.09,0.,.09,.168)]
        points += [point(p['valve'],np.array([0.,.015,.007]))]
        points += [point(p['barrel'],np.array([x,0.,0.])) for x in (-.047,0.,.062)]
        points += [point(p['plunger'],np.array([x,0.,0.])) for x in (-.072,0.,.035)]
        hands_clear=all(np.linalg.norm(h-q)>=.060 for h in hands for q in points)
        stopped=True
        for k in BODY_PATHS:
            obj=objects.get(k)
            if obj is None:raise ValueError('Measured body velocity required')
            velocity=np.asarray(obj['lin_vel'] if isinstance(obj,dict) else obj.lin_vel,float)
            omega=np.asarray(obj['ang_vel'] if isinstance(obj,dict) else obj.ang_vel,float)
            if velocity.shape!=(3,) or omega.shape!=(3,) or not np.isfinite(np.r_[velocity,omega]).all():raise ValueError('Finite measured velocity required')
            stopped &= bool(np.linalg.norm(velocity)<=.015 and np.linalg.norm(omega)<=.2)
        final=bool(self.withdrawn and collapsed and not m['connected'] and catheter_parked
                   and barrel_parked and hands_clear and stopped and not self.failed)
        self.dwell=self.dwell+1 if final else 0
        success=self.dwell>=20
        score=0. if self.failed else (1. if success else .7 if self.withdrawn and collapsed else .4 if collapsed and self.deflated else .1 if self.connection else 0.)
        return {'score':score,'success':success,'failed':self.failed,'failure':self.reason,
                'connection_observed':self.connection,'deflation_observed':self.deflated,
                'withdrawal_observed':self.withdrawn,'catheter_parked':catheter_parked,
                'receiver_parked':barrel_parked,'hands_clear':hands_clear,'held_frames':self.dwell,
                'primary_score_version':3,'opposing_grasps':bool(supported)}
