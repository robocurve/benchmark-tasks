"""Contact-history scoring for clean physical scooping and one-ball serving."""
from dataclasses import asdict, dataclass
import math
import numpy as np


def rotation(q):
    q=np.asarray(q,dtype=float);q=q/np.linalg.norm(q)
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


@dataclass
class BodyState:
    pose: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    low: np.ndarray
    high: np.ndarray

    @property
    def center(self):
        return (self.low+self.high)/2


@dataclass(frozen=True)
class Rules:
    plate_center_radius_m: float = .100
    plate_floor_tolerance_m: float = .003
    source_center_radius_m: float = .105
    minimum_lift_above_rim_m: float = .005
    scoop_support_s: float = .25
    recent_contact_s: float = .10
    release_no_contact_s: float = .20
    maximum_speed_m_s: float = .010
    maximum_spin_rad_s: float = .5
    tcp_release_distance_m: float = .060
    hold_s: float = 1.0


class TaskScorer:
    def __init__(self, layout, *, hz=20, rules=None):
        self.rules=rules or Rules();self.hz=hz
        self.table=float(layout['table_top_z'])
        self.plate=np.array(layout['objects']['Plate']['position'],float)
        self.source=np.array(layout['objects']['MeatballBowl']['position'],float)
        self.rim=self.table+layout['meatball_bowl']['rim_height_m']
        shape=layout['spoon_bowl']
        self.cavity_radii=np.array([shape['inner_length_m'],shape['inner_width_m']])/2+.004
        self.cavity_power=shape.get('footprint_power',2)
        self.balls=tuple(sorted(n for n in layout['objects'] if n.startswith('Meatball_')))
        self.tools=('CleanSpoon','UsedSpoon')
        self.sources={'UsedSpoon','RawPlate',*(f'RawChicken_{i}' for i in range(1,6))}
        self.reset()

    def configuration(self):
        return {**asdict(self.rules),'cavity_radii_m':self.cavity_radii.tolist(),
                'cavity_footprint_power':self.cavity_power,'source_rim_world_z':self.rim}

    def reset(self):
        self.step=0;self.dirty=set(self.sources);self.handled=set();self.transported=set()
        self.last_spoon_contact={n:-100000 for n in self.balls}
        self.last_plate_contact={n:-100000 for n in self.balls}
        self.last_tool_contact={t:dict.fromkeys(self.balls,-100000) for t in self.tools}
        self.support_steps={t:dict.fromkeys(self.balls,0) for t in self.tools}
        self.transported_by={n:set() for n in self.balls};self.clean_transported=set()
        self.history=[];self._candidate=0.;self._held=0;self.score=0.

    def update(self,bodies,tcps,contacts,*,contact_healthy=True):
        self.step+=1;r=self.rules
        if not all(np.isfinite(t).all() for t in tcps.values()):
            self.score=0.;self._held=0
            return {'score':0.,'success':False,'valid_state':False,'reason':'nonfinite robot state'}
        for b in bodies.values():
            if not all(np.isfinite(v).all() for v in (b.pose,b.linear_velocity,b.angular_velocity,b.low,b.high)) or np.linalg.norm(b.pose[3:])<1e-8:
                self.score=0.;self._held=0
                return {'score':0.,'success':False,'valid_state':False,'reason':'nonfinite body state'}
        pairs={tuple(sorted(p)) for p in contacts}
        # Closed contact graph for this step; contamination persists until reset.
        changed=True
        while changed:
            changed=False
            for a,b in pairs:
                if (a in self.dirty) != (b in self.dirty):
                    self.dirty.update((a,b));changed=True
        carried=[];on_plate=[];in_source=[];spilled=[]
        for name in self.balls:
            ball=bodies[name]
            if tuple(sorted(('Plate',name))) in pairs:
                self.last_plate_contact[name]=self.step
            if any(name in p and ('left_robot' in p or 'right_robot' in p) for p in pairs):
                self.handled.add(name)
            lifted=ball.low[2]>=self.rim+r.minimum_lift_above_rim_m
            supported=False
            for tool in self.tools:
                spoon=bodies[tool];sr=rotation(spoon.pose[3:])
                if tuple(sorted((tool,name))) in pairs:
                    self.last_spoon_contact[name]=self.last_tool_contact[tool][name]=self.step
                local=sr.T@(ball.center-spoon.pose[:3])
                cavity=(np.sum(np.abs((local[:2]-[.054,0])/self.cavity_radii)**self.cavity_power)<=1
                        and .010<=local[2]<=.075 and sr[2,2]>=.60)
                recent=self.step-self.last_tool_contact[tool][name]<=math.ceil(r.recent_contact_s*self.hz)
                eligible=cavity and recent and lifted and name not in self.handled
                self.support_steps[tool][name]=self.support_steps[tool][name]+1 if eligible else 0
                if self.support_steps[tool][name]>=math.ceil(r.scoop_support_s*self.hz) and tool not in self.transported_by[name]:
                    clean=tool not in self.dirty and name not in self.dirty
                    self.transported.add(name);self.transported_by[name].add(tool)
                    if clean:self.clean_transported.add(name)
                    self.history.append({'event':'clean_scoop' if clean else 'dirty_scoop','tool':tool,'ball':name,'step':self.step})
                supported=supported or (cavity and recent and tool in self.transported_by[name])
            if supported:carried.append(name)
            xy=float(np.linalg.norm(ball.center[:2]-self.plate[:2]))
            at_floor=abs(ball.low[2]-(self.plate[2]+.004))<=r.plate_floor_tolerance_m
            if xy<=r.plate_center_radius_m and at_floor:on_plate.append(name)
            source_xy=np.linalg.norm(ball.center[:2]-self.source[:2])<=r.source_center_radius_m
            if source_xy and self.table+.002<=ball.low[2] and ball.center[2]<=self.rim+.040:in_source.append(name)
            on_table=ball.low[2]<=self.table+.003 and np.linalg.norm(ball.linear_velocity)<=r.maximum_speed_m_s
            if (on_table and name not in on_plate and name not in in_source and not supported) or ball.low[2]<self.table-.003:
                spilled.append(name)
        invalid_food=bool(spilled)
        delivered=None;released=False;settled=False;plate_contact=False
        if len(on_plate)==1:
            name=on_plate[0];ball=bodies[name]
            no_tool=self.step-self.last_spoon_contact[name]>math.ceil(r.release_no_contact_s*self.hz)
            released=no_tool and bool(tcps) and all(np.linalg.norm(t-ball.center)>=r.tcp_release_distance_m for t in tcps.values())
            settled=(np.linalg.norm(ball.linear_velocity)<=r.maximum_speed_m_s and np.linalg.norm(ball.angular_velocity)<=r.maximum_spin_rad_s)
            plate_contact=self.step-self.last_plate_contact[name]<=math.ceil(r.recent_contact_s*self.hz)
            if (name in self.transported and name not in self.handled
                    and all(n in in_source for n in self.balls if n!=name) and released and settled and plate_contact):delivered=name
        valid=contact_healthy and not invalid_food
        clean_delivery=bool(delivered and 'CleanSpoon' in self.transported_by[delivered]
                            and all(n not in self.dirty for n in (delivered,'CleanSpoon','Plate','MeatballBowl')))
        raw=(1. if clean_delivery else .5) if valid and delivered else 0.
        self._held=self._held+1 if raw>0 and raw==self._candidate else (1 if raw>0 else 0)
        self._candidate=raw;self.score=raw if self._held>=math.ceil(r.hold_s*self.hz) else 0.
        return {'score':self.score,'success':self.score==1.,'candidate_score':raw,'held_steps':self._held,
                'valid_state':valid,'contact_reports_healthy':bool(contact_healthy),'delivered_ball':delivered,
                'on_plate':on_plate,'in_source':in_source,'currently_carried':carried,
                'clean_scoops':sorted(self.clean_transported),'scooped_by':{n:sorted(t) for n,t in self.transported_by.items() if t},
                'clean_delivery':clean_delivery,'contaminated':sorted(self.dirty),
                'directly_handled':sorted(self.handled),'spilled_food':spilled,
                'ball_released':released,'ball_settled':bool(settled),'plate_contact':bool(plate_contact)}
