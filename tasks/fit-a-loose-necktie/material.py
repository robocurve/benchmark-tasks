"""Reduced ribbon-threading and sliding-friction model of a pre-wrapped necktie.

The loose wide blade must pass through the open front sleeve. This is not
a full cloth solver. Only after that passage does the prepared wrap keep its
threaded topology. Closing fingers can capture visible material, and relative motion
between a held knot and wide blade snugs the wrap. Holding the narrow blade
then allows the knot to slide up; pulling the wide blade cannot perform that step.
The neckband, wrap and two tails share a fixed 1.26 m material budget. Pulling
one unsupported blade moves the assembly instead of tightening the wrap.
The knot stays on the front plane of the form, like a guided draping model.
No target pose, score, action label or elapsed trial time drives this model.
"""
from dataclasses import dataclass, field
import numpy as np
from ribbon_thread import LooseBlade,N

NECK=np.array([.40,0.,1.25]);RADIUS=.064;LENGTH=1.26
KNOT_X=.319;KNOT_Z=1.09;WIDE_LENGTH=.25;WRAP_LENGTH=.32

def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

def neck_length(z,y):
    # The band follows 270 degrees of the collar, then descends from its front
    # sides. It must not take a straight shortcut through the neck cylinder.
    return 1.5*np.pi*RADIUS+sum(np.linalg.norm(np.array([.40-RADIUS/np.sqrt(2),sign*RADIUS/np.sqrt(2),1.25])-
                            np.array([KNOT_X+.009,y,z+.020])) for sign in (-1,1))

@dataclass
class Tie:
    z: float=KNOT_Z
    y: float=.025
    drawn: float=0.
    wide_direction: np.ndarray=field(default_factory=lambda:np.array([0.,0.,-1.]))
    narrow_direction: np.ndarray=field(default_factory=lambda:np.array([0.,0.,-1.]))
    wide_twist: float=0.
    grips: dict=field(default_factory=dict)
    failed: bool=False
    released_s: float=0.
    events: list=field(default_factory=list)
    reaction_forces: dict=field(default_factory=dict)
    sliding_work_j: float=0.
    threaded: bool=False
    wide_base: float=.33
    wrap_base: float=.24
    loose: object=None

    def __post_init__(self):
        if self.loose is None:
            self.loose=LooseBlade(self.center)
            if not self.threaded:
                for _ in range(80):self.loose.advance()


    @property
    def slack(self):return max(0.,.05-self.drawn)
    @property
    def half_height(self):return .026+.05*self.slack
    @property
    def center(self):return np.array([KNOT_X,self.y,self.z])
    @property
    def wide_length(self):return self.wide_base+self.drawn
    @property
    def narrow_length(self):return LENGTH-neck_length(self.z,self.y)-(self.wrap_base-self.drawn)-self.wide_length
    @property
    def wide_root(self):return self.center+[-.006,0.,-self.half_height] if self.threaded else self.loose.root
    @property
    def narrow_root(self):return self.center+[.008,0.,-self.half_height]

    def surfaces(self):
        """Visible sample points used for contact, independent of the policy."""
        angle=np.linspace(0,2*np.pi,64,endpoint=False);rx=.018+.06*self.slack;ry=.026+.22*self.slack
        knot=np.array([self.center+[rx*np.cos(t),ry*np.sin(t),h] for h in [-.012,0.,.012] for t in angle])
        if not self.threaded:
            points,distances,ids=self.loose.samples()
            return knot,np.repeat(points,5,axis=0)+np.tile([[0,y,0] for y in [-.012,-.006,0,.006,.012]],(len(points),1)),np.repeat(distances,5)
        # Material coordinate is distance from the tip, retained by a grasp.
        samples=[];coordinates=[]
        for distance in np.linspace(.02,max(.021,self.wide_length-.03),48):
            c=self.wide_root+self.wide_direction*(self.wide_length-distance)
            for across in [-.012,-.006,0.,.006,.012]:
                samples.append(c+[0,across,0]);coordinates.append(distance)
        return knot,np.array(samples),np.array(coordinates)

    def narrow_surface(self):
        coords=np.linspace(.015,max(.016,self.narrow_length-.018),32)
        samples=[];dist=[]
        for d in coords:
            c=self.narrow_root+self.narrow_direction*(self.narrow_length-d)
            for across in [-.010,0.,.010]:samples.append(c+[0,across,0]);dist.append(d)
        return np.array(samples),np.array(dist)

    def update(self,hands,dt=.05):
        self.reaction_forces={side:np.zeros(3) for side in hands}
        if self.failed:
            self.grips.clear();self.released_s=0.;return
        knot,wide,coord=self.surfaces();narrow,ncoord=self.narrow_surface()
        for side,h in hands.items():
            gap=h['opening']*.094
            if gap>.040:
                if side in self.grips:self.events.append({'event':'release','side':side,'part':self.grips[side]['part']})
                self.grips.pop(side,None);continue
            if side in self.grips or gap>.030:continue
            candidates=[]
            for part,points in [('knot',knot),('wide',wide),('narrow',narrow)]:
                local=(points-h['tcp'])@h['rotation']
                hit=(abs(local[:,0])<=gap/2+.0015)&(abs(local[:,1])<.012)&(local[:,2]>-.028)&(local[:,2]<.012)
                for i in np.flatnonzero(hit):candidates.append((np.linalg.norm(local[i]-[0,0,-.008]),part,int(i),local[i]))
            if not candidates:continue
            _,part,i,local=min(candidates,key=lambda x:x[0]);point={'knot':knot,'wide':wide,'narrow':narrow}[part][i]
            root=self.wide_root if part=='wide' else self.narrow_root
            direction=self.wide_direction if part=='wide' else self.narrow_direction
            length=self.wide_length if part=='wide' else self.narrow_length
            coordinates=coord if part=='wide' else ncoord
            self.grips[side]=dict(part=part,local=local.copy(),last=point.copy(),distance=float(coordinates[i]) if part!='knot' else None,
                offset=point-self.center if part=='knot' else point-(root+direction*(length-coordinates[i])),
                last_angle=float(np.arctan2(h['rotation'][1,0],h['rotation'][0,0])))
            if part=='wide' and not self.threaded:
                node=int(np.clip(round((.33-self.grips[side]['distance'])/(.33/(N-1))),1,N-1))
                self.grips[side]['offset']=point-self.loose.points[node]
            self.events.append(dict(event='grasp',side=side,part=part))
        motions={}
        for side,g in list(self.grips.items()):
            h=hands[side];target=h['tcp']+h['rotation']@g['local'];delta=target-g['last'];g['last']=target
            # A missed or dragged-off grip slips rather than teleporting cloth.
            if np.linalg.norm(delta)>.035 or abs(target[0]-KNOT_X)>.12:
                self.grips.pop(side);self.events.append(dict(event='slip',side=side));continue
            motions.setdefault(g['part'],[]).append((delta,target,g,h))
        # Quasistatic, regularized dry sliding resistance. These equal and
        # opposite loads are applied to the native grippers, not a force that
        # moves the material toward a target. The 3 N level is an uncalibrated
        # engineering approximation for a loose polyester knot.
        if motions.get('knot'):
            ks=next(s for s,g in self.grips.items() if g['part']=='knot')
            kd=motions['knot'][0][0]
            for part in ('wide','narrow'):
                if motions.get(part):
                    side=next(s for s,g in self.grips.items() if g['part']==part)
                    relative=motions[part][0][0]-kd
                    speed=np.linalg.norm(relative)/dt
                    if speed>1e-9:
                        f=-3.*np.tanh(speed/.002)*relative/np.linalg.norm(relative)
                        self.reaction_forces[side]+=f;self.reaction_forces[ks]-=f
                        self.sliding_work_j+=float(-f@relative)
        if not self.threaded:
            grip=None;wide_motion=motions.get('wide')
            if wide_motion:
                delta,target,g,h=wide_motion[0]
                index=int(np.clip(round((.33-g['distance'])/(.33/(N-1))),1,N-1))
                grip=(index,target-g['offset'])
            complete=self.loose.advance(grip,dt)
            if self.loose.max_stretch>1.15:
                self.failed=True;self.grips.clear();self.events.append(dict(event="threading_constraint_overload"));return
            if complete and grip is not None:
                exit_point=self.center+[-.006,0.,-self.half_height]
                vector=grip[1]-exit_point
                self.wide_base=float(np.linalg.norm(vector)+g['distance'])
                self.wrap_base=.57-self.wide_base
                self.wide_direction=vector/np.linalg.norm(vector)
                self.threaded=True
                self.events.append(dict(event='wide_blade_passed_through_front_loop',free_length_m=self.wide_base))
            self.released_s=self.released_s+dt if not self.grips else 0.
            return
        dz=0.
        if motions.get('knot'):
            delta=np.mean([v[0] for v in motions['knot']],axis=0)
            old=self.z
            # The narrow blade is the standing part on which the knot slides.
            # Holding the wide blade only allows snugging of its outer wrap.
            if motions.get('narrow'):
                self.z=float(np.clip(self.z+delta[2],.94,1.177))
            dz=self.z-old
            self.y=float(np.clip(self.y+delta[1],-.10,.10))
        if motions.get('wide'):
            delta,target,g,h=motions['wide'][0]
            if motions.get('knot'):
                # Relative sliding feed. A reversed pull loosens the wrap;
                # there is no monotonic progress flag or automatic tightening.
                self.drawn=float(np.clip(self.drawn-delta[2],-.245,.08))
            else:
                self.z=float(np.clip(self.z+min(0.,delta[2]),.94,1.177))
                self.drawn-=max(0.,delta[2])
                self.y=float(np.clip(self.y+delta[1],-.10,.10))
            vector=target-self.wide_root
            if np.linalg.norm(vector)>.01:self.wide_direction=vector/np.linalg.norm(vector)
            # Jaw plane records axial twist; the visible ribbon carries it.
            normal=h['rotation'][:,0];angle=float(np.arctan2(normal[1],normal[0]));delta_angle=(angle-g['last_angle']+np.pi)%(2*np.pi)-np.pi
            self.wide_twist+=delta_angle;g['last_angle']=angle
        else:
            # Quasistatic gravitational drape of the released blade; does not
            # tighten the wrap, move the knot or select the desired score.
            u=1-np.exp(-dt*5.);self.wide_direction=(1-u)*self.wide_direction+u*np.array([0.,0.,-1.]);self.wide_direction/=np.linalg.norm(self.wide_direction)
        if motions.get('narrow'):
            delta,target,g,h=motions['narrow'][0]
            if not motions.get('knot'):
                # An unsupported narrow-end pull moves the knot down rather
                # than gifting a fitted outcome.
                self.z=float(np.clip(self.z+min(0.,delta[2]),.94,1.177))
            vector=target-self.narrow_root
            if np.linalg.norm(vector)>.01:self.narrow_direction=vector/np.linalg.norm(vector)
        else:
            u=1-np.exp(-dt*5.);self.narrow_direction=(1-u)*self.narrow_direction+u*np.array([0.,0.,-1.]);self.narrow_direction/=np.linalg.norm(self.narrow_direction)
        if self.wide_length<.035 or self.narrow_length<.025:
            self.failed=True;self.events.append(dict(event='end_pulled_out'));self.grips.clear()
        for side,g in list(self.grips.items()):
            if g['part']=='knot':visible=self.center+g['offset']
            else:
                part=g['part'];root=getattr(self,part+'_root');direction=getattr(self,part+'_direction');length=getattr(self,part+'_length')
                visible=root+direction*(length-g['distance'])+g['offset']
            if np.linalg.norm(visible-g['last'])>.025:
                self.grips.pop(side);self.events.append(dict(event='material_slipped',side=side))
        self.released_s=self.released_s+dt if not self.grips else 0.

    def measures(self):
        wide_tip=self.wide_root+self.wide_direction*self.wide_length if self.threaded else self.loose.points[-1]
        narrow_tip=self.narrow_root+self.narrow_direction*self.narrow_length
        return dict(threaded=self.threaded,threading_entered=self.loose.entered,threading_exited=self.loose.exited,threading_max_stretch=self.loose.max_stretch,knot_width_mm=(.052+.44*self.slack)*1000,
            knot_height_mm=2*self.half_height*1000,knot_top_below_collar_mm=max(0.,1.2-self.z-self.half_height)*1000,
            knot_offset_mm=abs(self.y)*1000,wide_tip_below_collar_mm=(1.2-wide_tip[2])*1000,
            narrow_tip_above_wide_mm=(narrow_tip[2]-wide_tip[2])*1000,narrow_lateral_offset_mm=abs(narrow_tip[1]-wide_tip[1])*1000,
            wide_twist_deg=abs(np.degrees(self.wide_twist)),released_s=self.released_s,
            failed=self.failed,sliding_work_j=self.sliding_work_j,drawn_mm=self.drawn*1000,neckband_length_mm=neck_length(self.z,self.y)*1000,
            material_length_m=neck_length(self.z,self.y)+self.wrap_base-self.drawn+self.wide_length+self.narrow_length)

def score(m):
    if not m['threaded'] or m['failed'] or m['released_s']<5.-1e-8:return 0.
    if m['knot_width_mm']>70. or m['knot_height_mm']>70. or m['knot_top_below_collar_mm']>60.:return 0.
    fitted=(m['knot_top_below_collar_mm']<=20. and m['knot_offset_mm']<=20.
            and 350.<=m['wide_tip_below_collar_mm']<=440.
            and m['narrow_tip_above_wide_mm']>=10. and m['wide_twist_deg']<=30. and m['narrow_lateral_offset_mm']<=15.)
    return 1. if fitted else .5
