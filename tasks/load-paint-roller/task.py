"""Load an even coat onto the mini paint roller and unload excess on the tray ramp."""
from __future__ import annotations

from dataclasses import dataclass,field
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from plan import OPEN,TABLE,TIP,Track,check_tracks,waypoints
from runtime import bodies,contacts,link_pose,paint_prim,register,reset_contacts,tick

SCENE="station"
SCORE_OVER="end"
SUCCESS_HOLD_S=1.
MAX_STEPS=1400
INSTRUCTION="Load the paint roller evenly. Roll it through the well and over the ribbed ramp to remove excess paint. Leave it on the tray's flat rest."
TRAY=(.345,-.047,TABLE)
START=(.324,-.225,TABLE+.022)
GRIP_X,GRIP_Y=-.111,-.018
GRIP_WIDTH,GRIP_HEIGHT=.024,.024
FRAME_PITCH=math.radians(22)
GRIP_REACH=GRIP_X*math.cos(FRAME_PITCH)
GRIP_RISE=-GRIP_X*math.sin(FRAME_PITCH)
FRAME_ROT=(0.,math.sin(FRAME_PITCH/2),0.,math.cos(FRAME_PITCH/2))
FRAME_MASS,SLEEVE_MASS=.115,.049
SUPPORT=(START[0]+GRIP_REACH,START[1]+GRIP_Y,TABLE)
SUPPORT_HEIGHT=START[2]-TABLE+GRIP_RISE-GRIP_HEIGHT/(2*math.cos(FRAME_PITCH))
R=.021
BANDS,SECTORS=4,12
MIN_SECTORS=10
WELL=(.004,.153)
FRONT_WALL=.156
WELL_FAR=FRONT_WALL-R-.003
RAMP=(-.150,0.)
REST=-.179
PARK_SUPPORT=(TRAY[0]+REST-.173*math.cos(FRAME_PITCH),TRAY[1]+GRIP_Y,TABLE)
PARK_SUPPORT_HEIGHT=.068
RAMP_HEIGHT=.031
WELL_BOTTOM=.006
BAND_CENTERS=(-.0375,-.0125,.0125,.0375)
WET=(.22,.49,.63)
CLEAN=(.91,.91,.87)


@dataclass
class Sample:
    x: float
    y: float
    bottom: float
    angle: float
    level: bool
    held: bool
    rotation: np.ndarray=field(default_factory=lambda: np.eye(3))


@dataclass
class Trial:
    wet: np.ndarray=field(default_factory=lambda: np.zeros((BANDS,SECTORS),dtype=bool))
    prior: Sample|None=None
    conditioning: float=0.
    hold: float=0.
    invalid: bool=False
    park_pose: Sample|None=None

    def step(self,s,dt):
        if s.bottom < -.025:
            self.invalid=True
        if self.prior is not None and s.level and s.held:
            dx=s.x-self.prior.x
            da=math.atan2(math.sin(s.angle-self.prior.angle),math.cos(s.angle-self.prior.angle))
            roll=abs(dx)>.0004 and .40 < abs(R*da/dx) < 2.2
            if roll and WELL[0] <= s.x <= WELL[1] and .003 <= s.bottom <= .015:
                n=max(1,math.ceil(abs(da)*SECTORS/(2*math.pi)*2))
                for theta in np.linspace(self.prior.angle,self.prior.angle+da,n+1):
                    sector=int(((-theta+math.pi)%(2*math.pi))*SECTORS/(2*math.pi))
                    for band,y in enumerate(BAND_CENTERS):
                        if abs(s.y+y)<.063:
                            self.wet[band,sector]=True
                self.conditioning=0.
            ramp_surface=WELL_BOTTOM - min(0.,s.x)*RAMP_HEIGHT/abs(RAMP[0])
            if (roll and self.bands==BANDS and abs(s.y)<.020
                    and RAMP[0] <= s.x < RAMP[1] and abs(s.bottom-ramp_surface)<.011):
                self.conditioning+=min(abs(dx),abs(R*da))
        parked=(abs(s.x-REST)<.024 and abs(s.y)<.022 and
                abs(s.bottom-(WELL_BOTTOM+RAMP_HEIGHT))<.010 and s.level and not s.held)
        ready=self.bands==BANDS and self.conditioning>=.069 and parked
        if not ready:
            self.park_pose,self.hold=None,0.
        elif self.park_pose is None:
            self.park_pose=s
        else:
            p=self.park_pose
            translation=math.dist((s.x,s.y,s.bottom),(p.x,p.y,p.bottom))
            cosine=(np.trace(p.rotation.T@s.rotation)-1)/2
            turn=math.acos(float(np.clip(cosine,-1.,1.)))
            if translation>.002 or turn>math.radians(3):
                self.park_pose,self.hold=s,0.
            else:
                self.hold+=dt
        self.prior=s

    @property
    def bands(self):
        return int(np.sum(self.wet.sum(axis=1)>=MIN_SECTORS))

    def value(self):
        if self.invalid:
            return 0.
        return 1. if self.hold>=5.-1e-8 else .2*self.bands


trial=Trial()
_detail={}


def score(objects):
    return trial.value()


def success(objects):
    return score(objects)==1.


def telemetry():
    return dict(_detail,covered=trial.wet.sum(axis=1).tolist(),
                ramp_mm=round(trial.conditioning*1000,1),hold=round(trial.hold,2))


def setup():
    from scenekit import UsdBody

    tray=UsdBody("paint_tray",usd="assets/paint_tray.usda",pos=TRAY,size=(.35,.15,.05),mass=None,
                 friction=(1.04,.80))
    roller=UsdBody("roller",usd="assets/paint_roller.usda",pos=START,size=(.23,.12,.05),
                   articulation=True,joint_pos=(("Axle",0.),),joint_damping=.0005,
                   rot_xyzw=FRAME_ROT,friction=(1.10,.90),solver_iters=32)
    witness=UsdBody("witness",usd="assets/witness.usda",pos=(.347,.197,TABLE),
                    size=(.154,.13,.001),mass=None)

    support=UsdBody("handle_rest",usd="assets/handle_rest.usda",pos=SUPPORT,
                    size=(.058,.034,SUPPORT_HEIGHT+.010),mass=None)
    parking=UsdBody("park_handle_rest",usd="assets/park_handle_rest.usda",pos=PARK_SUPPORT,
                    size=(.032,.040,PARK_SUPPORT_HEIGHT),mass=None)

    def color_change(environment,before,after):
        for b,c in zip(*np.where(before!=after)):
            paint_prim(environment,"roller",f"Sleeve/Band{b}_Sector{c:02}",WET if after[b,c] else CLEAN)

    def reset(environment):
        global trial,_detail
        past=trial.wet
        trial,_detail=Trial(),{}
        reset_contacts()
        color_change(environment,past,trial.wet)

    def update(environment):
        hit=contacts(environment,["roller","roller/Sleeve"])
        roller_state=bodies(environment,["roller"])["roller"]
        p,m=link_pose(environment,"roller","Sleeve")
        local=p-TRAY
        bottom=float(local[2]-R)
        rot=roller_state.rotation()
        level=abs(float(m[2,1]))<.13 and float(rot[1,1])>.96
        s=Sample(float(local[0]),float(local[1]),bottom,roller_state.joints["Axle"],
                 level,any(hit.values()),rot)
        before=trial.wet.copy()
        trial.step(s,tick(environment))
        color_change(environment,before,trial.wet)
        pitch=math.degrees(math.atan2(-float(rot[2,0]),float(rot[0,0])))
        _detail.update(x_mm=round(s.x*1000,1),y_mm=round(s.y*1000,1),
                       bottom_mm=round(s.bottom*1000,1),pitch_deg=round(pitch,1),
                       axle_deg=round(math.degrees(s.angle),1),held=s.held,
                       frame_contact=hit["roller"],level=s.level)

    return register([tray,roller,witness,support,parking],reset,update)


def tracks():
    left,right=Track(),Track()
    closed=-.003
    clearance=TABLE+.145

    def grip(x,y,h):
        return (x+GRIP_REACH,y+GRIP_Y,TABLE+h+GRIP_RISE+TIP-.009)

    start=grip(*START[:2],START[2]-TABLE)
    right.go((start[0],start[1],clearance),OPEN,"above flat grip")
    right.go(start,OPEN,"around the handle")
    right.go(start,closed,"pinch flat grip",1.0)
    right.go((start[0],start[1],start[2]+.029),closed,"lift vertically off the support",1.4)
    right.go((start[0],start[1],clearance),closed,"raise sleeve above tray rim",1.5)
    well_z=WELL_BOTTOM+R-.0015
    far=grip(TRAY[0]+WELL_FAR,TRAY[1],well_z)
    right.go((far[0],far[1],clearance),closed,"center the level sleeve inside the well",2.4)
    right.go(far,closed,"lower sleeve onto the well floor",1.8)
    near=grip(TRAY[0]+.006,TRAY[1],well_z)
    right.go(near,closed,"roll across the well",4.3)
    for x,seconds in ((-.045,1.65),(-.095,1.65),(-.146,1.65),(-.058,2.8),(-.146,2.8)):
        right.go(grip(TRAY[0]+x,TRAY[1],well_z-x*RAMP_HEIGHT/abs(RAMP[0])),
                 closed,"roll the ribbed ramp",seconds)
    park=grip(TRAY[0]+REST-.003,TRAY[1],WELL_BOTTOM+R+RAMP_HEIGHT)
    park_hover=park[2]+.027
    right.go((*right[-1].pos[:2],park_hover),closed,"lift sleeve over the parking ridge",1.0)
    right.go((*park[:2],park_hover),closed,"center sleeve over the parking rest",1.2)
    right.go(park,closed,"lower sleeve onto the tray rest",1.3)
    right.go(park,closed,"settle the roller against the rest",.65)
    right.go(park,OPEN,"set it down",.8)
    right.go((park[0],park[1],clearance),OPEN,"withdraw from the tray")
    right.go((.205,-.21,clearance),OPEN,"leave it still",6.5)
    left.go((.23,.19,clearance),OPEN,"idle clear")
    left.wait_until(right.duration)
    return {"left":left,"right":right}


def demo():
    p=tracks()
    return waypoints(p["left"],p["right"])


def self_test(run_ik=False):
    # A full coating pass fits inside the well with the sleeve clear of the front wall.
    assert FRONT_WALL-WELL_FAR-R >= .0029
    assert (WELL_FAR-.006)/R >= MIN_SECTORS*2*math.pi/SECTORS
    t=Trial()
    for x in np.linspace(WELL_FAR,.006,88):
        t.step(Sample(x,0,.006,(WELL_FAR-x)/R,True,True),.05)
    assert t.bands==4 and abs(t.value()-.8)<1e-8
    for i in range(1,42):
        x=-.0035*i
        t.step(Sample(x,0,WELL_BOTTOM-x*RAMP_HEIGHT/.15,(WELL_FAR-x)/R,True,True),.05)
    assert t.conditioning>.069
    parked=Sample(REST,0,.037,0,True,False)
    t.step(parked,.05)
    t.step(parked,5)
    assert t.value()==1.
    t.step(Sample(REST+.003,0,.037,0,True,False),.05)
    assert t.value()==.8
    cheat=Trial()
    for i in range(50):
        cheat.step(Sample(.08-i*.001,0,.006,0,True,True),.05)
    assert cheat.value()==0
    t.step(Sample(0,0,-.03,0,True,False),.05)
    assert t.value()==0
    check_tracks(tracks(),MAX_STEPS,ik=run_ik)
    print("Paint contact, coverage, skid rejection and waypoint checks passed.")


if __name__=="__main__":
    self_test("--ik" in sys.argv)
else:
    OBJECTS=setup()
