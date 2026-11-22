"""Collect pair impulses at each native solver advance, including transient arm contacts."""
from math import isfinite
import numpy as np
from fixture import SIDES, HUMAN, WORLD, DIGIT_OWNER
from scoring import HIT_N

class ContactReader:
    def __init__(self,env):
        import carb.settings
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools
        base=env.unwrapped
        self.dt=float(base.sim.get_physics_dt())
        self.n=round(float(base.step_dt)/self.dt)
        if str(base.device)!="cpu" or abs(self.n*self.dt-.05)>1e-6:raise RuntimeError("Use CPU PhysX at 20 Hz with RTX cameras")
        def path(key):
            return str(base.scene[key].cfg.prim_path).replace("{ENV_REGEX_NS}","/World/envs/env_0").replace("env_.*","env_0").replace("env_[^/]+","env_0").rstrip("/")
        self.fixture=path("rescue_trainer")
        self.robots=[path(s+"_robot") for s in SIDES]
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.api=get_physx_simulation_interface();self.sim=base.sim
        self.settings=carb.settings.get_settings();self.key="/physics/disableContactProcessing"
        self.old=self.settings.get(self.key);self.settings.set_bool(self.key,False)
        self.saved=self.sim.step;self.rows=[];self.event=None;self.error=None
        self.sub=self.api.subscribe_contact_report_events(self.callback);self.sim.step=self.advance

    def body(self,actor):
        name=str(self.decode(actor))
        if name.startswith(self.fixture+"/"):
            part=name.rsplit("/",1)[-1];return DIGIT_OWNER.get(part,part)
        if any(name.startswith(p+"/") for p in self.robots):return "robot"
        return "other"

    def decode_step(self,headers,points):
        digit=np.zeros(2);seat=np.zeros(2);landing=np.zeros(2);strike=False;touch=False;key_touch=False
        for h in headers:
            if not h.num_contact_data:continue
            pair={self.body(h.actor0),self.body(h.actor1)}
            touch |= "robot" in pair and bool(pair-{"robot","other"})
            key_touch |= "robot" in pair and bool(pair & {"key_left","key_right"})
            exposure=bool(pair & HUMAN) and bool(pair & {"robot","key_left","key_right"})
            if "panel" not in pair and not exposure:continue
            landing_side=None
            if pair=={"panel","frame"}:
                for field in ("collider0","collider1"):
                    collider=getattr(h,field,None)
                    if collider:
                        leaf=str(self.decode(collider)).rsplit("/",1)[-1]
                        if leaf in ("landing_left","landing_right"):
                            landing_side=0 if leaf=="landing_left" else 1
                            break
            force=0.
            for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                pt=points[k];val=pt.impulse
                impulse=abs(float(val)) if np.isscalar(val) else abs(float(np.dot(tuple(val),tuple(pt.normal))))
                if not isfinite(impulse):raise ValueError("invalid contact impulse")
                normal_force=impulse/self.dt
                force+=normal_force
                if pair=={"panel","frame"}:
                    # Both landing pads are on the fixed frame. Resolve individual witnesses
                    # by the contact point on the physical top face in station coordinates.
                    x,y,z=(float(k)-w for k,w in zip(pt.position,WORLD))
                    side=landing_side
                    if side is None and abs(x)<=.014 and abs(abs(y)-.052)<=.007 and abs(z-.050)<=.003:
                        side=0 if y>0 else 1
                    if side is not None:
                        landing[side]+=normal_force*abs(float(pt.normal[2]))
            strike |= exposure and force>HIT_N
            for i,side in enumerate(SIDES):
                if "panel" not in pair:continue
                target=digit if "digits_"+side in pair else seat if "key_"+side in pair else None
                if target is not None:target[i]+=force
        return digit,seat,strike,touch,landing,key_touch

    def callback(self,headers,points):
        try:
            if self.event is not None:raise RuntimeError("Multiple contact packets in a solver advance")
            self.event=self.decode_step(headers,points)
        except Exception as exc:self.error=exc

    def advance(self,*args,**kwargs):
        self.event=None
        result=self.saved(*args,**kwargs)
        h,p=self.api.get_contact_report()[:2]
        if self.error:raise RuntimeError("PhysX contact decode failed") from self.error
        self.rows.append(self.decode_step(h,p) if len(h) or self.event is None else self.event)
        return result

    def read(self):
        rows,self.rows=self.rows,[]
        if len(rows)!=self.n:raise RuntimeError(f"Missing solver steps: {len(rows)}/{self.n}")
        digit,seat,h,t,landing,key_touch=zip(*rows)
        return tuple(np.mean(digit,axis=0)),tuple(np.mean(seat,axis=0)),any(h),any(t),tuple(np.mean(landing,axis=0)),any(key_touch)

    def close(self):
        self.sim.step=self.saved
        self.sub=None
        self.settings.set_bool(self.key,bool(self.old))
