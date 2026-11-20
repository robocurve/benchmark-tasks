"""Contact-gated gel wetting and exposure-dependent, force-limited bonds.

This is a reduced adhesive law, not a chemical cure model. Neither wetting nor
lamp time moves the tip into alignment; native contact must seat it first.
"""
import numpy as np
from lamp_shadow import shadowed
from native_model import NativeModel
from geometry import BASE,LAMP,BUTTON,PATCHES,arch,tip_width,FRAME
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.grasp_state={};self.wet=np.zeros(len(PATCHES));self.dose=np.zeros(len(PATCHES));self.lamp_remaining=0.;self.button_down=False;self.spill=False;self.proof_elapsed=0.;self.proof_origin=None;self.proof_slip=0.;self.proof_force=0.
 def configure_station(self):
  from pad_covers import install
  install()
 def bind_native(self,base):
  super().bind_native(base)
  from native_wrench import NativeWrench
  self.proof_wrench=NativeWrench(self,['extension'])
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,['extension'],.0058,.003,.002,(-.009,.003))
 def material(self,hands,dt):
  from pxr import Gf
  self.pinch.update(hands)
  depth=float(-self.joint_gap('button_spring')[2]);pressed=depth>.0008
  if pressed and not self.button_down:self.lamp_remaining=60.;self.events.append(dict(event='lamp_60_second_cycle',time=self.time))
  self.button_down=pressed;lit=self.lamp_remaining>0;self.lamp_remaining=max(0.,self.lamp_remaining-dt)
  for i,p in enumerate(PATCHES):
   point=self.p(p['name'])+[0,0,.00012];q=self.orientation('extension').T@(point-self.p('extension'))
   within=-.020<=q[0]<=.008 and abs(q[1])<=tip_width(q[0])
   gap=float(q[2]-(.003-arch(np.clip(q[1],-.013,.013))))
   contact=within and -.00015<gap<.00020
   if contact:self.wet[i]=min(1.,self.wet[i]+dt)
   elif abs(gap)>.001 and self.dose[i]<10:self.wet[i]=0.
   # A narrow distal pinch leaves the gel bed exposed; any hand over a patch
   # masks the lamp rather than allowing light through an opaque gripper.
   shadow=shadowed(point,hands,LAMP[2]+.041)
   name='bond_'+p['name']
   attached=name in self.enabled and name not in self.broken
   # A live flash-cured bond keeps the same wetted interface attached while
   # its finite compliance permits small elastic displacement. Further light
   # exposure cures that bond; a broken bond gains no such exemption.
   if lit and (contact or attached) and self.wet[i]>=1. and not shadow:self.dose[i]+=dt
   if self.dose[i]>=10. and name not in self.enabled:
    j=self.joints[name];j.GetLocalPos0Attr().Set(Gf.Vec3f(*(FRAME.T@(point-BASE))));j.GetLocalPos1Attr().Set(Gf.Vec3f(*q));self.enable(name,'contact_gel_flash_cured')
   if name in self.enabled:self.joints[name].GetBreakForceAttr().Set(.08*min(1.,self.dose[i]/40.))
  # Gel carried outside the nail can smear onto the surrounding practice skin.
  for p in PATCHES:
   point=self.p('extension',[p['x'],p['y'],.003-arch(p['y'])])
   q=FRAME.T@(point-BASE)
   outside=not(-.0205<q[0]<-.0055 and abs(q[1])<.0065)
   local=(q-[-.027,0,-.009])/np.array([.032,.011,.010])
   if outside and abs(np.linalg.norm(local)-1.)<.04:self.spill=True
  # A one-newton outward proof load after release tests the actual native bonds.
  force=0.
  if self.release>=3 and np.count_nonzero(self.dose>=40)>=len(PATCHES)//2:
   if self.proof_origin is None:self.proof_origin=self.p('extension').copy()
   self.proof_slip=max(self.proof_slip,float(np.linalg.norm(self.p('extension')-self.proof_origin)))
   self.proof_elapsed=min(4.,self.proof_elapsed+dt);t=self.proof_elapsed;force=min(1.,t/.5,max(0.,(4-t)/.5))
  else:self.proof_elapsed=0.;self.proof_origin=None;self.proof_slip=0.
  self.proof_force=force;self.proof_wrench.set([[0.,0.,force]])
 def measure(self):
  reference=np.array([-.020,0,.003]);delta=FRAME.T@(self.p('extension',reference)-(BASE+FRAME@reference));R=FRAME.T@self.orientation('extension');tilt=float(np.degrees(np.arccos(np.clip((np.trace(R)-1)/2,-1,1))))
  bonded=[i for i,p in enumerate(PATCHES) if self.dose[i]>=40. and 'bond_'+p['name'] in self.enabled and 'bond_'+p['name'] not in self.broken]
  return dict(bonded_fraction=len(bonded)/len(PATCHES),alignment_mm=float(np.linalg.norm(delta[:2])*1000),tilt_deg=tilt,proof_complete=self.proof_elapsed>=4.-1e-8,proof_slip_mm=self.proof_slip*1000,released_seconds=self.release,wetted_fraction=float(np.mean(self.wet>=.95)),gel_on_skin=self.spill,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed()
 def score(self):
  m=self.measure()
  if self.failed or self.spill or not m['proof_complete'] or m['proof_slip_mm']>1. or m['alignment_mm']>1.5 or m['tilt_deg']>8:return 0.
  f=m['bonded_fraction'];return 1. if f>=.9 else .5 if f>=.75 else .25 if f>=.5 else 0.
 def state(self):return dict(**super().state(),grasp_state=self.grasp_state,wet=self.wet,dose=self.dose,lamp_remaining=self.lamp_remaining,button_down=self.button_down,spill=self.spill,proof_elapsed=self.proof_elapsed,proof_origin=self.proof_origin,proof_slip=self.proof_slip,proof_force=self.proof_force)
 def restore(self,state):
  s=dict(state)
  for key in ['wet','dose','proof_origin']:
   value=s.pop(key);setattr(self,key,None if value is None else np.asarray(value))
  super().restore(s)
