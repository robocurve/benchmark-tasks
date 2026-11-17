"""Wetting under compressive sponge load, then transfer during backing removal.

This is a reduced adhesive/water law. Paper, sponge and arms remain native
contact bodies. The law never places paper or moves either robot.
"""
import numpy as np
from native_model import NativeModel
from geometry import INK,INK_BODY,INK_ORIGINAL,ARM_RADIUS,DX,DY,PADCOUNT
class Model(NativeModel):
 def __init__(self):
  super().__init__();self.wet={n:0. for n in INK};self.contact_uv={};self.transferred={};self.lost=set();self.max_load=0.
 def p(self,name,local=(0,0,0)):
  if name in INK_BODY:
   body,offset,index=INK_BODY[name];return super().p(body,np.asarray(offset)+local)
  return super().p(name,local)
 def orientation(self,name):return super().orientation(INK_BODY[name][0] if name in INK_BODY else name)
 def validate_reset(self):
  from geometry import PAPER
  centre=(self.p('paper_band_5')+self.p('paper_band_6'))/2
  if np.linalg.norm(centre-PAPER)>.010:raise RuntimeError('Backing moved off its specified starting rest during settling.')
 def material(self,hands,dt):
  from pxr import Gf,UsdPhysics
  # Compression is measured from passive native spring motion, not a requested pose.
  compression=np.array([max(0.,float((self.orientation('sponge').T@self.joint_gap('compression_'+str(i)))[2])) for i in range(PADCOUNT)])
  load=100*compression;self.max_load=max(self.max_load,float(load.sum()))
  previous_wet=self.wet.copy()
  for name in INK:
   p=self.p(name);local=self.orientation('forearm').T@(p-self.p('forearm'));radius=np.linalg.norm(local[1:]);normal=np.r_[0.,local[1:]/max(radius,1e-9)]
   paper_normal=self.orientation('forearm').T@self.orientation(name)[:,2]
   on_arm=abs(local[0])<.09 and abs(radius-ARM_RADIUS)<.0006 and paper_normal@normal>.8
   pressure=0.
   for i in range(PADCOUNT):
    q=self.orientation('sponge_pad_'+str(i)).T@(p-self.p('sponge_pad_'+str(i)))
    # Actual paper lies against the loaded underside of this sponge pad.
    if abs(q[0])<.006 and abs(q[1])<.006 and abs(q[2]+.005)<.0007:pressure+=load[i]
   tack='tack_'+name
   # Water spreads through the continuous backing from a genuinely wetted
   # neighbour. This keeps the printed face in actual skin contact; it does
   # not treat a sponge hovering over dry paper as a wetting source.
   wicked=any(other!=name and previous_wet[other]>=1. and np.linalg.norm(np.asarray(INK_ORIGINAL[other])-INK_ORIGINAL[name])<=.0071 and np.linalg.norm(self.p(other)-p)<=.008 for other in INK)
   if on_arm and (pressure>.03 or wicked) and name not in self.lost:
    self.wet[name]+=dt
    # The coating becomes adhesive after actual wet contact. Before wetting,
    # paper is supported by sponge contact, not glued rigidly onto the form.
    if self.wet[name]>=30. and tack not in self.enabled:
     j=self.joints[tack];j.GetLocalPos0Attr().Set(Gf.Vec3f(*local));j.GetLocalPos1Attr().Set(Gf.Vec3f(*INK_BODY[name][1]));j.GetBreakForceAttr().Set(.015)
     self.contact_uv[name]=local.copy();self.enable(tack,'wetted_coating_adhesive_contact')
   if 0.<self.wet[name]<30. and abs(radius-ARM_RADIUS)>.003 and name not in self.lost:
    self.lost.add(name);self.events.append(dict(event='backing_lifted_before_transfer',patch=name,time=self.time))
   if tack in self.broken and name not in self.transferred:
    self.transferred[name]=self.contact_uv[name].copy();self.events.append(dict(event='wet_image_transferred',patch=name,time=self.time))
  self.failed |= any(n.startswith(('sheet_','compression_')) for n in self.broken)
 def measure(self):
  from geometry import BODIES,PAPER
  names=list(self.transferred);positions=[self.transferred[n] for n in names]
  uv=np.array([[p[0],np.arctan2(p[1],p[2])*ARM_RADIUS] for p in positions]).reshape(-1,2)
  coherent=np.ones(len(names),dtype=bool)
  if len(names)>1:
   original=INK_ORIGINAL
   src=np.array([original[n] for n in names]);a=src-src.mean(axis=0);b=uv-uv.mean(axis=0)
   U,_,V=np.linalg.svd(a.T@b);Q=U@V
   if np.linalg.det(Q)<0:U[:,-1]*=-1;Q=U@V
   residual=np.linalg.norm(a@Q-b,axis=1);dist=np.linalg.norm(uv[:,None]-uv[None,:],axis=-1);np.fill_diagonal(dist,np.inf)
   coherent=(residual<.003)&(dist.min(axis=1)>.0035)
  good=[p for p,ok in zip(positions,coherent) if ok and abs(p[0])<.020 and abs(np.arctan2(p[1],p[2])*ARM_RADIUS)<.0175]
  paper=np.array([self.p(n) for n in self.names if n.startswith('paper_')]);local=(paper-self.p('forearm'))@self.orientation('forearm')
  backing_clear=bool(np.all((np.abs(local[:,0])>.11)|(np.abs(np.linalg.norm(local[:,1:],axis=1)-ARM_RADIUS)>.010)))
  return dict(transferred_fraction=len(self.transferred)/len(INK),registered_fraction=len(good)/len(INK),backing_clear=backing_clear,backing_intact=not self.failed,released_seconds=self.release,max_sponge_load_N=self.max_load,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed() or (any(n.startswith(('sheet_','compression_')) for n in self.broken))
 def score(self):
  self.failed |= self.integrity_failed()
  m=self.measure()
  if self.failed:return 0.
  f=m['registered_fraction']
  if f>=.95 and m['backing_clear'] and self.release>=3:return 1.
  return .5 if f>=.75 and m['backing_clear'] else .25 if f>=.5 else 0.
 def state(self):
  return dict(**super().state(),wet=self.wet,contact_uv=self.contact_uv,transferred=self.transferred,lost=sorted(self.lost),max_load=self.max_load)
 def draw(self,d):
  from pxr import UsdGeom
  for name,p in self.transferred.items():
   # Only the image coating changes substrate. Native paper motion is untouched.
   body,offset,index=INK_BODY[name]
   prims=[x for x in self.stage.Traverse() if str(x.GetPath()).endswith('/'+body+'/shape_'+str(index))]
   for prim in prims:UsdGeom.Imageable(prim).MakeInvisible()
   angle=np.arctan2(p[1],p[2]);pts=[]
   for x,y in [(-DX/2,-DY/2),(DX/2,-DY/2),(DX/2,DY/2),(-DX/2,DY/2)]:
    a=angle+y/ARM_RADIUS;local=np.array([p[0]+x,(ARM_RADIUS+.0003)*np.sin(a),(ARM_RADIUS+.0003)*np.cos(a)])
    pts.append(self.p('forearm',local))
   d.mesh('transferred_'+name,np.array(pts),[(0,1,2,3)],(.8,.03,.05))
